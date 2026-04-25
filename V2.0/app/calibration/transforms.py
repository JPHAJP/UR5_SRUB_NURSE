from __future__ import annotations

import math
from typing import Dict, Iterable, List, Sequence, Tuple

import numpy as np


def compute_homography(
    image_points: Sequence[Sequence[float]],
    world_points: Sequence[Sequence[float]],
) -> List[List[float]]:
    if len(image_points) < 4 or len(world_points) < 4:
        raise ValueError("Se requieren al menos 4 puntos para calcular la homografia.")

    a_rows = []
    for (u, v), (x, y) in zip(image_points, world_points):
        a_rows.append([-u, -v, -1, 0, 0, 0, x * u, x * v, x])
        a_rows.append([0, 0, 0, -u, -v, -1, y * u, y * v, y])

    matrix = np.asarray(a_rows, dtype=float)
    _, _, vh = np.linalg.svd(matrix)
    homography = vh[-1].reshape(3, 3)
    homography /= homography[2, 2]
    return homography.tolist()


def apply_homography(point_xy: Sequence[float], homography: Sequence[Sequence[float]]) -> Tuple[float, float]:
    matrix = np.asarray(homography, dtype=float)
    point = np.asarray([point_xy[0], point_xy[1], 1.0], dtype=float)
    transformed = matrix @ point
    transformed /= transformed[2]
    return float(transformed[0]), float(transformed[1])


def deproject_pixel_to_camera_xyz(
    point_xy: Sequence[float],
    depth_mm: float,
    intrinsics: Dict[str, float] | Sequence[float],
) -> List[float]:
    if isinstance(intrinsics, dict):
        fx = float(intrinsics.get("fx", 0.0) or 0.0)
        fy = float(intrinsics.get("fy", 0.0) or 0.0)
        cx = float(intrinsics.get("cx", 0.0) or 0.0)
        cy = float(intrinsics.get("cy", 0.0) or 0.0)
    else:
        fx, fy, cx, cy = [float(value) for value in intrinsics[:4]]

    if fx <= 0.0 or fy <= 0.0:
        raise ValueError("Intrinsecos invalidos para deproyeccion.")

    z_mm = float(depth_mm)
    u = float(point_xy[0])
    v = float(point_xy[1])
    x_mm = (u - cx) * z_mm / fx
    y_mm = (v - cy) * z_mm / fy
    return [float(x_mm), float(y_mm), z_mm]


def build_transform_matrix(
    translation_mm: Sequence[float],
    rotation_rpy_deg: Sequence[float],
) -> List[List[float]]:
    tx, ty, tz = [float(value) for value in translation_mm[:3]]
    roll, pitch, yaw = [math.radians(float(value)) for value in rotation_rpy_deg[:3]]

    rx = np.array(
        [
            [1.0, 0.0, 0.0],
            [0.0, math.cos(roll), -math.sin(roll)],
            [0.0, math.sin(roll), math.cos(roll)],
        ],
        dtype=float,
    )
    ry = np.array(
        [
            [math.cos(pitch), 0.0, math.sin(pitch)],
            [0.0, 1.0, 0.0],
            [-math.sin(pitch), 0.0, math.cos(pitch)],
        ],
        dtype=float,
    )
    rz = np.array(
        [
            [math.cos(yaw), -math.sin(yaw), 0.0],
            [math.sin(yaw), math.cos(yaw), 0.0],
            [0.0, 0.0, 1.0],
        ],
        dtype=float,
    )

    rotation = rz @ ry @ rx
    transform = np.eye(4, dtype=float)
    transform[:3, :3] = rotation
    transform[:3, 3] = np.asarray([tx, ty, tz], dtype=float)
    return transform.tolist()


def build_transform_from_ur_pose(pose_mm: Sequence[float]) -> List[List[float]]:
    if len(pose_mm) < 6:
        raise ValueError("La pose UR debe incluir x, y, z, rx, ry, rz.")

    x_mm, y_mm, z_mm = [float(value) for value in pose_mm[:3]]
    rotation_vector = np.asarray([float(value) for value in pose_mm[3:6]], dtype=float)
    angle = float(np.linalg.norm(rotation_vector))

    if angle <= 1e-12:
        rotation = np.eye(3, dtype=float)
    else:
        axis = rotation_vector / angle
        kx, ky, kz = axis
        skew = np.asarray(
            [
                [0.0, -kz, ky],
                [kz, 0.0, -kx],
                [-ky, kx, 0.0],
            ],
            dtype=float,
        )
        rotation = (
            np.eye(3, dtype=float)
            + math.sin(angle) * skew
            + (1.0 - math.cos(angle)) * (skew @ skew)
        )

    transform = np.eye(4, dtype=float)
    transform[:3, :3] = rotation
    transform[:3, 3] = np.asarray([x_mm, y_mm, z_mm], dtype=float)
    return transform.tolist()


def compose_transform_matrices(*transforms: Sequence[Sequence[float]]) -> List[List[float]]:
    composed = np.eye(4, dtype=float)
    for transform in transforms:
        matrix = np.asarray(transform, dtype=float)
        if matrix.shape != (4, 4):
            raise ValueError("Cada matriz de transformacion debe ser 4x4.")
        composed = composed @ matrix
    return composed.tolist()


def apply_rigid_transform(
    point_xyz_mm: Sequence[float],
    transform_matrix: Sequence[Sequence[float]],
) -> List[float]:
    matrix = np.asarray(transform_matrix, dtype=float)
    if matrix.shape != (4, 4):
        raise ValueError("La matriz de transformacion debe ser 4x4.")

    point = np.asarray([float(point_xyz_mm[0]), float(point_xyz_mm[1]), float(point_xyz_mm[2]), 1.0], dtype=float)
    transformed = matrix @ point
    return [float(transformed[0]), float(transformed[1]), float(transformed[2])]


def fit_depth_compensation(samples: Sequence[Dict[str, float] | Sequence[float]]) -> Dict[str, float]:
    normalized = []
    for sample in samples:
        if isinstance(sample, dict):
            camera_depth_mm = float(sample.get("camera_depth_mm", 0.0) or 0.0)
            robot_z_mm = float(sample.get("robot_z_mm", 0.0) or 0.0)
        else:
            camera_depth_mm = float(sample[0])
            robot_z_mm = float(sample[1])
        if camera_depth_mm > 0.0:
            normalized.append((camera_depth_mm, robot_z_mm))

    if not normalized:
        return {
            "gain": 1.0,
            "offset_mm": 0.0,
            "last_error_abs_mm": 0.0,
            "last_error_mean_mm": 0.0,
        }

    if len(normalized) == 1:
        camera_depth_mm, robot_z_mm = normalized[0]
        predicted = camera_depth_mm
        error = robot_z_mm - predicted
        return {
            "gain": 1.0,
            "offset_mm": error,
            "last_error_abs_mm": abs(error),
            "last_error_mean_mm": abs(error),
        }

    depths = np.asarray([item[0] for item in normalized], dtype=float)
    robot_z = np.asarray([item[1] for item in normalized], dtype=float)
    design = np.column_stack([depths, np.ones(len(depths), dtype=float)])
    gain, offset_mm = np.linalg.lstsq(design, robot_z, rcond=None)[0]
    predicted = gain * depths + offset_mm
    errors = robot_z - predicted
    return {
        "gain": float(gain),
        "offset_mm": float(offset_mm),
        "last_error_abs_mm": float(abs(errors[-1])),
        "last_error_mean_mm": float(np.mean(np.abs(errors))),
    }


def workspace_contains(workspace: Dict[str, Iterable[float]], point_xyz_mm: Sequence[float]) -> bool:
    x_range = list(workspace.get("x", (-1e9, 1e9)))
    y_range = list(workspace.get("y", (-1e9, 1e9)))
    z_range = list(workspace.get("z", (-1e9, 1e9)))
    x, y, z = point_xyz_mm
    return x_range[0] <= x <= x_range[1] and y_range[0] <= y <= y_range[1] and z_range[0] <= z <= z_range[1]
