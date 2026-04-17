from __future__ import annotations

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


def workspace_contains(workspace: Dict[str, Iterable[float]], point_xyz_mm: Sequence[float]) -> bool:
    x_range = list(workspace.get("x", (-1e9, 1e9)))
    y_range = list(workspace.get("y", (-1e9, 1e9)))
    z_range = list(workspace.get("z", (-1e9, 1e9)))
    x, y, z = point_xyz_mm
    return x_range[0] <= x <= x_range[1] and y_range[0] <= y <= y_range[1] and z_range[0] <= z <= z_range[1]
