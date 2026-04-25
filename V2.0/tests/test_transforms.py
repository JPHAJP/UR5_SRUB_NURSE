import os
import sys

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))

from app.calibration.transforms import (
    apply_homography,
    apply_rigid_transform,
    build_transform_matrix,
    build_transform_from_ur_pose,
    compose_transform_matrices,
    compute_homography,
    deproject_pixel_to_camera_xyz,
    fit_depth_compensation,
)


def test_homography_maps_rectangle_corners():
    image_points = [(0.0, 0.0), (100.0, 0.0), (100.0, 50.0), (0.0, 50.0)]
    world_points = [(0.0, 0.0), (200.0, 0.0), (200.0, 100.0), (0.0, 100.0)]
    homography = compute_homography(image_points, world_points)

    x_mm, y_mm = apply_homography((50.0, 25.0), homography)
    assert round(x_mm, 2) == 100.0
    assert round(y_mm, 2) == 50.0


def test_deproject_pixel_to_camera_xyz_uses_intrinsics():
    xyz = deproject_pixel_to_camera_xyz(
        point_xy=(420.0, 260.0),
        depth_mm=500.0,
        intrinsics={"fx": 500.0, "fy": 500.0, "cx": 320.0, "cy": 240.0},
    )

    assert xyz == [100.0, 20.0, 500.0]


def test_build_and_apply_rigid_transform_translation_only():
    matrix = build_transform_matrix([100.0, -50.0, 25.0], [0.0, 0.0, 0.0])
    world = apply_rigid_transform([10.0, 20.0, 30.0], matrix)

    assert world == [110.0, -30.0, 55.0]


def test_ur_pose_composes_tcp_to_camera_offset():
    base_to_tcp = build_transform_from_ur_pose([100.0, 200.0, 300.0, 0.0, 0.0, 0.0])
    tcp_to_camera = build_transform_matrix([55.0, -30.0, 0.0], [0.0, 0.0, 0.0])
    base_to_camera = compose_transform_matrices(base_to_tcp, tcp_to_camera)

    world = apply_rigid_transform([10.0, 20.0, 30.0], base_to_camera)

    assert world == [165.0, 190.0, 330.0]


def test_ur_rotation_vector_rotates_tcp_camera_offset():
    base_to_tcp = build_transform_from_ur_pose([0.0, 0.0, 0.0, 0.0, 0.0, 1.5707963267948966])
    tcp_to_camera = build_transform_matrix([55.0, -30.0, 0.0], [0.0, 0.0, 0.0])
    base_to_camera = compose_transform_matrices(base_to_tcp, tcp_to_camera)

    world = apply_rigid_transform([0.0, 0.0, 0.0], base_to_camera)

    assert round(world[0], 3) == 30.0
    assert round(world[1], 3) == 55.0
    assert round(world[2], 3) == 0.0


def test_fit_depth_compensation_with_single_sample_keeps_gain_one():
    fit = fit_depth_compensation([{"camera_depth_mm": 480.0, "robot_z_mm": 500.0}])

    assert fit["gain"] == 1.0
    assert fit["offset_mm"] == 20.0


def test_fit_depth_compensation_with_multiple_samples_solves_affine():
    fit = fit_depth_compensation(
        [
            {"camera_depth_mm": 100.0, "robot_z_mm": 130.0},
            {"camera_depth_mm": 200.0, "robot_z_mm": 250.0},
            {"camera_depth_mm": 300.0, "robot_z_mm": 370.0},
        ]
    )

    assert round(fit["gain"], 3) == 1.2
    assert round(fit["offset_mm"], 3) == 10.0
