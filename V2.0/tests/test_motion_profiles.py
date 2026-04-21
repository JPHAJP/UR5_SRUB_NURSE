import os
import sys

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))

from app.robot.motion_profiles import (
    apply_deadzone,
    estimate_joint_move_duration_deg,
    smooth_velocity,
    velocity_from_error_mm,
)


def test_deadzone_zeroes_small_values():
    assert apply_deadzone(2.0, 5.0) == 0.0
    assert apply_deadzone(8.0, 5.0) == 8.0


def test_velocity_from_error_clamps_and_respects_sign():
    assert velocity_from_error_mm(500.0, 120.0, 10.0) == 120.0
    assert velocity_from_error_mm(-500.0, 120.0, 10.0) == -120.0


def test_smooth_velocity_interpolates():
    result = smooth_velocity([0.0, 0.0, 0.0], [100.0, -100.0, 50.0], 0.25)
    assert result == [25.0, -25.0, 12.5]


def test_estimated_joint_move_duration_grows_with_joint_distance():
    short_move = estimate_joint_move_duration_deg(
        [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        [5.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        speed_rad_s=1.5,
        acceleration_rad_s2=2.5,
    )
    long_move = estimate_joint_move_duration_deg(
        [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        [90.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        speed_rad_s=1.5,
        acceleration_rad_s2=2.5,
    )

    assert short_move > 0.0
    assert long_move > short_move
