from __future__ import annotations

import math
from typing import Iterable, List


def clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


def apply_deadzone(value: float, threshold: float) -> float:
    return 0.0 if abs(value) < threshold else value


def smooth_velocity(previous: Iterable[float], current: Iterable[float], alpha: float) -> List[float]:
    return [
        alpha * float(new) + (1.0 - alpha) * float(old)
        for old, new in zip(previous, current)
    ]


def limit_velocity_acceleration(
    previous: Iterable[float],
    current: Iterable[float],
    max_accel_mm_s2: float,
    dt_s: float,
) -> List[float]:
    if max_accel_mm_s2 <= 0.0 or dt_s <= 0.0:
        return [float(value) for value in current]

    max_delta = float(max_accel_mm_s2) * float(dt_s)
    return [
        float(old) + clamp(float(new) - float(old), -max_delta, max_delta)
        for old, new in zip(previous, current)
    ]


def velocity_from_error_mm(
    error_mm: float,
    max_speed_mm_s: float,
    deadzone_mm: float,
    proportional_gain: float = 0.4,
) -> float:
    filtered_error = apply_deadzone(error_mm, deadzone_mm)
    if filtered_error == 0.0:
        return 0.0
    raw_velocity = filtered_error * proportional_gain
    return clamp(raw_velocity, -max_speed_mm_s, max_speed_mm_s)


def estimate_joint_move_duration_deg(
    start_joints_deg: Iterable[float],
    target_joints_deg: Iterable[float],
    speed_rad_s: float,
    acceleration_rad_s2: float,
) -> float:
    if speed_rad_s <= 0.0 or acceleration_rad_s2 <= 0.0:
        return 0.0

    max_delta_rad = max(
        abs(math.radians(float(target) - float(start)))
        for start, target in zip(start_joints_deg, target_joints_deg)
    )
    if max_delta_rad <= 0.0:
        return 0.0

    accel_time_s = speed_rad_s / acceleration_rad_s2
    accel_distance_rad = 0.5 * acceleration_rad_s2 * (accel_time_s ** 2)

    if max_delta_rad <= 2.0 * accel_distance_rad:
        return 2.0 * math.sqrt(max_delta_rad / acceleration_rad_s2)

    cruise_distance_rad = max_delta_rad - (2.0 * accel_distance_rad)
    return (2.0 * accel_time_s) + (cruise_distance_rad / speed_rad_s)


def estimate_linear_move_duration_mm(
    start_xyz_mm: Iterable[float],
    target_xyz_mm: Iterable[float],
    speed_m_s: float,
    acceleration_m_s2: float,
) -> float:
    if speed_m_s <= 0.0 or acceleration_m_s2 <= 0.0:
        return 0.0

    deltas_mm = [
        float(target) - float(start)
        for start, target in zip(start_xyz_mm, target_xyz_mm)
    ]
    distance_m = math.sqrt(sum((delta / 1000.0) ** 2 for delta in deltas_mm))
    if distance_m <= 0.0:
        return 0.0

    accel_time_s = speed_m_s / acceleration_m_s2
    accel_distance_m = 0.5 * acceleration_m_s2 * (accel_time_s ** 2)

    if distance_m <= 2.0 * accel_distance_m:
        return 2.0 * math.sqrt(distance_m / acceleration_m_s2)

    cruise_distance_m = distance_m - (2.0 * accel_distance_m)
    return (2.0 * accel_time_s) + (cruise_distance_m / speed_m_s)
