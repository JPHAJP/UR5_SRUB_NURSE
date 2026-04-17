from __future__ import annotations

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
