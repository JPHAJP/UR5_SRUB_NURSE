from __future__ import annotations

from typing import Dict, List, Sequence


DEFAULT_ORIENTATION = [0.0, 3.142, 0.0]


def build_greet_sequence(greet_sequence_deg: Sequence[Sequence[float]]) -> List[List[float]]:
    return [list(step) for step in greet_sequence_deg]


def build_pick_waypoints(
    target_xy_mm: Sequence[float],
    plane_z_mm: float,
    approach_lift_mm: float,
) -> Dict[str, List[float]]:
    x_mm, y_mm = float(target_xy_mm[0]), float(target_xy_mm[1])
    approach = [x_mm, y_mm, plane_z_mm + approach_lift_mm]
    pick = [x_mm, y_mm, plane_z_mm]
    retreat = [x_mm, y_mm, plane_z_mm + approach_lift_mm]
    return {
        "approach": approach,
        "pick": pick,
        "retreat": retreat,
    }
