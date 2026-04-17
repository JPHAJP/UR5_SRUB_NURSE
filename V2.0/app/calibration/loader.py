from __future__ import annotations

import json
from pathlib import Path

from .schema import CameraCalibration


DEFAULT_CALIBRATION = CameraCalibration.from_dict(
    {
        "camera": {
            "index": 0,
            "width": 1280,
            "height": 720,
            "fps": 30,
        },
        "hand_follow": {
            "plane_z_mm": 350.0,
            "image_points": [],
            "world_points_mm": [],
            "homography": [],
            "workspace": {
                "x": [-450.0, 450.0],
                "y": [-700.0, 150.0],
                "z": [150.0, 700.0],
            },
        },
        "object_pick": {
            "plane_z_mm": 120.0,
            "image_points": [],
            "world_points_mm": [],
            "homography": [],
            "workspace": {
                "x": [-450.0, 450.0],
                "y": [-700.0, 150.0],
                "z": [20.0, 600.0],
            },
        },
    }
)


def load_calibration(path: Path) -> CameraCalibration:
    if not path.exists():
        return DEFAULT_CALIBRATION

    try:
        data = json.loads(path.read_text(encoding="utf-8"))
        return CameraCalibration.from_dict(data)
    except (json.JSONDecodeError, OSError, TypeError, ValueError):
        return DEFAULT_CALIBRATION


def save_calibration(path: Path, calibration: CameraCalibration) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(
        json.dumps(calibration.to_dict(), indent=2, ensure_ascii=True),
        encoding="utf-8",
    )
