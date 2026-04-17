from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any, Dict, List


@dataclass(slots=True)
class ModeCalibration:
    plane_z_mm: float
    image_points: List[List[float]] = field(default_factory=list)
    world_points_mm: List[List[float]] = field(default_factory=list)
    homography: List[List[float]] = field(default_factory=list)
    workspace: Dict[str, Any] = field(default_factory=dict)

    @classmethod
    def from_dict(cls, data: Dict[str, Any]) -> "ModeCalibration":
        return cls(
            plane_z_mm=float(data.get("plane_z_mm", 0.0)),
            image_points=list(data.get("image_points", [])),
            world_points_mm=list(data.get("world_points_mm", [])),
            homography=list(data.get("homography", [])),
            workspace=dict(data.get("workspace", {})),
        )

    def to_dict(self) -> Dict[str, Any]:
        return {
            "plane_z_mm": self.plane_z_mm,
            "image_points": self.image_points,
            "world_points_mm": self.world_points_mm,
            "homography": self.homography,
            "workspace": self.workspace,
        }


@dataclass(slots=True)
class CameraCalibration:
    camera: Dict[str, Any]
    hand_follow: ModeCalibration
    object_pick: ModeCalibration

    @classmethod
    def from_dict(cls, data: Dict[str, Any]) -> "CameraCalibration":
        return cls(
            camera=dict(data.get("camera", {})),
            hand_follow=ModeCalibration.from_dict(data.get("hand_follow", {})),
            object_pick=ModeCalibration.from_dict(data.get("object_pick", {})),
        )

    def to_dict(self) -> Dict[str, Any]:
        return {
            "camera": self.camera,
            "hand_follow": self.hand_follow.to_dict(),
            "object_pick": self.object_pick.to_dict(),
        }
