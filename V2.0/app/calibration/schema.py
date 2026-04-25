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
class CameraIntrinsics:
    width: int = 0
    height: int = 0
    fx: float = 0.0
    fy: float = 0.0
    cx: float = 0.0
    cy: float = 0.0
    frame_id: str = ""
    topic: str = ""

    @classmethod
    def from_dict(cls, data: Dict[str, Any]) -> "CameraIntrinsics":
        return cls(
            width=int(data.get("width", 0) or 0),
            height=int(data.get("height", 0) or 0),
            fx=float(data.get("fx", 0.0) or 0.0),
            fy=float(data.get("fy", 0.0) or 0.0),
            cx=float(data.get("cx", 0.0) or 0.0),
            cy=float(data.get("cy", 0.0) or 0.0),
            frame_id=str(data.get("frame_id", "") or ""),
            topic=str(data.get("topic", "") or ""),
        )

    def to_dict(self) -> Dict[str, Any]:
        return {
            "width": self.width,
            "height": self.height,
            "fx": self.fx,
            "fy": self.fy,
            "cx": self.cx,
            "cy": self.cy,
            "frame_id": self.frame_id,
            "topic": self.topic,
        }

    def is_valid(self) -> bool:
        return self.width > 0 and self.height > 0 and self.fx > 0.0 and self.fy > 0.0


@dataclass(slots=True)
class CameraTransformCalibration:
    translation_mm: List[float] = field(default_factory=lambda: [0.0, 0.0, 0.0])
    rotation_rpy_deg: List[float] = field(default_factory=lambda: [0.0, 0.0, 0.0])
    matrix_4x4: List[List[float]] = field(default_factory=list)
    configured: bool = False

    @classmethod
    def from_dict(cls, data: Dict[str, Any]) -> "CameraTransformCalibration":
        return cls(
            translation_mm=list(data.get("translation_mm", [0.0, 0.0, 0.0])),
            rotation_rpy_deg=list(data.get("rotation_rpy_deg", [0.0, 0.0, 0.0])),
            matrix_4x4=list(data.get("matrix_4x4", [])),
            configured=bool(data.get("configured", False)),
        )

    def to_dict(self) -> Dict[str, Any]:
        return {
            "translation_mm": self.translation_mm,
            "rotation_rpy_deg": self.rotation_rpy_deg,
            "matrix_4x4": self.matrix_4x4,
            "configured": self.configured,
        }


@dataclass(slots=True)
class DepthCalibrationSample:
    camera_depth_mm: float
    robot_z_mm: float
    compensated_depth_mm: float = 0.0
    error_mm: float = 0.0
    timestamp: float = 0.0

    @classmethod
    def from_dict(cls, data: Dict[str, Any]) -> "DepthCalibrationSample":
        return cls(
            camera_depth_mm=float(data.get("camera_depth_mm", 0.0) or 0.0),
            robot_z_mm=float(data.get("robot_z_mm", 0.0) or 0.0),
            compensated_depth_mm=float(data.get("compensated_depth_mm", 0.0) or 0.0),
            error_mm=float(data.get("error_mm", 0.0) or 0.0),
            timestamp=float(data.get("timestamp", 0.0) or 0.0),
        )

    def to_dict(self) -> Dict[str, Any]:
        return {
            "camera_depth_mm": self.camera_depth_mm,
            "robot_z_mm": self.robot_z_mm,
            "compensated_depth_mm": self.compensated_depth_mm,
            "error_mm": self.error_mm,
            "timestamp": self.timestamp,
        }


@dataclass(slots=True)
class DepthCompensationCalibration:
    gain: float = 1.0
    offset_mm: float = 0.0
    last_error_abs_mm: float = 0.0
    last_error_mean_mm: float = 0.0
    configured: bool = False
    samples: List[DepthCalibrationSample] = field(default_factory=list)

    @classmethod
    def from_dict(cls, data: Dict[str, Any]) -> "DepthCompensationCalibration":
        return cls(
            gain=float(data.get("gain", 1.0) or 1.0),
            offset_mm=float(data.get("offset_mm", 0.0) or 0.0),
            last_error_abs_mm=float(data.get("last_error_abs_mm", 0.0) or 0.0),
            last_error_mean_mm=float(data.get("last_error_mean_mm", 0.0) or 0.0),
            configured=bool(data.get("configured", False)),
            samples=[
                DepthCalibrationSample.from_dict(item)
                for item in list(data.get("samples", []))
            ],
        )

    def to_dict(self) -> Dict[str, Any]:
        return {
            "gain": self.gain,
            "offset_mm": self.offset_mm,
            "last_error_abs_mm": self.last_error_abs_mm,
            "last_error_mean_mm": self.last_error_mean_mm,
            "configured": self.configured,
            "samples": [item.to_dict() for item in self.samples],
        }


@dataclass(slots=True)
class CameraCalibration:
    camera: Dict[str, Any]
    hand_follow: ModeCalibration
    object_pick: ModeCalibration
    backend: str = "hp60c_ros2"
    topics: Dict[str, str] = field(default_factory=dict)
    intrinsics: Dict[str, CameraIntrinsics] = field(default_factory=dict)
    camera_to_robot: CameraTransformCalibration = field(default_factory=CameraTransformCalibration)
    tcp_to_camera: CameraTransformCalibration = field(default_factory=CameraTransformCalibration)
    depth_compensation: DepthCompensationCalibration = field(default_factory=DepthCompensationCalibration)

    @classmethod
    def from_dict(cls, data: Dict[str, Any]) -> "CameraCalibration":
        intrinsics_raw = dict(data.get("intrinsics", {}))
        return cls(
            camera=dict(data.get("camera", {})),
            hand_follow=ModeCalibration.from_dict(data.get("hand_follow", {})),
            object_pick=ModeCalibration.from_dict(data.get("object_pick", {})),
            backend=str(data.get("backend", "hp60c_ros2") or "hp60c_ros2"),
            topics=dict(data.get("topics", {})),
            intrinsics={
                "rgb": CameraIntrinsics.from_dict(dict(intrinsics_raw.get("rgb", {}))),
                "depth": CameraIntrinsics.from_dict(dict(intrinsics_raw.get("depth", {}))),
            },
            camera_to_robot=CameraTransformCalibration.from_dict(dict(data.get("camera_to_robot", {}))),
            tcp_to_camera=CameraTransformCalibration.from_dict(dict(data.get("tcp_to_camera", {}))),
            depth_compensation=DepthCompensationCalibration.from_dict(dict(data.get("depth_compensation", {}))),
        )

    def to_dict(self) -> Dict[str, Any]:
        return {
            "camera": self.camera,
            "hand_follow": self.hand_follow.to_dict(),
            "object_pick": self.object_pick.to_dict(),
            "backend": self.backend,
            "topics": self.topics,
            "intrinsics": {
                "rgb": self.intrinsics.get("rgb", CameraIntrinsics()).to_dict(),
                "depth": self.intrinsics.get("depth", CameraIntrinsics()).to_dict(),
            },
            "camera_to_robot": self.camera_to_robot.to_dict(),
            "tcp_to_camera": self.tcp_to_camera.to_dict(),
            "depth_compensation": self.depth_compensation.to_dict(),
        }
