import os
import sys
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))

from app.services.vision_service import VisionService
from app.state import AppState


def build_stub_config(tmp_path: Path):
    calibration_path = tmp_path / "camera_calibration.json"
    return SimpleNamespace(
        repo_dir=tmp_path,
        vision_model_path=tmp_path / "model.pt",
        vision_confidence_threshold=0.5,
        vision_backend="hp60c_ros2",
        depth_sampling_ratio=0.10,
        ros_rgb_topic="/ascamera/camera_publisher/rgb0/image",
        ros_depth_topic="/ascamera/camera_publisher/depth0/image_raw",
        ros_rgb_camera_info_topic="/ascamera/camera_publisher/rgb0/camera_info",
        ros_depth_camera_info_topic="/ascamera/camera_publisher/depth0/camera_info",
        ros_auto_launch_camera=False,
        ros_camera_namespace="/ascamera",
        ros_workspace_dir=tmp_path / "workspace",
        ros_workspace_setup_file=tmp_path / "workspace" / "install" / "setup.bash",
        ros_ascamera_config_path=tmp_path / "workspace" / "src" / "ascamera" / "configurationfiles",
        ros_camera_wait_timeout_s=2.5,
        ros_camera_boot_delay_s=0.0,
        ros_camera_rgb_width=640,
        ros_camera_rgb_height=480,
        ros_camera_depth_width=640,
        ros_camera_depth_height=480,
        ros_camera_fps=15,
        v4l2_device="/dev/video2",
        camera_index=0,
        camera_width=640,
        camera_height=480,
        camera_fps=30,
        calibration_file=lambda: calibration_path,
    )


def make_service(tmp_path: Path) -> VisionService:
    service = VisionService(build_stub_config(tmp_path), AppState())
    service.calibration.intrinsics["rgb"].width = 640
    service.calibration.intrinsics["rgb"].height = 480
    service.calibration.intrinsics["rgb"].fx = 500.0
    service.calibration.intrinsics["rgb"].fy = 500.0
    service.calibration.intrinsics["rgb"].cx = 320.0
    service.calibration.intrinsics["rgb"].cy = 240.0
    service.calibration.intrinsics["depth"].width = 640
    service.calibration.intrinsics["depth"].height = 480
    service.calibration.intrinsics["depth"].fx = 500.0
    service.calibration.intrinsics["depth"].fy = 500.0
    service.calibration.intrinsics["depth"].cx = 320.0
    service.calibration.intrinsics["depth"].cy = 240.0
    service.calibration.camera_to_robot.configured = True
    service.calibration.camera_to_robot.matrix_4x4 = [
        [1.0, 0.0, 0.0, 10.0],
        [0.0, 1.0, 0.0, 20.0],
        [0.0, 0.0, 1.0, 30.0],
        [0.0, 0.0, 0.0, 1.0],
    ]
    service.calibration.depth_compensation.gain = 1.0
    service.calibration.depth_compensation.offset_mm = 5.0
    service._latest_frame = np.zeros((480, 640, 3), dtype=np.uint8)
    service._latest_depth_frame = np.full((480, 640), 400, dtype=np.uint16)
    return service


def test_build_detection_marks_mano_as_hand(tmp_path):
    service = make_service(tmp_path)

    detection = service._build_detection("Mano", 0.91, [10.0, 20.0, 110.0, 220.0])

    assert detection["type"] == "hand"
    assert detection["center_px"] == [60.0, 120.0]


def test_build_detection_keeps_objects_separate(tmp_path):
    service = make_service(tmp_path)

    detection = service._build_detection("Pinzas", 0.88, [0.0, 0.0, 20.0, 30.0])

    assert detection["type"] == "object"


def test_select_hand_target_prefers_highest_confidence(tmp_path):
    service = make_service(tmp_path)

    detections = [
        service._build_detection("Mano", 0.55, [0.0, 0.0, 10.0, 10.0]),
        service._build_detection("Mano", 0.95, [10.0, 10.0, 30.0, 30.0]),
        service._build_detection("Pinzas", 0.99, [0.0, 0.0, 50.0, 50.0]),
    ]

    hand = service._select_hand_target(detections)

    assert hand is not None
    assert hand["confidence"] == 0.95
    assert hand["type"] == "hand"


def test_build_detection_adds_depth_and_world_coordinates(tmp_path):
    service = make_service(tmp_path)

    detection = service._build_detection("Pinzas", 0.88, [300.0, 220.0, 340.0, 260.0])

    assert detection["depth_valid"] is True
    assert detection["depth_avg_mm"] == 400.0
    assert detection["depth_compensated_mm"] == 405.0
    assert detection["camera_xyz_mm"] == [0.0, 0.0, 405.0]
    assert detection["world_mm"] == [10.0, 20.0, 435.0]
    assert len(detection["depth_samples"]) == 9


def test_build_detection_keeps_world_none_when_depth_is_invalid(tmp_path):
    service = make_service(tmp_path)
    service._latest_depth_frame = np.zeros((480, 640), dtype=np.uint16)

    detection = service._build_detection("Pinzas", 0.88, [300.0, 220.0, 340.0, 260.0])

    assert detection["depth_valid"] is False
    assert detection["world_mm"] is None


def test_mjpeg_stream_uses_rgb_preview_when_yolo_frame_missing(tmp_path):
    service = make_service(tmp_path)
    service._latest_jpeg = None
    service._latest_rgb_jpeg = b"rgb-preview"

    chunk = next(service.mjpeg_stream("annotated"))

    assert b"rgb-preview" in chunk


def test_numpy_guard_rejects_numpy_2(monkeypatch, tmp_path):
    service = make_service(tmp_path)
    fake_numpy = SimpleNamespace(__version__="2.4.4")

    monkeypatch.setitem(sys.modules, "numpy", fake_numpy)

    with pytest.raises(RuntimeError, match="numpy<2"):
        service._ensure_numpy_compatible_with_cv_bridge()


def test_numpy_guard_accepts_numpy_1(monkeypatch, tmp_path):
    service = make_service(tmp_path)
    fake_numpy = SimpleNamespace(__version__="1.26.4")

    monkeypatch.setitem(sys.modules, "numpy", fake_numpy)

    service._ensure_numpy_compatible_with_cv_bridge()
