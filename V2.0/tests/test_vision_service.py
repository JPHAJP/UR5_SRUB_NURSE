import os
import sys
from pathlib import Path
from types import SimpleNamespace

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))

from app.services.vision_service import VisionService
from app.state import AppState


def build_stub_config(tmp_path: Path):
    empty_mode = SimpleNamespace(homography=[], plane_z_mm=0.0)
    calibration = SimpleNamespace(hand_follow=empty_mode, object_pick=empty_mode)
    calibration_path = tmp_path / "camera_calibration.json"

    return SimpleNamespace(
        vision_model_path=tmp_path / "model.pt",
        camera_index=0,
        camera_width=640,
        camera_height=480,
        camera_fps=30,
        calibration_file=lambda: calibration_path,
    ), calibration


def test_build_detection_marks_mano_as_hand(tmp_path):
    config, _ = build_stub_config(tmp_path)
    service = VisionService(config, AppState())

    detection = service._build_detection("Mano", 0.91, [10.0, 20.0, 110.0, 220.0])

    assert detection["type"] == "hand"
    assert detection["center_px"] == [60.0, 120.0]


def test_build_detection_keeps_objects_separate(tmp_path):
    config, _ = build_stub_config(tmp_path)
    service = VisionService(config, AppState())

    detection = service._build_detection("Pinzas", 0.88, [0.0, 0.0, 20.0, 30.0])

    assert detection["type"] == "object"


def test_select_hand_target_prefers_highest_confidence(tmp_path):
    config, _ = build_stub_config(tmp_path)
    service = VisionService(config, AppState())

    detections = [
        service._build_detection("Mano", 0.55, [0.0, 0.0, 10.0, 10.0]),
        service._build_detection("Mano", 0.95, [10.0, 10.0, 30.0, 30.0]),
        service._build_detection("Pinzas", 0.99, [0.0, 0.0, 50.0, 50.0]),
    ]

    hand = service._select_hand_target(detections)

    assert hand is not None
    assert hand["confidence"] == 0.95
    assert hand["type"] == "hand"
