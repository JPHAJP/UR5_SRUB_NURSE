import os
import sys

from flask import Flask

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))

from app.routes_api import api_blueprint


class FakeGateway:
    def current_pose_mm(self):
        return [100.0, 200.0, 300.0, 0.0, 0.0, 0.0]

    def refresh_status(self):
        return {"connected": False}


class FakeVision:
    def get_depth_diagnostics(self):
        return {"depth_ok": True, "points": [], "average_depth_mm": 420.0}

    def get_depth_calibration_summary(self, robot_pose_mm=None):
        return {"depth_compensation": {"gain": 1.0, "offset_mm": 0.0}, "robot_reference": {"robot_z_mm": robot_pose_mm[2]}}

    def set_depth_compensation(self, gain=None, offset_mm=None):
        return {"gain": gain, "offset_mm": offset_mm}

    def capture_depth_calibration_sample(self, robot_pose_mm):
        return {"camera_depth_mm": 420.0, "robot_z_mm": robot_pose_mm[2]}

    def fit_depth_calibration(self):
        return {"gain": 1.01, "offset_mm": -2.0}

    def save_calibration_to_disk(self):
        return {"saved": True}

    def set_camera_transform(self, translation_mm, rotation_rpy_deg):
        return {"translation_mm": translation_mm, "rotation_rpy_deg": rotation_rpy_deg}

    def get_hand_follow_calibration(self):
        return {"ok": True, "plane_z_mm": 350.0, "z_offset_mm": 200.0, "target_z_mm": 550.0}

    def set_hand_follow_calibration(self, plane_z_mm=None, save=False):
        return {
            "plane_z_mm": float(plane_z_mm),
            "z_offset_mm": 200.0,
            "target_z_mm": float(plane_z_mm) + 200.0,
            "saved": bool(save),
        }


def make_client():
    app = Flask(__name__)
    app.extensions["silvia_services"] = {
        "gateway": FakeGateway(),
        "vision": FakeVision(),
        "dispatch_action": lambda *_args, **_kwargs: {"ok": True},
        "conversation": type("Conversation", (), {"handle_user_text": lambda self, text, source="text": {"ok": True, "reply": text}})(),
        "voice": type(
            "Voice",
            (),
            {
                "speech_base64": lambda self, text: "",
                "process_browser_audio": lambda self, file: {"ok": True},
                "transcribe_browser_audio": lambda self, file: {"ok": True},
                "process_push_to_talk_audio": lambda self, file: {"ok": True},
            },
        )(),
    }
    app.register_blueprint(api_blueprint, url_prefix="/api")
    return app.test_client()


def test_get_depth_diagnostics_endpoint():
    client = make_client()

    response = client.get("/api/vision/depth")

    assert response.status_code == 200
    assert response.get_json()["average_depth_mm"] == 420.0


def test_capture_depth_sample_endpoint():
    client = make_client()

    response = client.post("/api/calibration/depth/sample")

    assert response.status_code == 200
    payload = response.get_json()
    assert payload["ok"] is True
    assert payload["sample"]["robot_z_mm"] == 300.0


def test_set_camera_transform_endpoint():
    client = make_client()

    response = client.post(
        "/api/calibration/camera-transform",
        json={"translation_mm": [1.0, 2.0, 3.0], "rotation_rpy_deg": [4.0, 5.0, 6.0]},
    )

    assert response.status_code == 200
    payload = response.get_json()
    assert payload["camera_to_robot"]["translation_mm"] == [1.0, 2.0, 3.0]


def test_get_hand_follow_calibration_endpoint():
    client = make_client()

    response = client.get("/api/calibration/hand-follow")

    assert response.status_code == 200
    assert response.get_json()["target_z_mm"] == 550.0


def test_set_hand_follow_calibration_endpoint():
    client = make_client()

    response = client.post("/api/calibration/hand-follow", json={"plane_z_mm": 340.0, "save": True})

    assert response.status_code == 200
    payload = response.get_json()
    assert payload["hand_follow"]["plane_z_mm"] == 340.0
    assert payload["hand_follow"]["target_z_mm"] == 540.0
    assert payload["hand_follow"]["saved"] is True
