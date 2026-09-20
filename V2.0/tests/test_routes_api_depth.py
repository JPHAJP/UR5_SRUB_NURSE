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
    def __init__(self, depth_enabled=True):
        self.depth_enabled = depth_enabled
        self.camera_catalog = {
            "ok": True,
            "backend": "webcam",
            "active_index": 0,
            "selected_index": 0,
            "updated_at_ms": 123,
            "cameras": [
                {"index": 0, "label": "Webcam 0", "width": 1280, "height": 720, "fps": 30, "active": True, "selected": True},
                {"index": 1, "label": "Webcam 1", "width": 640, "height": 480, "fps": 30, "active": False, "selected": False},
            ],
            "message": "2 webcam(s) detectada(s).",
        }

    def get_depth_diagnostics(self):
        if not self.depth_enabled:
            return {"depth_enabled": False, "message": "La profundidad esta deshabilitada en backend webcam."}
        return {"depth_ok": True, "points": [], "average_depth_mm": 420.0}

    def get_depth_calibration_summary(self, robot_pose_mm=None):
        if not self.depth_enabled:
            return {"depth_enabled": False, "message": "La profundidad esta deshabilitada en backend webcam."}
        return {"depth_compensation": {"gain": 1.0, "offset_mm": 0.0}, "robot_reference": {"robot_z_mm": robot_pose_mm[2]}}

    def set_depth_compensation(self, gain=None, offset_mm=None):
        if not self.depth_enabled:
            raise ValueError("La profundidad esta deshabilitada en backend webcam.")
        return {"gain": gain, "offset_mm": offset_mm}

    def capture_depth_calibration_sample(self, robot_pose_mm):
        if not self.depth_enabled:
            raise ValueError("La profundidad esta deshabilitada en backend webcam.")
        return {"camera_depth_mm": 420.0, "robot_z_mm": robot_pose_mm[2]}

    def fit_depth_calibration(self):
        if not self.depth_enabled:
            raise ValueError("La profundidad esta deshabilitada en backend webcam.")
        return {"gain": 1.01, "offset_mm": -2.0}

    def save_calibration_to_disk(self):
        if not self.depth_enabled:
            raise ValueError("La profundidad esta deshabilitada en backend webcam.")
        return {"saved": True}

    def set_camera_transform(self, translation_mm, rotation_rpy_deg):
        if not self.depth_enabled:
            raise ValueError("La transformacion TCP-camara esta deshabilitada en backend webcam.")
        return {"translation_mm": translation_mm, "rotation_rpy_deg": rotation_rpy_deg}

    def get_hand_follow_calibration(self):
        return {"ok": True, "plane_z_mm": 350.0, "z_offset_mm": 95.0, "target_z_mm": 255.0}

    def set_hand_follow_calibration(self, plane_z_mm=None, save=False):
        return {
            "plane_z_mm": float(plane_z_mm),
            "z_offset_mm": 95.0,
            "target_z_mm": float(plane_z_mm) - 95.0,
            "saved": bool(save),
        }

    def list_available_cameras(self, force_refresh=False):
        payload = dict(self.camera_catalog)
        payload["forced"] = bool(force_refresh)
        payload["cameras"] = [dict(camera) for camera in self.camera_catalog["cameras"]]
        return payload

    def select_camera(self, index):
        camera_index = int(index)
        if camera_index not in {0, 1}:
            raise ValueError(f"No se detecto la webcam indice {camera_index}.")
        cameras = []
        for camera in self.camera_catalog["cameras"]:
            updated = dict(camera)
            updated["active"] = updated["index"] == camera_index
            updated["selected"] = updated["index"] == camera_index
            cameras.append(updated)
        self.camera_catalog["active_index"] = camera_index
        self.camera_catalog["selected_index"] = camera_index
        self.camera_catalog["cameras"] = cameras
        return self.list_available_cameras(force_refresh=True)


def make_client(depth_enabled=True):
    app = Flask(__name__)
    app.extensions["silvia_services"] = {
        "gateway": FakeGateway(),
        "vision": FakeVision(depth_enabled=depth_enabled),
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


def test_get_available_cameras_endpoint():
    client = make_client()

    response = client.get("/api/vision/cameras?refresh=1")

    assert response.status_code == 200
    payload = response.get_json()
    assert payload["forced"] is True
    assert [camera["index"] for camera in payload["cameras"]] == [0, 1]


def test_select_camera_endpoint():
    client = make_client()

    response = client.post("/api/vision/camera", json={"index": 1})

    assert response.status_code == 200
    payload = response.get_json()
    assert payload["selected_index"] == 1
    assert payload["active_index"] == 1


def test_select_camera_endpoint_rejects_unknown_index():
    client = make_client()

    response = client.post("/api/vision/camera", json={"index": 9})

    assert response.status_code == 400
    assert "No se detecto" in response.get_json()["message"]


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
    assert response.get_json()["target_z_mm"] == 255.0


def test_set_hand_follow_calibration_endpoint():
    client = make_client()

    response = client.post("/api/calibration/hand-follow", json={"plane_z_mm": 340.0, "save": True})

    assert response.status_code == 200
    payload = response.get_json()
    assert payload["hand_follow"]["plane_z_mm"] == 340.0
    assert payload["hand_follow"]["target_z_mm"] == 245.0
    assert payload["hand_follow"]["saved"] is True


def test_depth_post_endpoints_return_400_when_webcam_backend():
    client = make_client(depth_enabled=False)

    response = client.post("/api/calibration/depth/sample")

    assert response.status_code == 400
    assert "deshabilitada" in response.get_json()["message"]
