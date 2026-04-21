import os
import sys
from types import SimpleNamespace

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))

from app.services.tracking_service import TrackingService
from app.state import AppState


class FakeVision:
    def __init__(self, hand_target=None, object_target=None):
        self._hand_target = hand_target
        self._object_target = object_target

    def get_latest_hand_target(self):
        return self._hand_target

    def find_object_target(self, label=None):
        return self._object_target


class FakeGateway:
    def __init__(self):
        self.speed_calls = []
        self.move_calls = []
        self.magnet_calls = []
        self.stop_calls = 0

    def current_pose_mm(self):
        return [100.0, 200.0, 300.0, 0.0, 0.0, 0.0]

    def speed_linear_mm(self, velocity):
        self.speed_calls.append(list(velocity))
        return True, "ok"

    def move_linear_mm(self, pose):
        self.move_calls.append(list(pose))
        return True, "ok"

    def stop_motion(self):
        self.stop_calls += 1
        return True, "ok"

    def set_magnet(self, enabled):
        self.magnet_calls.append(bool(enabled))
        return True, "ok"

    def go_home(self):
        return True, "ok"


class FakeSafety:
    def is_locked(self):
        return False

    def evaluate_target(self, mode, point_xyz_mm):
        return True, "ok"


def build_config():
    return SimpleNamespace(
        target_loss_timeout_s=1.0,
        max_track_speed_xy_mm_s=120.0,
        max_track_speed_z_mm_s=60.0,
        hand_deadzone_mm=1.0,
        track_smoothing_alpha=1.0,
        pick_approach_lift_mm=80.0,
    )


def test_follow_hand_step_uses_real_depth_for_z_error():
    state = AppState()
    state.set_mode("hand_follow")
    vision = FakeVision(hand_target={"world_mm": [130.0, 210.0, 360.0], "type": "hand"})
    gateway = FakeGateway()
    service = TrackingService(build_config(), state, vision, gateway, FakeSafety())

    service._follow_hand_step()

    assert gateway.speed_calls
    vx, vy, vz = gateway.speed_calls[-1]
    assert vx > 0.0
    assert vy > 0.0
    assert vz > 0.0


def test_execute_pick_uses_detected_object_depth_for_waypoints(monkeypatch):
    state = AppState()
    vision = FakeVision(object_target={"label": "Pinzas", "world_mm": [150.0, 250.0, 340.0], "type": "object"})
    gateway = FakeGateway()
    service = TrackingService(build_config(), state, vision, gateway, FakeSafety())

    monkeypatch.setattr("app.services.tracking_service.time.sleep", lambda *_args, **_kwargs: None)

    service._execute_pick({"label": "Pinzas", "deliver_to_hand": False})

    assert gateway.move_calls[0] == [150.0, 250.0, 420.0]
    assert gateway.move_calls[1] == [150.0, 250.0, 340.0]
    assert gateway.move_calls[2] == [150.0, 250.0, 420.0]
