import os
import sys
import time
from types import SimpleNamespace

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))

from app.services.tracking_service import TrackingService
from app.state import AppState


class FakeVision:
    def __init__(self, hand_target=None, object_target=None):
        self._hand_target = hand_target
        self._object_target = object_target
        self.stream_health = {
            "rgb_ok": True,
            "depth_ok": True,
            "camera_open": True,
            "rgb_age_s": 0.01,
            "depth_age_s": 0.01,
        }
        self.hand_calibration = {
            "plane_z_mm": 350.0,
            "z_offset_mm": 200.0,
            "target_z_mm": 550.0,
        }

    def get_latest_hand_target(self):
        return self._hand_target

    def get_stream_health(self):
        return self.stream_health

    def get_hand_follow_calibration(self):
        return self.hand_calibration

    def find_object_target(self, label=None):
        return self._object_target


class FakeGateway:
    def __init__(self):
        self.move_calls = []
        self.pick_sequence_calls = []
        self.magnet_calls = []
        self.stop_calls = 0
        self.refresh_status_calls = 0
        self.pose_mm = [100.0, 200.0, 300.0, 0.0, 0.0, 0.0]

    def current_pose_mm(self):
        return list(self.pose_mm)

    def refresh_status(self):
        self.refresh_status_calls += 1
        return {
            "current_pose_mm": list(self.pose_mm),
            "joint_positions_deg": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            "connected": True,
            "remote_control": "true",
            "safety_status": "NORMAL",
        }

    def move_linear_mm(self, pose, orientation=None, speed=None, acceleration=None):
        self.move_calls.append(
            {
                "pose": list(pose),
                "orientation": None if orientation is None else list(orientation),
                "speed": speed,
                "acceleration": acceleration,
            }
        )
        return True, "ok"

    def execute_pick_sequence_mm(self, waypoints, **kwargs):
        self.pick_sequence_calls.append(
            {
                "waypoints": {name: list(values) for name, values in waypoints.items()},
                "kwargs": dict(kwargs),
            }
        )
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
    def __init__(self):
        self.targets = []

    def is_locked(self):
        return False

    def evaluate_target(self, mode, point_xyz_mm):
        self.targets.append((mode, list(point_xyz_mm)))
        return True, "ok"


def build_config():
    return SimpleNamespace(
        target_loss_timeout_s=1.0,
        max_track_speed_xy_mm_s=120.0,
        max_track_speed_z_mm_s=60.0,
        hand_deadzone_mm=1.0,
        track_smoothing_alpha=1.0,
        pick_approach_lift_mm=80.0,
        hand_follow_height_mm=550.0,
        hand_follow_z_offset_mm=200.0,
        hand_target_max_age_s=0.5,
        track_command_hz=10.0,
        max_track_accel_mm_s2=300.0,
        hand_follow_move_speed_m_s=0.5,
        hand_follow_move_acceleration_m_s2=0.5,
        hand_follow_position_tolerance_ratio=0.10,
        hand_follow_stop_speed_mm_s=10.0,
        hand_follow_motion_start_delay_s=0.12,
        pick_pre_grasp_offset_mm=45.0,
        pick_linear_speed_m_s=0.16,
        pick_linear_acceleration_m_s2=0.45,
        pick_blend_radius_m=0.012,
        pick_settle_s=0.15,
        pick_magnet_settle_s=0.25,
        home_joints_deg=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
    )


def fresh_hand_target(world_mm=None):
    return {"world_mm": world_mm or [130.0, 210.0, 360.0], "type": "hand", "timestamp_s": time.time()}


def test_follow_hand_step_uses_plane_plus_offset_for_z_target():
    state = AppState()
    state.set_mode("hand_follow")
    vision = FakeVision(hand_target=fresh_hand_target())
    gateway = FakeGateway()
    safety = FakeSafety()
    service = TrackingService(build_config(), state, vision, gateway, safety)

    service._follow_hand_step()

    assert gateway.move_calls
    assert safety.targets[-1] == ("hand_follow", [130.0, 210.0, 550.0])
    assert state.snapshot()["tracking_status"]["target_xyz_mm"] == [130.0, 210.0, 550.0]
    assert gateway.move_calls[-1]["pose"] == [130.0, 210.0, 550.0]


def test_follow_hand_step_uses_absolute_movel_like_v1():
    state = AppState()
    state.set_mode("hand_follow")
    vision = FakeVision(hand_target=fresh_hand_target([600.0, 700.0, 360.0]))
    gateway = FakeGateway()
    service = TrackingService(build_config(), state, vision, gateway, FakeSafety())

    service._follow_hand_step()

    assert gateway.move_calls
    assert gateway.move_calls[-1]["pose"] == [600.0, 700.0, 550.0]
    assert gateway.move_calls[-1]["speed"] == 0.5
    assert gateway.move_calls[-1]["acceleration"] == 0.5


def test_follow_hand_step_waits_until_previous_move_finishes():
    state = AppState()
    state.set_mode("hand_follow")
    vision = FakeVision(hand_target=fresh_hand_target([600.0, 700.0, 360.0]))
    gateway = FakeGateway()
    service = TrackingService(build_config(), state, vision, gateway, FakeSafety())

    service._follow_hand_step()
    service._follow_hand_step()

    assert len(gateway.move_calls) == 1
    assert state.snapshot()["tracking_status"]["stop_reason"] == "robot_moving"


def test_follow_hand_step_stops_when_target_is_stale():
    state = AppState()
    state.set_mode("hand_follow")
    stale_target = {"world_mm": [130.0, 210.0, 360.0], "type": "hand", "timestamp_s": time.time() - 2.0}
    vision = FakeVision(hand_target=stale_target)
    gateway = FakeGateway()
    service = TrackingService(build_config(), state, vision, gateway, FakeSafety())

    service._follow_hand_step()

    assert gateway.stop_calls == 1
    assert state.snapshot()["tracking_status"]["stop_reason"] == "target_stale"


def test_follow_hand_step_stops_when_camera_is_stale():
    state = AppState()
    state.set_mode("hand_follow")
    vision = FakeVision(hand_target=fresh_hand_target())
    vision.stream_health["rgb_ok"] = False
    gateway = FakeGateway()
    service = TrackingService(build_config(), state, vision, gateway, FakeSafety())

    service._follow_hand_step()

    assert gateway.stop_calls == 1
    assert state.snapshot()["tracking_status"]["stop_reason"] == "camera_stale"


def test_repeated_stop_reason_is_throttled():
    state = AppState()
    state.set_mode("hand_follow")
    vision = FakeVision(hand_target=fresh_hand_target())
    gateway = FakeGateway()
    service = TrackingService(build_config(), state, vision, gateway, FakeSafety())

    service._stop_tracking("camera_stale")
    service._stop_tracking("camera_stale")

    assert gateway.stop_calls == 1


def test_execute_pick_uses_detected_object_depth_for_waypoints(monkeypatch):
    state = AppState()
    vision = FakeVision(object_target={"label": "Pinzas", "world_mm": [150.0, 250.0, 340.0], "type": "object"})
    gateway = FakeGateway()
    service = TrackingService(build_config(), state, vision, gateway, FakeSafety())

    monkeypatch.setattr("app.services.tracking_service.time.sleep", lambda *_args, **_kwargs: None)

    service._execute_pick({"label": "Pinzas", "deliver_to_hand": False})

    call = gateway.pick_sequence_calls[0]
    assert call["waypoints"]["approach"] == [150.0, 250.0, 420.0]
    assert call["waypoints"]["pre_pick"] == [150.0, 250.0, 385.0]
    assert call["waypoints"]["pick"] == [150.0, 250.0, 340.0]
    assert call["waypoints"]["retreat"] == [150.0, 250.0, 420.0]
    assert call["kwargs"]["return_home"] is True
