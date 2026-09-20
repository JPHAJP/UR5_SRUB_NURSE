import importlib.util
import json
import os
import sys
import tempfile
from pathlib import Path


MODULE_PATH = Path(__file__).resolve().parents[1] / "scripts" / "diagnose_hand_follow.py"
SPEC = importlib.util.spec_from_file_location("diagnose_hand_follow", MODULE_PATH)
diagnose_hand_follow = importlib.util.module_from_spec(SPEC)
assert SPEC is not None and SPEC.loader is not None
SPEC.loader.exec_module(diagnose_hand_follow)


def write_jsonl(path, rows):
    path.write_text(
        "".join(json.dumps(row, ensure_ascii=True) + "\n" for row in rows),
        encoding="utf-8",
    )


class FakeResponse:
    def __init__(self, payload):
        self._payload = payload

    def raise_for_status(self):
        return None

    def json(self):
        return self._payload


class FakeSession:
    def __init__(self, get_payloads, post_payloads=None):
        self._get_payloads = list(get_payloads)
        self._post_payloads = list(post_payloads or [])
        self.posts = []
        self.gets = []

    def get(self, url, timeout=10):
        self.gets.append((url, timeout))
        return FakeResponse(self._get_payloads.pop(0))

    def post(self, url, json=None, timeout=10):
        self.posts.append((url, dict(json or {}), timeout))
        if self._post_payloads:
            return FakeResponse(self._post_payloads.pop(0))
        return FakeResponse({"ok": True})


class FakeClock:
    def __init__(self):
        self.value = 0.0

    def monotonic(self):
        return self.value

    def sleep(self, delta_s):
        self.value += delta_s


def test_analyze_normalized_samples_stable_sequence_has_no_oscillation():
    samples = [
        {
            "timestamp_s": 1.0,
            "tcp_pose_mm": [0.0, 0.0, 0.0],
            "hand_world_mm": [100.0, 0.0, 0.0],
            "target_xyz_mm": [100.0, 0.0, 0.0],
            "distance_tcp_to_hand_mm": 100.0,
            "approaching_hand": None,
            "active_axis": "x",
            "command_state": "command_sent",
        },
        {
            "timestamp_s": 2.0,
            "tcp_pose_mm": [20.0, 0.0, 0.0],
            "hand_world_mm": [100.0, 0.0, 0.0],
            "target_xyz_mm": [100.0, 0.0, 0.0],
            "distance_tcp_to_hand_mm": 80.0,
            "approaching_hand": True,
            "active_axis": "x",
            "command_state": "robot_moving",
        },
        {
            "timestamp_s": 3.0,
            "tcp_pose_mm": [45.0, 0.0, 0.0],
            "hand_world_mm": [100.0, 0.0, 0.0],
            "target_xyz_mm": [100.0, 0.0, 0.0],
            "distance_tcp_to_hand_mm": 55.0,
            "approaching_hand": True,
            "active_axis": "x",
            "command_state": "robot_moving",
        },
    ]

    report = diagnose_hand_follow.analyze_normalized_samples(samples)

    assert report["diagnosis"] == "ok"
    assert report["oscillation_detected"] is False
    assert report["direction_reversals"] == 0
    assert report["approach_toggles"] == 0


def test_analyze_normalized_samples_detects_oscillation():
    samples = [
        {
            "timestamp_s": 1.0,
            "tcp_pose_mm": [0.0, 0.0, 0.0],
            "hand_world_mm": [100.0, 0.0, 0.0],
            "target_xyz_mm": [100.0, 0.0, 0.0],
            "distance_tcp_to_hand_mm": 100.0,
            "approaching_hand": None,
            "active_axis": "x",
            "command_state": "command_sent",
        },
        {
            "timestamp_s": 2.0,
            "tcp_pose_mm": [20.0, 0.0, 0.0],
            "hand_world_mm": [100.0, 0.0, 0.0],
            "target_xyz_mm": [100.0, 0.0, 0.0],
            "distance_tcp_to_hand_mm": 80.0,
            "approaching_hand": True,
            "active_axis": "x",
            "command_state": "robot_moving",
        },
        {
            "timestamp_s": 3.0,
            "tcp_pose_mm": [10.0, 0.0, 0.0],
            "hand_world_mm": [100.0, 0.0, 0.0],
            "target_xyz_mm": [100.0, 0.0, 0.0],
            "distance_tcp_to_hand_mm": 90.0,
            "approaching_hand": False,
            "active_axis": "x",
            "command_state": "robot_moving",
        },
        {
            "timestamp_s": 4.0,
            "tcp_pose_mm": [35.0, 0.0, 0.0],
            "hand_world_mm": [100.0, 0.0, 0.0],
            "target_xyz_mm": [100.0, 0.0, 0.0],
            "distance_tcp_to_hand_mm": 65.0,
            "approaching_hand": True,
            "active_axis": "x",
            "command_state": "robot_moving",
        },
        {
            "timestamp_s": 5.0,
            "tcp_pose_mm": [25.0, 0.0, 0.0],
            "hand_world_mm": [100.0, 0.0, 0.0],
            "target_xyz_mm": [100.0, 0.0, 0.0],
            "distance_tcp_to_hand_mm": 75.0,
            "approaching_hand": False,
            "active_axis": "x",
            "command_state": "robot_moving",
        },
        {
            "timestamp_s": 6.0,
            "tcp_pose_mm": [50.0, 0.0, 0.0],
            "hand_world_mm": [100.0, 0.0, 0.0],
            "target_xyz_mm": [100.0, 0.0, 0.0],
            "distance_tcp_to_hand_mm": 50.0,
            "approaching_hand": True,
            "active_axis": "x",
            "command_state": "robot_moving",
        },
    ]

    report = diagnose_hand_follow.analyze_normalized_samples(samples)

    assert report["diagnosis"] == "oscillation_detected"
    assert report["oscillation_detected"] is True
    assert report["direction_reversals"] >= 3
    assert report["approach_toggles"] >= 3


def test_analyze_normalized_samples_flags_insufficient_tracking_data():
    samples = [
        {
            "timestamp_s": 1.0,
            "tcp_pose_mm": [0.0, 0.0, 0.0],
            "hand_world_mm": None,
            "target_xyz_mm": None,
            "distance_tcp_to_hand_mm": None,
            "approaching_hand": None,
            "active_axis": "x",
            "command_state": "hand_not_visible",
        },
        {
            "timestamp_s": 2.0,
            "tcp_pose_mm": [0.0, 0.0, 0.0],
            "hand_world_mm": None,
            "target_xyz_mm": None,
            "distance_tcp_to_hand_mm": None,
            "approaching_hand": None,
            "active_axis": "x",
            "command_state": "camera_stale",
        },
        {
            "timestamp_s": 3.0,
            "tcp_pose_mm": [0.0, 0.0, 0.0],
            "hand_world_mm": None,
            "target_xyz_mm": None,
            "distance_tcp_to_hand_mm": None,
            "approaching_hand": None,
            "active_axis": "x",
            "command_state": "target_stale",
        },
        {
            "timestamp_s": 4.0,
            "tcp_pose_mm": [5.0, 0.0, 0.0],
            "hand_world_mm": [100.0, 0.0, 0.0],
            "target_xyz_mm": [100.0, 0.0, 0.0],
            "distance_tcp_to_hand_mm": 95.0,
            "approaching_hand": None,
            "active_axis": "x",
            "command_state": "command_sent",
        },
    ]

    report = diagnose_hand_follow.analyze_normalized_samples(samples)

    assert report["diagnosis"] == "insufficient_tracking_data"
    assert report["insufficient_tracking_data"] is True
    assert report["oscillation_detected"] is False


def test_analyze_normalized_samples_short_sequence_does_not_flag_oscillation():
    samples = [
        {
            "timestamp_s": 1.0,
            "tcp_pose_mm": [0.0, 0.0, 0.0],
            "hand_world_mm": [100.0, 0.0, 0.0],
            "target_xyz_mm": [100.0, 0.0, 0.0],
            "distance_tcp_to_hand_mm": 100.0,
            "approaching_hand": None,
            "active_axis": "x",
            "command_state": "command_sent",
        },
        {
            "timestamp_s": 2.0,
            "tcp_pose_mm": [20.0, 0.0, 0.0],
            "hand_world_mm": [100.0, 0.0, 0.0],
            "target_xyz_mm": [100.0, 0.0, 0.0],
            "distance_tcp_to_hand_mm": 80.0,
            "approaching_hand": True,
            "active_axis": "x",
            "command_state": "robot_moving",
        },
    ]

    report = diagnose_hand_follow.analyze_normalized_samples(samples)

    assert report["oscillation_detected"] is False
    assert report["samples_valid"] == 2


def test_analyze_normalized_samples_accepts_xy_active_axis():
    samples = [
        {
            "timestamp_s": 1.0,
            "tcp_pose_mm": [0.0, 0.0, 0.0],
            "hand_world_mm": [100.0, 100.0, 0.0],
            "target_xyz_mm": [100.0, 100.0, 0.0],
            "distance_tcp_to_hand_mm": 141.421,
            "approaching_hand": None,
            "active_axis": "xy",
            "command_state": "command_sent",
        },
        {
            "timestamp_s": 2.0,
            "tcp_pose_mm": [40.0, 40.0, 0.0],
            "hand_world_mm": [100.0, 100.0, 0.0],
            "target_xyz_mm": [100.0, 100.0, 0.0],
            "distance_tcp_to_hand_mm": 84.853,
            "approaching_hand": True,
            "active_axis": "xy",
            "command_state": "robot_moving",
        },
        {
            "timestamp_s": 3.0,
            "tcp_pose_mm": [80.0, 80.0, 0.0],
            "hand_world_mm": [100.0, 100.0, 0.0],
            "target_xyz_mm": [100.0, 100.0, 0.0],
            "distance_tcp_to_hand_mm": 28.284,
            "approaching_hand": True,
            "active_axis": "xy",
            "command_state": "robot_moving",
        },
    ]

    report = diagnose_hand_follow.analyze_normalized_samples(samples)

    assert report["diagnosis"] == "ok"
    assert report["samples_valid"] == 3


def test_run_analyze_mode_reads_jsonl_and_writes_report(tmp_path):
    input_path = tmp_path / "hand_follow_distance_sample.jsonl"
    write_jsonl(
        input_path,
        [
            {
                "timestamp_s": 1.0,
                "tcp_pose_mm": [0.0, 0.0, 0.0],
                "hand_world_mm": [100.0, 0.0, 0.0],
                "target_xyz_mm": [100.0, 0.0, 0.0],
                "distance_tcp_to_hand_mm": 100.0,
                "approaching_hand": None,
                "active_axis": "x",
                "command_state": "command_sent",
            },
            {
                "timestamp_s": 2.0,
                "tcp_pose_mm": [20.0, 0.0, 0.0],
                "hand_world_mm": [100.0, 0.0, 0.0],
                "target_xyz_mm": [100.0, 0.0, 0.0],
                "distance_tcp_to_hand_mm": 80.0,
                "approaching_hand": True,
                "active_axis": "x",
                "command_state": "robot_moving",
            },
            {
                "timestamp_s": 3.0,
                "tcp_pose_mm": [45.0, 0.0, 0.0],
                "hand_world_mm": [100.0, 0.0, 0.0],
                "target_xyz_mm": [100.0, 0.0, 0.0],
                "distance_tcp_to_hand_mm": 55.0,
                "approaching_hand": True,
                "active_axis": "x",
                "command_state": "robot_moving",
            },
        ],
    )

    report = diagnose_hand_follow.run_analyze_mode(input_path, tmp_path)

    assert report["mode"] == "analyze"
    assert report["input_path"] == str(input_path)
    assert Path(report["report_path"]).exists()


def test_run_live_mode_controls_session_and_captures_tracking_log(tmp_path):
    tracking_log_path = tmp_path / "hand_follow_distance_live.jsonl"
    session = FakeSession(
        get_payloads=[
            {
                "safety_locked": False,
                "safety_message": "Sistema listo.",
                "robot_status": {"current_pose_mm": [0.0, 0.0, 0.0]},
                "tracking_status": {},
            },
            {
                "safety_locked": False,
                "robot_status": {"current_pose_mm": [0.0, 0.0, 0.0]},
                "hand_target": {"world_mm": [100.0, 0.0, 0.0]},
                "tracking_status": {
                    "hand_world_mm": [100.0, 0.0, 0.0],
                    "target_xyz_mm": [100.0, 0.0, 0.0],
                    "distance_tcp_to_hand_mm": 100.0,
                    "approaching_hand": None,
                    "active_axis": "x",
                    "stop_reason": "command_sent",
                    "distance_log_path": str(tracking_log_path),
                },
            },
            {
                "safety_locked": False,
                "robot_status": {"current_pose_mm": [20.0, 0.0, 0.0]},
                "hand_target": {"world_mm": [100.0, 0.0, 0.0]},
                "tracking_status": {
                    "hand_world_mm": [100.0, 0.0, 0.0],
                    "target_xyz_mm": [100.0, 0.0, 0.0],
                    "distance_tcp_to_hand_mm": 80.0,
                    "approaching_hand": True,
                    "active_axis": "x",
                    "stop_reason": "robot_moving",
                    "distance_log_path": str(tracking_log_path),
                },
            },
            {
                "safety_locked": False,
                "robot_status": {"current_pose_mm": [45.0, 0.0, 0.0]},
                "hand_target": {"world_mm": [100.0, 0.0, 0.0]},
                "tracking_status": {
                    "hand_world_mm": [100.0, 0.0, 0.0],
                    "target_xyz_mm": [100.0, 0.0, 0.0],
                    "distance_tcp_to_hand_mm": 55.0,
                    "approaching_hand": True,
                    "active_axis": "x",
                    "stop_reason": "robot_moving",
                    "distance_log_path": str(tracking_log_path),
                },
            },
        ],
        post_payloads=[{"ok": True}, {"ok": True}, {"ok": True}],
    )
    clock = FakeClock()

    report = diagnose_hand_follow.run_live_mode(
        base_url="http://127.0.0.1:5050",
        duration_s=0.3,
        poll_hz=10.0,
        output_dir=tmp_path,
        session=session,
        sleeper=clock.sleep,
        monotonic=clock.monotonic,
    )

    assert [payload["action"] if "action" in payload else payload["mode"] for _, payload, _ in session.posts] == [
        "go_home",
        "hand_follow",
        "idle",
    ]
    assert report["mode"] == "live"
    assert report["tracking_log_path"] == str(tracking_log_path)
    assert Path(report["state_capture_path"]).exists()
    assert Path(report["report_path"]).exists()


def test_run_live_mode_aborts_when_safety_is_locked(tmp_path):
    session = FakeSession(
        get_payloads=[
            {
                "safety_locked": True,
                "safety_message": "Bloqueo activo.",
                "robot_status": {"current_pose_mm": [0.0, 0.0, 0.0]},
                "tracking_status": {},
            }
        ]
    )

    report = diagnose_hand_follow.run_live_mode(
        base_url="http://127.0.0.1:5050",
        duration_s=0.1,
        poll_hz=10.0,
        output_dir=tmp_path,
        session=session,
    )

    assert report["diagnosis"] == "precondition_failed"
    assert report["reason"] == "safety_locked"
    assert session.posts == []
