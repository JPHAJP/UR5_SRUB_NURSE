import os
import signal
import subprocess
import sys
import time
from io import BytesIO
from pathlib import Path
from types import SimpleNamespace, ModuleType

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
        vision_device="auto",
        vision_confidence_threshold=0.5,
        vision_inference_fps=8.0,
        vision_preview_fps=8.0,
        vision_depth_preview_fps=4.0,
        vision_detector_mode="full",
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
        ros_camera_color_pcl=False,
        ros_camera_restart_delay_s=3.0,
        ros_camera_startup_grace_s=8.0,
        ros_camera_stall_timeout_s=3.0,
        ros_camera_no_camera_info_timeout_s=4.0,
        ros_camera_hard_restart_timeout_s=6.0,
        ros_camera_healthy_reset_s=20.0,
        ros_camera_backoff_sequence_s=[2.0, 5.0, 10.0, 20.0],
        v4l2_device="/dev/video2",
        camera_index=0,
        camera_width=640,
        camera_height=480,
        camera_fps=30,
        log_dir=tmp_path / "logs",
        hand_plane_z_mm=350.0,
        object_plane_z_mm=120.0,
        hand_follow_height_mm=550.0,
        hand_follow_z_offset_mm=200.0,
        hand_landmarker_model_path=tmp_path / "hand_landmarker.task",
        hand_landmarker_delegate="auto",
        tcp_to_camera_translation_mm=[55.0, -30.0, 0.0],
        tcp_to_camera_rotation_rpy_deg=[0.0, 0.0, 0.0],
        hand_workspace={"x": [-450.0, 450.0], "y": [-700.0, 150.0], "z": [150.0, 700.0]},
        object_workspace={"x": [-450.0, 450.0], "y": [-700.0, 150.0], "z": [20.0, 600.0]},
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
    service.calibration.tcp_to_camera.configured = True
    service.calibration.tcp_to_camera.translation_mm = [55.0, -30.0, 0.0]
    service.calibration.tcp_to_camera.rotation_rpy_deg = [0.0, 0.0, 0.0]
    service.calibration.tcp_to_camera.matrix_4x4 = [
        [1.0, 0.0, 0.0, 55.0],
        [0.0, 1.0, 0.0, -30.0],
        [0.0, 0.0, 1.0, 0.0],
        [0.0, 0.0, 0.0, 1.0],
    ]
    service.calibration.depth_compensation.gain = 1.0
    service.calibration.depth_compensation.offset_mm = 5.0
    service._latest_frame = np.zeros((480, 640, 3), dtype=np.uint8)
    service._latest_depth_frame = np.full((480, 640), 400, dtype=np.uint16)
    service.state.set_robot_status({"current_pose_mm": [10.0, 20.0, 30.0, 0.0, 0.0, 0.0]})
    return service


class FakeTensor:
    def __init__(self, values):
        self.values = values

    def __len__(self):
        return len(self.values)

    def cpu(self):
        return self

    def tolist(self):
        return self.values


class FakeYoloModel:
    names = {1: "Mano", 3: "Pinzas"}

    def __call__(self, *_args, **_kwargs):
        boxes = SimpleNamespace(
            xyxy=FakeTensor([[10.0, 20.0, 110.0, 220.0], [300.0, 220.0, 340.0, 260.0]]),
            cls=FakeTensor([1.0, 3.0]),
            conf=FakeTensor([0.91, 0.88]),
        )
        result = SimpleNamespace(obb=None, boxes=boxes, plot=lambda: np.zeros((480, 640, 3), dtype=np.uint8))
        return [result]


def test_yolo_filters_hand_labels_and_keeps_instruments(tmp_path):
    service = make_service(tmp_path)
    service._model = FakeYoloModel()
    service._inference_device = "cpu"

    detections, _frame = service._detect_with_yolo(np.zeros((480, 640, 3), dtype=np.uint8))

    assert [item["label"] for item in detections] == ["Pinzas"]
    assert detections[0]["type"] == "object"


def test_yolo_can_return_hand_targets_for_follow_mode(tmp_path):
    service = make_service(tmp_path)
    service._model = FakeYoloModel()
    service._inference_device = "cpu"

    detections, _frame = service._detect_with_yolo(
        np.zeros((480, 640, 3), dtype=np.uint8),
        frame_id=7,
        include_hand=True,
    )

    assert [item["label"] for item in detections] == ["Mano", "Pinzas"]
    assert detections[0]["type"] == "hand"
    assert detections[0]["source"] == "yolo"
    assert detections[0]["frame_id"] == 7


def test_annotate_frame_draws_object_boxes_without_yolo_plot(tmp_path):
    service = make_service(tmp_path)

    class FakeCV2:
        FONT_HERSHEY_SIMPLEX = object()

        def __init__(self):
            self.rectangles = []
            self.texts = []

        def rectangle(self, frame, start, end, color, thickness):
            self.rectangles.append((start, end, color, thickness))

        def putText(self, frame, text, origin, *_args):
            self.texts.append((text, origin))

    fake_cv2 = FakeCV2()
    service._cv2 = fake_cv2
    frame = np.zeros((480, 640, 3), dtype=np.uint8)
    detections = [
        service._build_detection("Pinzas", 0.88, [300.0, 220.0, 340.0, 260.0]),
    ]

    service._annotate_frame(frame, detections, None, "yolo_objects")

    assert fake_cv2.rectangles
    assert any("Pinzas 0.88" in text for text, _origin in fake_cv2.texts)


def test_build_detection_keeps_objects_separate(tmp_path):
    service = make_service(tmp_path)

    detection = service._build_detection("Pinzas", 0.88, [0.0, 0.0, 20.0, 30.0])

    assert detection["type"] == "object"


def test_select_hand_target_uses_first_hand_like_v1(tmp_path):
    service = make_service(tmp_path)

    detections = [
        service._build_detection("Mano", 0.55, [0.0, 0.0, 10.0, 10.0]),
        service._build_detection("Mano", 0.95, [10.0, 10.0, 30.0, 30.0]),
        service._build_detection("Pinzas", 0.99, [0.0, 0.0, 50.0, 50.0]),
    ]

    hand = service._select_hand_target(detections)

    assert hand is not None
    assert hand["confidence"] == 0.55
    assert hand["type"] == "hand"


def test_build_detection_adds_depth_and_world_coordinates(tmp_path):
    service = make_service(tmp_path)

    detection = service._build_detection("Pinzas", 0.88, [300.0, 220.0, 340.0, 260.0])

    assert detection["depth_valid"] is True
    assert detection["depth_avg_mm"] == 400.0
    assert detection["depth_compensated_mm"] == 405.0
    assert detection["camera_xyz_mm"] == [0.0, 0.0, 405.0]
    assert detection["world_mm"] == [65.0, -10.0, 435.0]
    assert len(detection["depth_samples"]) == 9


def test_mediapipe_hand_target_uses_frame_resolution_and_tcp_offset(tmp_path):
    service = make_service(tmp_path)
    service.calibration.intrinsics["rgb"].width = 1280
    service.calibration.intrinsics["rgb"].height = 720
    service.calibration.intrinsics["rgb"].fx = 800.0
    service.calibration.intrinsics["rgb"].fy = 800.0
    service.calibration.intrinsics["rgb"].cx = 640.0
    service.calibration.intrinsics["rgb"].cy = 360.0
    service.calibration.intrinsics["depth"].width = 1280
    service.calibration.intrinsics["depth"].height = 720
    service._latest_frame = np.zeros((720, 1280, 3), dtype=np.uint8)
    service._latest_depth_frame = np.full((720, 1280), 400, dtype=np.uint16)
    service.state.set_robot_status({"current_pose_mm": [100.0, 200.0, 300.0, 0.0, 0.0, 0.0]})
    landmarks = [SimpleNamespace(x=0.5, y=0.5, z=0.0) for _ in range(21)]

    target = service._build_mediapipe_hand_target(
        hand_landmarks=landmarks,
        handedness=[SimpleNamespace(score=0.77, category_name="Right")],
        frame_shape=(720, 1280, 3),
    )

    assert target is not None
    assert target["source"] == "mediapipe"
    assert target["center_px"] == [640.0, 360.0]
    assert target["confidence"] == 0.77
    assert target["world_mm"] == [155.0, 170.0, 705.0]
    assert target["follow_plane_z_mm"] == 350.0
    assert target["follow_z_offset_mm"] == 200.0
    assert target["follow_height_mm"] == 550.0
    assert "timestamp_s" in target


def test_hand_only_mode_uses_yolo_hand_detection(tmp_path):
    config = build_stub_config(tmp_path)
    config.vision_detector_mode = "hand_only"
    service = make_service(tmp_path)
    service.config.vision_detector_mode = "hand_only"
    service._model = FakeYoloModel()
    service._inference_device = "cpu"

    detections, _frame = service._detect_with_yolo(
        np.zeros((480, 640, 3), dtype=np.uint8),
        include_hand=True,
        hand_only=True,
    )
    target = service._select_hand_target(detections)

    assert service._is_hand_only_mode() is True
    assert target is not None
    assert target["source"] == "yolo"
    assert [item["label"] for item in detections] == ["Mano"]


def test_hand_only_inference_initialization_keeps_yolo_enabled(monkeypatch, tmp_path):
    config = build_stub_config(tmp_path)
    config.vision_detector_mode = "hand_only"
    service = VisionService(config, AppState())
    service._running = False
    configure_called = {"value": False}
    load_called = {"value": False}

    monkeypatch.setattr(
        service,
        "_configure_inference_device",
        lambda: configure_called.__setitem__("value", True),
    )
    monkeypatch.setattr(
        service,
        "_load_model",
        lambda: load_called.__setitem__("value", True),
    )
    monkeypatch.setitem(sys.modules, "torch", ModuleType("torch"))
    ultralytics_module = ModuleType("ultralytics")
    ultralytics_module.YOLO = object()
    monkeypatch.setitem(sys.modules, "ultralytics", ultralytics_module)

    service._inference_loop()

    assert service._yolo is not None
    assert configure_called["value"] is True
    assert load_called["value"] is True


def test_auto_launch_skips_second_process_when_bundle_is_running(monkeypatch, tmp_path):
    config = build_stub_config(tmp_path)
    config.ros_ascamera_config_path.mkdir(parents=True, exist_ok=True)
    service = VisionService(config, AppState())

    popen_called = {"value": False}
    monkeypatch.setattr(
        service,
        "_find_external_camera_processes",
        lambda: [{"pid": 999, "cmdline": "ros2 launch ascamera", "stdout": "/tmp/log", "stderr": "/tmp/log"}],
    )
    monkeypatch.setattr("app.services.vision_service.subprocess.Popen", lambda *args, **kwargs: popen_called.__setitem__("value", True))

    service._maybe_launch_camera_node()

    assert popen_called["value"] is False
    assert "no lanzara un segundo nodo" in service._last_camera_launch_error.lower()


def test_auto_launch_uses_bundle_like_color_pcl_and_writes_logs(monkeypatch, tmp_path):
    config = build_stub_config(tmp_path)
    config.ros_workspace_dir.mkdir(parents=True, exist_ok=True)
    config.ros_ascamera_config_path.mkdir(parents=True, exist_ok=True)
    service = VisionService(config, AppState())

    captured = {}

    class DummyProcess:
        pid = 321

        @staticmethod
        def poll():
            return None

    monkeypatch.setattr(service, "_find_external_camera_processes", lambda: [])
    monkeypatch.setattr(service, "_open_camera_process_log", lambda: BytesIO())

    def fake_popen(command, **kwargs):
        captured["command"] = command
        captured["kwargs"] = kwargs
        return DummyProcess()

    monkeypatch.setattr("app.services.vision_service.subprocess.Popen", fake_popen)

    service._maybe_launch_camera_node()

    assert "color_pcl:=false" in captured["command"]
    assert captured["kwargs"]["cwd"] == str(config.ros_workspace_dir)
    assert captured["kwargs"]["stderr"] == subprocess.STDOUT
    assert captured["kwargs"]["start_new_session"] is True


def test_auto_launch_replaces_stale_old_silvia_process(monkeypatch, tmp_path):
    config = build_stub_config(tmp_path)
    config.ros_workspace_dir.mkdir(parents=True, exist_ok=True)
    config.ros_ascamera_config_path.mkdir(parents=True, exist_ok=True)
    service = VisionService(config, AppState())

    captured = {"terminated": None}

    class DummyProcess:
        pid = 654

        @staticmethod
        def poll():
            return None

    stale_process = {
        "pid": 12345,
        "cmdline": (
            "ascamera_node --ros-args "
            f"-p confiPath:={str(config.ros_ascamera_config_path).lower()} "
            "-p color_pcl:=true"
        ),
        "stdout": "/dev/null",
        "stderr": "/dev/null",
    }

    monkeypatch.setattr(service, "_find_external_camera_processes", lambda: [stale_process])
    monkeypatch.setattr(service, "_terminate_external_camera_process", lambda pid: captured.__setitem__("terminated", pid) or True)
    monkeypatch.setattr(service, "_open_camera_process_log", lambda: BytesIO())
    monkeypatch.setattr("app.services.vision_service.subprocess.Popen", lambda *args, **kwargs: DummyProcess())

    service._maybe_launch_camera_node()

    assert captured["terminated"] == 12345
    assert service._camera_process is not None


def test_stale_process_detection_matches_old_silvia_signature(tmp_path):
    config = build_stub_config(tmp_path)
    service = VisionService(config, AppState())

    stale_process = {
        "pid": 1,
        "cmdline": (
            "ascamera_node --ros-args "
            f"-p confiPath:={str(config.ros_ascamera_config_path).lower()} "
            "-p color_pcl:=true -p fps:=15"
        ),
        "stdout": "/dev/null",
        "stderr": "/dev/null",
    }
    healthy_process = {
        "pid": 2,
        "cmdline": "ros2 launch ascamera ascamera.launch.py",
        "stdout": "/tmp/ascamera.log",
        "stderr": "/tmp/ascamera.log",
    }

    assert service._should_replace_stale_external_camera_process(stale_process) is True
    assert service._should_replace_stale_external_camera_process(healthy_process) is False


def test_stale_process_detection_matches_current_silvia_log_signature(tmp_path):
    config = build_stub_config(tmp_path)
    service = VisionService(config, AppState())

    current_silvia_process = {
        "pid": 7,
        "cmdline": (
            "ascamera_node --ros-args "
            f"-p confiPath:={str(config.ros_ascamera_config_path).lower()} "
            "-p color_pcl:=false -p fps:=10"
        ),
        "stdout": str(service._camera_process_log_path),
        "stderr": str(service._camera_process_log_path),
    }

    assert service._should_replace_stale_external_camera_process(current_silvia_process) is True


def test_find_external_camera_process_ignores_other_network_namespace(monkeypatch, tmp_path):
    config = build_stub_config(tmp_path)
    service = VisionService(config, AppState())

    monkeypatch.setattr("app.services.vision_service.os.getpid", lambda: 500)
    monkeypatch.setattr(service, "_safe_process_namespace", lambda proc_dir, namespace: "net:[host]" if str(proc_dir) == "/proc/self" else "net:[other]")
    monkeypatch.setattr("app.services.vision_service.Path.iterdir", lambda _self: [Path("/proc/600")])
    monkeypatch.setattr("pathlib.Path.read_bytes", lambda self: b"ascamera_node\x00--ros-args\x00")

    assert service._find_external_camera_process() is None


def test_find_external_camera_process_keeps_same_network_namespace(monkeypatch, tmp_path):
    config = build_stub_config(tmp_path)
    service = VisionService(config, AppState())

    monkeypatch.setattr("app.services.vision_service.os.getpid", lambda: 500)
    monkeypatch.setattr(service, "_safe_process_namespace", lambda _proc_dir, _namespace: "net:[host]")
    monkeypatch.setattr("app.services.vision_service.Path.iterdir", lambda _self: [Path("/proc/600")])
    monkeypatch.setattr("pathlib.Path.read_bytes", lambda self: b"ascamera_node\x00--ros-args\x00")
    monkeypatch.setattr(service, "_safe_process_fd_target", lambda _proc_dir, fd: f"/tmp/fd{fd}")

    process = service._find_external_camera_process()

    assert process is not None
    assert process["pid"] == 600
    assert process["net_namespace"] == "net:[host]"


def test_stop_camera_process_kills_process_group_and_stale_orphans(monkeypatch, tmp_path):
    service = make_service(tmp_path)
    terminated = []

    class DummyProcess:
        pid = 4321

        @staticmethod
        def poll():
            return None

        @staticmethod
        def wait(timeout=None):
            return None

    service._camera_process = DummyProcess()
    service._camera_process_started_by_service = True
    monkeypatch.setattr("app.services.vision_service.os.getpgid", lambda pid: pid)
    monkeypatch.setattr("app.services.vision_service.os.killpg", lambda pgid, sig: terminated.append((pgid, sig)))
    monkeypatch.setattr(
        service,
        "_find_external_camera_processes",
        lambda: [
            {
                "pid": 9876,
                "cmdline": (
                    "ascamera_node --ros-args "
                    f"-p confiPath:={str(service.config.ros_ascamera_config_path).lower()} "
                    "-p color_pcl:=false"
                ),
                "stdout": str(service._camera_process_log_path),
                "stderr": str(service._camera_process_log_path),
            }
        ],
    )
    monkeypatch.setattr(
        service,
        "_terminate_external_camera_process",
        lambda pid, timeout_s=3.0: terminated.append(("external", pid, timeout_s)) or True,
    )

    service._stop_camera_process()

    assert (4321, signal.SIGTERM) in terminated
    assert ("external", 9876, 3.0) in terminated
    assert service._camera_process is None


def test_latest_camera_process_issue_extracts_recent_driver_error(tmp_path):
    service = make_service(tmp_path)
    service._camera_process_log_path.parent.mkdir(parents=True, exist_ok=True)
    service._camera_process_log_path.write_text(
        "\n".join(
            [
                "[INFO] startup",
                "[INFO] still waiting",
                "[ERROR] [UvcCamera.cpp] [247] [start] uvc_start_streaming:No such device",
            ]
        ),
        encoding="utf-8",
    )

    assert "No such device" in service._latest_camera_process_issue()


def test_latest_camera_process_issue_ignores_old_errors_before_current_launch(tmp_path):
    service = make_service(tmp_path)
    service._camera_process_log_path.parent.mkdir(parents=True, exist_ok=True)
    service._camera_process_log_path.write_text(
        "[ERROR] [UvcCamera.cpp] [247] [start] uvc_start_streaming:No such device\n",
        encoding="utf-8",
    )
    service._camera_process_log_offset = service._camera_process_log_path.stat().st_size
    with service._camera_process_log_path.open("a", encoding="utf-8") as handle:
        handle.write("[INFO] camera recovered\n")

    assert service._latest_camera_process_issue() == ""


def test_camera_recovery_stays_starting_during_startup_grace(tmp_path):
    service = make_service(tmp_path)
    service.config.ros_auto_launch_camera = True
    now = time.time()

    class DummyProcess:
        pid = 101

        @staticmethod
        def poll():
            return None

    service._camera_process = DummyProcess()
    service._camera_process_started_by_service = True
    service._camera_process_started_at = now - 2.0
    service._latest_rgb_timestamp = 0.0
    service._latest_depth_timestamp = 0.0
    service._latest_camera_info_timestamp = 0.0

    service._refresh_stream_health(now)
    service._refresh_camera_recovery(now)

    assert service._camera_recovery_state == "starting"


def test_camera_recovery_marks_stalled_before_hard_restart(monkeypatch, tmp_path):
    service = make_service(tmp_path)
    now = time.time()
    restarted = {}

    class DummyProcess:
        pid = 202

        @staticmethod
        def poll():
            return None

    service._camera_process = DummyProcess()
    service._camera_process_started_by_service = True
    service._camera_process_started_at = now - 10.0
    service._latest_rgb_timestamp = now - 5.0
    service._latest_depth_timestamp = now - 5.0
    service._latest_camera_info_timestamp = now - 1.0
    monkeypatch.setattr(
        service,
        "_perform_camera_hard_restart",
        lambda now, reason, driver_issue="": restarted.update({"reason": reason, "driver_issue": driver_issue}),
    )

    service._refresh_stream_health(now)
    service._refresh_camera_recovery(now)

    assert service._camera_recovery_state == "stalled"
    assert restarted == {}
    assert service._latest_rgb_jpeg is None
    assert service._latest_depth_jpeg is None

    service._refresh_camera_recovery(now + 6.1)

    assert "reason" in restarted


def test_schedule_camera_retry_uses_backoff_sequence(tmp_path):
    service = make_service(tmp_path)

    service._schedule_camera_retry(now=100.0, reason="primer fallo", driver_issue="usb")
    assert service._camera_restart_count == 1
    assert service._next_camera_launch_attempt_at == pytest.approx(102.0)
    assert service._camera_backoff_index == 1

    service._schedule_camera_retry(now=103.0, reason="segundo fallo", driver_issue="usb")
    assert service._camera_restart_count == 2
    assert service._next_camera_launch_attempt_at == pytest.approx(108.0)
    assert service._camera_backoff_index == 2


def test_camera_recovery_resets_backoff_after_healthy_window(tmp_path):
    service = make_service(tmp_path)
    now = time.time()

    class DummyProcess:
        pid = 303

        @staticmethod
        def poll():
            return None

    service._camera_process = DummyProcess()
    service._camera_process_started_by_service = True
    service._camera_process_started_at = now - 30.0
    service._camera_backoff_index = 3
    service._camera_recovery_state = "healthy"
    service._camera_healthy_since_at = now - 21.0
    service._latest_rgb_timestamp = now - 0.2
    service._latest_depth_timestamp = now - 0.2
    service._latest_camera_info_timestamp = now - 0.2

    service._refresh_stream_health(now)
    service._refresh_camera_recovery(now)

    assert service._camera_backoff_index == 0
    assert service._camera_recovery_state == "healthy"


def test_clear_live_stream_state_clears_previews_and_targets(tmp_path):
    service = make_service(tmp_path)
    service._latest_jpeg = b"annotated"
    service._latest_rgb_jpeg = b"rgb"
    service._latest_depth_jpeg = b"depth"
    service._latest_detections = [{"label": "Mano"}]
    service._latest_hand_target = {"label": "Mano"}
    service.state.set_detections([{"label": "Mano", "type": "hand"}])
    service.state.set_hand_target({"label": "Mano"})

    service._clear_live_stream_state(clear_timestamps=True)

    assert service._latest_jpeg is None
    assert service._latest_rgb_jpeg is None
    assert service._latest_depth_jpeg is None
    assert service._latest_detections == []
    assert service._latest_hand_target is None
    assert service._latest_rgb_timestamp == 0.0
    assert service._latest_depth_timestamp == 0.0
    assert service.state.snapshot()["hand_target"] is None


def test_publish_status_exposes_camera_recovery_and_hides_stale_previews(tmp_path):
    service = make_service(tmp_path)
    service._latest_rgb_jpeg = b"rgb"
    service._latest_depth_jpeg = b"depth"
    service._latest_rgb_timestamp = time.time() - 10.0
    service._latest_depth_timestamp = time.time() - 10.0
    service._camera_recovery_state = "failed"
    service._next_camera_launch_attempt_at = time.time() + 4.0

    service._publish_status(ok=False)
    status = service.state.snapshot()["vision_status"]

    assert status["rgb_preview_ready"] is False
    assert status["depth_preview_ready"] is False
    assert status["camera_recovery"]["state"] == "failed"
    assert status["camera_recovery"]["next_retry_in_s"] is not None


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
    service._latest_rgb_timestamp = time.time()

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


def test_configure_inference_device_prefers_cuda_when_available(tmp_path):
    service = make_service(tmp_path)
    service._torch = SimpleNamespace(
        version=SimpleNamespace(cuda="12.1"),
        cuda=SimpleNamespace(
            is_available=lambda: True,
            device_count=lambda: 1,
            get_device_name=lambda index: "Fake GPU",
        ),
    )

    service._configure_inference_device()

    assert service._inference_device == "cuda:0"
    assert "Fake GPU" in service._inference_device_reason


def test_configure_inference_device_falls_back_to_cpu_when_cuda_init_fails(tmp_path):
    service = make_service(tmp_path)
    service._torch = SimpleNamespace(
        version=SimpleNamespace(cuda="13.0"),
        cuda=SimpleNamespace(
            is_available=lambda: False,
            device_count=lambda: 1,
            get_device_name=lambda index: "Fake GPU",
        ),
    )

    service._configure_inference_device()

    assert service._inference_device == "cpu"
    assert "CPU" in service._inference_device_reason


def test_hand_delegate_candidates_try_gpu_then_cpu_for_auto():
    assert VisionService._hand_delegate_candidates("auto") == ["gpu", "cpu"]
    assert VisionService._hand_delegate_candidates("gpu") == ["gpu", "cpu"]
    assert VisionService._hand_delegate_candidates("cpu") == ["cpu"]


def test_mediapipe_cpp_log_level_maps_alias_and_keeps_errors(monkeypatch):
    monkeypatch.setenv("MEDIAPIPE_CPP_MIN_LOG_LEVEL", "error")
    monkeypatch.delenv("GLOG_minloglevel", raising=False)
    monkeypatch.delenv("ABSL_MIN_LOG_LEVEL", raising=False)
    monkeypatch.delenv("TF_CPP_MIN_LOG_LEVEL", raising=False)

    VisionService._apply_mediapipe_cpp_log_level()

    assert os.environ["MEDIAPIPE_CPP_MIN_LOG_LEVEL"] == "2"
    assert os.environ["GLOG_minloglevel"] == "2"
    assert os.environ["ABSL_MIN_LOG_LEVEL"] == "2"
    assert os.environ["TF_CPP_MIN_LOG_LEVEL"] == "2"


def test_preview_refresh_respects_fps_limit(tmp_path):
    service = make_service(tmp_path)

    assert service._should_refresh_preview(10.0, 0.0, 8.0) is True
    assert service._should_refresh_preview(10.05, 10.0, 8.0) is False
    assert service._should_refresh_preview(10.13, 10.0, 8.0) is True
    assert service._should_refresh_preview(10.01, 10.0, 0.0) is True
