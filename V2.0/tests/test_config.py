import os
import sys

import pytest

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))

from app.config import AppConfig


def test_ssl_context_disabled_by_default(monkeypatch):
    monkeypatch.delenv("FLASK_SSL_MODE", raising=False)
    monkeypatch.delenv("FLASK_SSL_CERT_FILE", raising=False)
    monkeypatch.delenv("FLASK_SSL_KEY_FILE", raising=False)

    config = AppConfig.load()

    assert config.ssl_context() is None


def test_ssl_context_supports_adhoc(monkeypatch):
    monkeypatch.setenv("FLASK_SSL_MODE", "adhoc")
    monkeypatch.delenv("FLASK_SSL_CERT_FILE", raising=False)
    monkeypatch.delenv("FLASK_SSL_KEY_FILE", raising=False)

    config = AppConfig.load()

    assert config.ssl_context() == "adhoc"


def test_ssl_context_requires_existing_cert_files(monkeypatch, tmp_path):
    cert_path = tmp_path / "cert.pem"
    key_path = tmp_path / "key.pem"
    cert_path.write_text("cert", encoding="utf-8")
    key_path.write_text("key", encoding="utf-8")

    monkeypatch.setenv("FLASK_SSL_MODE", "cert")
    monkeypatch.setenv("FLASK_SSL_CERT_FILE", str(cert_path))
    monkeypatch.setenv("FLASK_SSL_KEY_FILE", str(key_path))

    config = AppConfig.load()

    assert config.ssl_context() == (str(cert_path), str(key_path))


def test_ssl_context_rejects_missing_cert_files(monkeypatch, tmp_path):
    monkeypatch.setenv("FLASK_SSL_MODE", "cert")
    monkeypatch.setenv("FLASK_SSL_CERT_FILE", str(tmp_path / "missing-cert.pem"))
    monkeypatch.setenv("FLASK_SSL_KEY_FILE", str(tmp_path / "missing-key.pem"))

    config = AppConfig.load()

    with pytest.raises(RuntimeError, match="No se encontraron los archivos SSL"):
        config.ssl_context()


def test_loads_vision_confidence_threshold(monkeypatch):
    monkeypatch.setenv("VISION_CONFIDENCE_THRESHOLD", "0.5")

    config = AppConfig.load()

    assert config.vision_confidence_threshold == 0.5


def test_loads_vision_device(monkeypatch):
    monkeypatch.setenv("VISION_DEVICE", "cuda:0")

    config = AppConfig.load()

    assert config.vision_device == "cuda:0"


def test_loads_vision_fps_limits(monkeypatch):
    monkeypatch.setenv("VISION_INFERENCE_FPS", "6")
    monkeypatch.setenv("VISION_PREVIEW_FPS", "5")
    monkeypatch.setenv("VISION_DEPTH_PREVIEW_FPS", "3")
    monkeypatch.setenv("HAND_LANDMARKER_DELEGATE", "gpu")

    config = AppConfig.load()

    assert config.vision_inference_fps == 6.0
    assert config.vision_preview_fps == 5.0
    assert config.vision_depth_preview_fps == 3.0
    assert config.hand_landmarker_delegate == "gpu"


def test_loads_hand_only_detector_mode(monkeypatch):
    monkeypatch.setenv("VISION_DETECTOR_MODE", "hand_only")

    config = AppConfig.load()

    assert config.vision_detector_mode == "hand_only"


def test_loads_hand_follow_height_and_tcp_camera_offset(monkeypatch):
    monkeypatch.setenv("HAND_FOLLOW_HEIGHT_MM", "550")
    monkeypatch.setenv("TCP_TO_CAMERA_TRANSLATION_MM", "55,-30,0")
    monkeypatch.setenv("TCP_TO_CAMERA_ROTATION_RPY_DEG", "0,0,0")

    config = AppConfig.load()

    assert config.hand_follow_height_mm == 550.0
    assert config.tcp_to_camera_translation_mm == [55.0, -30.0, 0.0]
    assert config.tcp_to_camera_rotation_rpy_deg == [0.0, 0.0, 0.0]


def test_loads_hand_follow_z_offset(monkeypatch):
    monkeypatch.setenv("HAND_PLANE_Z_MM", "350")
    monkeypatch.setenv("HAND_FOLLOW_Z_OFFSET_MM", "200")
    monkeypatch.setenv("HAND_FOLLOW_HEIGHT_MM", "999")

    config = AppConfig.load()

    assert config.hand_follow_z_offset_mm == 200.0
    assert config.hand_follow_height_mm == 550.0


def test_derives_hand_follow_offset_from_legacy_height(monkeypatch):
    monkeypatch.setenv("HAND_PLANE_Z_MM", "300")
    monkeypatch.setenv("HAND_FOLLOW_HEIGHT_MM", "525")
    monkeypatch.delenv("HAND_FOLLOW_Z_OFFSET_MM", raising=False)

    config = AppConfig.load()

    assert config.hand_follow_z_offset_mm == 225.0
    assert config.hand_follow_height_mm == 525.0


def test_loads_tracking_freshness_and_smoothing_limits(monkeypatch):
    monkeypatch.setenv("HAND_TARGET_MAX_AGE_S", "0.4")
    monkeypatch.setenv("TRACK_COMMAND_HZ", "12")
    monkeypatch.setenv("MAX_TRACK_ACCEL_MM_S2", "250")

    config = AppConfig.load()

    assert config.hand_target_max_age_s == 0.4
    assert config.track_command_hz == 12.0
    assert config.max_track_accel_mm_s2 == 250.0


def test_dashboard_command_logs_can_be_toggled(monkeypatch):
    monkeypatch.setenv("UR5_DASHBOARD_LOG_COMMANDS", "true")

    config = AppConfig.load()

    assert config.ur5_dashboard_log_commands is True


def test_dashboard_command_logs_default_to_quiet(monkeypatch):
    monkeypatch.delenv("UR5_DASHBOARD_LOG_COMMANDS", raising=False)

    config = AppConfig.load()

    assert config.ur5_dashboard_log_commands is False


def test_hp60c_auto_launch_enabled_by_default(monkeypatch):
    monkeypatch.delenv("ROS_AUTO_LAUNCH_CAMERA", raising=False)

    config = AppConfig.load()

    assert config.ros_auto_launch_camera is True


def test_hp60c_auto_launch_defaults_match_bundle(monkeypatch):
    monkeypatch.delenv("ROS_CAMERA_COLOR_PCL", raising=False)
    monkeypatch.delenv("ROS_CAMERA_RESTART_DELAY_S", raising=False)
    monkeypatch.delenv("ROS_CAMERA_STARTUP_GRACE_S", raising=False)
    monkeypatch.delenv("ROS_CAMERA_STALL_TIMEOUT_S", raising=False)
    monkeypatch.delenv("ROS_CAMERA_NO_CAMERA_INFO_TIMEOUT_S", raising=False)
    monkeypatch.delenv("ROS_CAMERA_HARD_RESTART_TIMEOUT_S", raising=False)
    monkeypatch.delenv("ROS_CAMERA_HEALTHY_RESET_S", raising=False)
    monkeypatch.delenv("ROS_CAMERA_BACKOFF_SEQUENCE_S", raising=False)

    config = AppConfig.load()

    assert config.ros_camera_color_pcl is False
    assert config.ros_camera_restart_delay_s == 3.0
    assert config.ros_camera_startup_grace_s == 15.0
    assert config.ros_camera_stall_timeout_s == 6.0
    assert config.ros_camera_no_camera_info_timeout_s == 10.0
    assert config.ros_camera_hard_restart_timeout_s == 15.0
    assert config.ros_camera_healthy_reset_s == 30.0
    assert config.ros_camera_backoff_sequence_s == [3.0, 8.0, 15.0, 30.0]


def test_hp60c_recovery_windows_can_be_overridden(monkeypatch):
    monkeypatch.setenv("ROS_CAMERA_STARTUP_GRACE_S", "9")
    monkeypatch.setenv("ROS_CAMERA_STALL_TIMEOUT_S", "4")
    monkeypatch.setenv("ROS_CAMERA_NO_CAMERA_INFO_TIMEOUT_S", "5")
    monkeypatch.setenv("ROS_CAMERA_HARD_RESTART_TIMEOUT_S", "7")
    monkeypatch.setenv("ROS_CAMERA_HEALTHY_RESET_S", "30")
    monkeypatch.setenv("ROS_CAMERA_BACKOFF_SEQUENCE_S", "1,3,6")

    config = AppConfig.load()

    assert config.ros_camera_startup_grace_s == 9.0
    assert config.ros_camera_stall_timeout_s == 4.0
    assert config.ros_camera_no_camera_info_timeout_s == 5.0
    assert config.ros_camera_hard_restart_timeout_s == 7.0
    assert config.ros_camera_healthy_reset_s == 30.0
    assert config.ros_camera_backoff_sequence_s == [1.0, 3.0, 6.0]
