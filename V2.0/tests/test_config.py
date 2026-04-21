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


def test_hp60c_auto_launch_enabled_by_default(monkeypatch):
    monkeypatch.delenv("ROS_AUTO_LAUNCH_CAMERA", raising=False)

    config = AppConfig.load()

    assert config.ros_auto_launch_camera is True
