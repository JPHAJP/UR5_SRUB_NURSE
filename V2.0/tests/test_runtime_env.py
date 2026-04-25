import os
import sys
from io import StringIO
from pathlib import Path

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))

from app.runtime_env import (
    _FilteredStderr,
    _install_mediapipe_stderr_filter,
    _preload_app_env_files,
    ROS_BOOTSTRAP_MARKER,
    _apply_environment_updates,
    _candidate_setup_scripts,
    _needs_process_reexec,
    bootstrap_runtime_environment,
)


def test_apply_environment_updates_prepends_pythonpath(monkeypatch, tmp_path):
    original_sys_path = list(sys.path)
    try:
        target = tmp_path / "ros_site_packages"
        target.mkdir()
        monkeypatch.delenv("PYTHONPATH", raising=False)

        _apply_environment_updates({"PYTHONPATH": str(target)})

        assert os.environ["PYTHONPATH"] == str(target)
        assert sys.path[0] == str(target)
    finally:
        sys.path[:] = original_sys_path


def test_candidate_setup_scripts_discovers_repo_workspace(monkeypatch, tmp_path):
    monkeypatch.delenv("ROS_SETUP_FILE", raising=False)
    monkeypatch.delenv("ROS_WORKSPACE_SETUP_FILE", raising=False)
    monkeypatch.setenv("ROS_DISTRO", "jazzy")

    ros_setup = tmp_path / "opt" / "ros" / "jazzy" / "setup.bash"
    ros_setup.parent.mkdir(parents=True)
    ros_setup.write_text("", encoding="utf-8")

    workspace_setup = tmp_path / "hp60c_portable_bundle" / "workspace" / "install" / "setup.bash"
    workspace_setup.parent.mkdir(parents=True)
    workspace_setup.write_text("", encoding="utf-8")

    monkeypatch.setenv("ROS_SETUP_FILE", str(ros_setup))
    scripts = _candidate_setup_scripts(str(tmp_path))

    assert ros_setup in scripts
    assert workspace_setup in scripts


def test_needs_process_reexec_when_ros_paths_change(monkeypatch):
    monkeypatch.delenv(ROS_BOOTSTRAP_MARKER, raising=False)
    monkeypatch.setenv("LD_LIBRARY_PATH", "/usr/lib")
    monkeypatch.setenv("PYTHONPATH", "")

    assert _needs_process_reexec(
        {
            "LD_LIBRARY_PATH": "/opt/ros/jazzy/lib:/usr/lib",
            "PYTHONPATH": "/opt/ros/jazzy/lib/python3.12/site-packages",
        }
    )


def test_needs_process_reexec_skips_after_bootstrap(monkeypatch):
    monkeypatch.setenv(ROS_BOOTSTRAP_MARKER, "1")
    monkeypatch.setenv("LD_LIBRARY_PATH", "/usr/lib")
    monkeypatch.setenv("PYTHONPATH", "")

    assert not _needs_process_reexec(
        {
            "LD_LIBRARY_PATH": "/opt/ros/jazzy/lib:/usr/lib",
            "PYTHONPATH": "/opt/ros/jazzy/lib/python3.12/site-packages",
        }
    )


def test_bootstrap_runtime_environment_reexecs_before_applying_paths(monkeypatch, tmp_path):
    class ReexecTriggered(Exception):
        pass

    calls = {"reexec": None, "applied": False}

    def fake_reexec(env_updates):
        calls["reexec"] = dict(env_updates)
        raise ReexecTriggered()

    monkeypatch.setattr("app.runtime_env._ros_imports_available", lambda: False)
    monkeypatch.setattr(
        "app.runtime_env._capture_ros_environment",
        lambda repo_dir: {
            "LD_LIBRARY_PATH": "/opt/ros/jazzy/lib",
            "PYTHONPATH": str(tmp_path),
        },
    )
    monkeypatch.setattr("app.runtime_env._reexec_with_environment", fake_reexec)
    monkeypatch.setattr(
        "app.runtime_env._apply_environment_updates",
        lambda env_updates: calls.__setitem__("applied", True),
    )

    try:
        bootstrap_runtime_environment(tmp_path)
    except ReexecTriggered:
        pass
    else:
        raise AssertionError("Se esperaba relanzamiento del proceso.")

    assert calls["reexec"] == {
        "LD_LIBRARY_PATH": "/opt/ros/jazzy/lib",
        "PYTHONPATH": str(tmp_path),
    }
    assert calls["applied"] is False


def test_bootstrap_runtime_environment_returns_visible_imports_without_setup(monkeypatch, tmp_path):
    monkeypatch.setattr("app.runtime_env._ros_imports_available", lambda: True)
    monkeypatch.setattr("app.runtime_env._capture_ros_environment", lambda repo_dir: {})

    assert bootstrap_runtime_environment(tmp_path) is True


def test_preload_app_env_files_reads_v2_env(monkeypatch, tmp_path):
    monkeypatch.delenv("MEDIAPIPE_STDERR_FILTER", raising=False)
    v2_dir = tmp_path / "V2.0"
    v2_dir.mkdir()
    (v2_dir / ".env").write_text("MEDIAPIPE_STDERR_FILTER=false\n", encoding="utf-8")

    _preload_app_env_files(tmp_path)

    assert os.environ["MEDIAPIPE_STDERR_FILTER"] == "false"


def test_filtered_stderr_suppresses_known_mediapipe_noise():
    target = StringIO()
    stream = _FilteredStderr(
        target,
        (
            "inference_feedback_manager.cc:114",
            "landmark_projection_calculator.cc:78",
        ),
    )

    stream.write("normal line\n")
    stream.write("W0000 ... inference_feedback_manager.cc:114 noisy\n")
    stream.write("W0000 ... landmark_projection_calculator.cc:78 noisy\n")
    stream.flush()

    assert target.getvalue() == "normal line\n"


def test_install_mediapipe_stderr_filter_can_be_disabled(monkeypatch):
    original_stderr = sys.stderr
    monkeypatch.setenv("MEDIAPIPE_STDERR_FILTER", "false")
    monkeypatch.setattr(sys, "stderr", StringIO())

    _install_mediapipe_stderr_filter()

    assert not getattr(sys.stderr, "_silvia_filter_installed", False)
    monkeypatch.setattr(sys, "stderr", original_stderr)
