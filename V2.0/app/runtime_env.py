from __future__ import annotations

import importlib.util
import json
import logging
import os
import shlex
import subprocess
import sys
from functools import lru_cache
from io import TextIOBase
from pathlib import Path

logger = logging.getLogger(__name__)

ROS_IMPORTS = ("rclpy", "cv_bridge", "sensor_msgs")
ROS_BOOTSTRAP_MARKER = "_SILVIA_ROS_BOOTSTRAPPED"
ROS_ENV_KEYS = (
    "PYTHONPATH",
    "LD_LIBRARY_PATH",
    "AMENT_PREFIX_PATH",
    "CMAKE_PREFIX_PATH",
    "COLCON_PREFIX_PATH",
    "PATH",
    "ROS_DISTRO",
    "ROS_PYTHON_VERSION",
    "ROS_VERSION",
)
MEDIAPIPE_STDERR_FILTER_PATTERNS = (
    "WARNING: All log messages before absl::InitializeLog() is called are written to STDERR",
    "gl_context_egl.cc:",
    "gl_context.cc:",
    "INFO: Created TensorFlow Lite XNNPACK delegate for CPU.",
    "inference_feedback_manager.cc:114",
    "landmark_projection_calculator.cc:78",
)


def bootstrap_runtime_environment(repo_dir: Path | None = None) -> bool:
    imports_available = _ros_imports_available()
    repo_root = repo_dir or Path(__file__).resolve().parents[2]
    _preload_app_env_files(repo_root)
    _install_mediapipe_stderr_filter()
    env_updates = _capture_ros_environment(str(repo_root))
    if not env_updates:
        return imports_available

    if _needs_process_reexec(env_updates):
        _reexec_with_environment(env_updates)

    _apply_environment_updates(env_updates)
    available = _ros_imports_available()
    if available:
        logger.info("Entorno ROS2 cargado automaticamente para HP60C.")
    return available


def _ros_imports_available() -> bool:
    return all(importlib.util.find_spec(module_name) is not None for module_name in ROS_IMPORTS)


def _load_env_file(path: Path) -> None:
    if not path.exists():
        return

    for raw_line in path.read_text(encoding="utf-8").splitlines():
        line = raw_line.strip()
        if not line or line.startswith("#") or "=" not in line:
            continue
        key, value = line.split("=", 1)
        key = key.strip()
        if key and key not in os.environ:
            os.environ[key] = value.strip().strip("'").strip('"')


def _preload_app_env_files(repo_root: Path) -> None:
    _load_env_file(repo_root / ".env")
    _load_env_file(repo_root / "V2.0" / ".env")


def _parse_bool_env(name: str, default: bool) -> bool:
    raw = str(os.getenv(name, str(default))).strip().lower()
    if raw in {"1", "true", "yes", "on"}:
        return True
    if raw in {"0", "false", "no", "off"}:
        return False
    return default


class _FilteredStderr(TextIOBase):
    def __init__(self, wrapped, patterns: tuple[str, ...]) -> None:
        self._wrapped = wrapped
        self._patterns = patterns
        self._buffer = ""
        self._silvia_filter_installed = True

    def write(self, data):
        if not data:
            return 0

        self._buffer += data
        while "\n" in self._buffer:
            line, self._buffer = self._buffer.split("\n", 1)
            self._write_line(line + "\n")
        return len(data)

    def flush(self) -> None:
        if self._buffer:
            self._write_line(self._buffer)
            self._buffer = ""
        self._wrapped.flush()

    def isatty(self):
        return self._wrapped.isatty()

    def fileno(self):
        return self._wrapped.fileno()

    @property
    def encoding(self):
        return getattr(self._wrapped, "encoding", None)

    def writable(self):
        return True

    def _write_line(self, line: str) -> None:
        if any(pattern in line for pattern in self._patterns):
            return
        self._wrapped.write(line)

    def __getattr__(self, name: str):
        return getattr(self._wrapped, name)


def _install_mediapipe_stderr_filter() -> None:
    if not _parse_bool_env("MEDIAPIPE_STDERR_FILTER", True):
        return
    if getattr(sys.stderr, "_silvia_filter_installed", False):
        return
    sys.stderr = _FilteredStderr(sys.stderr, MEDIAPIPE_STDERR_FILTER_PATTERNS)


def _candidate_setup_scripts(repo_dir: str) -> list[Path]:
    repo_root = Path(repo_dir)
    ros_distro = (os.getenv("ROS_DISTRO", "jazzy") or "jazzy").strip()
    candidates = [
        os.getenv("ROS_SETUP_FILE", ""),
        f"/opt/ros/{ros_distro}/setup.bash",
        os.getenv("ROS_WORKSPACE_SETUP_FILE", ""),
        str(repo_root / "hp60c_portable_bundle" / "workspace" / "install" / "setup.bash"),
    ]

    unique_paths: list[Path] = []
    for candidate in candidates:
        raw = str(candidate).strip()
        if not raw:
            continue
        resolved = Path(raw).expanduser()
        if resolved.exists() and resolved not in unique_paths:
            unique_paths.append(resolved)
    return unique_paths


@lru_cache(maxsize=4)
def _capture_ros_environment(repo_dir: str) -> dict[str, str]:
    setup_scripts = _candidate_setup_scripts(repo_dir)
    if not setup_scripts:
        logger.warning("No se encontraron scripts setup.bash para ROS2/HP60C.")
        return {}

    source_steps = "\n".join(
        f"source {shlex.quote(str(script))} >/dev/null 2>&1" for script in setup_scripts
    )
    keys_json = json.dumps(list(ROS_ENV_KEYS))
    command = f"""
set -e
{source_steps}
python3 - <<'PY'
import json
import os

keys = {keys_json}
print(json.dumps({{key: os.environ.get(key, "") for key in keys if os.environ.get(key)}}))
PY
""".strip()

    result = subprocess.run(
        ["bash", "-lc", command],
        capture_output=True,
        text=True,
        check=False,
    )
    if result.returncode != 0:
        logger.warning(
            "No se pudo cargar el entorno ROS2 automaticamente (codigo %s).",
            result.returncode,
        )
        if result.stderr.strip():
            logger.debug("stderr bootstrap ROS2: %s", result.stderr.strip())
        return {}

    raw_output = result.stdout.strip()
    if not raw_output:
        return {}

    try:
        return {
            key: value
            for key, value in json.loads(raw_output).items()
            if isinstance(value, str) and value.strip()
        }
    except json.JSONDecodeError:
        logger.warning("Salida invalida al cargar el entorno ROS2: %s", raw_output)
        return {}


def _apply_environment_updates(env_updates: dict[str, str]) -> None:
    for key, value in env_updates.items():
        os.environ[key] = value

    python_path = env_updates.get("PYTHONPATH", "")
    if not python_path:
        return

    for entry in reversed(python_path.split(os.pathsep)):
        normalized = entry.strip()
        if normalized and normalized not in sys.path:
            sys.path.insert(0, normalized)


def _needs_process_reexec(env_updates: dict[str, str]) -> bool:
    if os.environ.get(ROS_BOOTSTRAP_MARKER) == "1":
        return False

    critical_keys = ("LD_LIBRARY_PATH", "PYTHONPATH")
    return any(os.environ.get(key, "") != env_updates.get(key, "") for key in critical_keys)


def _reexec_with_environment(env_updates: dict[str, str]) -> None:
    updated_env = dict(os.environ)
    updated_env.update(env_updates)
    updated_env[ROS_BOOTSTRAP_MARKER] = "1"
    logger.info("Relanzando proceso con entorno ROS2 cargado para HP60C.")
    os.execve(sys.executable, [sys.executable, *sys.argv], updated_env)
