from __future__ import annotations

import importlib.util
import json
import logging
import os
import shlex
import subprocess
import sys
from functools import lru_cache
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


def bootstrap_runtime_environment(repo_dir: Path | None = None) -> bool:
    imports_available = _ros_imports_available()
    repo_root = repo_dir or Path(__file__).resolve().parents[2]
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
