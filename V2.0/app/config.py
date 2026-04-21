from __future__ import annotations

import json
import os
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Dict, List


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


def _parse_json_env(name: str, default: Dict[str, Any]) -> Dict[str, Any]:
    raw = os.getenv(name)
    if not raw:
        return default
    try:
        return json.loads(raw)
    except json.JSONDecodeError:
        return default


def _parse_float(name: str, default: float) -> float:
    try:
        return float(os.getenv(name, default))
    except (TypeError, ValueError):
        return default


def _parse_int(name: str, default: int) -> int:
    try:
        return int(os.getenv(name, default))
    except (TypeError, ValueError):
        return default


def _parse_joint_list(name: str, default: List[float]) -> List[float]:
    raw = os.getenv(name)
    if not raw:
        return default
    try:
        values = [float(part.strip()) for part in raw.split(",")]
        return values if len(values) == 6 else default
    except ValueError:
        return default


def _parse_optional_path(name: str, repo_dir: Path) -> Path | None:
    raw = os.getenv(name, "").strip()
    if not raw:
        return None
    candidate = Path(raw).expanduser()
    if not candidate.is_absolute():
        candidate = repo_dir / candidate
    return candidate.resolve()


@dataclass(slots=True)
class AppConfig:
    base_dir: Path
    repo_dir: Path
    data_dir: Path
    calibration_dir: Path
    log_dir: Path
    host: str
    port: int
    debug: bool
    ssl_mode: str
    ssl_cert_file: Path | None
    ssl_key_file: Path | None
    secret_key: str
    openai_api_key: str
    openai_realtime_model: str
    openai_text_model: str
    openai_stt_model: str
    openai_stt_prompt: str
    openai_tts_model: str
    openai_realtime_voice: str
    openai_tts_voice: str
    openai_voice_speed: float
    wake_word: str
    voice_timeout_s: float
    voice_chunk_ms: int
    vision_model_path: Path
    vision_confidence_threshold: float
    hand_landmarker_model_path: Path
    hand_landmarker_model_url: str
    camera_index: int
    camera_width: int
    camera_height: int
    camera_fps: int
    ur5_host: str
    ur5_cmd_port: int
    ur5_state_port: int
    ur5_dashboard_port: int
    magnet_do_pin: int
    hand_plane_z_mm: float
    object_plane_z_mm: float
    hand_workspace: Dict[str, Any]
    object_workspace: Dict[str, Any]
    max_track_speed_xy_mm_s: float
    max_track_speed_z_mm_s: float
    hand_deadzone_mm: float
    track_smoothing_alpha: float
    target_loss_timeout_s: float
    safety_poll_ms: int
    pick_approach_lift_mm: float
    home_joints_deg: List[float]
    greet_sequence_deg: List[List[float]]
    greet_speed_rad_s: float
    greet_acceleration_rad_s2: float
    greet_blend_radius_m: float

    @classmethod
    def load(cls) -> "AppConfig":
        base_dir = Path(__file__).resolve().parents[1]
        repo_dir = base_dir.parent
        _load_env_file(repo_dir / ".env")
        _load_env_file(base_dir / ".env")

        default_home = [-51.9, -71.85, -112.7, -85.96, 90.0, 38.0]
        default_greet = [
            default_home,
            [-71.96, -90.60, -99.21, 2.23, 122.58, 37.98],
            [-17.52, -134.93, -70.15, 30.38, 68.76, 42.41],
            [-65.14, -147.04, -70.09, 42.23, 117.62, 44.25],
            [-65.21, -65.16, -69.04, -62.73, 110.99, 37.60],
        ]

        default_hand_workspace = {
            "x": [-450.0, 450.0],
            "y": [-700.0, 150.0],
            "z": [150.0, 700.0],
        }
        default_object_workspace = {
            "x": [-450.0, 450.0],
            "y": [-700.0, 150.0],
            "z": [20.0, 600.0],
        }

        greet_raw = os.getenv("GREET_SEQUENCE_DEG")
        if greet_raw:
            try:
                greet_sequence = json.loads(greet_raw)
            except json.JSONDecodeError:
                greet_sequence = default_greet
        else:
            greet_sequence = default_greet

        return cls(
            base_dir=base_dir,
            repo_dir=repo_dir,
            data_dir=base_dir / "data",
            calibration_dir=base_dir / "data" / "calibration",
            log_dir=base_dir / "data" / "logs",
            host=os.getenv("FLASK_HOST", "0.0.0.0"),
            port=_parse_int("FLASK_PORT", 5050),
            debug=os.getenv("FLASK_DEBUG", "false").lower() == "true",
            ssl_mode=os.getenv("FLASK_SSL_MODE", "off").strip().lower(),
            ssl_cert_file=_parse_optional_path("FLASK_SSL_CERT_FILE", repo_dir),
            ssl_key_file=_parse_optional_path("FLASK_SSL_KEY_FILE", repo_dir),
            secret_key=os.getenv("SECRET_KEY", "silvia-v2-secret"),
            openai_api_key=os.getenv("OPENAI_API_KEY", ""),
            openai_realtime_model=os.getenv("OPENAI_REALTIME_MODEL", "gpt-realtime-1.5"),
            openai_text_model=os.getenv("OPENAI_TEXT_MODEL", "gpt-5.4-mini"),
            openai_stt_model=os.getenv("OPENAI_STT_MODEL", "gpt-4o-transcribe"),
            openai_stt_prompt=os.getenv(
                "OPENAI_STT_PROMPT",
                "",
            ),
            openai_tts_model=os.getenv("OPENAI_TTS_MODEL", "gpt-4o-mini-tts"),
            openai_realtime_voice=os.getenv("OPENAI_REALTIME_VOICE", "shimmer"),
            openai_tts_voice=os.getenv("OPENAI_TTS_VOICE", "shimmer"),
            openai_voice_speed=_parse_float("OPENAI_VOICE_SPEED", 1.5),
            wake_word=os.getenv("WAKE_WORD", "silvia"),
            voice_timeout_s=_parse_float("VOICE_TIMEOUT_S", 14.0),
            voice_chunk_ms=_parse_int("VOICE_CHUNK_MS", 2500),
            vision_model_path=Path(
                os.getenv(
                    "VISION_MODEL_PATH",
                    str(repo_dir / "V_COVA" / "Dashboard" / "V6_best.pt"),
                )
            ),
            vision_confidence_threshold=_parse_float("VISION_CONFIDENCE_THRESHOLD", 0.5),
            hand_landmarker_model_path=Path(
                os.getenv(
                    "HAND_LANDMARKER_MODEL_PATH",
                    str(base_dir / "data" / "models" / "hand_landmarker.task"),
                )
            ),
            hand_landmarker_model_url=os.getenv(
                "HAND_LANDMARKER_MODEL_URL",
                "https://storage.googleapis.com/mediapipe-models/hand_landmarker/hand_landmarker/float16/1/hand_landmarker.task",
            ),
            camera_index=_parse_int("CAMERA_INDEX", 0),
            camera_width=_parse_int("CAMERA_WIDTH", 1280),
            camera_height=_parse_int("CAMERA_HEIGHT", 720),
            camera_fps=_parse_int("CAMERA_FPS", 30),
            ur5_host=os.getenv("UR5_HOST", "192.168.1.1"),
            ur5_cmd_port=_parse_int("UR5_CMD_PORT", 30002),
            ur5_state_port=_parse_int("UR5_STATE_PORT", 30001),
            ur5_dashboard_port=_parse_int("UR5_DASHBOARD_PORT", 29999),
            magnet_do_pin=_parse_int("MAGNET_DO_PIN", 0),
            hand_plane_z_mm=_parse_float("HAND_PLANE_Z_MM", 350.0),
            object_plane_z_mm=_parse_float("OBJECT_PLANE_Z_MM", 120.0),
            hand_workspace=_parse_json_env("HAND_WORKSPACE_JSON", default_hand_workspace),
            object_workspace=_parse_json_env("OBJECT_WORKSPACE_JSON", default_object_workspace),
            max_track_speed_xy_mm_s=_parse_float("MAX_TRACK_SPEED_XY_MM_S", 120.0),
            max_track_speed_z_mm_s=_parse_float("MAX_TRACK_SPEED_Z_MM_S", 60.0),
            hand_deadzone_mm=_parse_float("HAND_DEADZONE_MM", 12.0),
            track_smoothing_alpha=_parse_float("TRACK_SMOOTHING_ALPHA", 0.35),
            target_loss_timeout_s=_parse_float("TARGET_LOSS_TIMEOUT_S", 1.0),
            safety_poll_ms=_parse_int("SAFETY_POLL_MS", 500),
            pick_approach_lift_mm=_parse_float("PICK_APPROACH_LIFT_MM", 120.0),
            home_joints_deg=_parse_joint_list("HOME_JOINTS_DEG", default_home),
            greet_sequence_deg=greet_sequence,
            greet_speed_rad_s=_parse_float("GREET_SPEED_RAD_S", 1.0),
            greet_acceleration_rad_s2=_parse_float("GREET_ACCELERATION_RAD_S2", 1.0),
            greet_blend_radius_m=_parse_float("GREET_BLEND_RADIUS_M", 0.02),
        )

    def calibration_file(self) -> Path:
        return self.calibration_dir / "camera_calibration.json"

    def ssl_context(self) -> str | tuple[str, str] | None:
        if self.ssl_mode in {"", "off", "false", "disabled"}:
            return None
        if self.ssl_mode == "adhoc":
            return "adhoc"
        if self.ssl_mode == "cert":
            if not self.ssl_cert_file or not self.ssl_key_file:
                raise RuntimeError(
                    "FLASK_SSL_MODE=cert requiere FLASK_SSL_CERT_FILE y FLASK_SSL_KEY_FILE."
                )
            missing = [path for path in (self.ssl_cert_file, self.ssl_key_file) if not path.exists()]
            if missing:
                missing_str = ", ".join(str(path) for path in missing)
                raise RuntimeError(f"No se encontraron los archivos SSL configurados: {missing_str}")
            return (str(self.ssl_cert_file), str(self.ssl_key_file))
        raise RuntimeError(
            "FLASK_SSL_MODE invalido. Usa 'off', 'adhoc' o 'cert'."
        )

    def public_settings(self) -> Dict[str, Any]:
        return {
            "wake_word": self.wake_word,
            "voice_chunk_ms": self.voice_chunk_ms,
            "network": {
                "host": self.host,
                "port": self.port,
                "ssl_mode": self.ssl_mode,
                "https_enabled": self.ssl_mode in {"adhoc", "cert"},
            },
            "vision_model_name": self.vision_model_path.name,
            "vision_confidence_threshold": self.vision_confidence_threshold,
            "camera": {
                "index": self.camera_index,
                "width": self.camera_width,
                "height": self.camera_height,
                "fps": self.camera_fps,
            },
            "models": {
                "text": self.openai_text_model,
                "realtime": self.openai_realtime_model,
                "stt": self.openai_stt_model,
                "tts": self.openai_tts_model,
            },
            "voices": {
                "realtime": self.openai_realtime_voice,
                "tts": self.openai_tts_voice,
            },
            "voice_speed": self.openai_voice_speed,
        }
