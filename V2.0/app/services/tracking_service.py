from __future__ import annotations

import logging
import threading
import time
from typing import Any, Dict, Optional, Tuple

from ..robot.motion_profiles import limit_velocity_acceleration, smooth_velocity, velocity_from_error_mm
from ..robot.sequences import build_pick_waypoints

logger = logging.getLogger(__name__)


class TrackingService:
    def __init__(self, config, state, vision_service, gateway, safety_service) -> None:
        self.config = config
        self.state = state
        self.vision = vision_service
        self.gateway = gateway
        self.safety = safety_service
        self._running = False
        self._thread: threading.Thread | None = None
        self._stop_event = threading.Event()
        self._pending_pick: Optional[Dict[str, object]] = None
        self._last_hand_seen_ts = 0.0
        self._prev_velocity = [0.0, 0.0, 0.0]
        self._last_command_ts = 0.0
        self._last_stop_ts = 0.0
        self._last_stop_reason = ""

    def start(self) -> None:
        if self._running:
            return
        self._running = True
        self._thread = threading.Thread(target=self._worker_loop, daemon=True)
        self._thread.start()

    def shutdown(self) -> None:
        self._running = False
        self._stop_event.set()
        if self._thread and self._thread.is_alive():
            self._thread.join(timeout=1.0)

    def set_mode(self, mode: str) -> Tuple[bool, str]:
        if mode not in {"idle", "hand_follow", "object_pick"}:
            return False, "Modo no valido."

        if mode == "idle":
            self._pending_pick = None
            self._stop_tracking("idle")

        self.state.set_mode(mode)
        if mode == "hand_follow":
            self._publish_tracking_status(
                {
                    "active": True,
                    "stop_reason": "",
                    "velocity_mm_s": [0.0, 0.0, 0.0],
                }
            )
        self.state.add_event("system_event", f"Modo cambiado a {mode}.", {"mode": mode})
        return True, f"Modo {mode} activado."

    def request_pick(self, label: str | None = None, deliver_to_hand: bool = False) -> Tuple[bool, str]:
        self._pending_pick = {
            "label": label,
            "deliver_to_hand": deliver_to_hand,
        }
        self.state.set_mode("object_pick")
        self.state.set_selected_object_label(label)
        return True, "Solicitud de recoleccion programada."

    def cancel(self) -> Tuple[bool, str]:
        self._pending_pick = None
        self.state.set_mode("idle")
        self._stop_tracking("cancel")
        self.gateway.set_magnet(False)
        self.state.clear_voice_session()
        return True, "Operacion cancelada."

    def _worker_loop(self) -> None:
        while self._running and not self._stop_event.is_set():
            if self.safety.is_locked():
                self._stop_tracking("safety_locked")
                time.sleep(0.1)
                continue

            if self._pending_pick:
                request = dict(self._pending_pick)
                self._pending_pick = None
                self._execute_pick(request)
                time.sleep(0.1)
                continue

            if self.state.snapshot()["mode"] == "hand_follow":
                self._follow_hand_step()
            else:
                self._prev_velocity = [0.0, 0.0, 0.0]
                self._publish_tracking_status({"active": False, "stop_reason": "idle"})

            time.sleep(0.05)

    def _follow_hand_step(self) -> None:
        stream_health = self.vision.get_stream_health()
        if not stream_health.get("rgb_ok") or not stream_health.get("depth_ok"):
            self._stop_tracking("camera_stale", {"stream_health": stream_health})
            return

        target = self.vision.get_latest_hand_target()
        if not target or not target.get("world_mm"):
            if time.time() - self._last_hand_seen_ts > min(
                float(self.config.target_loss_timeout_s),
                float(getattr(self.config, "hand_target_max_age_s", 0.5)),
            ):
                self._stop_tracking("hand_not_visible", {"stream_health": stream_health})
            return

        now = time.time()
        target_age_s = self._target_age_s(target, now)
        if target_age_s is None or target_age_s > float(getattr(self.config, "hand_target_max_age_s", 0.5)):
            self._stop_tracking("target_stale", {"target_age_s": target_age_s, "stream_health": stream_health})
            return

        self._last_hand_seen_ts = now
        current_pose = self.gateway.current_pose_mm()
        target_xyz = list(target["world_mm"])
        target_z_mm = self.vision.get_hand_follow_calibration()["target_z_mm"]
        target_xyz[2] = float(target_z_mm)
        ok, message = self.safety.evaluate_target("hand_follow", target_xyz)
        if not ok:
            self._stop_tracking("safety_rejected", {"target": target_xyz, "message": message})
            self.state.lock_safety(message, {"mode": "hand_follow", "target": target_xyz})
            return

        error_x = target_xyz[0] - current_pose[0]
        error_y = target_xyz[1] - current_pose[1]
        error_z = target_xyz[2] - current_pose[2]

        raw_velocity = [
            velocity_from_error_mm(error_x, self.config.max_track_speed_xy_mm_s, self.config.hand_deadzone_mm),
            velocity_from_error_mm(error_y, self.config.max_track_speed_xy_mm_s, self.config.hand_deadzone_mm),
            velocity_from_error_mm(error_z, self.config.max_track_speed_z_mm_s, self.config.hand_deadzone_mm),
        ]
        smoothed = smooth_velocity(self._prev_velocity, raw_velocity, self.config.track_smoothing_alpha)
        command_interval_s = self._command_interval_s()
        if self._last_command_ts and now - self._last_command_ts < command_interval_s:
            self._publish_tracking_status(
                self._tracking_payload(
                    target,
                    target_xyz,
                    [error_x, error_y, error_z],
                    self._prev_velocity,
                    target_age_s,
                    "rate_limited",
                    stream_health,
                )
            )
            return

        dt_s = max(command_interval_s, now - self._last_command_ts) if self._last_command_ts else command_interval_s
        limited = limit_velocity_acceleration(
            self._prev_velocity,
            smoothed,
            float(getattr(self.config, "max_track_accel_mm_s2", 300.0)),
            dt_s,
        )
        self._prev_velocity = limited
        self._last_command_ts = now

        if max(abs(value) for value in limited) < 0.5:
            self._stop_tracking(
                "deadzone",
                self._tracking_payload(
                    target,
                    target_xyz,
                    [error_x, error_y, error_z],
                    limited,
                    target_age_s,
                    "deadzone",
                    stream_health,
                ),
            )
            return

        self.gateway.speed_linear_mm(limited)
        self._publish_tracking_status(
            self._tracking_payload(
                target,
                target_xyz,
                [error_x, error_y, error_z],
                limited,
                target_age_s,
                "",
                stream_health,
            )
        )

    def _stop_tracking(self, reason: str, extra: Dict[str, Any] | None = None) -> None:
        now = time.time()
        should_send_stop = (
            reason != self._last_stop_reason
            or self._last_stop_ts <= 0.0
            or (now - self._last_stop_ts) >= 0.5
        )
        if should_send_stop:
            self.gateway.stop_motion()
            self._last_stop_ts = now
            self._last_stop_reason = reason
        self._prev_velocity = [0.0, 0.0, 0.0]
        self._last_command_ts = 0.0
        payload = {
            "active": self.state.snapshot().get("mode") == "hand_follow",
            "velocity_mm_s": [0.0, 0.0, 0.0],
            "stop_reason": reason,
        }
        if extra:
            payload.update(extra)
        self._publish_tracking_status(payload)

    def _target_age_s(self, target: Dict[str, Any], now: float) -> float | None:
        timestamp = target.get("timestamp_s")
        if timestamp is None:
            return None
        try:
            return max(0.0, now - float(timestamp))
        except (TypeError, ValueError):
            return None

    def _command_interval_s(self) -> float:
        try:
            hz = float(getattr(self.config, "track_command_hz", 10.0))
        except (TypeError, ValueError):
            hz = 10.0
        return 1.0 / hz if hz > 0.0 else 0.0

    def _tracking_payload(
        self,
        target: Dict[str, Any],
        target_xyz: list[float],
        error_xyz: list[float],
        velocity_xyz: list[float],
        target_age_s: float | None,
        stop_reason: str,
        stream_health: Dict[str, Any],
    ) -> Dict[str, Any]:
        return {
            "active": True,
            "hand_world_mm": list(target.get("world_mm") or []),
            "target_xyz_mm": [round(float(value), 3) for value in target_xyz],
            "error_xyz_mm": [round(float(value), 3) for value in error_xyz],
            "velocity_mm_s": [round(float(value), 3) for value in velocity_xyz],
            "target_age_s": round(float(target_age_s), 3) if target_age_s is not None else None,
            "stop_reason": stop_reason,
            "stream_health": stream_health,
        }

    def _publish_tracking_status(self, payload: Dict[str, Any]) -> None:
        status = {
            "active": False,
            "hand_world_mm": None,
            "target_xyz_mm": None,
            "error_xyz_mm": None,
            "velocity_mm_s": [0.0, 0.0, 0.0],
            "target_age_s": None,
            "stop_reason": "",
            "command_hz": float(getattr(self.config, "track_command_hz", 10.0)),
            "max_accel_mm_s2": float(getattr(self.config, "max_track_accel_mm_s2", 300.0)),
        }
        status.update(payload)
        self.state.set_tracking_status(status)

    def _execute_pick(self, request: Dict[str, object]) -> None:
        label = request.get("label")
        target = self.vision.find_object_target(label if isinstance(label, str) else None)
        if not target or not target.get("world_mm"):
            self.state.add_event("system_event", "No se encontro el objeto solicitado.", {"label": label})
            return

        target_xyz = target["world_mm"]
        ok, message = self.safety.evaluate_target("object_pick", target_xyz)
        if not ok:
            self.state.lock_safety(message, {"mode": "object_pick", "target": target_xyz})
            return

        self.state.add_event("robot_log", f"Recogiendo {target['label']}.", {"target": target})
        waypoints = build_pick_waypoints(target_xyz[:2], target_xyz[2], self.config.pick_approach_lift_mm)

        self.gateway.move_linear_mm(waypoints["approach"])
        time.sleep(0.8)
        self.gateway.move_linear_mm(waypoints["pick"])
        time.sleep(0.8)
        self.gateway.set_magnet(True)
        time.sleep(0.4)
        self.gateway.move_linear_mm(waypoints["retreat"])
        time.sleep(0.8)

        if bool(request.get("deliver_to_hand")):
            self.state.set_mode("hand_follow")
            self.state.add_event("robot_log", "Objeto recogido. Cambiando a seguir mano.", {"label": label})
        else:
            self.gateway.go_home()
            self.state.add_event("robot_log", "Objeto recogido. Regresando a home.", {"label": label})
