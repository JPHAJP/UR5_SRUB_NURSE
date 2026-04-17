from __future__ import annotations

import logging
import threading
import time
from typing import Dict, Optional, Tuple

from ..robot.motion_profiles import smooth_velocity, velocity_from_error_mm
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
            self.gateway.stop_motion()

        self.state.set_mode(mode)
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
        self.gateway.stop_motion()
        self.gateway.set_magnet(False)
        self.state.clear_voice_session()
        return True, "Operacion cancelada."

    def _worker_loop(self) -> None:
        while self._running and not self._stop_event.is_set():
            if self.safety.is_locked():
                self.gateway.stop_motion()
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

            time.sleep(0.05)

    def _follow_hand_step(self) -> None:
        target = self.vision.get_latest_hand_target()
        if not target or not target.get("world_mm"):
            if time.time() - self._last_hand_seen_ts > self.config.target_loss_timeout_s:
                self.gateway.stop_motion()
            return

        self._last_hand_seen_ts = time.time()
        current_pose = self.gateway.current_pose_mm()
        target_xyz = target["world_mm"]
        ok, message = self.safety.evaluate_target("hand_follow", target_xyz)
        if not ok:
            self.gateway.stop_motion()
            self.state.lock_safety(message, {"mode": "hand_follow", "target": target_xyz})
            return

        error_x = target_xyz[0] - current_pose[0]
        error_y = target_xyz[1] - current_pose[1]
        error_z = self.config.hand_plane_z_mm - current_pose[2]

        raw_velocity = [
            velocity_from_error_mm(error_x, self.config.max_track_speed_xy_mm_s, self.config.hand_deadzone_mm),
            velocity_from_error_mm(error_y, self.config.max_track_speed_xy_mm_s, self.config.hand_deadzone_mm),
            velocity_from_error_mm(error_z, self.config.max_track_speed_z_mm_s, self.config.hand_deadzone_mm),
        ]
        smoothed = smooth_velocity(self._prev_velocity, raw_velocity, self.config.track_smoothing_alpha)
        self._prev_velocity = smoothed

        if max(abs(value) for value in smoothed) < 0.5:
            self.gateway.stop_motion()
            return

        self.gateway.speed_linear_mm(smoothed)

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
        waypoints = build_pick_waypoints(target_xyz[:2], self.config.object_plane_z_mm, self.config.pick_approach_lift_mm)

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
