from __future__ import annotations

import logging
import math
import threading
import time
from typing import Dict, List, Tuple

from .motion_profiles import clamp
from .sequences import DEFAULT_ORIENTATION, build_greet_sequence
from .ur_dashboard_client import URDashboardClient
from .ur_script_client import URScriptClient
from .ur_state_client import URStateClient

logger = logging.getLogger(__name__)


class RobotGateway:
    def __init__(self, config, state) -> None:
        self.config = config
        self.state = state
        self._lock = threading.RLock()
        self.script_client = URScriptClient(config.ur5_host, config.ur5_cmd_port)
        self.state_client = URStateClient(config.ur5_host, config.ur5_state_port)
        self.dashboard_client = URDashboardClient(config.ur5_host, config.ur5_dashboard_port)
        self._last_status: Dict[str, object] = {
            "connected": False,
            "current_pose_mm": [0.0, 0.0, 0.0, *DEFAULT_ORIENTATION],
            "joint_positions_deg": list(config.home_joints_deg),
            "robot_mode": "DISCONNECTED",
            "program_state": "STOPPED",
            "safety_status": "UNKNOWN",
            "remote_control": "false",
        }

    def refresh_status(self) -> Dict[str, object]:
        with self._lock:
            dashboard = self.dashboard_client.get_status()
            state_snapshot = self.state_client.read_state()
            connected = bool(dashboard) or bool(state_snapshot.get("connected"))

            if state_snapshot.get("tcp_pose_mm"):
                current_pose_mm = state_snapshot["tcp_pose_mm"]
            else:
                current_pose_mm = self._last_status["current_pose_mm"]

            joint_positions_deg = self._last_status["joint_positions_deg"]
            if state_snapshot.get("joint_positions_rad"):
                joint_positions_deg = [
                    round(math.degrees(value), 2)
                    for value in state_snapshot["joint_positions_rad"]
                ]

            self._last_status = {
                "connected": connected,
                "current_pose_mm": current_pose_mm,
                "joint_positions_deg": joint_positions_deg,
                "robot_mode": dashboard.get("robot_mode", "UNKNOWN"),
                "program_state": dashboard.get("program_state", "UNKNOWN"),
                "safety_status": dashboard.get("safety_status", "UNKNOWN"),
                "remote_control": dashboard.get("remote_control", "false"),
                "magnet_enabled": self.state.snapshot()["magnet_enabled"],
            }
            self.state.set_robot_status(self._last_status)
            return dict(self._last_status)

    def current_pose_mm(self) -> List[float]:
        return list(self.refresh_status()["current_pose_mm"])

    def is_remote_control_enabled(self) -> bool:
        status = str(self.refresh_status().get("remote_control", "")).lower()
        return "true" in status or "remote" in status

    def send_urscript(self, command: str) -> Tuple[bool, str]:
        if self.script_client.send_command(command):
            return True, "Comando URScript enviado."
        return False, "No se pudo enviar el comando al robot."

    def move_joints_deg(self, joints_deg: List[float], speed: float = 1.5, acceleration: float = 2.5) -> Tuple[bool, str]:
        joints_rad = [math.radians(value) for value in joints_deg]
        joint_values = ", ".join(f"{value:.5f}" for value in joints_rad)
        return self.send_urscript(f"movej([{joint_values}], a = {acceleration}, v = {speed})")

    def move_linear_mm(
        self,
        pose_xyz_mm: List[float],
        orientation: List[float] | None = None,
        speed: float = 0.25,
        acceleration: float = 1.2,
    ) -> Tuple[bool, str]:
        current = self.current_pose_mm()
        orientation = orientation or current[3:] or DEFAULT_ORIENTATION
        x_m, y_m, z_m = [float(value) / 1000.0 for value in pose_xyz_mm[:3]]
        rx, ry, rz = orientation
        command = (
            f"movel(p[{x_m:.6f},{y_m:.6f},{z_m:.6f},{rx:.6f},{ry:.6f},{rz:.6f}], "
            f"a = {acceleration}, v = {speed}, t = 0, r = 0)"
        )
        return self.send_urscript(command)

    def speed_linear_mm(
        self,
        velocity_xyz_mm_s: List[float],
        angular_velocity: List[float] | None = None,
        acceleration: float = 0.5,
        time_step: float = 0.1,
    ) -> Tuple[bool, str]:
        angular_velocity = angular_velocity or [0.0, 0.0, 0.0]
        velocity_m = [float(value) / 1000.0 for value in velocity_xyz_mm_s]
        command = (
            "speedl(["
            f"{velocity_m[0]:.5f}, {velocity_m[1]:.5f}, {velocity_m[2]:.5f}, "
            f"{angular_velocity[0]:.5f}, {angular_velocity[1]:.5f}, {angular_velocity[2]:.5f}"
            f"], {acceleration}, {time_step})"
        )
        return self.send_urscript(command)

    def stop_motion(self, acceleration: float = 1.5) -> Tuple[bool, str]:
        ok_l, _ = self.send_urscript(f"stopl({acceleration})")
        ok_j, _ = self.send_urscript(f"stopj({acceleration})")
        return ok_l or ok_j, "Movimiento detenido."

    def set_magnet(self, enabled: bool) -> Tuple[bool, str]:
        command = f"set_standard_digital_out({self.config.magnet_do_pin}, {'True' if enabled else 'False'})"
        ok, message = self.send_urscript(command)
        if ok:
            self.state.set_magnet_enabled(enabled)
            message = "Electroiman activado." if enabled else "Electroiman desactivado."
        return ok, message

    def go_home(self) -> Tuple[bool, str]:
        return self.move_joints_deg(list(self.config.home_joints_deg))

    def greet(self) -> Tuple[bool, str]:
        sequence = build_greet_sequence(self.config.greet_sequence_deg)
        ok_all = True
        for step in sequence:
            ok, _ = self.move_joints_deg(step)
            ok_all = ok_all and ok
            time.sleep(1.2)
        self.go_home()
        return ok_all, "Secuencia de saludo ejecutada."

    def disconnect(self) -> None:
        with self._lock:
            self.script_client.close()
            self.state_client.close()
            self.dashboard_client.close()
