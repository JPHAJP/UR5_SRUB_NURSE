from __future__ import annotations

import logging
import math
import threading
from typing import Dict, List, Tuple

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
        self.dashboard_client = URDashboardClient(
            config.ur5_host,
            config.ur5_dashboard_port,
            log_commands=config.ur5_dashboard_log_commands,
        )
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

    @staticmethod
    def _build_movej_command_deg(
        joints_deg: List[float],
        speed: float,
        acceleration: float,
        blend_radius: float = 0.0,
    ) -> str:
        joints_rad = [math.radians(value) for value in joints_deg]
        joint_values = ", ".join(f"{value:.5f}" for value in joints_rad)
        return (
            f"movej([{joint_values}], a = {acceleration}, v = {speed}, "
            f"t = 0, r = {max(0.0, blend_radius)})"
        )

    @classmethod
    def _build_joint_path_program_deg(
        cls,
        path_deg: List[List[float]],
        speed: float,
        acceleration: float,
        blend_radius: float = 0.0,
        program_name: str = "joint_path",
    ) -> str:
        if not path_deg:
            return ""

        commands = []
        last_index = len(path_deg) - 1
        for index, joints_deg in enumerate(path_deg):
            step_blend = blend_radius if index < last_index else 0.0
            commands.append(
                f"  {cls._build_movej_command_deg(joints_deg, speed, acceleration, step_blend)}"
            )

        return "\n".join(
            [
                f"def {program_name}():",
                *commands,
                "end",
                f"{program_name}()",
            ]
        )

    def move_joints_deg(
        self,
        joints_deg: List[float],
        speed: float = 1.5,
        acceleration: float = 2.5,
        blend_radius: float = 0.0,
    ) -> Tuple[bool, str]:
        command = self._build_movej_command_deg(joints_deg, speed, acceleration, blend_radius)
        return self.send_urscript(command)

    def move_linear_mm(
        self,
        pose_xyz_mm: List[float],
        orientation: List[float] | None = None,
        speed: float = 0.25,
        acceleration: float = 1.2,
    ) -> Tuple[bool, str]:
        current = self.current_pose_mm()
        orientation = orientation or current[3:] or DEFAULT_ORIENTATION
        command = self._build_movel_command_mm(
            pose_xyz_mm,
            orientation=orientation,
            speed=speed,
            acceleration=acceleration,
        )
        return self.send_urscript(command)

    @staticmethod
    def _build_movel_command_mm(
        pose_xyz_mm: List[float],
        orientation: List[float],
        speed: float,
        acceleration: float,
        blend_radius: float = 0.0,
    ) -> str:
        x_m, y_m, z_m = [float(value) / 1000.0 for value in pose_xyz_mm[:3]]
        rx, ry, rz = [float(value) for value in orientation[:3]]
        return (
            f"movel(p[{x_m:.6f},{y_m:.6f},{z_m:.6f},{rx:.6f},{ry:.6f},{rz:.6f}], "
            f"a = {acceleration}, v = {speed}, t = 0, r = {max(0.0, blend_radius)})"
        )

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

    def execute_pick_sequence_mm(
        self,
        waypoints: Dict[str, List[float]],
        orientation: List[float] | None = None,
        speed: float = 0.16,
        acceleration: float = 0.45,
        blend_radius: float = 0.012,
        settle_s: float = 0.15,
        magnet_settle_s: float = 0.25,
        return_home: bool = False,
        home_joints_deg: List[float] | None = None,
        home_speed: float = 1.5,
        home_acceleration: float = 2.5,
    ) -> Tuple[bool, str]:
        current = self.current_pose_mm()
        orientation = orientation or current[3:] or DEFAULT_ORIENTATION
        commands = [
            "def pick_sequence():",
            f"  {self._build_movel_command_mm(waypoints['approach'], orientation, speed, acceleration, blend_radius)}",
            f"  {self._build_movel_command_mm(waypoints['pre_pick'], orientation, speed, acceleration, blend_radius)}",
            f"  {self._build_movel_command_mm(waypoints['pick'], orientation, speed, acceleration, 0.0)}",
        ]

        if settle_s > 0.0:
            commands.append(f"  sleep({max(0.0, float(settle_s)):.3f})")
        commands.append(f"  set_standard_digital_out({self.config.magnet_do_pin}, True)")
        if magnet_settle_s > 0.0:
            commands.append(f"  sleep({max(0.0, float(magnet_settle_s)):.3f})")
        commands.append(f"  {self._build_movel_command_mm(waypoints['retreat'], orientation, speed, acceleration, 0.0)}")

        if return_home and home_joints_deg:
            commands.append(
                f"  {self._build_movej_command_deg(home_joints_deg, home_speed, home_acceleration, 0.0)}"
            )

        commands.extend(["end", "pick_sequence()"])
        return self.send_urscript("\n".join(commands))

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
        if not sequence:
            return False, "No hay una secuencia de saludo configurada."

        full_path = [list(step) for step in sequence]
        home_joints = list(self.config.home_joints_deg)
        if full_path[-1] != home_joints:
            full_path.append(home_joints)

        command = self._build_joint_path_program_deg(
            path_deg=full_path,
            speed=self.config.greet_speed_rad_s,
            acceleration=self.config.greet_acceleration_rad_s2,
            blend_radius=self.config.greet_blend_radius_m,
            program_name="greet_sequence",
        )
        return self.send_urscript(command)

    def disconnect(self) -> None:
        with self._lock:
            self.script_client.close()
            self.state_client.close()
            self.dashboard_client.close()
