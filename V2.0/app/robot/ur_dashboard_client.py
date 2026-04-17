from __future__ import annotations

import logging
import socket
from typing import Dict, Optional

logger = logging.getLogger(__name__)


class URDashboardClient:
    def __init__(self, host: str, port: int, timeout: float = 1.5) -> None:
        self.host = host
        self.port = port
        self.timeout = timeout
        self._socket: Optional[socket.socket] = None

    def connect(self) -> bool:
        if self._socket:
            return True
        try:
            self._socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self._socket.settimeout(self.timeout)
            self._socket.connect((self.host, self.port))
            try:
                self._socket.recv(1024)
            except OSError:
                pass
            return True
        except OSError as error:
            logger.warning("No se pudo conectar URDashboardClient: %s", error)
            self._socket = None
            return False

    def send_command(self, command: str) -> str:
        if not self.connect():
            return ""

        assert self._socket is not None
        try:
            self._socket.sendall((command.strip() + "\n").encode("utf-8"))
            response = self._socket.recv(1024).decode("utf-8", errors="ignore").strip()
            logger.info("Dashboard %s -> %s", command, response)
            return response
        except OSError as error:
            logger.warning("Error en Dashboard command '%s': %s", command, error)
            self.close()
            return ""

    def get_status(self) -> Dict[str, str]:
        return {
            "robot_mode": self.send_command("robotmode"),
            "program_state": self.send_command("programState"),
            "safety_status": self.send_command("safetystatus"),
            "remote_control": self.send_command("is in remote control"),
        }

    def unlock_protective_stop(self) -> str:
        return self.send_command("unlock protective stop")

    def close_safety_popup(self) -> str:
        return self.send_command("close safety popup")

    def close(self) -> None:
        if self._socket:
            try:
                self._socket.close()
            except OSError:
                pass
        self._socket = None
