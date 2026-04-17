from __future__ import annotations

import logging
import socket
import struct
from typing import Dict, Optional

logger = logging.getLogger(__name__)


class URStateClient:
    def __init__(self, host: str, port: int, timeout: float = 1.0) -> None:
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
            return True
        except OSError as error:
            logger.warning("No se pudo conectar URStateClient: %s", error)
            self._socket = None
            return False

    def read_state(self) -> Dict[str, object]:
        if not self.connect():
            return {"connected": False}

        assert self._socket is not None
        try:
            data = self._socket.recv(4096)
        except OSError as error:
            logger.warning("No se pudo leer estado del robot: %s", error)
            self.close()
            return {"connected": False}

        if not data or len(data) < 18:
            return {"connected": False}

        packet_length = struct.unpack("!i", data[0:4])[0]
        packet_type = struct.unpack("!b", data[4:5])[0]

        if packet_type != 16:
            return {"connected": True}

        pose = None
        joints = [0.0] * 6
        index = 0

        while index + 10 < min(packet_length, len(data)):
            try:
                message_length = struct.unpack("!i", data[5 + index : 9 + index])[0]
                message_type = struct.unpack("!b", data[9 + index : 10 + index])[0]
            except struct.error:
                break

            if message_length <= 0:
                break

            if message_type == 1 and 10 + index + 6 * 41 <= len(data):
                for joint_index in range(6):
                    start = 10 + index + joint_index * 41
                    joints[joint_index] = struct.unpack("!d", data[start : start + 8])[0]
            elif message_type == 4 and 10 + index + 48 <= len(data):
                values = []
                for offset in range(6):
                    start = 10 + index + offset * 8
                    values.append(struct.unpack("!d", data[start : start + 8])[0])
                pose = values

            index += message_length

        pose_mm = None
        if pose:
            pose_mm = [
                round(pose[0] * 1000.0, 2),
                round(pose[1] * 1000.0, 2),
                round(pose[2] * 1000.0, 2),
                pose[3],
                pose[4],
                pose[5],
            ]

        return {
            "connected": True,
            "tcp_pose_m": pose,
            "tcp_pose_mm": pose_mm,
            "joint_positions_rad": joints,
        }

    def close(self) -> None:
        if self._socket:
            try:
                self._socket.close()
            except OSError:
                pass
        self._socket = None
