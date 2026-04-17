from __future__ import annotations

import logging
import socket
from typing import Optional

logger = logging.getLogger(__name__)


class URScriptClient:
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
            logger.warning("No se pudo conectar URScriptClient: %s", error)
            self._socket = None
            return False

    def send_command(self, command: str) -> bool:
        if not self.connect():
            return False
        assert self._socket is not None
        try:
            if not command.endswith("\n"):
                command = command + "\n"
            self._socket.sendall(command.encode("utf-8"))
            logger.info("URScript enviado: %s", command.strip())
            return True
        except OSError as error:
            logger.warning("Error enviando URScript: %s", error)
            self.close()
            return False

    def close(self) -> None:
        if self._socket:
            try:
                self._socket.close()
            except OSError:
                pass
        self._socket = None
