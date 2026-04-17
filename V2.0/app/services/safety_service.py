from __future__ import annotations

import logging
import threading
import time
from typing import Dict, Tuple

from ..calibration.transforms import workspace_contains

logger = logging.getLogger(__name__)


class SafetyService:
    def __init__(self, config, state, gateway) -> None:
        self.config = config
        self.state = state
        self.gateway = gateway
        self._running = False
        self._thread: threading.Thread | None = None

    def start(self) -> None:
        if self._running:
            return
        self._running = True
        self._thread = threading.Thread(target=self._monitor_loop, daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._running = False
        if self._thread and self._thread.is_alive():
            self._thread.join(timeout=1.0)

    def is_locked(self) -> bool:
        return self.state.snapshot()["safety_locked"]

    def manual_reset(self) -> Tuple[bool, str]:
        status = self.gateway.refresh_status()
        if status.get("connected") and not self._is_status_safe(status):
            return False, "No puedo liberar el bloqueo mientras el robot siga en fault."
        self.state.unlock_safety("Bloqueo manual liberado.")
        return True, "Bloqueo manual liberado."

    def evaluate_target(self, mode: str, point_xyz_mm) -> Tuple[bool, str]:
        if self.is_locked():
            return False, self.state.snapshot()["safety_message"]

        status = self.gateway.refresh_status()
        if status.get("connected") and not self._is_status_safe(status):
            self.state.lock_safety("Estado inseguro detectado en el robot.", status)
            return False, self.state.snapshot()["safety_message"]

        workspace = self.config.hand_workspace if mode == "hand_follow" else self.config.object_workspace
        if not workspace_contains(workspace, point_xyz_mm):
            return False, "El objetivo salio del workspace seguro."

        return True, "Objetivo valido."

    def _monitor_loop(self) -> None:
        while self._running:
            status = self.gateway.refresh_status()
            if status.get("connected") and not self._is_status_safe(status):
                self.state.lock_safety("Falla de seguridad detectada. Se requiere revision manual.", status)
            time.sleep(self.config.safety_poll_ms / 1000.0)

    def _is_status_safe(self, status: Dict[str, object]) -> bool:
        safety_status = str(status.get("safety_status", "")).upper()
        remote_control = str(status.get("remote_control", "")).lower()

        if "NORMAL" not in safety_status and "REDUCED" not in safety_status and "UNKNOWN" not in safety_status:
            logger.warning("Safety status no seguro: %s", safety_status)
            return False
        if status.get("connected") and "true" not in remote_control and "remote" not in remote_control:
            logger.warning("Remote control no habilitado: %s", remote_control)
            return False
        return True
