import os
import sys

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))

from app.config import AppConfig
from app.services.safety_service import SafetyService
from app.state import AppState


class FakeGateway:
    def __init__(self, status):
        self._status = status

    def refresh_status(self):
        return self._status


def test_workspace_validation_rejects_out_of_bounds():
    config = AppConfig.load()
    state = AppState()
    gateway = FakeGateway(
        {
            "connected": True,
            "safety_status": "NORMAL",
            "remote_control": "true",
        }
    )
    safety = SafetyService(config, state, gateway)
    ok, message = safety.evaluate_target("object_pick", [9999.0, 0.0, 120.0])
    assert not ok
    assert "workspace" in message.lower()


def test_manual_reset_refuses_if_fault_persists():
    config = AppConfig.load()
    state = AppState()
    state.lock_safety("Fault.")
    gateway = FakeGateway(
        {
            "connected": True,
            "safety_status": "PROTECTIVE_STOP",
            "remote_control": "false",
        }
    )
    safety = SafetyService(config, state, gateway)
    ok, _ = safety.manual_reset()
    assert not ok
