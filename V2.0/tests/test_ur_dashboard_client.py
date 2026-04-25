import os
import sys

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))

from app.robot import ur_dashboard_client
from app.robot.ur_dashboard_client import URDashboardClient


class FakeSocket:
    def __init__(self, response=b"Robotmode: RUNNING\n"):
        self.response = response
        self.sent = []

    def sendall(self, payload):
        self.sent.append(payload)

    def recv(self, _size):
        return self.response


def test_dashboard_command_logging_is_quiet_by_default(monkeypatch):
    client = URDashboardClient("127.0.0.1", 29999)
    client._socket = FakeSocket()
    monkeypatch.setattr(client, "connect", lambda: True)
    log_calls = []
    monkeypatch.setattr(ur_dashboard_client.logger, "info", lambda *args, **kwargs: log_calls.append(args))

    response = client.send_command("robotmode")

    assert response == "Robotmode: RUNNING"
    assert log_calls == []


def test_dashboard_command_logging_can_be_enabled(monkeypatch):
    client = URDashboardClient("127.0.0.1", 29999, log_commands=True)
    client._socket = FakeSocket()
    monkeypatch.setattr(client, "connect", lambda: True)
    log_calls = []
    monkeypatch.setattr(ur_dashboard_client.logger, "info", lambda *args, **kwargs: log_calls.append(args))

    response = client.send_command("robotmode")

    assert response == "Robotmode: RUNNING"
    assert log_calls
