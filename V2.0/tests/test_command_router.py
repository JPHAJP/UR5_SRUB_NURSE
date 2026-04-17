import os
import sys

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))

from app.config import AppConfig
from app.services.command_router import CommandRouter


def test_route_greet():
    router = CommandRouter(AppConfig.load())
    decision = router.route("Silvia, por favor saluda al equipo")
    assert decision.action == "greet"


def test_route_hand_mode():
    router = CommandRouter(AppConfig.load())
    decision = router.route("Activa el modo seguir mano")
    assert decision.action == "set_mode"
    assert decision.payload["mode"] == "hand_follow"


def test_route_pick_object():
    router = CommandRouter(AppConfig.load())
    decision = router.route("Recoge el bisturi de la mesa")
    assert decision.action == "pick_object"
    assert decision.payload["label"] == "Bisturi"


def test_route_new_conversation():
    router = CommandRouter(AppConfig.load())
    decision = router.route("Nueva conversacion")
    assert decision.action == "new_conversation"
