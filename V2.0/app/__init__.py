from __future__ import annotations

import atexit
import logging
from typing import Any, Dict

from .config import AppConfig
from .runtime_env import bootstrap_runtime_environment

try:  # pragma: no cover - dependency fallback for local test environments
    from flask import Flask
    from flask_socketio import SocketIO
except ImportError:  # pragma: no cover - dependency fallback
    Flask = object  # type: ignore[assignment]

    class SocketIO:  # type: ignore[override]
        def __init__(self, *args, **kwargs) -> None:
            pass

        def init_app(self, *args, **kwargs) -> None:
            raise RuntimeError("Flask-SocketIO no esta instalado.")

        def emit(self, *args, **kwargs) -> None:
            pass

        def start_background_task(self, *args, **kwargs) -> None:
            pass

        def run(self, *args, **kwargs) -> None:
            raise RuntimeError("Flask-SocketIO no esta instalado.")

socketio = SocketIO(async_mode="threading", cors_allowed_origins="*")


def create_app() -> Flask:
    bootstrap_runtime_environment()

    from .routes_api import api_blueprint
    from .routes_realtime import realtime_blueprint
    from .routes_web import web_blueprint
    from .robot.robot_gateway import RobotGateway
    from .services.command_router import CommandRouter
    from .services.conversation_service import ConversationService
    from .services.safety_service import SafetyService
    from .services.tracking_service import TrackingService
    from .services.vision_service import VisionService
    from .services.voice_service import VoiceService
    from .sockets import register_socketio
    from .state import AppState

    config = AppConfig.load()
    _configure_logging(config)

    app = Flask(
        __name__,
        template_folder="templates",
        static_folder="static",
    )
    app.config["SECRET_KEY"] = config.secret_key

    state = AppState()
    gateway = RobotGateway(config, state)
    safety = SafetyService(config, state, gateway)
    vision = VisionService(config, state)
    tracker = TrackingService(config, state, vision, gateway, safety)
    router = CommandRouter(config)
    conversation = ConversationService(config, state, router)
    voice = VoiceService(config, state, conversation)

    services: Dict[str, Any] = {
        "state": state,
        "gateway": gateway,
        "safety": safety,
        "vision": vision,
        "tracker": tracker,
        "router": router,
        "conversation": conversation,
        "voice": voice,
    }

    def dispatch_action(action: str, payload: Dict[str, Any] | None = None, source: str = "api") -> Dict[str, Any]:
        payload = payload or {}
        ok = False
        message = "Accion no reconocida."

        if action == "set_mode":
            ok, message = tracker.set_mode(payload.get("mode", "idle"))
        elif action == "pick_object":
            ok, message = tracker.request_pick(
                label=payload.get("label"),
                deliver_to_hand=bool(payload.get("deliver_to_hand", False)),
            )
        elif action == "greet":
            tracker.set_mode("idle")
            ok, message = gateway.greet()
        elif action == "go_home":
            tracker.set_mode("idle")
            ok, message = gateway.go_home()
        elif action == "cancel":
            ok, message = tracker.cancel()
        elif action == "release_object":
            ok, message = gateway.set_magnet(False)
        elif action == "magnet_on":
            ok, message = gateway.set_magnet(True)
        elif action == "magnet_off":
            ok, message = gateway.set_magnet(False)
        elif action == "new_conversation":
            state.reset_conversation()
            state.clear_voice_session()
            ok, message = True, "He iniciado una nueva conversacion."
        elif action == "safety_reset":
            ok, message = safety.manual_reset()

        event_kind = "robot_log" if ok else "system_event"
        state.add_event(event_kind, message, {"action": action, "source": source, "ok": ok})
        socketio.emit(
            event_kind,
            {
                "message": message,
                "action": action,
                "source": source,
                "ok": ok,
            },
        )

        if not ok and safety.is_locked():
            socketio.emit("safety_fault", {"message": state.safety_message})

        return {"ok": ok, "message": message, "action": action}

    services["dispatch_action"] = dispatch_action
    conversation.set_action_handler(dispatch_action)

    app.extensions["silvia_config"] = config
    app.extensions["silvia_services"] = services

    app.register_blueprint(web_blueprint)
    app.register_blueprint(api_blueprint, url_prefix="/api")
    app.register_blueprint(realtime_blueprint, url_prefix="/api")

    socketio.init_app(app)
    register_socketio(app, socketio)

    vision.start()
    tracker.start()
    safety.start()

    @atexit.register
    def _shutdown_services() -> None:
        tracker.shutdown()
        safety.stop()
        vision.stop()
        gateway.disconnect()

    return app


def _configure_logging(config: AppConfig) -> None:
    config.log_dir.mkdir(parents=True, exist_ok=True)
    logging.basicConfig(
        level=logging.DEBUG if config.debug else logging.INFO,
        format="%(asctime)s [%(levelname)s] %(name)s: %(message)s",
    )
