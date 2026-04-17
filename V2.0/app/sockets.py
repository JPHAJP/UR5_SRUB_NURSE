from __future__ import annotations

import threading
import time

_broadcast_started = False
_broadcast_lock = threading.Lock()


def register_socketio(app, socketio) -> None:
    @socketio.on("connect")
    def on_connect():
        services = app.extensions["silvia_services"]
        services["gateway"].refresh_status()
        socketio.emit("state", services["state"].snapshot())
        _ensure_background_broadcast(app, socketio)


def _ensure_background_broadcast(app, socketio) -> None:
    global _broadcast_started

    with _broadcast_lock:
        if _broadcast_started:
            return
        _broadcast_started = True

    def _broadcast_loop():
        while True:
            with app.app_context():
                services = app.extensions["silvia_services"]
                services["gateway"].refresh_status()
                snapshot = services["state"].snapshot()
            socketio.emit("state", snapshot)
            time.sleep(1.0)

    socketio.start_background_task(_broadcast_loop)
