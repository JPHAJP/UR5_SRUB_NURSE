import os

os.environ["VISION_DETECTOR_MODE"] = "hand_only"
os.environ.setdefault("HAND_FOLLOW_Z_OFFSET_MM", "200")
os.environ["VISION_INFERENCE_FPS"] = os.getenv("HAND_DEMO_INFERENCE_FPS", "5")

from app import create_app, socketio

app = create_app()


if __name__ == "__main__":
    config = app.extensions["silvia_config"]
    ssl_context = config.ssl_context()
    socketio.run(
        app,
        host=config.host,
        port=config.port,
        debug=config.debug,
        allow_unsafe_werkzeug=True,
        ssl_context=ssl_context,
    )
