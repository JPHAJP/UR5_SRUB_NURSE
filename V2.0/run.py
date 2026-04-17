from app import create_app, socketio

app = create_app()


if __name__ == "__main__":
    config = app.extensions["silvia_config"]
    socketio.run(
        app,
        host=config.host,
        port=config.port,
        debug=config.debug,
        allow_unsafe_werkzeug=True,
    )
