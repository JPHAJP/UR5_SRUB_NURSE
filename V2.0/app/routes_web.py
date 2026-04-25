from __future__ import annotations

from flask import Blueprint, Response, current_app, render_template

web_blueprint = Blueprint("web", __name__)


@web_blueprint.get("/")
def index():
    config = current_app.extensions["silvia_config"]
    return render_template("index.html", settings=config.public_settings())


@web_blueprint.get("/hand-demo")
def hand_demo():
    config = current_app.extensions["silvia_config"]
    return render_template("hand_demo.html", settings=config.public_settings())


@web_blueprint.get("/video_feed")
def video_feed():
    vision = current_app.extensions["silvia_services"]["vision"]
    return Response(
        vision.mjpeg_stream("annotated"),
        mimetype="multipart/x-mixed-replace; boundary=frame",
    )


@web_blueprint.get("/video_feed/rgb")
def video_feed_rgb():
    vision = current_app.extensions["silvia_services"]["vision"]
    return Response(
        vision.mjpeg_stream("rgb"),
        mimetype="multipart/x-mixed-replace; boundary=frame",
    )


@web_blueprint.get("/video_feed/depth")
def video_feed_depth():
    vision = current_app.extensions["silvia_services"]["vision"]
    return Response(
        vision.mjpeg_stream("depth"),
        mimetype="multipart/x-mixed-replace; boundary=frame",
    )
