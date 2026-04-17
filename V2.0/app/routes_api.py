from __future__ import annotations

from flask import Blueprint, current_app, jsonify, request

api_blueprint = Blueprint("api", __name__)


def _services():
    return current_app.extensions["silvia_services"]


@api_blueprint.get("/state")
def get_state():
    services = _services()
    services["gateway"].refresh_status()
    return jsonify(services["state"].snapshot())


@api_blueprint.post("/mode")
def set_mode():
    payload = request.get_json(silent=True) or {}
    result = _services()["dispatch_action"]("set_mode", payload, "api")
    return jsonify(result), 200 if result["ok"] else 400


@api_blueprint.post("/robot/greet")
def greet_robot():
    result = _services()["dispatch_action"]("greet", {}, "api")
    return jsonify(result), 200 if result["ok"] else 400


@api_blueprint.post("/conversation/reset")
def reset_conversation():
    result = _services()["dispatch_action"]("new_conversation", {}, "api")
    return jsonify(result), 200 if result["ok"] else 400


@api_blueprint.post("/conversation/message")
def send_message():
    payload = request.get_json(silent=True) or {}
    text = payload.get("text", "")
    result = _services()["conversation"].handle_user_text(text, source="text")
    return jsonify(result), 200 if result.get("ok") else 400


@api_blueprint.post("/agent/tool")
def execute_tool():
    payload = request.get_json(silent=True) or {}
    action = payload.get("action", "")
    result = _services()["dispatch_action"](action, payload.get("payload", payload), "agent")
    return jsonify(result), 200 if result["ok"] else 400


@api_blueprint.post("/voice/chunk")
def process_voice_chunk():
    if "audio" not in request.files:
        return jsonify({"ok": False, "message": "Falta archivo de audio."}), 400
    result = _services()["voice"].process_browser_audio(request.files["audio"])
    return jsonify(result), 200 if result.get("ok", False) else 400


@api_blueprint.post("/safety/reset")
def reset_safety():
    result = _services()["dispatch_action"]("safety_reset", {}, "api")
    return jsonify(result), 200 if result["ok"] else 400


@api_blueprint.post("/magnet")
def magnet_control():
    payload = request.get_json(silent=True) or {}
    action = "magnet_on" if bool(payload.get("enabled")) else "magnet_off"
    result = _services()["dispatch_action"](action, payload, "api")
    return jsonify(result), 200 if result["ok"] else 400
