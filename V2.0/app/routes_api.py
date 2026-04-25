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
    services = _services()
    result = services["conversation"].handle_user_text(text, source="text")
    result["audio_base64"] = services["voice"].speech_base64(result.get("reply", ""))
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


@api_blueprint.post("/voice/transcribe-test")
def transcribe_voice_test():
    if "audio" not in request.files:
        return jsonify({"ok": False, "message": "Falta archivo de audio."}), 400
    result = _services()["voice"].transcribe_browser_audio(request.files["audio"])
    return jsonify(result), 200 if result.get("ok", False) else 400


@api_blueprint.post("/voice/push-to-talk")
def process_push_to_talk():
    if "audio" not in request.files:
        return jsonify({"ok": False, "message": "Falta archivo de audio."}), 400
    result = _services()["voice"].process_push_to_talk_audio(request.files["audio"])
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


@api_blueprint.get("/vision/depth")
def get_depth_diagnostics():
    return jsonify(_services()["vision"].get_depth_diagnostics())


@api_blueprint.get("/calibration/depth")
def get_depth_calibration():
    services = _services()
    robot_pose_mm = services["gateway"].current_pose_mm()
    return jsonify(services["vision"].get_depth_calibration_summary(robot_pose_mm=robot_pose_mm))


@api_blueprint.post("/calibration/depth")
def set_depth_calibration():
    payload = request.get_json(silent=True) or {}
    gain = payload.get("gain")
    offset_mm = payload.get("offset_mm")
    result = _services()["vision"].set_depth_compensation(gain=gain, offset_mm=offset_mm)
    return jsonify({"ok": True, "depth_compensation": result})


@api_blueprint.post("/calibration/depth/sample")
def capture_depth_calibration_sample():
    services = _services()
    try:
        sample = services["vision"].capture_depth_calibration_sample(services["gateway"].current_pose_mm())
    except ValueError as error:
        return jsonify({"ok": False, "message": str(error)}), 400
    return jsonify({"ok": True, "sample": sample})


@api_blueprint.post("/calibration/depth/fit")
def fit_depth_calibration():
    result = _services()["vision"].fit_depth_calibration()
    return jsonify({"ok": True, "depth_compensation": result})


@api_blueprint.post("/calibration/depth/save")
def save_depth_calibration():
    result = _services()["vision"].save_calibration_to_disk()
    return jsonify({"ok": True, "calibration": result})


@api_blueprint.get("/calibration/hand-follow")
def get_hand_follow_calibration():
    return jsonify(_services()["vision"].get_hand_follow_calibration())


@api_blueprint.post("/calibration/hand-follow")
def set_hand_follow_calibration():
    payload = request.get_json(silent=True) or {}
    try:
        result = _services()["vision"].set_hand_follow_calibration(
            plane_z_mm=payload.get("plane_z_mm"),
            save=bool(payload.get("save", False)),
        )
    except ValueError as error:
        return jsonify({"ok": False, "message": str(error)}), 400
    return jsonify({"ok": True, "hand_follow": result})


@api_blueprint.post("/calibration/camera-transform")
def set_camera_transform():
    payload = request.get_json(silent=True) or {}
    translation_mm = payload.get("translation_mm", [0.0, 0.0, 0.0])
    rotation_rpy_deg = payload.get("rotation_rpy_deg", [0.0, 0.0, 0.0])
    result = _services()["vision"].set_camera_transform(translation_mm, rotation_rpy_deg)
    return jsonify({"ok": True, "camera_to_robot": result, "tcp_to_camera": result})
