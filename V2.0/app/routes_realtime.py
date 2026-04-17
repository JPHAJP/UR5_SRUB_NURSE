from __future__ import annotations

from flask import Blueprint, current_app, jsonify

realtime_blueprint = Blueprint("realtime", __name__)


@realtime_blueprint.post("/realtime/session")
def realtime_session():
    result = current_app.extensions["silvia_services"]["voice"].create_realtime_session()
    return jsonify(result), 200 if result.get("ok") else 400
