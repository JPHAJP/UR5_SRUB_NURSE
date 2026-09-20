from __future__ import annotations

import argparse
import json
import math
import sys
import time
from pathlib import Path
from typing import Any, Dict, Iterable, List

import requests


INVALID_TRACKING_STATES = {"hand_not_visible", "target_stale", "camera_stale"}
ACTIVE_AXIS_TO_INDEX = {"x": 0, "y": 1}
DEFAULT_DURATION_S = 20.0
DEFAULT_POLL_HZ = 10.0
DEFAULT_MIN_DELTA_MM = 5.0
DEFAULT_MIN_REVERSALS = 3
DEFAULT_MIN_TOGGLES = 3
DEFAULT_INSUFFICIENT_RATIO = 0.60


def default_output_dir() -> Path:
    return Path(__file__).resolve().parents[1] / "data" / "logs"


def load_jsonl(path: Path) -> list[dict[str, Any]]:
    return [
        json.loads(line)
        for line in path.read_text(encoding="utf-8").splitlines()
        if line.strip()
    ]


def write_jsonl(path: Path, rows: Iterable[dict[str, Any]]) -> None:
    with path.open("w", encoding="utf-8") as handle:
        for row in rows:
            handle.write(json.dumps(row, ensure_ascii=True) + "\n")


def _timestamp_label() -> str:
    return f"{time.strftime('%Y%m%dT%H%M%SZ', time.gmtime())}_{time.time_ns() % 1_000_000_000:09d}"


def _vector3(values: Any) -> list[float] | None:
    if not isinstance(values, list) or len(values) < 3:
        return None
    try:
        return [float(values[0]), float(values[1]), float(values[2])]
    except (TypeError, ValueError):
        return None


def _distance_xyz_mm(point_a: list[float] | None, point_b: list[float] | None) -> float | None:
    if point_a is None or point_b is None:
        return None
    return math.sqrt(
        sum((float(value_a) - float(value_b)) ** 2 for value_a, value_b in zip(point_a[:3], point_b[:3]))
    )


def _distance_xy_mm(point_a: list[float] | None, point_b: list[float] | None) -> float | None:
    if point_a is None or point_b is None:
        return None
    return math.sqrt(
        (float(point_a[0]) - float(point_b[0])) ** 2
        + (float(point_a[1]) - float(point_b[1])) ** 2
    )


def _normalize_bool(value: Any) -> bool | None:
    if isinstance(value, bool):
        return value
    return None


def normalize_tracking_rows(rows: list[dict[str, Any]]) -> list[dict[str, Any]]:
    normalized = []
    for row in rows:
        normalized.append(
            {
                "timestamp_s": row.get("timestamp_s"),
                "tcp_pose_mm": row.get("tcp_pose_mm"),
                "hand_world_mm": row.get("hand_world_mm"),
                "target_xyz_mm": row.get("target_xyz_mm"),
                "distance_tcp_to_hand_mm": row.get("distance_tcp_to_hand_mm"),
                "approaching_hand": _normalize_bool(row.get("approaching_hand")),
                "active_axis": row.get("active_axis"),
                "command_state": row.get("command_state", ""),
            }
        )
    return normalized


def normalize_state_capture_rows(rows: list[dict[str, Any]]) -> list[dict[str, Any]]:
    normalized = []
    for row in rows:
        state = row.get("state", {})
        tracking = state.get("tracking_status", {})
        normalized.append(
            {
                "timestamp_s": row.get("timestamp_s"),
                "tcp_pose_mm": (state.get("robot_status") or {}).get("current_pose_mm"),
                "hand_world_mm": tracking.get("hand_world_mm") or (state.get("hand_target") or {}).get("world_mm"),
                "target_xyz_mm": tracking.get("target_xyz_mm"),
                "distance_tcp_to_hand_mm": tracking.get("distance_tcp_to_hand_mm"),
                "approaching_hand": _normalize_bool(tracking.get("approaching_hand")),
                "active_axis": tracking.get("active_axis"),
                "command_state": tracking.get("stop_reason", ""),
            }
        )
    return normalized


def analyze_normalized_samples(
    samples: list[dict[str, Any]],
    insufficient_ratio_threshold: float = DEFAULT_INSUFFICIENT_RATIO,
    min_direction_delta_mm: float = DEFAULT_MIN_DELTA_MM,
    min_reversals: int = DEFAULT_MIN_REVERSALS,
    min_toggles: int = DEFAULT_MIN_TOGGLES,
) -> dict[str, Any]:
    total_samples = len(samples)
    invalid_state_count = sum(1 for sample in samples if sample.get("command_state") in INVALID_TRACKING_STATES)

    valid_samples: list[dict[str, Any]] = []
    for sample in samples:
        tcp_pose_mm = _vector3(sample.get("tcp_pose_mm"))
        hand_world_mm = _vector3(sample.get("hand_world_mm"))
        target_xyz_mm = _vector3(sample.get("target_xyz_mm"))
        active_axis = str(sample.get("active_axis") or "").strip().lower()
        axis_index = ACTIVE_AXIS_TO_INDEX.get(active_axis)
        axis_metric_mm = None
        if active_axis == "xy":
            axis_metric_mm = _distance_xy_mm(tcp_pose_mm, target_xyz_mm)
        elif axis_index is not None and tcp_pose_mm is not None:
            axis_metric_mm = tcp_pose_mm[axis_index]

        if tcp_pose_mm is None or hand_world_mm is None or target_xyz_mm is None or axis_metric_mm is None:
            continue

        distance_tcp_to_hand_mm = sample.get("distance_tcp_to_hand_mm")
        try:
            distance_tcp_to_hand_mm = (
                float(distance_tcp_to_hand_mm)
                if distance_tcp_to_hand_mm is not None
                else _distance_xyz_mm(tcp_pose_mm, hand_world_mm)
            )
        except (TypeError, ValueError):
            distance_tcp_to_hand_mm = _distance_xyz_mm(tcp_pose_mm, hand_world_mm)

        valid_samples.append(
            {
                "timestamp_s": sample.get("timestamp_s"),
                "tcp_pose_mm": tcp_pose_mm,
                "hand_world_mm": hand_world_mm,
                "target_xyz_mm": target_xyz_mm,
                "distance_tcp_to_hand_mm": distance_tcp_to_hand_mm,
                "approaching_hand": _normalize_bool(sample.get("approaching_hand")),
                "active_axis": active_axis,
                "tcp_active_axis_mm": axis_metric_mm,
            }
        )

    valid_sample_ratio = (len(valid_samples) / total_samples) if total_samples else 0.0
    invalid_state_ratio = (invalid_state_count / total_samples) if total_samples else 0.0
    insufficient_tracking_data = total_samples > 0 and invalid_state_ratio > insufficient_ratio_threshold

    direction_reversals = 0
    previous_direction_sign = 0
    previous_tcp_axis_mm: float | None = None
    approach_toggles = 0
    previous_approach: bool | None = None

    distances: list[float] = []
    for sample in valid_samples:
        distance = sample.get("distance_tcp_to_hand_mm")
        if distance is not None:
            distances.append(float(distance))

        current_axis_mm = float(sample["tcp_active_axis_mm"])
        if previous_tcp_axis_mm is not None:
            delta_axis_mm = current_axis_mm - previous_tcp_axis_mm
            if abs(delta_axis_mm) >= min_direction_delta_mm:
                current_sign = 1 if delta_axis_mm > 0.0 else -1
                if previous_direction_sign and current_sign != previous_direction_sign:
                    direction_reversals += 1
                previous_direction_sign = current_sign
        previous_tcp_axis_mm = current_axis_mm

        approach = sample.get("approaching_hand")
        if approach is None and len(distances) >= 2:
            approach = distances[-1] < distances[-2]
        if approach is not None:
            if previous_approach is not None and approach != previous_approach:
                approach_toggles += 1
            previous_approach = approach

    distance_start_mm = distances[0] if distances else None
    distance_end_mm = distances[-1] if distances else None
    distance_min_mm = min(distances) if distances else None
    distance_max_mm = max(distances) if distances else None
    net_distance_change_mm = (
        distance_end_mm - distance_start_mm
        if distance_start_mm is not None and distance_end_mm is not None
        else None
    )

    oscillation_detected = (
        not insufficient_tracking_data
        and len(valid_samples) >= 3
        and (
            direction_reversals >= min_reversals
            or approach_toggles >= min_toggles
        )
    )

    diagnosis = "ok"
    if insufficient_tracking_data:
        diagnosis = "insufficient_tracking_data"
    elif oscillation_detected:
        diagnosis = "oscillation_detected"

    return {
        "diagnosis": diagnosis,
        "oscillation_detected": oscillation_detected,
        "insufficient_tracking_data": insufficient_tracking_data,
        "samples_total": total_samples,
        "samples_valid": len(valid_samples),
        "valid_sample_ratio": round(valid_sample_ratio, 4),
        "invalid_state_ratio": round(invalid_state_ratio, 4),
        "direction_reversals": direction_reversals,
        "approach_toggles": approach_toggles,
        "distance_start_mm": round(distance_start_mm, 3) if distance_start_mm is not None else None,
        "distance_end_mm": round(distance_end_mm, 3) if distance_end_mm is not None else None,
        "distance_min_mm": round(distance_min_mm, 3) if distance_min_mm is not None else None,
        "distance_max_mm": round(distance_max_mm, 3) if distance_max_mm is not None else None,
        "net_distance_change_mm": round(net_distance_change_mm, 3) if net_distance_change_mm is not None else None,
    }


def _write_report(output_dir: Path, report: dict[str, Any]) -> Path:
    output_dir.mkdir(parents=True, exist_ok=True)
    report_path = output_dir / f"hand_follow_diagnostic_report_{_timestamp_label()}.json"
    report_path.write_text(json.dumps(report, ensure_ascii=True, indent=2) + "\n", encoding="utf-8")
    return report_path


def _print_report_summary(report: dict[str, Any]) -> None:
    print("Diagnostico hand_follow")
    print(f"- Diagnostico: {report.get('diagnosis')}")
    print(f"- Oscilacion detectada: {report.get('oscillation_detected')}")
    print(f"- Muestras totales: {report.get('samples_total')}")
    print(f"- Muestras validas: {report.get('samples_valid')}")
    print(f"- Ratio valido: {report.get('valid_sample_ratio')}")
    print(f"- Inversiones de direccion: {report.get('direction_reversals')}")
    print(f"- Alternancias de acercamiento: {report.get('approach_toggles')}")
    print(f"- Distancia inicial: {report.get('distance_start_mm')}")
    print(f"- Distancia final: {report.get('distance_end_mm')}")
    print(f"- Cambio neto: {report.get('net_distance_change_mm')}")
    if report.get("tracking_log_path"):
        print(f"- Tracking log: {report.get('tracking_log_path')}")
    if report.get("state_capture_path"):
        print(f"- State capture: {report.get('state_capture_path')}")
    if report.get("report_path"):
        print(f"- Reporte: {report.get('report_path')}")


def run_analyze_mode(input_path: Path, output_dir: Path | None = None) -> dict[str, Any]:
    rows = load_jsonl(input_path)
    if rows and "state" in rows[0]:
        normalized = normalize_state_capture_rows(rows)
        input_kind = "state_capture_jsonl"
    else:
        normalized = normalize_tracking_rows(rows)
        input_kind = "tracking_jsonl"

    report = analyze_normalized_samples(normalized)
    report.update(
        {
            "mode": "analyze",
            "input_kind": input_kind,
            "input_path": str(input_path),
        }
    )
    report_path = _write_report(output_dir or input_path.parent, report)
    report["report_path"] = str(report_path)
    report_path.write_text(json.dumps(report, ensure_ascii=True, indent=2) + "\n", encoding="utf-8")
    _print_report_summary(report)
    return report


def _http_get_json(session: requests.Session, url: str) -> dict[str, Any]:
    response = session.get(url, timeout=10)
    response.raise_for_status()
    return response.json()


def _http_post_json(session: requests.Session, url: str, payload: dict[str, Any]) -> dict[str, Any]:
    response = session.post(url, json=payload, timeout=10)
    response.raise_for_status()
    return response.json()


def _build_precondition_report(reason: str, output_dir: Path, extra: dict[str, Any] | None = None) -> dict[str, Any]:
    report = {
        "mode": "live",
        "diagnosis": "precondition_failed",
        "oscillation_detected": False,
        "reason": reason,
        "samples_total": 0,
        "samples_valid": 0,
        "valid_sample_ratio": 0.0,
        "direction_reversals": 0,
        "approach_toggles": 0,
        "distance_start_mm": None,
        "distance_end_mm": None,
        "distance_min_mm": None,
        "distance_max_mm": None,
        "net_distance_change_mm": None,
    }
    if extra:
        report.update(extra)
    report_path = _write_report(output_dir, report)
    report["report_path"] = str(report_path)
    report_path.write_text(json.dumps(report, ensure_ascii=True, indent=2) + "\n", encoding="utf-8")
    _print_report_summary(report)
    return report


def run_live_mode(
    base_url: str,
    duration_s: float = DEFAULT_DURATION_S,
    poll_hz: float = DEFAULT_POLL_HZ,
    output_dir: Path | None = None,
    session: requests.Session | None = None,
    sleeper=time.sleep,
    monotonic=time.monotonic,
) -> dict[str, Any]:
    output_dir = output_dir or default_output_dir()
    output_dir.mkdir(parents=True, exist_ok=True)
    base_url = base_url.rstrip("/")
    session = session or requests.Session()

    state_url = f"{base_url}/api/state"
    mode_url = f"{base_url}/api/mode"
    tool_url = f"{base_url}/api/agent/tool"

    initial_state = _http_get_json(session, state_url)
    if initial_state.get("safety_locked"):
        return _build_precondition_report(
            "safety_locked",
            output_dir,
            {
                "base_url": base_url,
                "safety_message": initial_state.get("safety_message"),
            },
        )

    go_home_response = _http_post_json(session, tool_url, {"action": "go_home"})
    if not go_home_response.get("ok", False):
        return _build_precondition_report(
            "go_home_failed",
            output_dir,
            {
                "base_url": base_url,
                "go_home_response": go_home_response,
            },
        )

    mode_response = _http_post_json(session, mode_url, {"mode": "hand_follow"})
    if not mode_response.get("ok", False):
        return _build_precondition_report(
            "hand_follow_activation_failed",
            output_dir,
            {
                "base_url": base_url,
                "mode_response": mode_response,
            },
        )

    capture_rows: list[dict[str, Any]] = []
    tracking_log_path: str | None = None
    interval_s = 1.0 / poll_hz if poll_hz > 0.0 else 0.1
    end_time = monotonic() + max(0.0, duration_s)
    try:
        while monotonic() < end_time:
            state = _http_get_json(session, state_url)
            capture_rows.append(
                {
                    "timestamp_s": round(time.time(), 6),
                    "state": state,
                }
            )
            current_tracking_log = ((state.get("tracking_status") or {}).get("distance_log_path") or "").strip()
            if current_tracking_log:
                tracking_log_path = current_tracking_log
            sleeper(interval_s)
    finally:
        _http_post_json(session, mode_url, {"mode": "idle"})

    capture_path = output_dir / f"hand_follow_state_capture_{_timestamp_label()}.jsonl"
    write_jsonl(capture_path, capture_rows)
    normalized = normalize_state_capture_rows(capture_rows)
    report = analyze_normalized_samples(normalized)
    report.update(
        {
            "mode": "live",
            "base_url": base_url,
            "duration_s": duration_s,
            "poll_hz": poll_hz,
            "tracking_log_path": tracking_log_path,
            "state_capture_path": str(capture_path),
        }
    )
    report_path = _write_report(output_dir, report)
    report["report_path"] = str(report_path)
    report_path.write_text(json.dumps(report, ensure_ascii=True, indent=2) + "\n", encoding="utf-8")
    _print_report_summary(report)
    return report


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Diagnostica oscilacion en hand_follow.")
    subparsers = parser.add_subparsers(dest="command", required=True)

    live_parser = subparsers.add_parser("live", help="Captura y diagnostica una sesion real contra la app.")
    live_parser.add_argument("--base-url", default="http://127.0.0.1:5050")
    live_parser.add_argument("--duration-s", type=float, default=DEFAULT_DURATION_S)
    live_parser.add_argument("--poll-hz", type=float, default=DEFAULT_POLL_HZ)
    live_parser.add_argument("--output-dir", type=Path, default=default_output_dir())

    analyze_parser = subparsers.add_parser("analyze", help="Analiza un JSONL existente.")
    analyze_parser.add_argument("--input", type=Path, required=True)
    analyze_parser.add_argument("--output-dir", type=Path, default=None)

    return parser.parse_args(argv)


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv)
    if args.command == "live":
        report = run_live_mode(
            base_url=args.base_url,
            duration_s=args.duration_s,
            poll_hz=args.poll_hz,
            output_dir=args.output_dir,
        )
    else:
        report = run_analyze_mode(
            input_path=args.input,
            output_dir=args.output_dir,
        )

    if report.get("diagnosis") == "precondition_failed":
        return 2
    if report.get("oscillation_detected"):
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
