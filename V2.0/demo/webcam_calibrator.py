from __future__ import annotations

import math
import os
import sys
import time
from pathlib import Path

import cv2

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))

from app.calibration.loader import DEFAULT_CALIBRATION, save_calibration
from app.calibration.transforms import compute_homography
from app.config import AppConfig
from app.robot.motion_profiles import estimate_joint_move_duration_deg
from app.robot.robot_gateway import RobotGateway
from app.state import AppState


COMMON_RESOLUTIONS = [
    (640, 480),
    (800, 600),
    (960, 540),
    (1280, 720),
    (1920, 1080),
]

clicked_points = []
CALIBRATION_JOINTS_DEG = [-51.12, -72.5, -71.58, -125.9, 90.91, -95.26]
CALIBRATION_MOVE_SPEED_RAD_S = 1.2
CALIBRATION_MOVE_ACCELERATION_RAD_S2 = 2.0
CALIBRATION_JOINT_TOLERANCE_DEG = 1.0
CALIBRATION_MOVE_TIMEOUT_PAD_S = 8.0


def on_mouse(event, x, y, *_args):
    if event == cv2.EVENT_LBUTTONDOWN and len(clicked_points) < 4:
        clicked_points.append([float(x), float(y)])


def detect_supported_resolutions(camera_index: int):
    capture = cv2.VideoCapture(camera_index)
    supported = []
    for width, height in COMMON_RESOLUTIONS:
        capture.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        capture.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        actual_width = int(capture.get(cv2.CAP_PROP_FRAME_WIDTH))
        actual_height = int(capture.get(cv2.CAP_PROP_FRAME_HEIGHT))
        if (actual_width, actual_height) == (width, height):
            supported.append((width, height))
    capture.release()
    return supported


def prepare_robot_plane_z(config: AppConfig) -> float:
    gateway = RobotGateway(config, AppState())
    try:
        status = gateway.refresh_status()
        if not bool(status.get("connected")):
            raise RuntimeError("No se pudo conectar con el UR5 para la calibracion.")

        start_joints = list(status.get("joint_positions_deg") or config.home_joints_deg)
        ok, message = gateway.move_joints_deg(
            CALIBRATION_JOINTS_DEG,
            speed=CALIBRATION_MOVE_SPEED_RAD_S,
            acceleration=CALIBRATION_MOVE_ACCELERATION_RAD_S2,
        )
        if not ok:
            raise RuntimeError(f"Fallo al enviar movej de calibracion: {message}")

        timeout_s = estimate_joint_move_duration_deg(
            start_joints_deg=start_joints,
            target_joints_deg=CALIBRATION_JOINTS_DEG,
            speed_rad_s=CALIBRATION_MOVE_SPEED_RAD_S,
            acceleration_rad_s2=CALIBRATION_MOVE_ACCELERATION_RAD_S2,
        )
        timeout_s = max(10.0, float(timeout_s) + CALIBRATION_MOVE_TIMEOUT_PAD_S)
        deadline = time.time() + timeout_s
        latest_status = status

        while time.time() < deadline:
            latest_status = gateway.refresh_status()
            current_joints = list(latest_status.get("joint_positions_deg") or [])
            if len(current_joints) == 6:
                max_joint_error_deg = max(
                    abs(float(current) - float(target))
                    for current, target in zip(current_joints, CALIBRATION_JOINTS_DEG)
                )
                if max_joint_error_deg <= CALIBRATION_JOINT_TOLERANCE_DEG:
                    break
            time.sleep(0.25)
        else:
            raise RuntimeError("El robot no llego a la postura de calibracion dentro del tiempo esperado.")

        tcp_pose_mm = list(latest_status.get("current_pose_mm") or [])
        if len(tcp_pose_mm) < 3:
            raise RuntimeError("No se pudo leer la pose TCP del robot para extraer la Z.")
        plane_z_mm = float(tcp_pose_mm[2])
        print(f"Robot listo en postura de calibracion. TCP Z base = {plane_z_mm:.2f} mm")
        return plane_z_mm
    finally:
        gateway.disconnect()


def main() -> None:
    config = AppConfig.load()
    calibration = DEFAULT_CALIBRATION
    mode = "object_pick"
    output_path = config.calibration_file()
    output_path.parent.mkdir(parents=True, exist_ok=True)
    plane_z_mm = prepare_robot_plane_z(config)

    supported = detect_supported_resolutions(config.camera_index)
    print("Resoluciones detectadas:", supported or "No se pudo confirmar; se usara la actual.")

    capture = cv2.VideoCapture(config.camera_index)
    capture.set(cv2.CAP_PROP_FRAME_WIDTH, config.camera_width)
    capture.set(cv2.CAP_PROP_FRAME_HEIGHT, config.camera_height)
    capture.set(cv2.CAP_PROP_FPS, config.camera_fps)

    cv2.namedWindow("SILVIA Calibrator")
    cv2.setMouseCallback("SILVIA Calibrator", on_mouse)

    while True:
        ok, frame = capture.read()
        if not ok:
            continue

        height, width, _ = frame.shape
        cv2.putText(frame, f"Modo: {mode}", (20, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (20, 230, 120), 2)
        cv2.putText(frame, f"Resolucion activa: {width}x{height}", (20, 56), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (240, 240, 240), 2)
        cv2.putText(frame, f"TCP Z base robot: {plane_z_mm:.1f} mm", (20, 84), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (240, 240, 240), 2)
        cv2.putText(frame, "Click P1..P4 en orden alrededor del rectangulo. Teclas: 1=mano 2=objeto s=guardar c=limpiar q=salir", (20, height - 20), cv2.FONT_HERSHEY_SIMPLEX, 0.48, (240, 240, 240), 1)

        for index, point in enumerate(clicked_points):
            cv2.circle(frame, (int(point[0]), int(point[1])), 6, (40, 170, 255), -1)
            cv2.putText(frame, str(index + 1), (int(point[0]) + 8, int(point[1]) - 8), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (40, 170, 255), 2)

        cv2.imshow("SILVIA Calibrator", frame)
        key = cv2.waitKey(10) & 0xFF

        if key == ord("1"):
            mode = "hand_follow"
            clicked_points.clear()
        elif key == ord("2"):
            mode = "object_pick"
            clicked_points.clear()
        elif key == ord("c"):
            clicked_points.clear()
        elif key == ord("s") and len(clicked_points) == 4:
            width_mm = float(input("Distancia horizontal entre P1 y P2 en mm: ").strip())
            height_mm = float(input("Distancia vertical entre P2 y P3 en mm: ").strip())
            world_points = [[0.0, 0.0], [width_mm, 0.0], [width_mm, height_mm], [0.0, height_mm]]
            homography = compute_homography(clicked_points, world_points)
            hfov_deg = round(math.degrees(2.0 * math.atan(width_mm / (2.0 * plane_z_mm))), 2)
            vfov_deg = round(math.degrees(2.0 * math.atan(height_mm / (2.0 * plane_z_mm))), 2)

            mode_config = getattr(calibration, mode)
            mode_config.plane_z_mm = plane_z_mm
            mode_config.image_points = list(clicked_points)
            mode_config.world_points_mm = world_points
            mode_config.homography = homography
            mode_config.workspace = {
                "x": [-50.0, width_mm + 50.0],
                "y": [-50.0, height_mm + 50.0],
                "z": [0.0, max(plane_z_mm + 300.0, 300.0)],
            }
            calibration.camera = {
                "index": config.camera_index,
                "width": width,
                "height": height,
                "fps": int(capture.get(cv2.CAP_PROP_FPS) or config.camera_fps),
                "hfov_deg": hfov_deg,
                "vfov_deg": vfov_deg,
            }
            save_calibration(output_path, calibration)
            print(f"Calibracion guardada en {output_path}")
            print(f"HFOV estimado: {hfov_deg} grados | VFOV estimado: {vfov_deg} grados")
            clicked_points.clear()
        elif key == ord("q"):
            break

    capture.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
