from __future__ import annotations

import math
from pathlib import Path

import cv2

from app.calibration.loader import DEFAULT_CALIBRATION, save_calibration
from app.calibration.transforms import compute_homography
from app.config import AppConfig


COMMON_RESOLUTIONS = [
    (640, 480),
    (800, 600),
    (960, 540),
    (1280, 720),
    (1920, 1080),
]

clicked_points = []


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


def main() -> None:
    config = AppConfig.load()
    calibration = DEFAULT_CALIBRATION
    mode = "object_pick"
    output_path = config.calibration_file()
    output_path.parent.mkdir(parents=True, exist_ok=True)

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
        cv2.putText(frame, "Click en 4 esquinas del area de trabajo. Teclas: 1=mano 2=objeto s=guardar c=limpiar q=salir", (20, height - 20), cv2.FONT_HERSHEY_SIMPLEX, 0.48, (240, 240, 240), 1)

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
            width_mm = float(input("Ancho visible del plano en mm: ").strip())
            height_mm = float(input("Alto visible del plano en mm: ").strip())
            plane_z_mm = float(input("Profundidad fija del plano en mm: ").strip())
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
