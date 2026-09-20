import os
import sys
import time
from pathlib import Path

import cv2
import numpy as np
from ultralytics import YOLO

REPO_ROOT = Path(__file__).resolve().parents[2]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from ur5_controller import UR5WebController


DEFAULT_MODEL_PATH = (
    Path(__file__).resolve().parents[1]
    / "Train"
    / "runs_intsoob"
    / "obb"
    / "ur5_yolo11n_obb_512_ultralight"
    / "weights"
    / "best.pt"
)
DEFAULT_HOME_JOINTS_DEG = [-51.9, -75.16, -84.35, -110.49, 90.0, 38.0]
HAND_LABELS = {"mano", "hand", "guante", "glove"}


ROBOT_IP = os.getenv("UR5_HOST", "192.168.100.100")
WEBCAM_INDEX = int(os.getenv("WEBCAM_INDEX", "2"))
WEBCAM_WIDTH = int(os.getenv("WEBCAM_WIDTH", "640"))
WEBCAM_HEIGHT = int(os.getenv("WEBCAM_HEIGHT", "480"))
WEBCAM_FPS = int(os.getenv("WEBCAM_FPS", "30"))
YOLO_CONFIDENCE = float(os.getenv("YOLO_CONFIDENCE", "0.5"))
HAND_CAMERA_DISTANCE_M = float(os.getenv("HAND_CAMERA_DISTANCE_M", "0.55"))
FOLLOW_Z_M = float(os.getenv("HAND_FOLLOW_Z_M", str(HAND_CAMERA_DISTANCE_M)))
USE_GRAYSCALE = os.getenv("JP_HAND_USE_GRAYSCALE", "true").strip().lower() not in {"0", "false", "no", "off"}


robot = UR5WebController(robot_ip=ROBOT_IP, robot_port=30002)
robot.home_joint_angles_deg = list(DEFAULT_HOME_JOINTS_DEG)
robot.home_joint_angles_rad = np.radians(DEFAULT_HOME_JOINTS_DEG)


def initialize_webcam():
    capture = cv2.VideoCapture(WEBCAM_INDEX)
    capture.set(cv2.CAP_PROP_FRAME_WIDTH, WEBCAM_WIDTH)
    capture.set(cv2.CAP_PROP_FRAME_HEIGHT, WEBCAM_HEIGHT)
    capture.set(cv2.CAP_PROP_FPS, WEBCAM_FPS)
    if not capture.isOpened():
        raise RuntimeError(f"No se pudo abrir la webcam indice {WEBCAM_INDEX}.")
    return capture


def get_frame_and_distance(capture):
    ok, frame = capture.read()
    if not ok or frame is None:
        print("Error: no se pudo leer frame de la webcam.")
        return None, None
    return frame, HAND_CAMERA_DISTANCE_M


def prepare_inference_frame(frame):
    if not USE_GRAYSCALE:
        return frame
    gray_image = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    return cv2.cvtColor(gray_image, cv2.COLOR_GRAY2BGR)


def run_yolo(model, image, conf_threshold=0.6):
    results = model(image, conf=conf_threshold, verbose=False)
    return results


def class_name_lookup(model):
    names = getattr(model, "names", {})
    if isinstance(names, dict):
        return {int(index): str(label) for index, label in names.items()}
    if isinstance(names, list):
        return {index: str(label) for index, label in enumerate(names)}
    return {}


def validate_hand_class(model, model_path):
    names = class_name_lookup(model)
    print(f"Modelo cargado: {model_path}")
    print(f"Clases disponibles: {names}")
    hand_classes = [label for label in names.values() if is_hand_label(label)]
    if not hand_classes:
        raise RuntimeError(
            "El modelo cargado no contiene una clase de mano reconocible (Mano/hand/guante/glove)."
        )
    print(f"Clase(s) de mano detectadas en el modelo: {hand_classes}")


def is_hand_label(label):
    normalized = "".join(char.lower() for char in str(label) if char.isalnum())
    return normalized in HAND_LABELS


def calculate_object_points(results, model):
    names = class_name_lookup(model)
    points = []
    for result in results:
        if getattr(result, "obb", None) is not None and len(result.obb.cls):
            boxes = result.obb.xyxy.cpu().tolist()
            classes = result.obb.cls.cpu().tolist()
            confidences = result.obb.conf.cpu().tolist()
        elif getattr(result, "boxes", None) is not None and len(result.boxes.cls):
            boxes = result.boxes.xyxy.cpu().tolist()
            classes = result.boxes.cls.cpu().tolist()
            confidences = result.boxes.conf.cpu().tolist()
        else:
            continue

        for box, cls_idx, confidence in zip(boxes, classes, confidences):
            center_x = int((box[0] + box[2]) / 2.0)
            center_y = int((box[1] + box[3]) / 2.0)
            class_name = names.get(int(cls_idx), str(cls_idx))
            points.append((center_x, center_y, class_name, round(float(confidence), 3)))
    return points


def draw_center_points(image, points):
    for point in points:
        color = (0, 0, 255) if is_hand_label(point[2]) else (255, 170, 0)
        cv2.circle(image, (int(point[0]), int(point[1])), 4, color, -1)
        cv2.putText(
            image,
            f"{point[2]} {point[3]:.2f}",
            (int(point[0]) + 8, int(point[1]) - 8),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.5,
            color,
            1,
        )
    return image


def display_image(window_name, image):
    cv2.imshow(window_name, image)


def ensure_robot_control():
    if robot.can_control():
        return True

    print("Robot sin control activo. Intentando reconectar sockets...")
    try:
        robot.initialize_robot()
    except Exception as error:
        print(f"Fallo reintentando conexion con el robot: {error}")
        return False

    if not robot.can_control():
        print("No hay control sobre el robot. Verifica IP, remote control y puertos 30001/30002.")
        return False
    return True


def map_value(value, from_low, from_high, to_low, to_high):
    return (value - from_low) * (to_high - to_low) / (from_high - from_low) + to_low


def transform_coordinates(x1, y1, ofx, ofy, theta):
    r = np.array([
        [0, -1, 0],
        [-1, 0, 0],
        [0, 0, 1],
    ])

    r_z = np.array([
        [np.cos(theta), -np.sin(theta), 0],
        [np.sin(theta), np.cos(theta), 0],
        [0, 0, 1],
    ])

    r0_f = np.dot(r_z, r)
    p0_f = np.array([[ofx], [ofy], [0]])
    h0_f = np.concatenate((r0_f, p0_f), 1)
    h0_f = np.concatenate((h0_f, [[0, 0, 0, 1]]), 0)

    point = np.array([[x1], [y1], [0], [1]])
    transformed = np.dot(h0_f, point)
    return transformed[0][0], transformed[1][0]


def frame_origin_coordinates(xtool, ytool, h_cam, v_cam, wrist3_deg):
    radio = 39.18 / 1000.0
    longitud_tangente = 11.5 / 1000.0

    theta = np.arctan2(ytool, xtool)
    angulo_tangencia = theta + np.radians(45.0) - np.radians(wrist3_deg) + np.radians(22.0)
    angulo_tangente = angulo_tangencia - np.pi / 2.0

    c_tcp = np.sqrt(radio**2 + longitud_tangente**2)
    theta1 = np.arccos(radio / c_tcp)
    theta2 = angulo_tangencia - theta1
    cfx = c_tcp * np.cos(theta2) + xtool
    cfy = c_tcp * np.sin(theta2) + ytool

    ofx = cfx - (h_cam / 2.0) * np.cos(angulo_tangente) - (v_cam / 2.0) * np.sin(angulo_tangente)
    ofy = cfy - (h_cam / 2.0) * np.sin(angulo_tangente) + (v_cam / 2.0) * np.cos(angulo_tangente)
    return ofx, ofy, angulo_tangencia


def robot_and_camera(distance_m, object_points, frame_shape):
    transformed_points = []
    xr, yr, _zr, _rxr, _ryr, _rzr = robot.get_current_tcp_pose()
    joint_positions = robot.get_current_joint_positions()
    wrist3_deg = np.rad2deg(joint_positions[5])

    frame_height, frame_width = frame_shape[:2]
    h_cam = distance_m * np.tan(np.radians(23.0))
    v_cam = abs(distance_m * np.tan(np.radians(83.96)))
    ofx, ofy, theta = frame_origin_coordinates(xr, yr, h_cam, v_cam, wrist3_deg)

    for point in object_points:
        x_cam = map_value(point[0], 0, frame_width, 0, h_cam)
        y_cam = map_value(point[1], 0, frame_height, 0, v_cam)
        x_obj, y_obj = transform_coordinates(x_cam, y_cam, ofx, ofy, theta)
        transformed_points.append((x_obj, y_obj, point[2], point[3]))

    return transformed_points


def go_home():
    return robot.go_home()


def robot_is_stopped():
    return not bool(getattr(robot, "movement_active", False))


def error_within_range(x_actual, y_actual, x_target, y_target, tolerance_ratio):
    error_x = abs(x_target - x_actual) / abs(x_target) if x_target != 0 else 0.0
    error_y = abs(y_target - y_actual) / abs(y_target) if y_target != 0 else 0.0
    return error_x <= tolerance_ratio and error_y <= tolerance_ratio


def move_robot(x_target, y_target, z_target):
    if not ensure_robot_control():
        return False

    if not robot.is_point_within_reach(x_target, y_target, z_target):
        print(f"Punto fuera de alcance: {(x_target, y_target, z_target)}")
        return False

    current_pose = robot.get_current_pose()
    success = robot.move_to_coordinates(
        x_target * 1000.0,
        y_target * 1000.0,
        z_target * 1000.0,
        current_pose[3],
        current_pose[4],
        current_pose[5],
    )
    print(f"Resultado move_to_coordinates: {success}")
    return bool(success)


def safe_move_to_home():
    if ensure_robot_control():
        go_home()


def follow_hand(object_points):
    hand_points = [point for point in object_points if is_hand_label(point[2])]
    hand_points.sort(key=lambda item: item[3], reverse=True)
    if not hand_points:
        print("No se encontraron puntos para la clase Mano.")
        return

    x_target, y_target, _label, confidence = hand_points[0]
    x_robot, y_robot, _z_robot, _rx, _ry, _rz = robot.get_current_tcp_pose()
    print(f"Mano objetivo: ({x_target:.4f}, {y_target:.4f}) conf={confidence:.3f}")
    print(f"TCP actual: ({x_robot:.4f}, {y_robot:.4f})")

    if error_within_range(x_robot, y_robot, x_target, y_target, tolerance_ratio=0.10):
        print("El robot ya esta dentro del margen de error, no se requiere correccion.")
        return

    while not robot_is_stopped():
        print("Esperando a que el robot se detenga...")
        time.sleep(0.05)

    print(f"Moviendo el robot a ({x_target:.4f}, {y_target:.4f}, {FOLLOW_Z_M:.4f})")
    if not move_robot(x_target, y_target, FOLLOW_Z_M):
        print("No se pudo enviar el movimiento al robot.")


def main():
    capture = initialize_webcam()
    model_path = Path(os.getenv("VISION_MODEL_PATH", str(DEFAULT_MODEL_PATH))).expanduser()
    model = YOLO(str(model_path))
    print(f"Webcam activa: indice {WEBCAM_INDEX}")
    print(f"Modelo YOLO: {model_path}")
    validate_hand_class(model, model_path)
    print(f"Robot conectado: {robot.is_connected()} | Control habilitado: {robot.can_control()}")

    safe_move_to_home()
    try:
        while True:
            frame, distance_m = get_frame_and_distance(capture)
            if frame is None:
                continue

            inference_frame = prepare_inference_frame(frame)
            results = run_yolo(model, inference_frame, conf_threshold=YOLO_CONFIDENCE)
            annotated = results[0].plot() if results else frame.copy()
            object_points = calculate_object_points(results, model)
            transformed_points = robot_and_camera(distance_m, object_points, frame.shape)

            draw_center_points(annotated, object_points)
            cv2.putText(
                annotated,
                f"Modelo: {model_path.name} | Webcam {WEBCAM_INDEX}",
                (16, 28),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                (255, 255, 255),
                2,
            )
            cv2.putText(
                annotated,
                f"Distancia fija: {distance_m:.3f} m | Z follow: {FOLLOW_Z_M:.3f} m",
                (16, 56),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                (80, 220, 120),
                2,
            )
            display_image("jp_hand_webcam", annotated)

            print("Coordenadas detectadas:", transformed_points)
            follow_hand(transformed_points)

            if cv2.waitKey(1) & 0xFF == ord("q"):
                break
    finally:
        try:
            robot.send_stopl(1.0)
            robot.send_stopj(1.0)
        except Exception:
            pass
        robot.disconnect()
        capture.release()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
