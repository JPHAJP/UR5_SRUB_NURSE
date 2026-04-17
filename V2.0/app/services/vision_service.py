from __future__ import annotations

import logging
import threading
import time
from typing import Any, Dict, List, Optional, Tuple

from ..calibration.loader import load_calibration
from ..calibration.transforms import apply_homography

logger = logging.getLogger(__name__)

FALLBACK_CLASS_NAMES = {
    0: "Bisturi",
    1: "Mano",
    2: "No_Objeto",
    3: "Pinzas",
    4: "Tijeras_curvas",
    5: "Tijeras_rectas",
}


class VisionService:
    def __init__(self, config, state) -> None:
        self.config = config
        self.state = state
        self.calibration = load_calibration(self.config.calibration_file())
        self._running = False
        self._thread: threading.Thread | None = None
        self._frame_lock = threading.RLock()
        self._latest_jpeg: bytes | None = None
        self._latest_detections: List[Dict[str, Any]] = []
        self._latest_hand_target: Optional[Dict[str, Any]] = None
        self._capture = None
        self._cv2 = None
        self._mp = None
        self._hands = None
        self._yolo = None
        self._model = None

    def start(self) -> None:
        if self._running:
            return
        self._running = True
        self._thread = threading.Thread(target=self._capture_loop, daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._running = False
        if self._thread and self._thread.is_alive():
            self._thread.join(timeout=1.0)
        if self._capture is not None:
            self._capture.release()
            self._capture = None

    def mjpeg_stream(self):
        while True:
            with self._frame_lock:
                frame = self._latest_jpeg
            if frame:
                yield (
                    b"--frame\r\n"
                    b"Content-Type: image/jpeg\r\n\r\n" + frame + b"\r\n"
                )
            time.sleep(0.05)

    def get_latest_hand_target(self) -> Dict[str, Any] | None:
        with self._frame_lock:
            return dict(self._latest_hand_target) if self._latest_hand_target else None

    def find_object_target(self, label: str | None = None) -> Dict[str, Any] | None:
        with self._frame_lock:
            detections = list(self._latest_detections)

        objects = [item for item in detections if item.get("type") == "object"]
        if label:
            objects = [item for item in objects if item.get("label") == label]
        if not objects:
            return None
        objects.sort(key=lambda item: item.get("confidence", 0.0), reverse=True)
        return objects[0]

    def _capture_loop(self) -> None:
        try:
            import cv2
            import mediapipe as mp
            from ultralytics import YOLO
        except Exception as error:  # pragma: no cover - dependency fallback
            self.state.set_vision_status({"ok": False, "message": f"Dependencias de vision no disponibles: {error}"})
            logger.warning("VisionService sin dependencias: %s", error)
            return

        self._cv2 = cv2
        self._mp = mp
        self._yolo = YOLO
        self._hands = mp.solutions.hands.Hands(
            static_image_mode=False,
            max_num_hands=1,
            min_detection_confidence=0.5,
            min_tracking_confidence=0.5,
        )

        if self.config.vision_model_path.exists():
            try:
                self._model = YOLO(str(self.config.vision_model_path))
            except Exception as error:  # pragma: no cover - dependency fallback
                logger.warning("No se pudo cargar el modelo YOLO: %s", error)
                self._model = None

        self._capture = cv2.VideoCapture(self.config.camera_index)
        self._capture.set(cv2.CAP_PROP_FRAME_WIDTH, self.config.camera_width)
        self._capture.set(cv2.CAP_PROP_FRAME_HEIGHT, self.config.camera_height)
        self._capture.set(cv2.CAP_PROP_FPS, self.config.camera_fps)

        while self._running:
            if not self._capture.isOpened():
                self.state.set_vision_status({"ok": False, "message": "No se pudo abrir la webcam."})
                time.sleep(1.0)
                continue

            ok, frame = self._capture.read()
            if not ok:
                self.state.set_vision_status({"ok": False, "message": "No se pudo leer un frame de la webcam."})
                time.sleep(0.2)
                continue

            detections = self._detect_objects(frame)
            hand_target = self._detect_hand(frame)
            annotated = self._annotate_frame(frame.copy(), detections, hand_target)
            success, buffer = cv2.imencode(".jpg", annotated)

            with self._frame_lock:
                self._latest_detections = detections
                self._latest_hand_target = hand_target
                self._latest_jpeg = buffer.tobytes() if success else None

            self.state.set_detections(detections)
            self.state.set_hand_target(hand_target)
            self.state.set_vision_status(
                {
                    "ok": True,
                    "camera_open": True,
                    "model_loaded": bool(self._model),
                    "detections": len(detections),
                    "hand_visible": bool(hand_target),
                    "calibrated_hand": bool(self.calibration.hand_follow.homography),
                    "calibrated_object": bool(self.calibration.object_pick.homography),
                }
            )

    def _detect_objects(self, frame) -> List[Dict[str, Any]]:
        if self._model is None:
            return []

        try:
            results = self._model(frame, verbose=False)
        except Exception as error:  # pragma: no cover - dependency fallback
            logger.warning("YOLO fallo: %s", error)
            return []

        result = results[0]
        names = getattr(self._model, "names", FALLBACK_CLASS_NAMES)
        detections: List[Dict[str, Any]] = []

        if getattr(result, "obb", None) is not None and len(result.obb.cls):
            boxes = result.obb.xyxy.cpu().tolist()
            classes = result.obb.cls.cpu().tolist()
            confidences = result.obb.conf.cpu().tolist()
        elif getattr(result, "boxes", None) is not None and len(result.boxes.cls):
            boxes = result.boxes.xyxy.cpu().tolist()
            classes = result.boxes.cls.cpu().tolist()
            confidences = result.boxes.conf.cpu().tolist()
        else:
            return []

        for box, cls_idx, confidence in zip(boxes, classes, confidences):
            label = names.get(int(cls_idx), str(cls_idx))
            center_x = (box[0] + box[2]) / 2.0
            center_y = (box[1] + box[3]) / 2.0
            world_xy = None
            if self.calibration.object_pick.homography:
                try:
                    x_mm, y_mm = apply_homography((center_x, center_y), self.calibration.object_pick.homography)
                    world_xy = [round(x_mm, 2), round(y_mm, 2), self.calibration.object_pick.plane_z_mm]
                except Exception:
                    world_xy = None
            detections.append(
                {
                    "type": "object",
                    "label": label,
                    "confidence": round(float(confidence), 3),
                    "bbox": [round(float(value), 2) for value in box],
                    "center_px": [round(center_x, 2), round(center_y, 2)],
                    "world_mm": world_xy,
                }
            )
        return detections

    def _detect_hand(self, frame) -> Dict[str, Any] | None:
        if self._hands is None:
            return None

        rgb = self._cv2.cvtColor(frame, self._cv2.COLOR_BGR2RGB)
        results = self._hands.process(rgb)
        if not results.multi_hand_landmarks:
            return None

        hand_landmarks = results.multi_hand_landmarks[0]
        height, width, _ = frame.shape
        landmark = hand_landmarks.landmark[self._mp.solutions.hands.HandLandmark.MIDDLE_FINGER_MCP]
        x_px = float(landmark.x * width)
        y_px = float(landmark.y * height)
        world_xyz = None
        if self.calibration.hand_follow.homography:
            try:
                x_mm, y_mm = apply_homography((x_px, y_px), self.calibration.hand_follow.homography)
                world_xyz = [round(x_mm, 2), round(y_mm, 2), self.calibration.hand_follow.plane_z_mm]
            except Exception:
                world_xyz = None

        return {
            "type": "hand",
            "label": "Mano",
            "center_px": [round(x_px, 2), round(y_px, 2)],
            "world_mm": world_xyz,
        }

    def _annotate_frame(self, frame, detections, hand_target):
        cv2 = self._cv2
        for detection in detections:
            box = detection["bbox"]
            label = f"{detection['label']} {detection['confidence']:.2f}"
            cv2.rectangle(frame, (int(box[0]), int(box[1])), (int(box[2]), int(box[3])), (34, 139, 230), 2)
            cv2.putText(frame, label, (int(box[0]), int(box[1] - 8)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)

        if hand_target:
            x_px, y_px = hand_target["center_px"]
            cv2.circle(frame, (int(x_px), int(y_px)), 8, (80, 220, 120), -1)
            cv2.putText(frame, "Mano", (int(x_px) + 10, int(y_px)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (80, 220, 120), 2)

        cv2.putText(
            frame,
            f"Modelo: {self.config.vision_model_path.name}",
            (16, 28),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            (245, 245, 245),
            2,
        )
        return frame
