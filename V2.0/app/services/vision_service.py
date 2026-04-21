from __future__ import annotations

import logging
import threading
import time
from typing import Any, Dict, List, Optional, Sequence

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

HAND_LABEL_ALIASES = {
    "glove",
    "guante",
    "hand",
    "mano",
}


class VisionService:
    def __init__(self, config, state) -> None:
        self.config = config
        self.state = state
        self.calibration = load_calibration(self.config.calibration_file())
        self._running = False
        self._capture_thread: threading.Thread | None = None
        self._inference_thread: threading.Thread | None = None
        self._stop_event = threading.Event()
        self._frame_lock = threading.RLock()
        self._latest_jpeg: bytes | None = None
        self._latest_detections: List[Dict[str, Any]] = []
        self._latest_hand_target: Optional[Dict[str, Any]] = None
        self._latest_frame = None
        self._latest_frame_id = 0
        self._capture = None
        self._cv2 = None
        self._yolo = None
        self._model = None
        self._camera_open = False
        self._last_capture_error = ""
        self._last_inference_error = ""

    def start(self) -> None:
        if self._running:
            return
        self._running = True
        self._stop_event.clear()
        self._capture_thread = threading.Thread(target=self._camera_loop, daemon=True, name="vision-camera")
        self._inference_thread = threading.Thread(target=self._inference_loop, daemon=True, name="vision-inference")
        self._capture_thread.start()
        self._inference_thread.start()

    def stop(self) -> None:
        self._running = False
        self._stop_event.set()
        if self._capture_thread and self._capture_thread.is_alive():
            self._capture_thread.join(timeout=1.0)
        if self._inference_thread and self._inference_thread.is_alive():
            self._inference_thread.join(timeout=1.0)
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

    def _camera_loop(self) -> None:
        try:
            import cv2
        except Exception as error:  # pragma: no cover - dependency fallback
            self._last_capture_error = f"OpenCV no disponible: {error}"
            self._publish_status(ok=False, message=self._last_capture_error)
            logger.warning("VisionService sin OpenCV: %s", error)
            return

        self._cv2 = cv2

        while self._running and not self._stop_event.is_set():
            if not self._ensure_capture():
                time.sleep(1.0)
                continue

            ok, frame = self._capture.read()
            if not ok or frame is None:
                self._camera_open = False
                self._last_capture_error = "No se pudo leer un frame de la webcam."
                self._publish_status(ok=False, message=self._last_capture_error)
                time.sleep(0.2)
                continue

            with self._frame_lock:
                self._latest_frame = frame.copy()
                self._latest_frame_id += 1

            self._camera_open = True
            self._last_capture_error = ""
            time.sleep(0.001)

    def _inference_loop(self) -> None:
        try:
            from ultralytics import YOLO
        except Exception as error:  # pragma: no cover - dependency fallback
            self._last_inference_error = f"Ultralytics/YOLO no disponible: {error}"
            self._publish_status(ok=False, message=self._last_inference_error)
            logger.warning("VisionService sin YOLO: %s", error)
            return

        self._yolo = YOLO
        self._load_model()

        last_processed_frame_id = 0
        while self._running and not self._stop_event.is_set():
            frame = None
            frame_id = 0
            with self._frame_lock:
                if self._latest_frame is not None and self._latest_frame_id != last_processed_frame_id:
                    frame = self._latest_frame.copy()
                    frame_id = self._latest_frame_id

            if frame is None:
                time.sleep(0.01)
                continue

            detections, plotted_frame = self._detect_with_yolo(frame)
            hand_target = self._select_hand_target(detections)
            annotated = self._annotate_frame(plotted_frame, detections, hand_target)
            success, buffer = self._cv2.imencode(".jpg", annotated)

            with self._frame_lock:
                self._latest_detections = detections
                self._latest_hand_target = hand_target
                self._latest_jpeg = buffer.tobytes() if success else None

            self.state.set_detections(detections)
            self.state.set_hand_target(hand_target)
            self._publish_status(ok=True)
            last_processed_frame_id = frame_id

    def _ensure_capture(self) -> bool:
        if self._capture is not None and self._capture.isOpened():
            return True

        if self._capture is not None:
            self._capture.release()

        self._capture = self._cv2.VideoCapture(self.config.camera_index)
        self._capture.set(self._cv2.CAP_PROP_FRAME_WIDTH, self.config.camera_width)
        self._capture.set(self._cv2.CAP_PROP_FRAME_HEIGHT, self.config.camera_height)
        self._capture.set(self._cv2.CAP_PROP_FPS, self.config.camera_fps)

        if not self._capture.isOpened():
            self._camera_open = False
            self._last_capture_error = "No se pudo abrir la webcam."
            self._publish_status(ok=False, message=self._last_capture_error)
            return False

        self._camera_open = True
        self._last_capture_error = ""
        return True

    def _load_model(self) -> None:
        if not self.config.vision_model_path.exists():
            self._last_inference_error = f"No se encontro el modelo YOLO en {self.config.vision_model_path}."
            self._publish_status(ok=False, message=self._last_inference_error)
            logger.warning(self._last_inference_error)
            self._model = None
            return

        try:
            self._model = self._yolo(str(self.config.vision_model_path))
            self._last_inference_error = ""
        except Exception as error:  # pragma: no cover - dependency fallback
            self._last_inference_error = f"No se pudo cargar el modelo YOLO: {error}"
            self._publish_status(ok=False, message=self._last_inference_error)
            logger.warning(self._last_inference_error)
            self._model = None

    def _detect_with_yolo(self, frame) -> tuple[List[Dict[str, Any]], Any]:
        if self._model is None:
            return [], frame

        try:
            results = self._model(
                frame,
                conf=self.config.vision_confidence_threshold,
                verbose=False,
            )
        except Exception as error:  # pragma: no cover - dependency fallback
            self._last_inference_error = f"YOLO fallo: {error}"
            logger.warning(self._last_inference_error)
            self._publish_status(ok=False, message=self._last_inference_error)
            return [], frame

        result = results[0]
        plotted_frame = result.plot()
        names = self._class_names()
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
            self._last_inference_error = ""
            return [], plotted_frame

        for box, cls_idx, confidence in zip(boxes, classes, confidences):
            label = names.get(int(cls_idx), str(cls_idx))
            detections.append(self._build_detection(label, confidence, box))

        self._last_inference_error = ""
        return detections, plotted_frame

    def _class_names(self) -> Dict[int, str]:
        names = getattr(self._model, "names", FALLBACK_CLASS_NAMES) if self._model is not None else FALLBACK_CLASS_NAMES
        if isinstance(names, dict):
            return {int(index): str(label) for index, label in names.items()}
        if isinstance(names, Sequence) and not isinstance(names, (str, bytes)):
            return {index: str(label) for index, label in enumerate(names)}
        return dict(FALLBACK_CLASS_NAMES)

    def _build_detection(self, label: str, confidence: float, box: Sequence[float]) -> Dict[str, Any]:
        center_x = (box[0] + box[2]) / 2.0
        center_y = (box[1] + box[3]) / 2.0
        is_hand = self._is_hand_label(label)

        world_xyz = None
        if is_hand and self.calibration.hand_follow.homography:
            try:
                x_mm, y_mm = apply_homography((center_x, center_y), self.calibration.hand_follow.homography)
                world_xyz = [round(x_mm, 2), round(y_mm, 2), self.calibration.hand_follow.plane_z_mm]
            except Exception:
                world_xyz = None
        elif not is_hand and self.calibration.object_pick.homography:
            try:
                x_mm, y_mm = apply_homography((center_x, center_y), self.calibration.object_pick.homography)
                world_xyz = [round(x_mm, 2), round(y_mm, 2), self.calibration.object_pick.plane_z_mm]
            except Exception:
                world_xyz = None

        return {
            "type": "hand" if is_hand else "object",
            "label": str(label),
            "confidence": round(float(confidence), 3),
            "bbox": [round(float(value), 2) for value in box],
            "center_px": [round(center_x, 2), round(center_y, 2)],
            "world_mm": world_xyz,
        }

    def _select_hand_target(self, detections: List[Dict[str, Any]]) -> Dict[str, Any] | None:
        hand_detections = [item for item in detections if item.get("type") == "hand"]
        if not hand_detections:
            return None

        hand_detections.sort(key=lambda item: item.get("confidence", 0.0), reverse=True)
        hand = dict(hand_detections[0])
        hand["type"] = "hand"
        return hand

    @staticmethod
    def _normalize_label(label: str) -> str:
        return "".join(char.lower() for char in str(label) if char.isalnum())

    @classmethod
    def _is_hand_label(cls, label: str) -> bool:
        normalized = cls._normalize_label(label)
        return normalized in HAND_LABEL_ALIASES

    def _annotate_frame(self, frame, detections, hand_target):
        cv2 = self._cv2

        if hand_target:
            box = hand_target.get("bbox")
            if box:
                cv2.rectangle(frame, (int(box[0]), int(box[1])), (int(box[2]), int(box[3])), (80, 220, 120), 2)
                cv2.putText(
                    frame,
                    f"{hand_target['label']} {hand_target['confidence']:.2f}",
                    (int(box[0]), max(22, int(box[1] - 8))),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.6,
                    (80, 220, 120),
                    2,
                )

        cv2.putText(
            frame,
            f"Modelo: {self.config.vision_model_path.name}",
            (16, 28),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            (245, 245, 245),
            2,
        )
        cv2.putText(
            frame,
            "Hand tracking: YOLO",
            (16, 56),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            (80, 220, 120),
            2,
        )
        cv2.putText(
            frame,
            f"Detecciones: {len(detections)} | Conf: {self.config.vision_confidence_threshold:.2f}",
            (16, 84),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            (245, 245, 245),
            2,
        )
        return frame

    def _publish_status(self, ok: bool, message: str | None = None) -> None:
        status = {
            "ok": ok and self._camera_open,
            "camera_open": self._camera_open,
            "capture_thread_running": bool(self._capture_thread and self._capture_thread.is_alive()),
            "inference_thread_running": bool(self._inference_thread and self._inference_thread.is_alive()),
            "model_loaded": bool(self._model),
            "detections": len(self._latest_detections),
            "hand_visible": bool(self._latest_hand_target),
            "hand_detector_mode": "yolo",
            "calibrated_hand": bool(self.calibration.hand_follow.homography),
            "calibrated_object": bool(self.calibration.object_pick.homography),
        }

        if message:
            status["message"] = message
        elif self._last_capture_error:
            status["message"] = self._last_capture_error
        elif self._last_inference_error:
            status["message"] = self._last_inference_error
        else:
            status["message"] = "Vision YOLO activa."

        self.state.set_vision_status(status)
