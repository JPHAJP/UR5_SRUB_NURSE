from __future__ import annotations

import logging
import os
import subprocess
import threading
import time
from typing import Any, Dict, List, Optional, Sequence

from ..calibration.loader import load_calibration, save_calibration
from ..calibration.schema import CameraIntrinsics, DepthCalibrationSample
from ..calibration.transforms import (
    apply_rigid_transform,
    build_transform_matrix,
    deproject_pixel_to_camera_xyz,
    fit_depth_compensation,
)

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
        self._sync_calibration_defaults()
        self._running = False
        self._capture_thread: threading.Thread | None = None
        self._inference_thread: threading.Thread | None = None
        self._stop_event = threading.Event()
        self._frame_lock = threading.RLock()
        self._latest_jpeg: bytes | None = None
        self._latest_rgb_jpeg: bytes | None = None
        self._latest_depth_jpeg: bytes | None = None
        self._latest_detections: List[Dict[str, Any]] = []
        self._latest_hand_target: Optional[Dict[str, Any]] = None
        self._latest_frame = None
        self._latest_depth_frame = None
        self._latest_frame_id = 0
        self._latest_depth_id = 0
        self._latest_rgb_timestamp = 0.0
        self._latest_depth_timestamp = 0.0
        self._latest_depth_report: Dict[str, Any] = {
            "source": "hp60c_ros2",
            "timestamp_ms": 0,
            "points": [],
            "average_depth_mm": None,
            "valid_points": 0,
            "span_ratio": self.config.depth_sampling_ratio,
        }
        self._cv2 = None
        self._yolo = None
        self._model = None
        self._camera_open = False
        self._rgb_stream_ok = False
        self._depth_stream_ok = False
        self._ros_node = None
        self._ros_subscriptions: List[Any] = []
        self._rclpy = None
        self._rclpy_owner = False
        self._camera_process: subprocess.Popen | None = None
        self._camera_process_started_by_service = False
        self._last_capture_error = ""
        self._last_inference_error = ""
        self._last_depth_error = ""
        self._last_camera_launch_error = ""

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

        if self._ros_node is not None:
            try:
                self._ros_node.destroy_node()
            except Exception:
                pass
            self._ros_node = None

        if self._rclpy is not None and self._rclpy_owner:
            try:
                self._rclpy.shutdown()
            except Exception:
                pass
            self._rclpy_owner = False
        self._stop_camera_process()

    def mjpeg_stream(self, stream_kind: str = "annotated"):
        while True:
            with self._frame_lock:
                if stream_kind == "rgb":
                    frame = self._latest_rgb_jpeg or self._latest_jpeg
                elif stream_kind == "depth":
                    frame = self._latest_depth_jpeg
                else:
                    frame = self._latest_jpeg or self._latest_rgb_jpeg
            if not frame:
                frame = self._placeholder_jpeg(stream_kind)
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

    def get_depth_diagnostics(self) -> Dict[str, Any]:
        with self._frame_lock:
            report = dict(self._latest_depth_report)
            report["points"] = [dict(point) for point in self._latest_depth_report.get("points", [])]
            report["depth_ok"] = self._depth_stream_ok
            report["rgb_ok"] = self._rgb_stream_ok
            report["intrinsics_ok"] = self._intrinsics_ok()
            report["alignment_ok"] = self._alignment_ok()
            report["camera_to_robot"] = self._camera_transform_summary()
            report["depth_compensation"] = self._depth_compensation_summary()
            report["topics"] = dict(self.calibration.topics)
            report["model_loaded"] = bool(self._model)
            report["model_path"] = str(self.config.vision_model_path)
            report["camera_process_running"] = self._camera_process_running()
            report["camera_auto_launch"] = bool(self.config.ros_auto_launch_camera)
            report["camera_launch_error"] = self._last_camera_launch_error
        return report

    def get_depth_calibration_summary(self, robot_pose_mm: Sequence[float] | None = None) -> Dict[str, Any]:
        with self._frame_lock:
            summary = {
                "backend": self.calibration.backend,
                "topics": dict(self.calibration.topics),
                "intrinsics": {
                    "rgb": self.calibration.intrinsics["rgb"].to_dict(),
                    "depth": self.calibration.intrinsics["depth"].to_dict(),
                },
                "camera_to_robot": self._camera_transform_summary(),
                "depth_compensation": self._depth_compensation_summary(),
                "latest_center_depth_mm": self._latest_depth_report.get("average_depth_mm"),
                "samples": [
                    sample.to_dict()
                    for sample in self.calibration.depth_compensation.samples
                ],
            }

        if robot_pose_mm and len(robot_pose_mm) >= 3 and summary["latest_center_depth_mm"] is not None:
            robot_z_mm = float(robot_pose_mm[2])
            center_depth_mm = float(summary["latest_center_depth_mm"])
            predicted_mm = self._apply_depth_compensation(center_depth_mm)
            summary["robot_reference"] = {
                "robot_z_mm": robot_z_mm,
                "predicted_depth_mm": predicted_mm,
                "error_mm": round(robot_z_mm - predicted_mm, 3),
            }
        return summary

    def set_depth_compensation(self, gain: float | None = None, offset_mm: float | None = None) -> Dict[str, Any]:
        with self._frame_lock:
            if gain is not None:
                self.calibration.depth_compensation.gain = float(gain)
            if offset_mm is not None:
                self.calibration.depth_compensation.offset_mm = float(offset_mm)
            self.calibration.depth_compensation.configured = True
            self._refresh_depth_sample_errors()
            return self._depth_compensation_summary()

    def set_camera_transform(
        self,
        translation_mm: Sequence[float],
        rotation_rpy_deg: Sequence[float],
    ) -> Dict[str, Any]:
        with self._frame_lock:
            self.calibration.camera_to_robot.translation_mm = [float(value) for value in translation_mm[:3]]
            self.calibration.camera_to_robot.rotation_rpy_deg = [float(value) for value in rotation_rpy_deg[:3]]
            self.calibration.camera_to_robot.matrix_4x4 = build_transform_matrix(
                self.calibration.camera_to_robot.translation_mm,
                self.calibration.camera_to_robot.rotation_rpy_deg,
            )
            self.calibration.camera_to_robot.configured = True
            return self._camera_transform_summary()

    def capture_depth_calibration_sample(self, robot_pose_mm: Sequence[float]) -> Dict[str, Any]:
        if len(robot_pose_mm) < 3:
            raise ValueError("La pose del robot no incluye z.")

        with self._frame_lock:
            center_depth_mm = self._latest_depth_report.get("average_depth_mm")
            if center_depth_mm is None:
                raise ValueError("No hay profundidad valida en el centro de la imagen.")

            compensated_depth_mm = self._apply_depth_compensation(float(center_depth_mm))
            robot_z_mm = float(robot_pose_mm[2])
            sample = DepthCalibrationSample(
                camera_depth_mm=float(center_depth_mm),
                robot_z_mm=robot_z_mm,
                compensated_depth_mm=compensated_depth_mm,
                error_mm=robot_z_mm - compensated_depth_mm,
                timestamp=time.time(),
            )
            self.calibration.depth_compensation.samples.append(sample)
            self._refresh_depth_sample_errors()
            return sample.to_dict()

    def fit_depth_calibration(self) -> Dict[str, Any]:
        with self._frame_lock:
            fit = fit_depth_compensation(
                [
                    {
                        "camera_depth_mm": sample.camera_depth_mm,
                        "robot_z_mm": sample.robot_z_mm,
                    }
                    for sample in self.calibration.depth_compensation.samples
                ]
            )
            self.calibration.depth_compensation.gain = float(fit["gain"])
            self.calibration.depth_compensation.offset_mm = float(fit["offset_mm"])
            self.calibration.depth_compensation.last_error_abs_mm = float(fit["last_error_abs_mm"])
            self.calibration.depth_compensation.last_error_mean_mm = float(fit["last_error_mean_mm"])
            self.calibration.depth_compensation.configured = bool(self.calibration.depth_compensation.samples)
            self._refresh_depth_sample_errors()
            return self._depth_compensation_summary()

    def save_calibration_to_disk(self) -> Dict[str, Any]:
        with self._frame_lock:
            save_calibration(self.config.calibration_file(), self.calibration)
            return self.get_depth_calibration_summary()

    def _camera_loop(self) -> None:
        try:
            import cv2
        except Exception as error:  # pragma: no cover - dependency fallback
            self._last_capture_error = f"OpenCV no disponible: {error}"
            self._publish_status(ok=False, message=self._last_capture_error)
            logger.warning("VisionService sin OpenCV: %s", error)
            return

        self._cv2 = cv2

        try:  # pragma: no cover - depends on ROS install
            self._ensure_numpy_compatible_with_cv_bridge()
            import rclpy
            from cv_bridge import CvBridge
            from sensor_msgs.msg import CameraInfo, Image
        except Exception as error:  # pragma: no cover - dependency fallback
            self._last_capture_error = f"ROS2/cv_bridge no disponible: {error}"
            self._publish_status(ok=False, message=self._last_capture_error)
            logger.warning("VisionService sin ROS2 HP60C: %s", error)
            return

        self._rclpy = rclpy
        self._ensure_rclpy_initialized()

        bridge = CvBridge()
        self._ros_node = rclpy.create_node("silvia_hp60c_bridge")
        self._ros_subscriptions = [
            self._ros_node.create_subscription(Image, self.config.ros_rgb_topic, self._make_rgb_callback(bridge), 10),
            self._ros_node.create_subscription(Image, self.config.ros_depth_topic, self._make_depth_callback(bridge), 10),
            self._ros_node.create_subscription(CameraInfo, self.config.ros_rgb_camera_info_topic, self._make_camera_info_callback("rgb"), 10),
            self._ros_node.create_subscription(CameraInfo, self.config.ros_depth_camera_info_topic, self._make_camera_info_callback("depth"), 10),
        ]
        auto_launch_deadline = time.time() + max(0.0, float(self.config.ros_camera_wait_timeout_s))

        while self._running and not self._stop_event.is_set():
            try:
                rclpy.spin_once(self._ros_node, timeout_sec=0.1)
            except Exception as error:  # pragma: no cover - depends on ROS runtime
                self._last_capture_error = f"ROS2 fallo al leer la HP60C: {error}"
                self._publish_status(ok=False, message=self._last_capture_error)
                logger.warning(self._last_capture_error)
                time.sleep(0.2)
                continue

            if (
                self.config.ros_auto_launch_camera
                and auto_launch_deadline is not None
                and time.time() >= auto_launch_deadline
                and not self._rgb_stream_ok
                and not self._camera_process_running()
            ):
                self._maybe_launch_camera_node()
                auto_launch_deadline = None

            if self._camera_process_started_by_service and self._camera_process is not None and self._camera_process.poll() is not None:
                exit_code = self._camera_process.poll()
                self._last_camera_launch_error = f"ascamera_node termino con codigo {exit_code}."
                self._camera_process_started_by_service = False

            self._refresh_stream_health()
            self._publish_status(ok=True)

        if self._ros_node is not None:
            try:
                self._ros_node.destroy_node()
            except Exception:
                pass
            self._ros_node = None
            self._ros_subscriptions = []

        if self._rclpy is not None and self._rclpy_owner:
            try:
                self._rclpy.shutdown()
            except Exception:
                pass
            self._rclpy_owner = False
        self._stop_camera_process()

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

    def _encode_jpeg(self, frame) -> bytes | None:
        if self._cv2 is None or frame is None:
            return None
        success, buffer = self._cv2.imencode(".jpg", frame)
        return buffer.tobytes() if success else None

    def _placeholder_jpeg(self, stream_kind: str) -> bytes | None:
        if self._cv2 is None:
            return None
        import numpy as np

        title = "RGB HP60C"
        if stream_kind == "depth":
            title = "Depth HP60C"
        elif stream_kind == "annotated":
            title = "RGB + YOLO"

        canvas = np.zeros((720, 1280, 3), dtype="uint8")
        canvas[:] = (15, 23, 32)
        line_y = 120
        self._cv2.putText(canvas, title, (48, line_y), self._cv2.FONT_HERSHEY_SIMPLEX, 1.2, (220, 240, 245), 3)

        lines = self._status_overlay_lines(stream_kind)
        for line in lines:
            line_y += 56
            self._cv2.putText(canvas, line, (48, line_y), self._cv2.FONT_HERSHEY_SIMPLEX, 0.85, (160, 214, 200), 2)

        return self._encode_jpeg(canvas)

    def _status_overlay_lines(self, stream_kind: str) -> List[str]:
        lines = []
        if self._camera_process_running():
            lines.append("ascamera_node corriendo. Esperando frames ROS2...")
        elif self.config.ros_auto_launch_camera:
            lines.append("Autoarranque HP60C habilitado.")

        if self._last_capture_error:
            lines.append(self._last_capture_error)
        if stream_kind == "depth" and self._last_depth_error:
            lines.append(self._last_depth_error)
        if stream_kind == "annotated" and self._last_inference_error:
            lines.append(self._last_inference_error)
        if self._last_camera_launch_error:
            lines.append(self._last_camera_launch_error)

        lines.append(f"Modelo YOLO: {self.config.vision_model_path.name}")
        lines.append(f"Modelo cargado: {'si' if self._model else 'no'}")
        lines.append(f"RGB topic: {self.config.ros_rgb_topic}")
        if stream_kind == "depth":
            lines.append(f"Depth topic: {self.config.ros_depth_topic}")
        return lines[:7]

    def _build_depth_preview(self, depth_frame, report: Dict[str, Any]):
        cv2 = self._cv2
        if cv2 is None:
            return depth_frame

        if len(depth_frame.shape) != 2:
            depth_frame = cv2.cvtColor(depth_frame, cv2.COLOR_BGR2GRAY)

        normalized = cv2.normalize(depth_frame, None, 0, 255, cv2.NORM_MINMAX)
        preview = cv2.applyColorMap(normalized.astype("uint8"), cv2.COLORMAP_TURBO)
        for point in report.get("points", []):
            x = int(point["x"])
            y = int(point["y"])
            valid = bool(point.get("valid"))
            color = (80, 220, 120) if valid else (64, 96, 220)
            cv2.circle(preview, (x, y), 6, color, -1)
            cv2.putText(
                preview,
                str(point["id"]),
                (x + 10, max(18, y - 8)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                (245, 245, 245),
                1,
            )

        avg = report.get("average_depth_mm")
        summary = f"Centro: {avg:.1f} mm" if isinstance(avg, (int, float)) else "Centro: sin profundidad valida"
        cv2.putText(preview, summary, (16, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (245, 245, 245), 2)
        return preview

    def _camera_process_running(self) -> bool:
        return self._camera_process is not None and self._camera_process.poll() is None

    def _maybe_launch_camera_node(self) -> None:
        if self._camera_process_running():
            return
        if not self.config.ros_ascamera_config_path.exists():
            self._last_camera_launch_error = (
                "No se encontro la configuracion de ascamera en "
                f"{self.config.ros_ascamera_config_path}."
            )
            logger.warning(self._last_camera_launch_error)
            return

        namespace = self.config.ros_camera_namespace.strip() or "/ascamera"
        command = [
            "ros2",
            "run",
            "ascamera",
            "ascamera_node",
            "--ros-args",
            "-r",
            f"__ns:={namespace}",
            "-p",
            "usb_bus_no:=-1",
            "-p",
            "usb_path:=null",
            "-p",
            f"confiPath:={self.config.ros_ascamera_config_path}",
            "-p",
            "color_pcl:=true",
            "-p",
            "pub_tfTree:=true",
            "-p",
            f"depth_width:={self.config.ros_camera_depth_width}",
            "-p",
            f"depth_height:={self.config.ros_camera_depth_height}",
            "-p",
            f"rgb_width:={self.config.ros_camera_rgb_width}",
            "-p",
            f"rgb_height:={self.config.ros_camera_rgb_height}",
            "-p",
            f"fps:={self.config.ros_camera_fps}",
        ]

        try:
            self._camera_process = subprocess.Popen(
                command,
                cwd=str(self.config.repo_dir),
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
                env=dict(os.environ),
            )
            self._camera_process_started_by_service = True
            self._last_camera_launch_error = ""
            self._last_capture_error = "Nodo HP60C iniciado por SILVIA. Esperando primeros frames RGB."
            logger.info("ascamera_node iniciado automaticamente desde SILVIA.")
            time.sleep(max(0.0, float(self.config.ros_camera_boot_delay_s)))
        except Exception as error:
            self._camera_process = None
            self._camera_process_started_by_service = False
            self._last_camera_launch_error = f"No se pudo iniciar ascamera_node: {error}"
            logger.warning(self._last_camera_launch_error)

    def _stop_camera_process(self) -> None:
        if self._camera_process is None:
            return
        if self._camera_process.poll() is None:
            try:
                self._camera_process.terminate()
                self._camera_process.wait(timeout=2.0)
            except Exception:
                try:
                    self._camera_process.kill()
                except Exception:
                    pass
        self._camera_process = None
        self._camera_process_started_by_service = False

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

        with self._frame_lock:
            depth_frame = None if self._latest_depth_frame is None else self._latest_depth_frame.copy()
            rgb_intrinsics = self.calibration.intrinsics["rgb"].to_dict()
            alignment_ok = self._alignment_ok()
            transform_ok = self.calibration.camera_to_robot.configured and bool(self.calibration.camera_to_robot.matrix_4x4)

        depth_payload = self._compute_detection_depth(
            box=box,
            depth_frame=depth_frame,
            rgb_intrinsics=rgb_intrinsics,
            alignment_ok=alignment_ok,
            transform_ok=transform_ok,
        )

        detection = {
            "type": "hand" if is_hand else "object",
            "label": str(label),
            "confidence": round(float(confidence), 3),
            "bbox": [round(float(value), 2) for value in box],
            "center_px": [round(center_x, 2), round(center_y, 2)],
            "world_mm": depth_payload["world_mm"],
            "depth_avg_mm": depth_payload["depth_avg_mm"],
            "depth_compensated_mm": depth_payload["depth_compensated_mm"],
            "depth_samples": depth_payload["depth_samples"],
            "depth_valid": depth_payload["depth_valid"],
            "camera_xyz_mm": depth_payload["camera_xyz_mm"],
        }
        return detection

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
                if hand_target.get("depth_valid") and hand_target.get("depth_avg_mm") is not None:
                    cv2.putText(
                        frame,
                        f"Z {hand_target['depth_avg_mm']:.1f} mm",
                        (int(box[0]), max(46, int(box[1] + 22))),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.55,
                        (235, 235, 235),
                        2,
                    )

        depth_avg_mm = self._latest_depth_report.get("average_depth_mm")
        depth_text = f"Centro: {depth_avg_mm:.1f} mm" if isinstance(depth_avg_mm, (int, float)) else "Centro: sin profundidad"

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
            "Hand tracking: YOLO + HP60C depth",
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
        cv2.putText(
            frame,
            depth_text,
            (16, 112),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            (245, 245, 245),
            2,
        )
        return frame

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

    def _ensure_numpy_compatible_with_cv_bridge(self) -> None:
        try:
            import numpy as np
        except Exception:
            return

        version = str(getattr(np, "__version__", "") or "").strip()
        major = self._parse_major_version(version)
        if major < 2:
            return

        raise RuntimeError(
            "NumPy "
            f"{version} no es compatible con el cv_bridge instalado en ROS Jazzy. "
            "Instala numpy<2 en este entorno, por ejemplo con: "
            "pip install --upgrade 'numpy<2' -r V2.0/requirements.txt"
        )

    @staticmethod
    def _parse_major_version(version: str) -> int:
        raw_major = str(version).split(".", 1)[0].strip()
        try:
            return int(raw_major)
        except (TypeError, ValueError):
            return 0

    def _sync_calibration_defaults(self) -> None:
        self.calibration.backend = self.config.vision_backend
        self.calibration.topics = {
            "rgb": self.config.ros_rgb_topic,
            "depth": self.config.ros_depth_topic,
            "rgb_camera_info": self.config.ros_rgb_camera_info_topic,
            "depth_camera_info": self.config.ros_depth_camera_info_topic,
        }
        self.calibration.camera.update(
            {
                "backend": self.config.vision_backend,
                "width": self.config.camera_width,
                "height": self.config.camera_height,
                "fps": self.config.camera_fps,
            }
        )

        if "rgb" not in self.calibration.intrinsics:
            self.calibration.intrinsics["rgb"] = CameraIntrinsics()
        if "depth" not in self.calibration.intrinsics:
            self.calibration.intrinsics["depth"] = CameraIntrinsics()

        self.calibration.intrinsics["rgb"].topic = self.config.ros_rgb_camera_info_topic
        self.calibration.intrinsics["depth"].topic = self.config.ros_depth_camera_info_topic

        if not self.calibration.camera_to_robot.matrix_4x4:
            self.calibration.camera_to_robot.matrix_4x4 = build_transform_matrix(
                self.calibration.camera_to_robot.translation_mm,
                self.calibration.camera_to_robot.rotation_rpy_deg,
            )

    def _ensure_rclpy_initialized(self) -> None:
        if self._rclpy is None:
            return
        if self._rclpy.ok():
            self._rclpy_owner = False
            return
        self._rclpy.init(args=None)
        self._rclpy_owner = True

    def _make_rgb_callback(self, bridge):
        def _callback(msg) -> None:
            try:
                frame = bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            except Exception as error:  # pragma: no cover - ROS runtime
                self._last_capture_error = f"Fallo al convertir RGB: {error}"
                return

            timestamp = time.time()
            encoded = self._encode_jpeg(frame)
            with self._frame_lock:
                self._latest_frame = frame.copy()
                self._latest_frame_id += 1
                self._latest_rgb_timestamp = timestamp
                self._latest_rgb_jpeg = encoded
                self._camera_open = True
                self._rgb_stream_ok = True
                self.calibration.camera["width"] = int(frame.shape[1])
                self.calibration.camera["height"] = int(frame.shape[0])
            self._last_capture_error = ""

        return _callback

    def _make_depth_callback(self, bridge):
        def _callback(msg) -> None:
            try:
                depth = bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
            except Exception as error:  # pragma: no cover - ROS runtime
                self._last_depth_error = f"Fallo al convertir profundidad: {error}"
                return

            depth_array = depth.copy()
            if len(depth_array.shape) != 2 and self._cv2 is not None:
                depth_array = self._cv2.cvtColor(depth_array, self._cv2.COLOR_BGR2GRAY)

            report = self._compute_center_depth_report(depth_array)
            depth_preview = self._build_depth_preview(depth_array, report)
            timestamp = time.time()
            with self._frame_lock:
                self._latest_depth_frame = depth_array
                self._latest_depth_id += 1
                self._latest_depth_timestamp = timestamp
                self._depth_stream_ok = bool(report.get("valid_points"))
                self._latest_depth_report = report
                self._latest_depth_jpeg = self._encode_jpeg(depth_preview)
            self._last_depth_error = "" if report.get("valid_points") else "Sin profundidad valida en el centro."

        return _callback

    def _make_camera_info_callback(self, stream_name: str):
        def _callback(msg) -> None:
            intrinsics = CameraIntrinsics(
                width=int(msg.width),
                height=int(msg.height),
                fx=float(msg.k[0]),
                fy=float(msg.k[4]),
                cx=float(msg.k[2]),
                cy=float(msg.k[5]),
                frame_id=str(msg.header.frame_id or ""),
                topic=self.calibration.topics.get(f"{stream_name}_camera_info", ""),
            )
            with self._frame_lock:
                self.calibration.intrinsics[stream_name] = intrinsics

        return _callback

    def _refresh_stream_health(self) -> None:
        now = time.time()
        with self._frame_lock:
            self._rgb_stream_ok = self._latest_rgb_timestamp > 0.0 and (now - self._latest_rgb_timestamp) < 1.5
            self._depth_stream_ok = self._latest_depth_timestamp > 0.0 and (now - self._latest_depth_timestamp) < 1.5
            self._camera_open = self._rgb_stream_ok
            if not self._rgb_stream_ok:
                self._last_capture_error = "Sin frames RGB recientes desde la HP60C."
            elif not self._depth_stream_ok:
                self._last_depth_error = "Sin frames de profundidad recientes desde la HP60C."

    def _normalize_depth_value_mm(self, raw_value: Any) -> float | None:
        try:
            value = float(raw_value)
        except (TypeError, ValueError):
            return None

        if value <= 0.0 or not (value == value):
            return None
        if value > 20.0:
            return value
        return value * 1000.0

    def _compute_center_depth_report(self, depth_frame) -> Dict[str, Any]:
        height, width = depth_frame.shape[:2]
        center_x = width // 2
        center_y = height // 2
        span_x = max(1, int(width * self.config.depth_sampling_ratio))
        span_y = max(1, int(height * self.config.depth_sampling_ratio))
        samples, average_depth_mm = self._sample_depth_grid(depth_frame, center_x, center_y, span_x, span_y)
        valid_points = len([item for item in samples if item["valid"]])
        return {
            "source": "hp60c_ros2",
            "timestamp_ms": int(time.time() * 1000),
            "points": samples,
            "average_depth_mm": average_depth_mm,
            "valid_points": valid_points,
            "span_ratio": self.config.depth_sampling_ratio,
        }

    def _sample_depth_grid(
        self,
        depth_frame,
        center_x: float,
        center_y: float,
        span_x: int,
        span_y: int,
    ) -> tuple[List[Dict[str, Any]], float | None]:
        height, width = depth_frame.shape[:2]
        xs = [
            max(0, min(width - 1, int(round(center_x - span_x)))),
            max(0, min(width - 1, int(round(center_x)))),
            max(0, min(width - 1, int(round(center_x + span_x)))),
        ]
        ys = [
            max(0, min(height - 1, int(round(center_y - span_y)))),
            max(0, min(height - 1, int(round(center_y)))),
            max(0, min(height - 1, int(round(center_y + span_y)))),
        ]

        samples: List[Dict[str, Any]] = []
        valid_values: List[float] = []
        index = 1
        for row, y in enumerate(ys):
            for col, x in enumerate(xs):
                value_mm = self._normalize_depth_value_mm(depth_frame[y, x])
                valid = value_mm is not None
                if valid:
                    valid_values.append(float(value_mm))
                samples.append(
                    {
                        "id": f"P{index}",
                        "row": row,
                        "col": col,
                        "x": int(x),
                        "y": int(y),
                        "valid": valid,
                        "value_mm": round(float(value_mm), 3) if valid else None,
                        "display": f"{float(value_mm):.2f} mm" if valid else "N/A",
                    }
                )
                index += 1

        average_depth_mm = round(sum(valid_values) / len(valid_values), 3) if valid_values else None
        return samples, average_depth_mm

    def _compute_detection_depth(
        self,
        box: Sequence[float],
        depth_frame,
        rgb_intrinsics: Dict[str, Any],
        alignment_ok: bool,
        transform_ok: bool,
    ) -> Dict[str, Any]:
        center_x = (float(box[0]) + float(box[2])) / 2.0
        center_y = (float(box[1]) + float(box[3])) / 2.0
        span_x = max(1, int(abs(float(box[2]) - float(box[0])) * self.config.depth_sampling_ratio))
        span_y = max(1, int(abs(float(box[3]) - float(box[1])) * self.config.depth_sampling_ratio))

        if depth_frame is None:
            return {
                "depth_valid": False,
                "depth_samples": [],
                "depth_avg_mm": None,
                "depth_compensated_mm": None,
                "camera_xyz_mm": None,
                "world_mm": None,
            }

        depth_samples, depth_avg_mm = self._sample_depth_grid(depth_frame, center_x, center_y, span_x, span_y)
        if depth_avg_mm is None:
            return {
                "depth_valid": False,
                "depth_samples": depth_samples,
                "depth_avg_mm": None,
                "depth_compensated_mm": None,
                "camera_xyz_mm": None,
                "world_mm": None,
            }

        depth_compensated_mm = self._apply_depth_compensation(depth_avg_mm)
        camera_xyz_mm = None
        world_mm = None

        if alignment_ok and self._intrinsics_valid_dict(rgb_intrinsics):
            try:
                camera_xyz_mm = [
                    round(value, 3)
                    for value in deproject_pixel_to_camera_xyz(
                        (center_x, center_y),
                        depth_compensated_mm,
                        rgb_intrinsics,
                    )
                ]
                if transform_ok:
                    world_mm = [
                        round(value, 3)
                        for value in apply_rigid_transform(
                            camera_xyz_mm,
                            self.calibration.camera_to_robot.matrix_4x4,
                        )
                    ]
            except Exception:
                camera_xyz_mm = None
                world_mm = None

        return {
            "depth_valid": True,
            "depth_samples": depth_samples,
            "depth_avg_mm": round(depth_avg_mm, 3),
            "depth_compensated_mm": round(depth_compensated_mm, 3),
            "camera_xyz_mm": camera_xyz_mm,
            "world_mm": world_mm,
        }

    def _apply_depth_compensation(self, depth_mm: float) -> float:
        gain = float(self.calibration.depth_compensation.gain)
        offset_mm = float(self.calibration.depth_compensation.offset_mm)
        return (float(depth_mm) * gain) + offset_mm

    def _refresh_depth_sample_errors(self) -> None:
        samples = self.calibration.depth_compensation.samples
        if not samples:
            self.calibration.depth_compensation.last_error_abs_mm = 0.0
            self.calibration.depth_compensation.last_error_mean_mm = 0.0
            return

        errors = []
        for sample in samples:
            sample.compensated_depth_mm = self._apply_depth_compensation(sample.camera_depth_mm)
            sample.error_mm = sample.robot_z_mm - sample.compensated_depth_mm
            errors.append(abs(sample.error_mm))

        self.calibration.depth_compensation.last_error_abs_mm = round(errors[-1], 3)
        self.calibration.depth_compensation.last_error_mean_mm = round(sum(errors) / len(errors), 3)

    def _intrinsics_valid_dict(self, intrinsics: Dict[str, Any]) -> bool:
        return (
            float(intrinsics.get("fx", 0.0) or 0.0) > 0.0
            and float(intrinsics.get("fy", 0.0) or 0.0) > 0.0
            and int(intrinsics.get("width", 0) or 0) > 0
            and int(intrinsics.get("height", 0) or 0) > 0
        )

    def _intrinsics_ok(self) -> bool:
        return self.calibration.intrinsics["rgb"].is_valid() and self.calibration.intrinsics["depth"].is_valid()

    def _alignment_ok(self) -> bool:
        if self._latest_frame is None or self._latest_depth_frame is None:
            return False

        rgb_height, rgb_width = self._latest_frame.shape[:2]
        depth_height, depth_width = self._latest_depth_frame.shape[:2]
        rgb_intrinsics = self.calibration.intrinsics["rgb"]
        depth_intrinsics = self.calibration.intrinsics["depth"]

        return (
            rgb_width == depth_width
            and rgb_height == depth_height
            and rgb_intrinsics.width == rgb_width
            and rgb_intrinsics.height == rgb_height
            and depth_intrinsics.width == depth_width
            and depth_intrinsics.height == depth_height
        )

    def _camera_transform_summary(self) -> Dict[str, Any]:
        transform = self.calibration.camera_to_robot
        return {
            "translation_mm": list(transform.translation_mm),
            "rotation_rpy_deg": list(transform.rotation_rpy_deg),
            "matrix_4x4": list(transform.matrix_4x4),
            "configured": bool(transform.configured),
        }

    def _depth_compensation_summary(self) -> Dict[str, Any]:
        comp = self.calibration.depth_compensation
        return {
            "gain": round(float(comp.gain), 6),
            "offset_mm": round(float(comp.offset_mm), 3),
            "last_error_abs_mm": round(float(comp.last_error_abs_mm), 3),
            "last_error_mean_mm": round(float(comp.last_error_mean_mm), 3),
            "configured": bool(comp.configured),
            "sample_count": len(comp.samples),
        }

    def _publish_status(self, ok: bool, message: str | None = None) -> None:
        status = {
            "ok": ok and self._rgb_stream_ok,
            "camera_open": self._camera_open,
            "capture_thread_running": bool(self._capture_thread and self._capture_thread.is_alive()),
            "inference_thread_running": bool(self._inference_thread and self._inference_thread.is_alive()),
            "camera_process_running": self._camera_process_running(),
            "camera_auto_launch": bool(self.config.ros_auto_launch_camera),
            "model_loaded": bool(self._model),
            "model_path": str(self.config.vision_model_path),
            "detections": len(self._latest_detections),
            "hand_visible": bool(self._latest_hand_target),
            "hand_detector_mode": "yolo",
            "calibrated_hand": bool(self.calibration.hand_follow.homography),
            "calibrated_object": bool(self.calibration.object_pick.homography),
            "rgb_ok": self._rgb_stream_ok,
            "depth_ok": self._depth_stream_ok,
            "rgb_preview_ready": bool(self._latest_rgb_jpeg),
            "depth_preview_ready": bool(self._latest_depth_jpeg),
            "intrinsics_ok": self._intrinsics_ok(),
            "alignment_ok": self._alignment_ok(),
            "calibration_ok": self._intrinsics_ok() and self._alignment_ok() and self.calibration.camera_to_robot.configured,
            "backend": self.calibration.backend,
            "depth_compensation": self._depth_compensation_summary(),
            "topics": dict(self.calibration.topics),
        }

        if message:
            status["message"] = message
        elif self._last_camera_launch_error:
            status["message"] = self._last_camera_launch_error
        elif status["camera_process_running"] and not self._rgb_stream_ok:
            status["message"] = "Nodo HP60C corriendo. Esperando frames RGB y depth."
        elif self._last_capture_error:
            status["message"] = self._last_capture_error
        elif self._last_depth_error:
            status["message"] = self._last_depth_error
        elif self._last_inference_error:
            status["message"] = self._last_inference_error
        elif not status["calibration_ok"]:
            status["message"] = "HP60C activa. Falta calibracion de profundidad o transformacion camara-robot."
        else:
            status["message"] = "Vision HP60C + YOLO activa."

        self.state.set_vision_status(status)
