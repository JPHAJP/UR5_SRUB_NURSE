from __future__ import annotations

import logging
import os
import signal
import subprocess
import threading
import time
import warnings
from pathlib import Path
from typing import Any, Dict, List, Optional, Sequence

from ..calibration.loader import load_calibration, save_calibration
from ..calibration.schema import CameraIntrinsics, DepthCalibrationSample
from ..calibration.transforms import (
    apply_rigid_transform,
    build_transform_matrix,
    build_transform_from_ur_pose,
    compose_transform_matrices,
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

MIDDLE_FINGER_MCP_INDEX = 9


class VisionService:
    def __init__(self, config, state) -> None:
        self.config = config
        self.state = state
        self._calibration_file_exists = self.config.calibration_file().exists()
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
        self._latest_camera_info_timestamp = 0.0
        self._latest_annotated_timestamp = 0.0
        self._latest_depth_report: Dict[str, Any] = {
            "source": "hp60c_ros2",
            "timestamp_ms": 0,
            "points": [],
            "average_depth_mm": None,
            "valid_points": 0,
            "span_ratio": self.config.depth_sampling_ratio,
        }
        self._cv2 = None
        self._torch = None
        self._yolo = None
        self._model = None
        self._mp = None
        self._mp_vision = None
        self._hand_landmarker = None
        self._last_hand_timestamp_ms = 0
        self._hand_landmarker_delegate = "cpu"
        self._hand_landmarker_delegate_reason = "Pendiente de inicializar."
        self._camera_open = False
        self._rgb_stream_ok = False
        self._depth_stream_ok = False
        self._ros_node = None
        self._ros_subscriptions: List[Any] = []
        self._rclpy = None
        self._rclpy_owner = False
        self._camera_process: subprocess.Popen | None = None
        self._camera_process_started_by_service = False
        self._camera_process_started_at = 0.0
        self._camera_process_log_handle = None
        self._camera_process_log_path = self.config.log_dir / "hp60c_autolaunch.log"
        self._camera_process_log_offset = 0
        self._next_camera_launch_attempt_at = 0.0
        self._last_capture_error = ""
        self._last_inference_error = ""
        self._last_hand_error = ""
        self._last_depth_error = ""
        self._last_camera_launch_error = ""
        self._inference_device = "cpu"
        self._inference_device_reason = "Pendiente de inicializar."
        self._torch_cuda_available = False
        self._torch_cuda_device_count = 0
        self._torch_cuda_version = ""
        self._last_rgb_preview_monotonic = 0.0
        self._last_depth_preview_monotonic = 0.0
        self._logged_first_rgb_frame = False
        self._logged_first_depth_frame = False
        self._last_no_frames_warning_at = 0.0
        self._camera_recovery_state = "failed"
        self._camera_restart_count = 0
        self._camera_last_restart_reason = ""
        self._camera_last_driver_issue = ""
        self._camera_stalled_since_at = 0.0
        self._camera_healthy_since_at = 0.0
        self._camera_backoff_index = 0

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

        if self._hand_landmarker is not None:
            try:
                self._hand_landmarker.close()
            except Exception:
                pass
            self._hand_landmarker = None

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
        preview_fps = (
            getattr(self.config, "vision_depth_preview_fps", 4.0)
            if stream_kind == "depth"
            else getattr(self.config, "vision_preview_fps", 8.0)
        )
        stream_interval = self._fps_interval(preview_fps)
        while True:
            now = time.time()
            with self._frame_lock:
                if stream_kind == "rgb":
                    frame = self._latest_rgb_jpeg if self._is_recent_timestamp(self._latest_rgb_timestamp, now) else None
                elif stream_kind == "depth":
                    frame = self._latest_depth_jpeg if self._is_recent_timestamp(self._latest_depth_timestamp, now) else None
                else:
                    frame = None
                    if self._is_recent_timestamp(self._latest_annotated_timestamp, now):
                        frame = self._latest_jpeg
                    elif self._is_recent_timestamp(self._latest_rgb_timestamp, now):
                        frame = self._latest_rgb_jpeg
            if not frame:
                frame = self._placeholder_jpeg(stream_kind)
            if frame:
                yield (
                    b"--frame\r\n"
                    b"Content-Type: image/jpeg\r\n\r\n" + frame + b"\r\n"
                )
            time.sleep(stream_interval if stream_interval > 0.0 else 0.005)

    def get_latest_hand_target(self) -> Dict[str, Any] | None:
        with self._frame_lock:
            return dict(self._latest_hand_target) if self._latest_hand_target else None

    def get_stream_health(self) -> Dict[str, Any]:
        now = time.time()
        with self._frame_lock:
            rgb_age_s = now - self._latest_rgb_timestamp if self._latest_rgb_timestamp else None
            depth_age_s = now - self._latest_depth_timestamp if self._latest_depth_timestamp else None
            rgb_ok = rgb_age_s is not None and rgb_age_s < float(self.config.ros_camera_stall_timeout_s)
            depth_ok = depth_age_s is not None and depth_age_s < float(self.config.ros_camera_stall_timeout_s)
            return {
                "camera_open": bool(rgb_ok),
                "rgb_ok": bool(rgb_ok),
                "depth_ok": bool(depth_ok),
                "rgb_age_s": round(rgb_age_s, 3) if rgb_age_s is not None else None,
                "depth_age_s": round(depth_age_s, 3) if depth_age_s is not None else None,
                "camera_info_age_s": round(now - self._latest_camera_info_timestamp, 3) if self._latest_camera_info_timestamp else None,
                "frame_id": self._latest_frame_id,
                "depth_id": self._latest_depth_id,
            }

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
        now = time.time()
        with self._frame_lock:
            report = dict(self._latest_depth_report)
            report["points"] = [dict(point) for point in self._latest_depth_report.get("points", [])]
            if not self._is_recent_timestamp(self._latest_depth_timestamp, now, float(self.config.ros_camera_stall_timeout_s)):
                report = self._empty_depth_report()
            report["depth_ok"] = self._depth_stream_ok
            report["rgb_ok"] = self._rgb_stream_ok
            report["intrinsics_ok"] = self._intrinsics_ok()
            report["alignment_ok"] = self._alignment_ok()
            report["camera_to_robot"] = self._camera_transform_summary()
            report["tcp_to_camera"] = self._camera_transform_summary()
            report["depth_compensation"] = self._depth_compensation_summary()
            report["topics"] = dict(self.calibration.topics)
            report["model_loaded"] = bool(self._model)
            report["model_path"] = str(self.config.vision_model_path)
            report["vision_inference_fps"] = getattr(self.config, "vision_inference_fps", 8.0)
            report["vision_preview_fps"] = getattr(self.config, "vision_preview_fps", 8.0)
            report["vision_depth_preview_fps"] = getattr(self.config, "vision_depth_preview_fps", 4.0)
            report["camera_process_running"] = self._camera_process_running()
            report["camera_auto_launch"] = bool(self.config.ros_auto_launch_camera)
            report["camera_launch_error"] = self._last_camera_launch_error
            report["camera_recovery"] = self._camera_recovery_summary(now)
            report["detector_mode"] = self._detector_mode()
            report["yolo_enabled"] = True
        return report

    def get_hand_follow_calibration(self) -> Dict[str, Any]:
        with self._frame_lock:
            plane_z_mm = float(self.calibration.hand_follow.plane_z_mm)
            offset_mm = float(getattr(self.config, "hand_follow_z_offset_mm", 200.0))
            return {
                "ok": True,
                "plane_z_mm": round(plane_z_mm, 3),
                "z_offset_mm": round(offset_mm, 3),
                "target_z_mm": round(plane_z_mm + offset_mm, 3),
                "workspace": dict(self.calibration.hand_follow.workspace or self.config.hand_workspace),
            }

    def set_hand_follow_calibration(self, plane_z_mm: Any, save: bool = False) -> Dict[str, Any]:
        try:
            parsed_plane_z_mm = float(plane_z_mm)
        except (TypeError, ValueError):
            raise ValueError("plane_z_mm debe ser numerico.")

        with self._frame_lock:
            self.calibration.hand_follow.plane_z_mm = parsed_plane_z_mm
            self.calibration.hand_follow.workspace = dict(self.config.hand_workspace)
            summary = self.get_hand_follow_calibration()
            if save:
                save_calibration(self.config.calibration_file(), self.calibration)
                summary["saved"] = True
            else:
                summary["saved"] = False
            return summary

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
                "tcp_to_camera": self._camera_transform_summary(),
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
            self.calibration.tcp_to_camera.translation_mm = [float(value) for value in translation_mm[:3]]
            self.calibration.tcp_to_camera.rotation_rpy_deg = [float(value) for value in rotation_rpy_deg[:3]]
            self.calibration.tcp_to_camera.matrix_4x4 = build_transform_matrix(
                self.calibration.tcp_to_camera.translation_mm,
                self.calibration.tcp_to_camera.rotation_rpy_deg,
            )
            self.calibration.tcp_to_camera.configured = True
            self.calibration.camera_to_robot.translation_mm = list(self.calibration.tcp_to_camera.translation_mm)
            self.calibration.camera_to_robot.rotation_rpy_deg = list(self.calibration.tcp_to_camera.rotation_rpy_deg)
            self.calibration.camera_to_robot.matrix_4x4 = list(self.calibration.tcp_to_camera.matrix_4x4)
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

        try:
            self._rclpy = rclpy
            self._ensure_rclpy_initialized()

            bridge = CvBridge()
            self._ros_node = rclpy.create_node("silvia_hp60c_bridge")
            self._ros_subscriptions = [
                self._ros_node.create_subscription(Image, self.config.ros_rgb_topic, self._make_rgb_callback(bridge), 2),
                self._ros_node.create_subscription(Image, self.config.ros_depth_topic, self._make_depth_callback(bridge), 2),
                self._ros_node.create_subscription(CameraInfo, self.config.ros_rgb_camera_info_topic, self._make_camera_info_callback("rgb"), 2),
                self._ros_node.create_subscription(CameraInfo, self.config.ros_depth_camera_info_topic, self._make_camera_info_callback("depth"), 2),
            ]
            external_camera_processes = self._find_external_camera_processes()
            healthy_external_camera_present = any(
                not self._should_replace_stale_external_camera_process(process_info)
                for process_info in external_camera_processes
            )
            initial_wait_s = max(0.0, float(self.config.ros_camera_wait_timeout_s)) if healthy_external_camera_present else 0.0
            self._next_camera_launch_attempt_at = time.time() + initial_wait_s
            self._camera_recovery_state = "starting" if self.config.ros_auto_launch_camera else "failed"
        except Exception as error:  # pragma: no cover - depends on ROS runtime
            self._last_capture_error = f"No se pudo inicializar ROS2 HP60C: {error}"
            self._publish_status(ok=False, message=self._last_capture_error)
            logger.exception(self._last_capture_error)
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
            return

        while self._running and not self._stop_event.is_set():
            try:
                rclpy.spin_once(self._ros_node, timeout_sec=0.1)
            except Exception as error:  # pragma: no cover - depends on ROS runtime
                error_detail = str(error).strip() or error.__class__.__name__
                error_text = error_detail.lower()
                if (
                    not self._running
                    or self._stop_event.is_set()
                    or "externalshutdownexception" in error_text
                    or "context is not valid" in error_text
                    or "rcl_shutdown() was called" in error_text
                ):
                    break
                self._last_capture_error = f"ROS2 fallo al leer la HP60C: {error_detail}"
                self._publish_status(ok=False, message=self._last_capture_error)
                logger.warning(self._last_capture_error)
                time.sleep(0.2)
                continue

            now = time.time()
            if self._camera_process_started_by_service and self._camera_process is not None and self._camera_process.poll() is not None:
                exit_code = self._camera_process.poll()
                driver_issue = self._latest_camera_process_issue()
                self._camera_recovery_state = "restarting"
                self._stop_camera_process()
                self._schedule_camera_retry(
                    now=now,
                    reason=f"ascamera_node termino con codigo {exit_code}.",
                    driver_issue=driver_issue or f"Codigo de salida {exit_code}.",
                )
                logger.warning(self._last_camera_launch_error)

            self._refresh_stream_health(now)
            self._refresh_camera_recovery(now)
            if (
                self.config.ros_auto_launch_camera
                and not (self._rgb_stream_ok and self._depth_stream_ok)
                and not self._camera_process_running()
                and now >= self._next_camera_launch_attempt_at
            ):
                self._maybe_launch_camera_node()
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
            import torch
            from ultralytics import YOLO
        except Exception as error:  # pragma: no cover - dependency fallback
            self._last_inference_error = f"Ultralytics/YOLO no disponible: {error}"
            self._publish_status(ok=False, message=self._last_inference_error)
            logger.warning("VisionService sin YOLO: %s", error)
            torch = None
            YOLO = None

        self._torch = torch
        self._yolo = YOLO
        if self._yolo is not None:
            self._configure_inference_device()
            self._load_model()
            if self._is_hand_only_mode():
                logger.info("VisionService en modo hand_only: seguimiento de mano por YOLO.")
        self._hand_landmarker = None
        self._mp = None
        self._mp_vision = None
        self._last_hand_error = ""
        self._hand_landmarker_delegate = "disabled"
        self._hand_landmarker_delegate_reason = "MediaPipe desactivado. Seguimiento por YOLO."

        last_processed_frame_id = 0
        inference_interval_s = self._fps_interval(getattr(self.config, "vision_inference_fps", 8.0))
        next_inference_at = 0.0
        while self._running and not self._stop_event.is_set():
            now_monotonic = time.monotonic()
            if inference_interval_s > 0.0 and now_monotonic < next_inference_at:
                time.sleep(min(0.02, next_inference_at - now_monotonic))
                continue

            frame = None
            frame_id = 0
            with self._frame_lock:
                if self._latest_frame is not None and self._latest_frame_id != last_processed_frame_id:
                    frame = self._latest_frame.copy()
                    frame_id = self._latest_frame_id

            if frame is None:
                time.sleep(0.01)
                continue
            if self._cv2 is None:
                time.sleep(0.01)
                continue

            processing_started_at = time.monotonic()
            mode = self.state.snapshot().get("mode", "idle")
            hand_tracking_active = self._is_hand_only_mode() or mode == "hand_follow"
            if hand_tracking_active:
                detections, plotted_frame = self._detect_with_yolo(
                    frame,
                    frame_id=frame_id,
                    include_hand=True,
                    hand_only=self._is_hand_only_mode(),
                )
                hand_target = self._select_hand_target(detections)
                detector_mode = "yolo_hand_only" if self._is_hand_only_mode() else "yolo_hand_follow"
            else:
                detections, plotted_frame = self._detect_with_yolo(frame, frame_id=frame_id)
                hand_target = None
                detector_mode = "yolo_objects"

            annotated = self._annotate_frame(plotted_frame, detections, hand_target, detector_mode)
            success, buffer = self._cv2.imencode(".jpg", annotated)

            with self._frame_lock:
                self._latest_detections = detections
                self._latest_hand_target = hand_target
                if success and buffer is not None:
                    self._latest_jpeg = buffer.tobytes()
                    self._latest_annotated_timestamp = time.time()

            self.state.set_detections(detections)
            self.state.set_hand_target(hand_target)
            self._publish_status(ok=True)
            last_processed_frame_id = frame_id
            if inference_interval_s > 0.0:
                next_inference_at = processing_started_at + inference_interval_s

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
        if self._camera_recovery_state in {"stalled", "restarting"}:
            lines.append("Recuperando HP60C...")
        elif self._camera_recovery_state == "failed" and self._next_camera_launch_attempt_at > time.time():
            seconds_left = max(0.0, self._next_camera_launch_attempt_at - time.time())
            lines.append(f"HP60C fallida. Reintento en {seconds_left:.1f}s.")
        elif self._camera_process_running():
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
        if self._is_hand_only_mode():
            lines.append("Detector: YOLO mano solamente")
        else:
            lines.append("Detector: YOLO objetos/mano")
        lines.append(f"Inferencia: {self._inference_device} @ {getattr(self.config, 'vision_inference_fps', 8.0):.1f} FPS")
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
            self._camera_recovery_state = "failed"
            logger.warning(self._last_camera_launch_error)
            return
        healthy_external_process = None
        for external_process in self._find_external_camera_processes():
            if self._should_replace_stale_external_camera_process(external_process):
                external_pid = int(external_process["pid"])
                logger.warning(
                    "Se detecto ascamera externo estancado (pid=%s, stdio=%s/%s). "
                    "SILVIA lo reemplazara para recuperar la HP60C.",
                    external_pid,
                    external_process.get("stdout", "") or "?",
                    external_process.get("stderr", "") or "?",
                )
                if not self._terminate_external_camera_process(external_pid):
                    self._last_camera_launch_error = (
                        "Se detecto un ascamera externo estancado, pero no se pudo terminar "
                        f"(pid={external_pid}). Cerralo manualmente y vuelve a intentar."
                    )
                    logger.warning(self._last_camera_launch_error)
                    return
                continue
            healthy_external_process = external_process
            break

        if healthy_external_process is not None:
            reason = (
                "Se detecto otro proceso ascamera ya corriendo. "
                "SILVIA no lanzara un segundo nodo para no pelear por la HP60C. "
                "Si el bundle publica en otro namespace, ajusta ROS_RGB_TOPIC/ROS_DEPTH_TOPIC "
                "o desactiva ROS_AUTO_LAUNCH_CAMERA."
            )
            self._last_camera_launch_error = reason
            self._camera_recovery_state = "failed"
            self._next_camera_launch_attempt_at = time.time() + max(
                1.0,
                float(getattr(self.config, "ros_camera_restart_delay_s", 3.0)),
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
            f"color_pcl:={'true' if self.config.ros_camera_color_pcl else 'false'}",
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
            log_handle = self._open_camera_process_log()
            self._camera_process_log_offset = log_handle.tell()
            process_cwd = self.config.ros_workspace_dir if self.config.ros_workspace_dir.exists() else self.config.repo_dir
            self._camera_process = subprocess.Popen(
                command,
                cwd=str(process_cwd),
                stdout=log_handle,
                stderr=subprocess.STDOUT,
                env=dict(os.environ),
                start_new_session=True,
            )
            self._camera_process_started_by_service = True
            self._camera_process_started_at = time.time()
            self._camera_recovery_state = "starting"
            self._camera_stalled_since_at = 0.0
            self._camera_healthy_since_at = 0.0
            self._last_no_frames_warning_at = 0.0
            self._next_camera_launch_attempt_at = 0.0
            self._last_camera_launch_error = ""
            self._last_capture_error = (
                "Nodo HP60C iniciado por SILVIA. "
                f"Logs en {self._camera_process_log_path}. Esperando primeros frames RGB."
            )
            logger.info("ascamera_node iniciado automaticamente desde SILVIA con logs en %s.", self._camera_process_log_path)
        except Exception as error:
            self._camera_process = None
            self._camera_process_started_by_service = False
            self._schedule_camera_retry(
                now=time.time(),
                reason=f"No se pudo iniciar ascamera_node: {error}",
                driver_issue=str(error),
            )
            logger.warning(self._last_camera_launch_error)
            self._close_camera_process_log()

    def _stop_camera_process(self) -> None:
        if self._camera_process is None:
            self._stop_stale_managed_camera_processes()
            self._close_camera_process_log()
            self._camera_process_log_offset = 0
            return
        process = self._camera_process
        process_group_id = None
        try:
            process_group_id = os.getpgid(process.pid)
        except OSError:
            process_group_id = None
        if process.poll() is None:
            try:
                if process_group_id == process.pid:
                    os.killpg(process_group_id, signal.SIGTERM)
                else:
                    process.terminate()
                process.wait(timeout=2.0)
            except Exception:
                try:
                    if process_group_id == process.pid:
                        os.killpg(process_group_id, signal.SIGKILL)
                    else:
                        process.kill()
                except Exception:
                    pass
                try:
                    process.wait(timeout=1.0)
                except Exception:
                    pass
        self._camera_process = None
        self._camera_process_started_by_service = False
        self._camera_process_started_at = 0.0
        self._stop_stale_managed_camera_processes()
        self._close_camera_process_log()
        self._camera_process_log_offset = 0

    def _open_camera_process_log(self):
        if self._camera_process_log_handle is not None and not self._camera_process_log_handle.closed:
            return self._camera_process_log_handle
        self.config.log_dir.mkdir(parents=True, exist_ok=True)
        self._camera_process_log_handle = self._camera_process_log_path.open("ab")
        return self._camera_process_log_handle

    def _close_camera_process_log(self) -> None:
        if self._camera_process_log_handle is None:
            return
        try:
            self._camera_process_log_handle.close()
        except Exception:
            pass
        self._camera_process_log_handle = None

    def _camera_backoff_sequence(self) -> List[float]:
        raw_sequence = list(getattr(self.config, "ros_camera_backoff_sequence_s", [2.0, 5.0, 10.0, 20.0]) or [])
        normalized = []
        for value in raw_sequence:
            try:
                normalized.append(max(1.0, float(value)))
            except (TypeError, ValueError):
                continue
        return normalized or [2.0, 5.0, 10.0, 20.0]

    def _current_camera_backoff_delay(self) -> float:
        sequence = self._camera_backoff_sequence()
        index = min(self._camera_backoff_index, len(sequence) - 1)
        return sequence[index]

    def _schedule_camera_retry(self, now: float, reason: str, driver_issue: str = "") -> None:
        delay_s = self._current_camera_backoff_delay()
        self._camera_restart_count += 1
        self._camera_recovery_state = "failed"
        self._camera_last_restart_reason = reason
        if driver_issue:
            self._camera_last_driver_issue = driver_issue
        self._camera_stalled_since_at = 0.0
        self._camera_healthy_since_at = 0.0
        self._next_camera_launch_attempt_at = now + delay_s
        self._last_camera_launch_error = f"HP60C fallida, reintentando en {delay_s:.1f}s. Motivo: {reason}"
        sequence = self._camera_backoff_sequence()
        self._camera_backoff_index = min(self._camera_backoff_index + 1, len(sequence) - 1)

    def _perform_camera_hard_restart(self, now: float, reason: str, driver_issue: str = "") -> None:
        self._camera_recovery_state = "restarting"
        self._camera_last_restart_reason = reason
        if driver_issue:
            self._camera_last_driver_issue = driver_issue
        logger.warning("Reiniciando HP60C por watchdog: %s", reason)
        self._clear_live_stream_state(clear_timestamps=False)
        self._stop_camera_process()
        self._schedule_camera_retry(now=now, reason=reason, driver_issue=driver_issue)

    def _clear_live_stream_state(self, clear_timestamps: bool = False) -> None:
        with self._frame_lock:
            self._latest_frame = None
            self._latest_depth_frame = None
            self._latest_jpeg = None
            self._latest_rgb_jpeg = None
            self._latest_depth_jpeg = None
            self._latest_detections = []
            self._latest_hand_target = None
            self._latest_depth_report = self._empty_depth_report()
            self._camera_open = False
            self._rgb_stream_ok = False
            self._depth_stream_ok = False
            self._last_rgb_preview_monotonic = 0.0
            self._last_depth_preview_monotonic = 0.0
            self._logged_first_rgb_frame = False
            self._logged_first_depth_frame = False
            if clear_timestamps:
                self._latest_rgb_timestamp = 0.0
                self._latest_depth_timestamp = 0.0
                self._latest_camera_info_timestamp = 0.0
                self._latest_annotated_timestamp = 0.0
        self.state.set_detections([])
        self.state.set_hand_target(None)

    def _refresh_camera_recovery(self, now: float) -> None:
        stream_timeout_s = max(1.0, float(getattr(self.config, "ros_camera_stall_timeout_s", 6.0)))
        startup_grace_s = max(stream_timeout_s, float(getattr(self.config, "ros_camera_startup_grace_s", 15.0)))
        camera_info_timeout_s = max(1.0, float(getattr(self.config, "ros_camera_no_camera_info_timeout_s", 10.0)))
        hard_restart_timeout_s = max(stream_timeout_s, float(getattr(self.config, "ros_camera_hard_restart_timeout_s", 15.0)))
        healthy_reset_s = max(stream_timeout_s, float(getattr(self.config, "ros_camera_healthy_reset_s", 30.0)))

        rgb_age_s = self._seconds_since(self._latest_rgb_timestamp, now)
        depth_age_s = self._seconds_since(self._latest_depth_timestamp, now)
        camera_info_age_s = self._seconds_since(self._latest_camera_info_timestamp, now)
        stream_stale = (
            rgb_age_s is None
            or depth_age_s is None
            or rgb_age_s >= stream_timeout_s
            or depth_age_s >= stream_timeout_s
        )
        camera_info_stale = camera_info_age_s is None or camera_info_age_s >= camera_info_timeout_s

        if self._camera_process_started_by_service and self._camera_process_running():
            running_for_s = max(0.0, now - self._camera_process_started_at)
            if not stream_stale:
                if self._camera_recovery_state != "healthy":
                    self._camera_recovery_state = "healthy"
                    self._camera_healthy_since_at = now
                elif self._camera_healthy_since_at <= 0.0:
                    self._camera_healthy_since_at = now
                if self._camera_healthy_since_at > 0.0 and (now - self._camera_healthy_since_at) >= healthy_reset_s:
                    self._camera_backoff_index = 0
                self._camera_stalled_since_at = 0.0
                self._last_camera_launch_error = ""
                return

            self._camera_healthy_since_at = 0.0
            if running_for_s < startup_grace_s:
                self._camera_recovery_state = "starting"
                return

            # --- USB re-enumeration detection ---
            # The HP60C firmware re-enumerates the USB device after opening
            # while configuring depth parameters via XU commands.  During
            # re-enumeration, the ascamera SDK logs "onAttached" + "this
            # camera exist" and frames temporarily pause.  Give the camera
            # some extra time to recover, but NOT indefinitely — if the
            # stream endpoint gets orphaned the node must be restarted.
            reenum_max_wait_s = max(
                hard_restart_timeout_s,
                float(getattr(self.config, "ros_camera_hard_restart_timeout_s", 15.0)),
            )
            if self._is_usb_reenumeration_in_progress():
                if self._camera_recovery_state != "reenumerating":
                    self._camera_recovery_state = "reenumerating"
                    if self._camera_stalled_since_at <= 0.0:
                        self._camera_stalled_since_at = now
                    logger.info(
                        "HP60C USB re-enumeracion detectada. "
                        "Esperando hasta %.0fs para que el stream se restablezca.",
                        reenum_max_wait_s,
                    )
                # Allow the re-enumeration grace period, but enforce a cap
                stalled_for_s = now - self._camera_stalled_since_at if self._camera_stalled_since_at > 0.0 else 0.0
                if stalled_for_s < reenum_max_wait_s:
                    return
                logger.warning(
                    "HP60C no se recupero tras %.1fs de re-enumeracion USB. "
                    "Forzando reinicio del nodo.",
                    stalled_for_s,
                )

            driver_issue = self._latest_camera_process_issue()
            if driver_issue:
                self._camera_last_driver_issue = driver_issue
            if self._camera_recovery_state not in {"stalled", "reenumerating"}:
                self._camera_recovery_state = "stalled"
                self._camera_stalled_since_at = now
                self._clear_live_stream_state(clear_timestamps=False)
            elif self._camera_stalled_since_at <= 0.0:
                self._camera_stalled_since_at = now

            should_restart = (
                (camera_info_stale and stream_stale)
                or ((now - self._camera_stalled_since_at) >= hard_restart_timeout_s)
            )
            if should_restart:
                if camera_info_stale and stream_stale:
                    reason = "Sin camera_info reciente y sin frames RGB/depth desde la HP60C."
                else:
                    reason = "ascamera_node sigue vivo pero el stream ROS2 quedo estancado."
                self._perform_camera_hard_restart(now=now, reason=reason, driver_issue=driver_issue)
                return

            if (now - self._last_no_frames_warning_at) >= 5.0:
                if driver_issue:
                    logger.warning(
                        "HP60C estancada: sin frames recientes y ascamera reporta %s",
                        driver_issue,
                    )
                else:
                    logger.warning("HP60C estancada: sin frames RGB/depth recientes.")
                self._last_no_frames_warning_at = now
            return

        if self._rgb_stream_ok and self._depth_stream_ok:
            if self._camera_recovery_state != "healthy":
                self._camera_recovery_state = "healthy"
                self._camera_healthy_since_at = now
            elif self._camera_healthy_since_at <= 0.0:
                self._camera_healthy_since_at = now
            if self._camera_healthy_since_at > 0.0 and (now - self._camera_healthy_since_at) >= healthy_reset_s:
                self._camera_backoff_index = 0
            self._camera_stalled_since_at = 0.0
            return

        self._camera_healthy_since_at = 0.0
        if self.config.ros_auto_launch_camera:
            self._camera_recovery_state = "failed"

    def _camera_recovery_summary(self, now: float | None = None) -> Dict[str, Any]:
        now_s = now or time.time()
        next_retry_in_s = None
        if self._camera_recovery_state == "failed" and self._next_camera_launch_attempt_at > now_s:
            next_retry_in_s = round(max(0.0, self._next_camera_launch_attempt_at - now_s), 3)
        managed_process_pid = None
        if self._camera_process_running() and self._camera_process is not None:
            managed_process_pid = int(self._camera_process.pid)
        return {
            "state": self._camera_recovery_state,
            "restart_count": int(self._camera_restart_count),
            "last_restart_reason": self._camera_last_restart_reason,
            "last_driver_issue": self._camera_last_driver_issue,
            "seconds_since_rgb_frame": self._rounded_seconds_since(self._latest_rgb_timestamp, now_s),
            "seconds_since_depth_frame": self._rounded_seconds_since(self._latest_depth_timestamp, now_s),
            "seconds_since_camera_info": self._rounded_seconds_since(self._latest_camera_info_timestamp, now_s),
            "managed_process_pid": managed_process_pid,
            "next_retry_in_s": next_retry_in_s,
        }

    def _empty_depth_report(self) -> Dict[str, Any]:
        return {
            "source": "hp60c_ros2",
            "timestamp_ms": 0,
            "points": [],
            "average_depth_mm": None,
            "valid_points": 0,
            "span_ratio": self.config.depth_sampling_ratio,
        }

    def _find_external_camera_processes(self) -> List[Dict[str, Any]]:
        current_pid = os.getpid()
        managed_pid = self._camera_process.pid if self._camera_process is not None else None
        current_net_namespace = self._safe_process_namespace(Path("/proc/self"), "net")
        processes: List[Dict[str, Any]] = []
        for proc_dir in Path("/proc").iterdir():
            if not proc_dir.name.isdigit():
                continue
            pid = int(proc_dir.name)
            if pid in {current_pid, managed_pid}:
                continue
            process_net_namespace = self._safe_process_namespace(proc_dir, "net")
            if current_net_namespace and process_net_namespace and process_net_namespace != current_net_namespace:
                continue
            try:
                raw_cmdline = (proc_dir / "cmdline").read_bytes()
            except OSError:
                continue
            cmdline = raw_cmdline.replace(b"\x00", b" ").decode("utf-8", errors="ignore").strip().lower()
            if not cmdline:
                continue
            if "ascamera_node" in cmdline or "ros2 launch ascamera" in cmdline or "ros2 run ascamera" in cmdline:
                processes.append(
                    {
                    "pid": pid,
                    "cmdline": cmdline,
                    "stdout": self._safe_process_fd_target(proc_dir, 1),
                    "stderr": self._safe_process_fd_target(proc_dir, 2),
                    "net_namespace": process_net_namespace,
                    }
                )
        processes.sort(key=lambda process_info: int(process_info.get("pid", 0)))
        return processes

    def _find_external_camera_process(self) -> Dict[str, Any] | None:
        processes = self._find_external_camera_processes()
        return processes[0] if processes else None

    def _external_camera_process_present(self) -> bool:
        return self._find_external_camera_process() is not None

    def _should_replace_stale_external_camera_process(self, process_info: Dict[str, Any]) -> bool:
        cmdline = str(process_info.get("cmdline", "") or "")
        stdout_target = str(process_info.get("stdout", "") or "")
        stderr_target = str(process_info.get("stderr", "") or "")
        config_path = str(self.config.ros_ascamera_config_path).lower()
        looks_like_old_silvia = config_path in cmdline and "color_pcl:=true" in cmdline
        looks_like_current_silvia = config_path in cmdline and (
            stdout_target == str(self._camera_process_log_path) or stderr_target == str(self._camera_process_log_path)
        )
        stdio_is_blackholed = stdout_target == "/dev/null" and stderr_target == "/dev/null"
        return looks_like_current_silvia or (looks_like_old_silvia and stdio_is_blackholed)

    def _stop_stale_managed_camera_processes(self) -> None:
        for process_info in self._find_external_camera_processes():
            if not self._should_replace_stale_external_camera_process(process_info):
                continue
            self._terminate_external_camera_process(int(process_info["pid"]))

    @staticmethod
    def _safe_process_fd_target(proc_dir: Path, fd: int) -> str:
        try:
            return os.path.realpath(proc_dir / "fd" / str(fd))
        except OSError:
            return ""

    @staticmethod
    def _safe_process_namespace(proc_dir: Path, namespace: str) -> str:
        try:
            return os.path.realpath(proc_dir / "ns" / namespace)
        except OSError:
            return ""

    def _terminate_external_camera_process(self, pid: int, timeout_s: float = 3.0) -> bool:
        try:
            os.kill(pid, signal.SIGTERM)
        except ProcessLookupError:
            return True
        except OSError as error:
            logger.warning("No se pudo enviar SIGTERM a ascamera externo pid=%s: %s", pid, error)
            return False

        if self._wait_for_process_exit(pid, timeout_s):
            return True

        try:
            os.kill(pid, signal.SIGKILL)
        except ProcessLookupError:
            return True
        except OSError as error:
            logger.warning("No se pudo enviar SIGKILL a ascamera externo pid=%s: %s", pid, error)
            return False

        return self._wait_for_process_exit(pid, 1.0)

    @staticmethod
    def _wait_for_process_exit(pid: int, timeout_s: float) -> bool:
        deadline = time.time() + max(0.1, float(timeout_s))
        proc_path = Path("/proc") / str(pid)
        while time.time() < deadline:
            if not proc_path.exists():
                return True
            time.sleep(0.05)
        return not proc_path.exists()

    def _detect_with_yolo(
        self,
        frame,
        frame_id: int = 0,
        include_hand: bool = False,
        hand_only: bool = False,
    ) -> tuple[List[Dict[str, Any]], Any]:
        if self._model is None:
            return [], frame

        inference_frame = frame.copy()

        try:
            detected_at_s = time.time()
            use_half = self._inference_device.startswith("cuda")
            results = self._model(
                inference_frame,
                conf=self.config.vision_confidence_threshold,
                device=self._inference_device,
                half=use_half,
                verbose=False,
            )
        except Exception as error:  # pragma: no cover - dependency fallback
            self._last_inference_error = f"YOLO fallo: {error}"
            logger.warning(self._last_inference_error)
            self._publish_status(ok=False, message=self._last_inference_error)
            return [], frame

        result = results[0]
        plotted_frame = frame.copy()
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
            is_hand = self._is_hand_label(label)
            if hand_only and not is_hand:
                continue
            if is_hand and not include_hand:
                continue
            detections.append(
                self._build_detection(
                    label,
                    confidence,
                    box,
                    frame_id=frame_id,
                    detected_at_s=detected_at_s,
                )
            )

        self._last_inference_error = ""
        return detections, plotted_frame

    def _load_hand_landmarker(self) -> None:
        model_path = self.config.hand_landmarker_model_path
        if not model_path.is_absolute():
            model_path = self.config.repo_dir / model_path

        if not model_path.exists():
            self._last_hand_error = f"No se encontro el modelo MediaPipe en {model_path}."
            self._hand_landmarker_delegate_reason = self._last_hand_error
            logger.warning(self._last_hand_error)
            return

        self._apply_mediapipe_cpp_log_level()

        try:
            import mediapipe as mp
            from mediapipe.tasks import python
            from mediapipe.tasks.python import vision
        except Exception as error:  # pragma: no cover - dependency fallback
            self._hand_landmarker = None
            self._last_hand_error = f"No se pudo importar MediaPipe HandLandmarker: {error}"
            self._hand_landmarker_delegate_reason = self._last_hand_error
            logger.warning(self._last_hand_error)
            return

        requested_delegate = str(getattr(self.config, "hand_landmarker_delegate", "auto") or "auto").strip().lower()
        if requested_delegate not in {"auto", "gpu", "cpu"}:
            requested_delegate = "auto"
        last_error = ""
        for delegate in self._hand_delegate_candidates(requested_delegate):
            try:
                options = vision.HandLandmarkerOptions(
                    base_options=self._mediapipe_base_options(python, model_path, delegate),
                    running_mode=vision.RunningMode.VIDEO,
                    num_hands=1,
                    min_hand_detection_confidence=0.5,
                    min_hand_presence_confidence=0.5,
                    min_tracking_confidence=0.5,
                )
                self._hand_landmarker = vision.HandLandmarker.create_from_options(options)
                self._mp = mp
                self._mp_vision = vision
                self._hand_landmarker_delegate = delegate
                if delegate == requested_delegate:
                    self._hand_landmarker_delegate_reason = f"MediaPipe usando delegate {delegate}."
                elif requested_delegate == "auto":
                    self._hand_landmarker_delegate_reason = f"MediaPipe auto eligio delegate {delegate}."
                else:
                    self._hand_landmarker_delegate_reason = (
                        f"MediaPipe no pudo usar {requested_delegate}; usando {delegate}."
                    )
                self._last_hand_error = ""
                logger.info(
                    "MediaPipe HandLandmarker cargado desde %s con delegate=%s.",
                    model_path,
                    delegate,
                )
                return
            except Exception as error:  # pragma: no cover - depends on MediaPipe runtime
                last_error = str(error)
                logger.warning("MediaPipe HandLandmarker no cargo con delegate=%s: %s", delegate, error)

        self._hand_landmarker = None
        self._last_hand_error = f"No se pudo cargar MediaPipe HandLandmarker: {last_error}"
        self._hand_landmarker_delegate_reason = self._last_hand_error
        logger.warning(self._last_hand_error)

    @staticmethod
    def _hand_delegate_candidates(requested_delegate: str) -> List[str]:
        if requested_delegate == "cpu":
            return ["cpu"]
        if requested_delegate == "gpu":
            return ["gpu", "cpu"]
        return ["gpu", "cpu"]

    @staticmethod
    def _apply_mediapipe_cpp_log_level() -> None:
        raw_level = str(os.getenv("MEDIAPIPE_CPP_MIN_LOG_LEVEL", "2") or "2").strip().lower()
        aliases = {
            "debug": "0",
            "info": "0",
            "warning": "1",
            "warn": "1",
            "error": "2",
            "fatal": "3",
            "off": "3",
        }
        level = aliases.get(raw_level, raw_level)
        if level not in {"0", "1", "2", "3"}:
            level = "2"

        os.environ["MEDIAPIPE_CPP_MIN_LOG_LEVEL"] = level
        os.environ.setdefault("GLOG_minloglevel", level)
        os.environ.setdefault("ABSL_MIN_LOG_LEVEL", level)
        os.environ.setdefault("TF_CPP_MIN_LOG_LEVEL", level)

    @staticmethod
    def _mediapipe_base_options(python_module, model_path, delegate: str):
        base_options = python_module.BaseOptions
        delegate_enum = getattr(base_options, "Delegate", None)
        if delegate_enum is None:
            return base_options(model_asset_path=str(model_path))
        if delegate == "gpu":
            return base_options(model_asset_path=str(model_path), delegate=delegate_enum.GPU)
        return base_options(model_asset_path=str(model_path), delegate=delegate_enum.CPU)

    def _detect_hand_with_mediapipe(self, frame, frame_id: int) -> Dict[str, Any] | None:
        if self._hand_landmarker is None or self._mp is None or self._cv2 is None:
            return None

        try:
            detected_at_s = time.time()
            rgb_frame = self._cv2.cvtColor(frame, self._cv2.COLOR_BGR2RGB)
            mp_image = self._mp.Image(image_format=self._mp.ImageFormat.SRGB, data=rgb_frame)
            timestamp_ms = max(self._last_hand_timestamp_ms + 1, int(detected_at_s * 1000), int(frame_id))
            self._last_hand_timestamp_ms = timestamp_ms
            result = self._hand_landmarker.detect_for_video(mp_image, timestamp_ms)
        except Exception as error:  # pragma: no cover - depends on MediaPipe runtime
            self._last_hand_error = f"MediaPipe mano fallo: {error}"
            logger.warning(self._last_hand_error)
            return None

        hand_landmarks = list(getattr(result, "hand_landmarks", []) or [])
        if not hand_landmarks:
            self._last_hand_error = ""
            return None

        handedness = list(getattr(result, "handedness", []) or [])
        self._last_hand_error = ""
        return self._build_mediapipe_hand_target(
            hand_landmarks=hand_landmarks[0],
            handedness=handedness[0] if handedness else [],
            frame_shape=frame.shape,
            frame_id=frame_id,
            detected_at_s=detected_at_s,
        )

    def _build_mediapipe_hand_target(
        self,
        hand_landmarks,
        handedness,
        frame_shape,
        frame_id: int | None = None,
        detected_at_s: float | None = None,
    ) -> Dict[str, Any] | None:
        if len(hand_landmarks) <= MIDDLE_FINGER_MCP_INDEX:
            return None

        height, width = frame_shape[:2]
        landmark = hand_landmarks[MIDDLE_FINGER_MCP_INDEX]
        center_x = self._clamp(float(getattr(landmark, "x", 0.0)) * width, 0.0, max(0.0, width - 1.0))
        center_y = self._clamp(float(getattr(landmark, "y", 0.0)) * height, 0.0, max(0.0, height - 1.0))

        xs = [
            self._clamp(float(getattr(item, "x", 0.0)) * width, 0.0, max(0.0, width - 1.0))
            for item in hand_landmarks
        ]
        ys = [
            self._clamp(float(getattr(item, "y", 0.0)) * height, 0.0, max(0.0, height - 1.0))
            for item in hand_landmarks
        ]
        box = [min(xs), min(ys), max(xs), max(ys)]
        span_x = max(1, int(max(12.0, abs(box[2] - box[0])) * self.config.depth_sampling_ratio))
        span_y = max(1, int(max(12.0, abs(box[3] - box[1])) * self.config.depth_sampling_ratio))

        with self._frame_lock:
            depth_frame = None if self._latest_depth_frame is None else self._latest_depth_frame.copy()
            rgb_intrinsics = self.calibration.intrinsics["rgb"].to_dict()
            alignment_ok = self._alignment_ok()

        depth_payload = self._compute_point_depth(
            point_xy=(center_x, center_y),
            span_x=span_x,
            span_y=span_y,
            depth_frame=depth_frame,
            rgb_intrinsics=rgb_intrinsics,
            alignment_ok=alignment_ok,
        )

        confidence = 1.0
        handedness_label = ""
        if handedness:
            category = handedness[0]
            confidence = float(getattr(category, "score", confidence) or confidence)
            handedness_label = str(getattr(category, "category_name", "") or "")

        timestamp_s = float(detected_at_s or time.time())
        hand_calibration = self.get_hand_follow_calibration()
        return {
            "type": "hand",
            "label": "Mano",
            "confidence": round(confidence, 3),
            "bbox": [round(float(value), 2) for value in box],
            "center_px": [round(center_x, 2), round(center_y, 2)],
            "world_mm": depth_payload["world_mm"],
            "depth_avg_mm": depth_payload["depth_avg_mm"],
            "depth_compensated_mm": depth_payload["depth_compensated_mm"],
            "depth_samples": depth_payload["depth_samples"],
            "depth_valid": depth_payload["depth_valid"],
            "camera_xyz_mm": depth_payload["camera_xyz_mm"],
            "source": "mediapipe",
            "landmark": "MIDDLE_FINGER_MCP",
            "handedness": handedness_label,
            "timestamp_s": timestamp_s,
            "frame_id": int(frame_id or 0),
            "follow_plane_z_mm": hand_calibration["plane_z_mm"],
            "follow_z_offset_mm": hand_calibration["z_offset_mm"],
            "follow_height_mm": hand_calibration["target_z_mm"],
        }

    def _class_names(self) -> Dict[int, str]:
        names = getattr(self._model, "names", FALLBACK_CLASS_NAMES) if self._model is not None else FALLBACK_CLASS_NAMES
        if isinstance(names, dict):
            return {int(index): str(label) for index, label in names.items()}
        if isinstance(names, Sequence) and not isinstance(names, (str, bytes)):
            return {index: str(label) for index, label in enumerate(names)}
        return dict(FALLBACK_CLASS_NAMES)

    def _build_detection(
        self,
        label: str,
        confidence: float,
        box: Sequence[float],
        frame_id: int = 0,
        detected_at_s: float | None = None,
    ) -> Dict[str, Any]:
        center_x = (box[0] + box[2]) / 2.0
        center_y = (box[1] + box[3]) / 2.0
        is_hand = self._is_hand_label(label)

        with self._frame_lock:
            depth_frame = None if self._latest_depth_frame is None else self._latest_depth_frame.copy()
            rgb_intrinsics = self.calibration.intrinsics["rgb"].to_dict()
            alignment_ok = self._alignment_ok()

        depth_payload = self._compute_detection_depth(
            box=box,
            depth_frame=depth_frame,
            rgb_intrinsics=rgb_intrinsics,
            alignment_ok=alignment_ok,
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
            "source": "yolo",
            "timestamp_s": float(detected_at_s or time.time()),
            "frame_id": int(frame_id or 0),
        }
        if is_hand:
            hand_calibration = self.get_hand_follow_calibration()
            detection["follow_plane_z_mm"] = hand_calibration["plane_z_mm"]
            detection["follow_z_offset_mm"] = hand_calibration["z_offset_mm"]
            detection["follow_height_mm"] = hand_calibration["target_z_mm"]
        return detection

    def _select_hand_target(self, detections: List[Dict[str, Any]]) -> Dict[str, Any] | None:
        for item in detections:
            if item.get("type") == "hand":
                hand = dict(item)
                hand["type"] = "hand"
                return hand
        return None

    @staticmethod
    def _normalize_label(label: str) -> str:
        return "".join(char.lower() for char in str(label) if char.isalnum())

    @classmethod
    def _is_hand_label(cls, label: str) -> bool:
        normalized = cls._normalize_label(label)
        return normalized in HAND_LABEL_ALIASES

    def _annotate_frame(self, frame, detections, hand_target, detector_mode: str = "yolo_objects"):
        cv2 = self._cv2

        selected_hand_box = hand_target.get("bbox") if hand_target else None
        for detection in detections:
            box = detection.get("bbox")
            if not box or len(box) < 4:
                continue

            x1, y1, x2, y2 = [int(value) for value in box[:4]]
            is_hand = detection.get("type") == "hand"
            is_selected_hand = bool(selected_hand_box) and list(box[:4]) == list(selected_hand_box[:4])
            color = (80, 220, 120) if is_hand else (60, 170, 255)
            thickness = 3 if is_selected_hand else 2

            cv2.rectangle(frame, (x1, y1), (x2, y2), color, thickness)
            cv2.putText(
                frame,
                f"{detection['label']} {detection['confidence']:.2f}",
                (x1, max(22, y1 - 8)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6,
                color,
                2,
            )
            if detection.get("depth_valid") and detection.get("depth_avg_mm") is not None:
                cv2.putText(
                    frame,
                    f"Z {detection['depth_avg_mm']:.1f} mm",
                    (x1, max(46, y1 + 22)),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.55,
                    (235, 235, 235),
                    2,
                )

        depth_avg_mm = self._latest_depth_report.get("average_depth_mm")
        depth_text = f"Centro: {depth_avg_mm:.1f} mm" if isinstance(depth_avg_mm, (int, float)) else "Centro: sin profundidad"
        model_text = f"Modelo: {self.config.vision_model_path.name}"
        detection_text = (
            "Detecciones: YOLO mano"
            if self._is_hand_only_mode()
            else f"Detecciones: {len(detections)} | Conf: {self.config.vision_confidence_threshold:.2f}"
        )

        cv2.putText(
            frame,
            model_text,
            (16, 28),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            (245, 245, 245),
            2,
        )
        cv2.putText(
            frame,
            f"Hand tracking: {detector_mode} + HP60C depth | {self._inference_device}",
            (16, 56),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            (80, 220, 120),
            2,
        )
        cv2.putText(
            frame,
            detection_text,
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
            if hasattr(self._model, "to"):
                self._model.to(self._inference_device)
            if self._inference_device.startswith("cuda") and hasattr(self._model, "half"):
                try:
                    self._model.half()
                    logger.info("YOLO model set to FP16 half-precision on %s.", self._inference_device)
                except Exception as half_err:
                    logger.warning("No se pudo activar FP16: %s. Continuando en FP32.", half_err)
            self._last_inference_error = ""
            logger.info(
                "YOLO cargado desde %s con device=%s (%s).",
                self.config.vision_model_path,
                self._inference_device,
                self._inference_device_reason,
            )
        except Exception as error:  # pragma: no cover - dependency fallback
            self._last_inference_error = f"No se pudo cargar el modelo YOLO: {error}"
            self._publish_status(ok=False, message=self._last_inference_error)
            logger.warning(self._last_inference_error)
            self._model = None

    def _configure_inference_device(self) -> None:
        requested_device = str(getattr(self.config, "vision_device", "auto") or "auto").strip().lower()
        torch_module = self._torch

        self._inference_device = "cpu"
        self._inference_device_reason = "CPU por default."
        self._torch_cuda_available = False
        self._torch_cuda_device_count = 0
        self._torch_cuda_version = ""

        if torch_module is None:
            self._inference_device_reason = "PyTorch no esta disponible en este entorno."
            return

        self._torch_cuda_version = str(getattr(getattr(torch_module, "version", None), "cuda", "") or "")
        cuda = getattr(torch_module, "cuda", None)
        if cuda is None:
            self._inference_device_reason = "PyTorch se cargo sin modulo CUDA."
            return

        cuda_warning = ""
        with warnings.catch_warnings(record=True) as caught:
            warnings.simplefilter("always")
            try:
                self._torch_cuda_available = bool(cuda.is_available())
            except Exception as error:
                cuda_warning = str(error)
                self._torch_cuda_available = False

        if not cuda_warning and caught:
            cuda_warning = " | ".join(str(item.message) for item in caught if item.message)

        try:
            self._torch_cuda_device_count = int(cuda.device_count())
        except Exception:
            self._torch_cuda_device_count = 0

        gpu_name = ""
        if self._torch_cuda_available and self._torch_cuda_device_count > 0:
            try:
                gpu_name = str(cuda.get_device_name(0))
            except Exception:
                gpu_name = ""

        if requested_device == "cpu":
            self._inference_device = "cpu"
            self._inference_device_reason = "Forzado por VISION_DEVICE=cpu."
            return

        if requested_device.startswith("cuda"):
            if self._torch_cuda_available:
                self._inference_device = requested_device
                self._inference_device_reason = f"GPU solicitada por config ({gpu_name or requested_device})."
                return
            self._inference_device = "cpu"
            self._inference_device_reason = self._build_cuda_fallback_reason(
                requested_device=requested_device,
                cuda_warning=cuda_warning,
            )
            logger.warning(self._inference_device_reason)
            return

        if self._torch_cuda_available:
            self._inference_device = "cuda:0"
            self._inference_device_reason = f"Auto detecto GPU {gpu_name or 'cuda:0'}."
            return

        self._inference_device = "cpu"
        self._inference_device_reason = self._build_cuda_fallback_reason(
            requested_device="auto",
            cuda_warning=cuda_warning,
        )
        logger.warning(self._inference_device_reason)

    def _build_cuda_fallback_reason(self, requested_device: str, cuda_warning: str) -> str:
        if self._torch_cuda_device_count > 0 and cuda_warning:
            return (
                f"{requested_device} cayo a CPU. CUDA detecta {self._torch_cuda_device_count} GPU(s), "
                f"pero no pudo inicializar: {cuda_warning}"
            )
        if self._torch_cuda_device_count > 0:
            return (
                f"{requested_device} cayo a CPU. Hay {self._torch_cuda_device_count} GPU(s) visibles, "
                "pero PyTorch no pudo habilitar CUDA."
            )
        return f"{requested_device} cayo a CPU. CUDA no esta disponible."

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

        if not self._calibration_file_exists:
            self.calibration.hand_follow.plane_z_mm = float(self.config.hand_plane_z_mm)
            self.calibration.object_pick.plane_z_mm = float(self.config.object_plane_z_mm)
            self.calibration.hand_follow.workspace = dict(self.config.hand_workspace)
            self.calibration.object_pick.workspace = dict(self.config.object_workspace)
        else:
            if not self.calibration.hand_follow.workspace:
                self.calibration.hand_follow.workspace = dict(self.config.hand_workspace)
            if not self.calibration.object_pick.workspace:
                self.calibration.object_pick.workspace = dict(self.config.object_workspace)

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

        if not self.calibration.tcp_to_camera.configured:
            self.calibration.tcp_to_camera.translation_mm = list(self.config.tcp_to_camera_translation_mm)
            self.calibration.tcp_to_camera.rotation_rpy_deg = list(self.config.tcp_to_camera_rotation_rpy_deg)
            self.calibration.tcp_to_camera.configured = True

        if not self.calibration.tcp_to_camera.matrix_4x4:
            self.calibration.tcp_to_camera.matrix_4x4 = build_transform_matrix(
                self.calibration.tcp_to_camera.translation_mm,
                self.calibration.tcp_to_camera.rotation_rpy_deg,
            )

        self.calibration.camera_to_robot.translation_mm = list(self.calibration.tcp_to_camera.translation_mm)
        self.calibration.camera_to_robot.rotation_rpy_deg = list(self.calibration.tcp_to_camera.rotation_rpy_deg)
        self.calibration.camera_to_robot.matrix_4x4 = list(self.calibration.tcp_to_camera.matrix_4x4)
        self.calibration.camera_to_robot.configured = bool(self.calibration.tcp_to_camera.configured)

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
            preview_timestamp = time.monotonic()
            encoded = None
            if self._should_refresh_preview(
                preview_timestamp,
                self._last_rgb_preview_monotonic,
                getattr(self.config, "vision_preview_fps", 8.0),
            ):
                encoded = self._encode_jpeg(frame)
            with self._frame_lock:
                self._latest_frame = frame
                self._latest_frame_id += 1
                self._latest_rgb_timestamp = timestamp
                if encoded is not None:
                    self._latest_rgb_jpeg = encoded
                    self._last_rgb_preview_monotonic = preview_timestamp
                self._camera_open = True
                self._rgb_stream_ok = True
                self.calibration.camera["width"] = int(frame.shape[1])
                self.calibration.camera["height"] = int(frame.shape[0])
            if not self._logged_first_rgb_frame:
                logger.info(
                    "HP60C RGB activo en %s con %sx%s.",
                    self.config.ros_rgb_topic,
                    frame.shape[1],
                    frame.shape[0],
                )
                self._logged_first_rgb_frame = True
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
            preview_timestamp = time.monotonic()
            encoded = None
            if self._should_refresh_preview(
                preview_timestamp,
                self._last_depth_preview_monotonic,
                getattr(self.config, "vision_depth_preview_fps", 4.0),
            ):
                depth_preview = self._build_depth_preview(depth_array, report)
                encoded = self._encode_jpeg(depth_preview)
            timestamp = time.time()
            with self._frame_lock:
                self._latest_depth_frame = depth_array
                self._latest_depth_id += 1
                self._latest_depth_timestamp = timestamp
                self._depth_stream_ok = bool(report.get("valid_points"))
                self._latest_depth_report = report
                if encoded is not None:
                    self._latest_depth_jpeg = encoded
                    self._last_depth_preview_monotonic = preview_timestamp
            if not self._logged_first_depth_frame:
                logger.info(
                    "HP60C depth activo en %s con %sx%s.",
                    self.config.ros_depth_topic,
                    depth_array.shape[1],
                    depth_array.shape[0],
                )
                self._logged_first_depth_frame = True
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
                self._latest_camera_info_timestamp = time.time()

        return _callback

    def _refresh_stream_health(self, now: float | None = None) -> None:
        now = now or time.time()
        freshness_timeout_s = max(1.0, float(getattr(self.config, "ros_camera_stall_timeout_s", 6.0)))
        with self._frame_lock:
            self._rgb_stream_ok = self._latest_rgb_timestamp > 0.0 and (now - self._latest_rgb_timestamp) < freshness_timeout_s
            self._depth_stream_ok = self._latest_depth_timestamp > 0.0 and (now - self._latest_depth_timestamp) < freshness_timeout_s
            self._camera_open = self._rgb_stream_ok
            if not self._rgb_stream_ok:
                self._last_capture_error = "Sin frames RGB recientes desde la HP60C."
            elif not self._depth_stream_ok:
                self._last_depth_error = "Sin frames de profundidad recientes desde la HP60C."

    def _is_usb_reenumeration_in_progress(self) -> bool:
        """Detect if HP60C is re-enumerating on USB.

        The ascamera SDK logs ``onAttached`` followed by ``this camera exist``
        when the USB device re-enumerates.  During this window (typically ~4-6s)
        frames stop temporarily but the camera will recover on its own.
        """
        if not self._camera_process_log_path.exists():
            return False

        try:
            with self._camera_process_log_path.open("rb") as handle:
                handle.seek(0, os.SEEK_END)
                file_size = handle.tell()
                # Only read the last 8 KB – re-enumeration messages are recent
                handle.seek(max(0, file_size - 8192))
                tail = handle.read().decode("utf-8", errors="ignore")
        except OSError:
            return False

        # Look for the characteristic SDK pattern in the most recent lines.
        lines = tail.splitlines()
        # Scan the last 30 lines (covers ~15s of SDK output at 10fps)
        recent = lines[-30:] if len(lines) > 30 else lines
        saw_attached = False
        saw_exists = False
        for line in reversed(recent):
            low = line.lower()
            if "this camera exist" in low or "this device exist" in low:
                saw_exists = True
            if "onattached" in low and "attached" in low:
                saw_attached = True
            # If we see streaming started again, the re-enumeration finished
            if "start streaming" in low or "camera opened" in low:
                return False
            if saw_attached and saw_exists:
                return True
        return False

    def _latest_camera_process_issue(self) -> str:
        if not self._camera_process_log_path.exists():
            return ""

        try:
            with self._camera_process_log_path.open("rb") as handle:
                handle.seek(0, os.SEEK_END)
                file_size = handle.tell()
                start_offset = max(0, min(file_size, int(self._camera_process_log_offset)))
                handle.seek(max(start_offset, file_size - 32768))
                content = handle.read().decode("utf-8", errors="ignore")
        except OSError:
            return ""

        interesting_markers = (
            "no such device",
            "uvc_start_streaming",
            "usb contorl transfer failed",
            "xucmd_write",
            "set confidence level failed",
        )
        for raw_line in reversed(content.splitlines()):
            line = " ".join(raw_line.strip().split())
            if not line:
                continue
            normalized = line.lower()
            if any(marker in normalized for marker in interesting_markers):
                return line[:220]
        return ""

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
    ) -> Dict[str, Any]:
        center_x = (float(box[0]) + float(box[2])) / 2.0
        center_y = (float(box[1]) + float(box[3])) / 2.0
        span_x = max(1, int(abs(float(box[2]) - float(box[0])) * self.config.depth_sampling_ratio))
        span_y = max(1, int(abs(float(box[3]) - float(box[1])) * self.config.depth_sampling_ratio))

        return self._compute_point_depth(
            point_xy=(center_x, center_y),
            span_x=span_x,
            span_y=span_y,
            depth_frame=depth_frame,
            rgb_intrinsics=rgb_intrinsics,
            alignment_ok=alignment_ok,
        )

    def _compute_point_depth(
        self,
        point_xy: Sequence[float],
        span_x: int,
        span_y: int,
        depth_frame,
        rgb_intrinsics: Dict[str, Any],
        alignment_ok: bool,
    ) -> Dict[str, Any]:
        center_x = float(point_xy[0])
        center_y = float(point_xy[1])

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
                transformed = self._camera_xyz_to_world(camera_xyz_mm)
                if transformed:
                    world_mm = [round(value, 3) for value in transformed]
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

    def _camera_xyz_to_world(self, camera_xyz_mm: Sequence[float]) -> List[float] | None:
        robot_pose_mm = self._current_robot_pose_mm()
        if not robot_pose_mm:
            return None

        with self._frame_lock:
            tcp_to_camera = list(self.calibration.tcp_to_camera.matrix_4x4)
            transform_ok = self.calibration.tcp_to_camera.configured and bool(tcp_to_camera)

        if not transform_ok:
            return None

        base_to_tcp = build_transform_from_ur_pose(robot_pose_mm)
        base_to_camera = compose_transform_matrices(base_to_tcp, tcp_to_camera)
        return apply_rigid_transform(camera_xyz_mm, base_to_camera)

    def _current_robot_pose_mm(self) -> List[float] | None:
        pose = self.state.snapshot().get("robot_status", {}).get("current_pose_mm")
        if not pose or len(pose) < 6:
            return None
        try:
            return [float(value) for value in pose[:6]]
        except (TypeError, ValueError):
            return None

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
        transform = self.calibration.tcp_to_camera
        return {
            "translation_mm": list(transform.translation_mm),
            "rotation_rpy_deg": list(transform.rotation_rpy_deg),
            "matrix_4x4": list(transform.matrix_4x4),
            "configured": bool(transform.configured),
        }

    @staticmethod
    def _clamp(value: float, low: float, high: float) -> float:
        return max(low, min(high, value))

    @staticmethod
    def _seconds_since(timestamp_s: float, now_s: float) -> float | None:
        if timestamp_s <= 0.0:
            return None
        return max(0.0, now_s - timestamp_s)

    @classmethod
    def _rounded_seconds_since(cls, timestamp_s: float, now_s: float) -> float | None:
        delta_s = cls._seconds_since(timestamp_s, now_s)
        return round(delta_s, 3) if delta_s is not None else None

    @staticmethod
    def _fps_interval(fps: Any) -> float:
        try:
            fps_value = float(fps)
        except (TypeError, ValueError):
            return 0.0
        if fps_value <= 0.0:
            return 0.0
        return 1.0 / fps_value

    def _should_refresh_preview(self, now_monotonic: float, last_monotonic: float, fps: Any) -> bool:
        interval = self._fps_interval(fps)
        return interval <= 0.0 or last_monotonic <= 0.0 or (now_monotonic - last_monotonic) >= interval

    @staticmethod
    def _is_recent_timestamp(timestamp_s: float, now_s: float, max_age_s: float = 1.5) -> bool:
        return timestamp_s > 0.0 and (now_s - timestamp_s) < max_age_s

    def _detector_mode(self) -> str:
        mode = str(getattr(self.config, "vision_detector_mode", "full") or "full").strip().lower()
        return mode if mode in {"full", "hand_only"} else "full"

    def _is_hand_only_mode(self) -> bool:
        return self._detector_mode() == "hand_only"

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
        now = time.time()
        preview_timeout_s = max(1.0, float(getattr(self.config, "ros_camera_stall_timeout_s", 6.0)))
        recovery = self._camera_recovery_summary(now)
        detections_fresh = self._is_recent_timestamp(self._latest_annotated_timestamp, now, preview_timeout_s)
        status = {
            "ok": ok and self._rgb_stream_ok and self._depth_stream_ok,
            "camera_open": self._camera_open,
            "capture_thread_running": bool(self._capture_thread and self._capture_thread.is_alive()),
            "inference_thread_running": bool(self._inference_thread and self._inference_thread.is_alive()),
            "camera_process_running": self._camera_process_running(),
            "camera_auto_launch": bool(self.config.ros_auto_launch_camera),
            "detector_mode": self._detector_mode(),
            "yolo_enabled": True,
            "model_loaded": bool(self._model),
            "model_path": str(self.config.vision_model_path),
            "inference_device": self._inference_device,
            "inference_device_reason": self._inference_device_reason,
            "vision_inference_fps": getattr(self.config, "vision_inference_fps", 8.0),
            "vision_preview_fps": getattr(self.config, "vision_preview_fps", 8.0),
            "vision_depth_preview_fps": getattr(self.config, "vision_depth_preview_fps", 4.0),
            "torch_cuda_available": self._torch_cuda_available,
            "torch_cuda_device_count": self._torch_cuda_device_count,
            "torch_cuda_version": self._torch_cuda_version,
            "detections": len(self._latest_detections) if detections_fresh else 0,
            "hand_visible": bool(self._latest_hand_target) if detections_fresh else False,
            "hand_detector_mode": "yolo_hand_only" if self._is_hand_only_mode() else "yolo",
            "calibrated_hand": bool(self.calibration.hand_follow.homography),
            "calibrated_object": bool(self.calibration.object_pick.homography),
            "rgb_ok": self._rgb_stream_ok,
            "depth_ok": self._depth_stream_ok,
            "rgb_preview_ready": bool(self._latest_rgb_jpeg) and self._is_recent_timestamp(self._latest_rgb_timestamp, now, preview_timeout_s),
            "depth_preview_ready": bool(self._latest_depth_jpeg) and self._is_recent_timestamp(self._latest_depth_timestamp, now, preview_timeout_s),
            "intrinsics_ok": self._intrinsics_ok(),
            "alignment_ok": self._alignment_ok(),
            "calibration_ok": self._intrinsics_ok() and self._alignment_ok() and self.calibration.tcp_to_camera.configured,
            "backend": self.calibration.backend,
            "tcp_to_camera": self._camera_transform_summary(),
            "depth_compensation": self._depth_compensation_summary(),
            "topics": dict(self.calibration.topics),
            "camera_recovery": recovery,
        }

        if message:
            status["message"] = message
        elif recovery["state"] in {"stalled", "restarting"}:
            status["message"] = "Recuperando HP60C..."
        elif (
            recovery["state"] == "failed"
            and recovery.get("next_retry_in_s") is not None
            and "otro proceso ascamera ya corriendo" not in self._last_camera_launch_error.lower()
        ):
            status["message"] = f"HP60C fallida, reintentando en {recovery['next_retry_in_s']:.1f}s."
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
        elif self._last_hand_error:
            status["message"] = self._last_hand_error
        elif not status["calibration_ok"]:
            status["message"] = "HP60C activa. Falta calibracion de profundidad o transformacion TCP-camara."
        elif self._is_hand_only_mode():
            status["message"] = "Vision HP60C + YOLO mano activa."
        else:
            status["message"] = "Vision HP60C + YOLO activa."

        self.state.set_vision_status(status)
