#!/usr/bin/env python3
import argparse
import glob
import os
import select
import subprocess
import threading
import time
from typing import Dict, List, Optional

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge, CvBridgeError
from flask import Flask, Response, jsonify, render_template_string, request
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import Image

HTML = """
<!doctype html>
<html lang=\"en\">
<head>
  <meta charset=\"utf-8\" />
  <meta name=\"viewport\" content=\"width=device-width, initial-scale=1\" />
  <title>Depth Camera Viewer</title>
  <style>
    :root {
      --bg1: #061423;
      --bg2: #16324f;
      --card: rgba(255, 255, 255, 0.08);
      --text: #eaf6ff;
      --muted: #b6d3ea;
      --accent: #7dd3fc;
    }
    * { box-sizing: border-box; }
    body {
      margin: 0;
      font-family: "Segoe UI", "Noto Sans", Arial, sans-serif;
      color: var(--text);
      background: radial-gradient(1200px 500px at 10% 0%, #1b4d79 0%, transparent 60%),
                  radial-gradient(900px 400px at 90% 100%, #0f3c5f 0%, transparent 60%),
                  linear-gradient(135deg, var(--bg1), var(--bg2));
      min-height: 100vh;
      padding: 18px;
    }
    .wrap { max-width: 1250px; margin: 0 auto; }
    .title { font-size: 1.4rem; font-weight: 700; margin: 0 0 8px; }
    .subtitle { color: var(--muted); margin: 0 0 16px; }
    .grid {
      display: grid;
      grid-template-columns: repeat(auto-fit, minmax(420px, 1fr));
      gap: 14px;
    }
    .card {
      background: var(--card);
      border: 1px solid rgba(125, 211, 252, 0.25);
      border-radius: 12px;
      padding: 10px;
      backdrop-filter: blur(3px);
    }
    .label { font-size: 0.95rem; color: var(--accent); margin-bottom: 8px; }
    .frame {
      width: 100%;
      aspect-ratio: 16 / 9;
      object-fit: contain;
      background: #000;
      border-radius: 10px;
      border: 1px solid rgba(255,255,255,0.08);
    }
    .status {
      margin-top: 14px;
      color: var(--muted);
      font-size: 0.95rem;
      display: flex;
      flex-wrap: wrap;
      gap: 14px;
    }
    code {
      background: rgba(255,255,255,0.1);
      padding: 2px 6px;
      border-radius: 6px;
    }
    .depth-grid-card {
      margin-top: 14px;
    }
    table {
      width: 100%;
      border-collapse: collapse;
      font-size: 0.92rem;
      color: var(--text);
    }
    th, td {
      border-bottom: 1px solid rgba(255, 255, 255, 0.12);
      padding: 8px 6px;
      text-align: left;
      vertical-align: top;
    }
    th {
      color: var(--accent);
      font-weight: 600;
    }
    .grid-meta {
      color: var(--muted);
      font-size: 0.9rem;
      margin: 0 0 10px;
    }
    .controls {
      display: flex;
      align-items: center;
      gap: 8px;
      margin: 0 0 10px;
      color: var(--muted);
      font-size: 0.95rem;
    }
    .controls.column {
      flex-direction: column;
      align-items: flex-start;
      gap: 10px;
    }
    .slider-row {
      display: flex;
      align-items: center;
      gap: 10px;
      width: 100%;
      max-width: 430px;
    }
    .slider-row input[type="range"] {
      flex: 1;
    }
    .slider-value {
      min-width: 72px;
      color: var(--accent);
      font-weight: 600;
    }
    .valid { color: #86efac; }
    .invalid { color: #fda4af; }
  </style>
</head>
<body>
  <div class=\"wrap\">
    <h1 class=\"title\">USB Depth Camera Viewer</h1>
    <p class=\"subtitle\">Live RGB and depth stream from ROS2 topics</p>
    <div class=\"grid\">
      <section class=\"card\">
        <div class=\"label\">RGB</div>
        <img class=\"frame\" src=\"/video/rgb\" alt=\"RGB stream\" />
      </section>
      <section class=\"card\">
        <div class=\"label\">Depth (colormap)</div>
        <img class=\"frame\" src=\"/video/depth\" alt=\"Depth stream\" />
      </section>
    </div>
    <div class=\"status\">
      <span>RGB topic: <code>{{ rgb_topic }}</code></span>
      <span>Depth topic: <code>{{ depth_topic }}</code></span>
      <span>Refresh page if stream was restarted.</span>
    </div>
    <section class=\"card depth-grid-card\">
      <div class=\"label\">Depth Values at Centered 3x3 Grid</div>
      <div class=\"controls column\">
        <div>
          <input id=\"overlay-text-toggle\" type=\"checkbox\" checked />
          <label for=\"overlay-text-toggle\">Show on-video point text labels</label>
        </div>
        <div class=\"slider-row\">
          <label for=\"span-slider\">Grid span:</label>
          <input id=\"span-slider\" type=\"range\" min=\"0.05\" max=\"0.45\" step=\"0.01\" value=\"0.17\" />
          <span id=\"span-value\" class=\"slider-value\">17%</span>
        </div>
      </div>
      <p class=\"grid-meta\" id=\"grid-meta\">Waiting for depth grid data...</p>
      <table>
        <thead>
          <tr>
            <th>Point</th>
            <th>Pixel (x, y)</th>
            <th>Depth Value</th>
          </tr>
        </thead>
        <tbody id=\"depth-grid-body\">
          <tr><td colspan=\"3\">Waiting for depth frames...</td></tr>
        </tbody>
      </table>
    </section>
  </div>
  <script>
    const gridBody = document.getElementById("depth-grid-body");
    const gridMeta = document.getElementById("grid-meta");
    const overlayToggle = document.getElementById("overlay-text-toggle");
    const spanSlider = document.getElementById("span-slider");
    const spanValue = document.getElementById("span-value");

    function esc(v) {
      return String(v)
        .replaceAll("&", "&amp;")
        .replaceAll("<", "&lt;")
        .replaceAll(">", "&gt;");
    }

    async function updateDepthGrid() {
      try {
        const resp = await fetch("/api/depth-grid", { cache: "no-store" });
        const data = await resp.json();
        const points = data.points || [];

        if (!points.length) {
          gridMeta.textContent = "Waiting for depth grid data...";
          gridBody.innerHTML = "<tr><td colspan='3'>Waiting for depth frames...</td></tr>";
          return;
        }

        const updated = data.timestamp_ms
          ? new Date(data.timestamp_ms).toLocaleTimeString()
          : "unknown";
        gridMeta.textContent = `Source: ${data.source || "unknown"} | Last update: ${updated}`;

        gridBody.innerHTML = points.map((p) => {
          const cls = p.valid ? "valid" : "invalid";
          return `
            <tr>
              <td>${esc(p.id)}</td>
              <td>(${esc(p.x)}, ${esc(p.y)})</td>
              <td class="${cls}">${esc(p.display)}</td>
            </tr>
          `;
        }).join("");
      } catch (err) {
        gridMeta.textContent = "Depth grid endpoint unavailable.";
      }
    }

    async function syncOverlayToggle() {
      try {
        const resp = await fetch("/api/grid-settings", { cache: "no-store" });
        const data = await resp.json();
        overlayToggle.checked = Boolean(data.overlay_text);
        const span = Number(data.span_ratio || 0.17);
        spanSlider.value = span.toFixed(2);
        spanValue.textContent = `${Math.round(span * 100)}%`;
      } catch (err) {
        // Keep default checked state.
      }
    }

    function updateSpanLabel() {
      const span = Number(spanSlider.value);
      spanValue.textContent = `${Math.round(span * 100)}%`;
    }

    updateSpanLabel();
    spanSlider.addEventListener("input", updateSpanLabel);

    overlayToggle.addEventListener("change", async () => {
      try {
        await fetch("/api/grid-settings", {
          method: "POST",
          headers: { "Content-Type": "application/json" },
          body: JSON.stringify({
            overlay_text: overlayToggle.checked,
            span_ratio: Number(spanSlider.value),
          }),
        });
      } catch (err) {
        // Ignore request error and keep current UI state.
      }
    });

    spanSlider.addEventListener("change", async () => {
      try {
        await fetch("/api/grid-settings", {
          method: "POST",
          headers: { "Content-Type": "application/json" },
          body: JSON.stringify({
            overlay_text: overlayToggle.checked,
            span_ratio: Number(spanSlider.value),
          }),
        });
      } catch (err) {
        // Ignore request error and keep current UI state.
      }
    });

    syncOverlayToggle();
    updateDepthGrid();
    setInterval(updateDepthGrid, 500);
  </script>
</body>
</html>
"""


class CameraBridge(Node):
    def __init__(
        self,
        rgb_topic: str,
        depth_topic: str,
        v4l2_device: str,
        enable_v4l2_fallback: bool,
        enable_opencv_fallback: bool,
    ):
        super().__init__("depth_web_ui_bridge")
        self.bridge = CvBridge()
        self.lock = threading.Lock()
        self.shutdown_event = threading.Event()
        self.v4l2_device = v4l2_device
        self.enable_v4l2_fallback = enable_v4l2_fallback
        self.enable_opencv_fallback = enable_opencv_fallback

        self.rgb_frame: Optional[np.ndarray] = None
        self.depth_frame: Optional[np.ndarray] = None
        self.depth_grid_points: List[Dict[str, object]] = []
        self.depth_grid_timestamp_ms = 0
        self.overlay_text_enabled = True
        self.grid_span_ratio = 0.17

        self.rgb_count = 0
        self.depth_count = 0
        self.fallback_rgb_count = 0
        self.fallback_depth_count = 0
        self.rgb_source = "none"
        self.depth_source = "none"

        self.create_subscription(Image, rgb_topic, self.rgb_callback, 10)
        self.create_subscription(Image, depth_topic, self.depth_callback, 10)
        self.create_timer(2.0, self.log_status)

        self.get_logger().info(f"Subscribed RGB topic: {rgb_topic}")
        self.get_logger().info(f"Subscribed depth topic: {depth_topic}")
        if self.enable_v4l2_fallback:
            self.get_logger().info(f"V4L2 fallback enabled on: {self.v4l2_device}")
            if self.enable_opencv_fallback:
                self.get_logger().info("OpenCV V4L2 fallback enabled")
            else:
                self.get_logger().info("OpenCV fallback disabled (FFmpeg-only fallback)")
            self.v4l2_thread = threading.Thread(target=self.v4l2_fallback_loop, daemon=True)
            self.v4l2_thread.start()

    def rgb_callback(self, msg: Image):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            with self.lock:
                self.rgb_frame = frame
                self.rgb_count += 1
                self.rgb_source = "ros"
        except CvBridgeError as exc:
            self.get_logger().warning(f"RGB conversion failed: {exc}")

    def depth_callback(self, msg: Image):
        try:
            depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
            if depth.ndim != 2:
                depth = cv2.cvtColor(depth, cv2.COLOR_BGR2GRAY)

            depth_vis = self.depth_to_colormap(depth)
            grid_points = self.compute_depth_grid(depth, pseudo=False, span_ratio=self.get_grid_span_ratio())
            depth_vis = self.overlay_depth_grid(depth_vis, grid_points, show_text=self.is_overlay_text_enabled())
            with self.lock:
                self.depth_frame = depth_vis
                self.depth_grid_points = grid_points
                self.depth_grid_timestamp_ms = int(time.time() * 1000)
                self.depth_count += 1
                self.depth_source = "ros"
        except CvBridgeError as exc:
            self.get_logger().warning(f"Depth conversion failed: {exc}")

    def log_status(self):
        self.get_logger().info(
            "Frames | "
            f"RGB ROS: {self.rgb_count}, RGB fallback: {self.fallback_rgb_count} ({self.rgb_source}) | "
            f"Depth ROS: {self.depth_count}, Depth fallback: {self.fallback_depth_count} ({self.depth_source})"
        )

    def v4l2_fallback_loop(self):
        warned_once = False
        while not self.shutdown_event.is_set():
            if not os.path.exists(self.v4l2_device):
                if not warned_once:
                    self.get_logger().warning(
                        f"V4L2 device {self.v4l2_device} does not exist. "
                        "Fallback capture will wait for device to appear."
                    )
                    warned_once = True
                time.sleep(2.0)
                continue

            # FFmpeg is generally more robust than OpenCV for this device.
            if self.ffmpeg_capture_loop():
                continue

            # OpenCV fallback can crash on some OpenCV/GStreamer builds.
            if self.enable_opencv_fallback and self.opencv_capture_loop():
                continue

            if not warned_once:
                self.get_logger().warning(
                    f"Could not decode frames from {self.v4l2_device} via fallback capture. "
                    "RGB panel will remain black until camera stream is available."
                )
                warned_once = True
            time.sleep(2.0)

    def apply_fallback_frame(self, frame: np.ndarray, source: str):
        # Boost low-light / low-contrast frames to avoid near-black output.
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        if float(gray.mean()) < 18.0 or float(gray.std()) < 8.0:
            frame = cv2.convertScaleAbs(frame, alpha=2.2, beta=18.0)

        with self.lock:
            if self.rgb_count == 0:
                self.rgb_frame = frame
                self.rgb_source = source
                self.fallback_rgb_count += 1

            if self.depth_count == 0:
                pseudo_depth = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
                pseudo = self.depth_to_colormap(pseudo_depth)
                grid_points = self.compute_depth_grid(pseudo_depth, pseudo=True, span_ratio=self.get_grid_span_ratio())
                pseudo = self.overlay_depth_grid(pseudo, grid_points, show_text=self.is_overlay_text_enabled())
                cv2.putText(
                    pseudo,
                    "Pseudo depth from RGB (ROS depth unavailable)",
                    (20, 40),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.9,
                    (255, 255, 255),
                    2,
                    cv2.LINE_AA,
                )
                self.depth_frame = pseudo
                self.depth_grid_points = grid_points
                self.depth_grid_timestamp_ms = int(time.time() * 1000)
                self.depth_source = "pseudo-from-rgb"
                self.fallback_depth_count += 1

    def ffmpeg_capture_loop(self) -> bool:
        cmd = [
            "ffmpeg",
            "-hide_banner",
            "-loglevel",
            "error",
            "-f",
            "v4l2",
            "-input_format",
            "mjpeg",
            "-video_size",
            "640x642",
            "-framerate",
            "30",
            "-i",
            self.v4l2_device,
            "-f",
            "mjpeg",
            "pipe:1",
        ]

        try:
            proc = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.DEVNULL, bufsize=0)
        except Exception:
            return False

        got_any = False
        buffer = bytearray()
        idle_ticks = 0
        start = time.time()
        try:
            while not self.shutdown_event.is_set():
                if proc.poll() is not None:
                    break

                ready, _, _ = select.select([proc.stdout], [], [], 0.5)
                if not ready:
                    idle_ticks += 1
                    # Bail out quickly if no bytes appear.
                    if idle_ticks > 12 and not got_any:
                        break
                    continue

                chunk = proc.stdout.read(4096)
                if not chunk:
                    break
                idle_ticks = 0
                buffer.extend(chunk)

                # Parse concatenated JPEGs.
                while True:
                    s = buffer.find(b"\xff\xd8")
                    if s < 0:
                        if len(buffer) > 2:
                            del buffer[:-2]
                        break
                    e = buffer.find(b"\xff\xd9", s + 2)
                    if e < 0:
                        if s > 0:
                            del buffer[:s]
                        if len(buffer) > 2_000_000:
                            del buffer[:-200_000]
                        break

                    jpg = bytes(buffer[s : e + 2])
                    del buffer[: e + 2]
                    arr = np.frombuffer(jpg, dtype=np.uint8)
                    frame = cv2.imdecode(arr, cv2.IMREAD_COLOR)
                    if frame is None:
                        continue
                    got_any = True
                    self.apply_fallback_frame(frame, source=f"ffmpeg:{self.v4l2_device}")

                if got_any and (time.time() - start) > 2.0 and self.rgb_count > 0 and self.depth_count > 0:
                    # ROS has taken over; no need to keep fallback capture hot.
                    break
        finally:
            try:
                proc.terminate()
            except Exception:
                pass
            try:
                proc.wait(timeout=1.0)
            except Exception:
                try:
                    proc.kill()
                except Exception:
                    pass

        if got_any:
            self.get_logger().info(f"V4L2 fallback active from {self.v4l2_device} via FFmpeg")
        return got_any

    def opencv_capture_loop(self) -> bool:
        cap = self.open_capture()
        if cap is None:
            return False

        cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 642)
        cap.set(cv2.CAP_PROP_FPS, 30)

        got_any = False
        failed_reads = 0
        try:
            while not self.shutdown_event.is_set():
                ok, frame = cap.read()
                if not ok or frame is None:
                    failed_reads += 1
                    if failed_reads > 100:
                        break
                    time.sleep(0.03)
                    continue
                got_any = True
                self.apply_fallback_frame(frame, source=f"v4l2:{self.v4l2_device}")
                time.sleep(1.0 / 30.0)
        finally:
            cap.release()

        if got_any:
            self.get_logger().info(f"V4L2 fallback active from {self.v4l2_device} via OpenCV")
        return got_any

    def open_capture(self):
        candidates = [(self.v4l2_device, cv2.CAP_V4L2)]
        if self.v4l2_device.startswith("/dev/video"):
            suffix = self.v4l2_device.replace("/dev/video", "")
            if suffix.isdigit():
                idx = int(suffix)
                candidates.extend([(idx, cv2.CAP_V4L2)])

        for dev in sorted(glob.glob("/dev/video*")):
            if dev == self.v4l2_device:
                continue
            candidates.extend([(dev, cv2.CAP_V4L2)])

        tried = []
        for target, backend in candidates:
            tried.append(f"{target}/{backend if backend is not None else 'default'}")
            cap = cv2.VideoCapture(target, backend) if backend is not None else cv2.VideoCapture(target)
            if cap.isOpened():
                return cap
            cap.release()

        self.get_logger().warning(f"Failed to open V4L2 capture candidates: {', '.join(tried)}")
        return None

    @staticmethod
    def depth_to_colormap(depth: np.ndarray) -> np.ndarray:
        depth_array = np.array(depth, copy=False)
        if depth_array.ndim != 2:
            depth_array = cv2.cvtColor(depth_array, cv2.COLOR_BGR2GRAY)

        depth_float = depth_array.astype(np.float32)
        valid_mask = np.isfinite(depth_float) & (depth_float > 0)

        if np.any(valid_mask):
            min_val = float(np.min(depth_float[valid_mask]))
            max_val = float(np.max(depth_float[valid_mask]))
            if max_val - min_val > 1e-6:
                norm = ((depth_float - min_val) * (255.0 / (max_val - min_val))).clip(0, 255).astype(np.uint8)
            else:
                norm = np.zeros_like(depth_float, dtype=np.uint8)
        else:
            norm = np.zeros_like(depth_float, dtype=np.uint8)

        depth_color = cv2.applyColorMap(norm, cv2.COLORMAP_TURBO)
        depth_color[~valid_mask] = (0, 0, 0)
        return depth_color

    @staticmethod
    def compute_depth_grid(depth: np.ndarray, pseudo: bool, span_ratio: float) -> List[Dict[str, object]]:
        h, w = depth.shape[:2]
        center_x = w // 2
        center_y = h // 2
        step = int(min(h, w) * max(0.01, float(span_ratio)))
        max_step = max(1, min(center_x, w - 1 - center_x, center_y, h - 1 - center_y))
        step = max(1, min(step, max_step))

        xs = [max(0, center_x - step), center_x, min(w - 1, center_x + step)]
        ys = [max(0, center_y - step), center_y, min(h - 1, center_y + step)]

        points: List[Dict[str, object]] = []
        idx = 1
        for row_i, y in enumerate(ys):
            for col_i, x in enumerate(xs):
                raw_value = depth[int(y), int(x)]
                valid = False
                short_display = "NA"
                full_display = "N/A"
                numeric_value = None

                if pseudo:
                    val = int(raw_value)
                    valid = True
                    numeric_value = val
                    short_display = f"I:{val}"
                    full_display = f"{val} intensity (pseudo)"
                else:
                    if np.issubdtype(depth.dtype, np.floating):
                        val = float(raw_value)
                        if np.isfinite(val) and val > 0:
                            valid = True
                            numeric_value = val
                            if val > 20.0:
                                mm = int(round(val))
                                meters = mm / 1000.0
                            else:
                                meters = val
                                mm = int(round(meters * 1000.0))
                            short_display = f"{meters:.2f}m"
                            full_display = f"{meters:.3f} m ({mm} mm)"
                    else:
                        val = int(raw_value)
                        if val > 0:
                            valid = True
                            numeric_value = val
                            meters = val / 1000.0
                            short_display = f"{val}mm"
                            full_display = f"{val} mm ({meters:.3f} m)"

                points.append(
                    {
                        "id": f"P{idx}",
                        "row": int(row_i),
                        "col": int(col_i),
                        "x": int(x),
                        "y": int(y),
                        "valid": bool(valid),
                        "value": numeric_value,
                        "short": short_display,
                        "display": full_display,
                    }
                )
                idx += 1

        return points

    @staticmethod
    def overlay_depth_grid(depth_vis: np.ndarray, points: List[Dict[str, object]], show_text: bool) -> np.ndarray:
        out = depth_vis.copy()
        h, w = out.shape[:2]
        for point in points:
            x = int(point["x"])
            y = int(point["y"])
            label = f'{point["id"]} {point["short"]}'
            valid = bool(point["valid"])

            dot_color = (0, 255, 0) if valid else (0, 0, 255)
            cv2.circle(out, (x, y), 6, dot_color, -1, cv2.LINE_AA)
            cv2.circle(out, (x, y), 10, (255, 255, 255), 1, cv2.LINE_AA)

            if show_text:
                text_x = min(max(6, x + 12), max(6, w - 170))
                text_y = min(max(14, y - 10), h - 6)
                cv2.putText(out, label, (text_x, text_y), cv2.FONT_HERSHEY_SIMPLEX, 0.44, (0, 0, 0), 3, cv2.LINE_AA)
                cv2.putText(out, label, (text_x, text_y), cv2.FONT_HERSHEY_SIMPLEX, 0.44, (255, 255, 255), 1, cv2.LINE_AA)

        if show_text:
            cv2.putText(
                out,
                "Centered 3x3 depth grid",
                (10, h - 14),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                (255, 255, 255),
                1,
                cv2.LINE_AA,
            )
        return out

    def get_rgb(self) -> np.ndarray:
        with self.lock:
            if self.rgb_frame is None:
                return self.placeholder("Waiting for RGB frames")
            return self.rgb_frame.copy()

    def get_depth(self) -> np.ndarray:
        with self.lock:
            if self.depth_frame is None:
                return self.placeholder("Waiting for depth frames")
            return self.depth_frame.copy()

    def get_depth_grid(self) -> Dict[str, object]:
        with self.lock:
            return {
                "source": self.depth_source,
                "timestamp_ms": int(self.depth_grid_timestamp_ms),
                "points": [dict(point) for point in self.depth_grid_points],
            }

    def is_overlay_text_enabled(self) -> bool:
        with self.lock:
            return bool(self.overlay_text_enabled)

    def set_overlay_text_enabled(self, enabled: bool):
        with self.lock:
            self.overlay_text_enabled = bool(enabled)

    def get_grid_span_ratio(self) -> float:
        with self.lock:
            return float(self.grid_span_ratio)

    def set_grid_span_ratio(self, span_ratio: float):
        clamped = max(0.01, min(0.49, float(span_ratio)))
        with self.lock:
            self.grid_span_ratio = clamped

    @staticmethod
    def placeholder(text: str) -> np.ndarray:
        img = np.zeros((720, 1280, 3), dtype=np.uint8)
        cv2.putText(img, text, (60, 360), cv2.FONT_HERSHEY_SIMPLEX, 1.2, (0, 255, 255), 2, cv2.LINE_AA)
        return img

    def destroy_node(self):
        self.shutdown_event.set()
        if hasattr(self, "v4l2_thread"):
            self.v4l2_thread.join(timeout=1.5)
        return super().destroy_node()


def encode_jpeg(frame: np.ndarray) -> bytes:
    ok, buff = cv2.imencode('.jpg', frame, [int(cv2.IMWRITE_JPEG_QUALITY), 85])
    if not ok:
        raise RuntimeError("JPEG encode failed")
    return buff.tobytes()


def make_app(bridge: CameraBridge, rgb_topic: str, depth_topic: str) -> Flask:
    app = Flask(__name__)

    @app.get("/")
    def index():
        return render_template_string(HTML, rgb_topic=rgb_topic, depth_topic=depth_topic)

    def frame_generator(kind: str):
        while True:
            try:
                frame = bridge.get_rgb() if kind == "rgb" else bridge.get_depth()
                jpg = encode_jpeg(frame)
                yield (b"--frame\r\n"
                       b"Content-Type: image/jpeg\r\n\r\n" + jpg + b"\r\n")
            except Exception:
                # Keep stream alive even if one frame fails
                time.sleep(0.05)
            time.sleep(1.0 / 20.0)

    @app.get("/video/rgb")
    def video_rgb():
        return Response(frame_generator("rgb"), mimetype='multipart/x-mixed-replace; boundary=frame')

    @app.get("/video/depth")
    def video_depth():
        return Response(frame_generator("depth"), mimetype='multipart/x-mixed-replace; boundary=frame')

    @app.get("/api/depth-grid")
    def api_depth_grid():
        return jsonify(bridge.get_depth_grid())

    @app.get("/api/grid-settings")
    def api_grid_settings_get():
        return jsonify(
            {
                "overlay_text": bridge.is_overlay_text_enabled(),
                "span_ratio": bridge.get_grid_span_ratio(),
            }
        )

    @app.post("/api/grid-settings")
    def api_grid_settings_post():
        payload = request.get_json(silent=True) or {}

        if "overlay_text" in payload:
            bridge.set_overlay_text_enabled(bool(payload.get("overlay_text")))

        if "span_ratio" in payload:
            try:
                bridge.set_grid_span_ratio(float(payload.get("span_ratio")))
            except (TypeError, ValueError):
                pass

        return jsonify(
            {
                "overlay_text": bridge.is_overlay_text_enabled(),
                "span_ratio": bridge.get_grid_span_ratio(),
            }
        )

    return app


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description="Flask web UI for ROS2 RGB + depth streams")
    p.add_argument("--rgb-topic", default="/ascamera/camera_publisher/rgb0/image")
    p.add_argument("--depth-topic", default="/ascamera/camera_publisher/depth0/image_raw")
    p.add_argument("--host", default="127.0.0.1")
    p.add_argument("--port", type=int, default=5000)
    p.add_argument("--v4l2-device", default="/dev/video2")
    p.add_argument("--disable-v4l2-fallback", action="store_true")
    p.add_argument("--enable-opencv-fallback", action="store_true")
    return p.parse_args()


def spin_bridge(bridge: CameraBridge):
    try:
        rclpy.spin(bridge)
    except ExternalShutdownException:
        pass


def main() -> int:
    args = parse_args()

    rclpy.init()
    bridge = CameraBridge(
        args.rgb_topic,
        args.depth_topic,
        v4l2_device=args.v4l2_device,
        enable_v4l2_fallback=not args.disable_v4l2_fallback,
        enable_opencv_fallback=args.enable_opencv_fallback,
    )

    spin_thread = threading.Thread(target=spin_bridge, args=(bridge,), daemon=True)
    spin_thread.start()

    app = make_app(bridge, args.rgb_topic, args.depth_topic)
    try:
        app.run(host=args.host, port=args.port, debug=False, threaded=True)
    finally:
        bridge.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
