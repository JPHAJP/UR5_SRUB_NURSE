const socket = io();

const els = {
  origin: document.getElementById("demo-origin"),
  detector: document.getElementById("demo-detector"),
  cameraStatus: document.getElementById("demo-camera-status"),
  cameraSelect: document.getElementById("demo-camera-select"),
  mode: document.getElementById("demo-mode-pill"),
  robotStatus: document.getElementById("demo-robot-status"),
  safetyStatus: document.getElementById("demo-safety-status"),
  handStatus: document.getElementById("demo-hand-status"),
  stopReason: document.getElementById("demo-stop-reason"),
  planeZInput: document.getElementById("demo-plane-z-input"),
  planeSummary: document.getElementById("demo-plane-summary"),
  trackingSummary: document.getElementById("demo-tracking-summary"),
  robotPose: document.getElementById("demo-robot-pose"),
};

const buttons = {
  start: document.getElementById("demo-start-btn"),
  stop: document.getElementById("demo-stop-btn"),
  home: document.getElementById("demo-home-btn"),
  resetSafety: document.getElementById("demo-reset-safety-btn"),
  calibratePlane: document.getElementById("demo-calibrate-plane-btn"),
  savePlane: document.getElementById("demo-save-plane-btn"),
  refreshCameras: document.getElementById("demo-refresh-cameras-btn"),
};

let latestState = null;
let latestCalibration = null;
let latestCameraCatalog = null;
let cameraSelectionBusy = false;

function postJSON(url, payload) {
  return fetch(url, {
    method: "POST",
    headers: { "Content-Type": "application/json" },
    body: JSON.stringify(payload || {}),
  }).then(async (response) => {
    const data = await response.json().catch(() => ({}));
    if (!response.ok) {
      throw new Error(data.message || "Operacion fallida");
    }
    return data;
  });
}

function formatNumber(value, digits = 1) {
  if (value === null || value === undefined || Number.isNaN(Number(value))) {
    return "N/A";
  }
  return Number(value).toFixed(digits);
}

function formatVector(values) {
  if (!Array.isArray(values) || !values.length) {
    return "N/A";
  }
  return values.map((value) => formatNumber(value, 1)).join(", ");
}

function formatApproachTrend(value) {
  if (value === true) {
    return "acercandose";
  }
  if (value === false) {
    return "alejandose";
  }
  return "sin referencia";
}

function formatHandDetectorLabel(value) {
  if (value === "mediapipe") {
    return "MediaPipe mano";
  }
  if (value === "yolo_fallback") {
    return "YOLO mano fallback";
  }
  if (value === "yolo") {
    return "YOLO mano";
  }
  return value || "N/A";
}

function syncPlaneInput(value) {
  if (document.activeElement !== els.planeZInput && value !== null && value !== undefined) {
    els.planeZInput.value = Number(value).toFixed(1);
  }
}

function renderCalibration(calibration) {
  latestCalibration = calibration;
  if (!calibration) {
    return;
  }
  syncPlaneInput(calibration.plane_z_mm);
  els.planeSummary.textContent = [
    `Plano base Z: ${formatNumber(calibration.plane_z_mm, 1)} mm`,
    `Offset hacia abajo: ${formatNumber(calibration.z_offset_mm, 1)} mm`,
    `Objetivo Z = plano - offset: ${formatNumber(calibration.target_z_mm, 1)} mm`,
  ].join("\n");
}

function formatCameraOption(camera) {
  const size = camera.width && camera.height ? `${camera.width}x${camera.height}` : "tamano desconocido";
  const fps = camera.fps ? `${camera.fps} fps` : "fps N/A";
  const flags = [];
  if (camera.active) flags.push("activa");
  if (camera.selected && !camera.active) flags.push("seleccionada");
  const suffix = flags.length ? ` [${flags.join(", ")}]` : "";
  return `${camera.label || `Webcam ${camera.index}`} (${size}, ${fps})${suffix}`;
}

function renderCameraCatalog(payload) {
  latestCameraCatalog = payload;
  if (!els.cameraSelect) {
    return;
  }

  const cameras = Array.isArray(payload?.cameras) ? payload.cameras : [];
  const selectedIndex = payload?.selected_index;
  const activeIndex = payload?.active_index;

  els.cameraSelect.innerHTML = "";
  if (!cameras.length) {
    const option = document.createElement("option");
    option.value = "";
    option.textContent = "Sin webcams detectadas";
    els.cameraSelect.appendChild(option);
    els.cameraSelect.disabled = true;
    return;
  }

  cameras.forEach((camera) => {
    const option = document.createElement("option");
    option.value = String(camera.index);
    option.textContent = formatCameraOption(camera);
    if (selectedIndex !== null && selectedIndex !== undefined && Number(camera.index) === Number(selectedIndex)) {
      option.selected = true;
    }
    els.cameraSelect.appendChild(option);
  });
  els.cameraSelect.disabled = cameraSelectionBusy;

  if ((selectedIndex === null || selectedIndex === undefined) && activeIndex !== null && activeIndex !== undefined) {
    els.cameraSelect.value = String(activeIndex);
  }
}

function syncCameraSelectionFromState(vision) {
  if (!els.cameraSelect || !vision) {
    return;
  }
  const selectedIndex = vision.selected_camera_index;
  const activeIndex = vision.active_camera_index;
  if (selectedIndex !== null && selectedIndex !== undefined && !cameraSelectionBusy) {
    els.cameraSelect.value = String(selectedIndex);
  } else if (activeIndex !== null && activeIndex !== undefined && !cameraSelectionBusy) {
    els.cameraSelect.value = String(activeIndex);
  }
}

async function refreshCameraCatalog(forceRefresh = false) {
  const response = await fetch(`/api/vision/cameras${forceRefresh ? "?refresh=1" : ""}`, { cache: "no-store" });
  const data = await response.json().catch(() => ({}));
  if (!response.ok) {
    throw new Error(data.message || "No se pudo leer la lista de webcams.");
  }
  renderCameraCatalog(data);
  return data;
}

async function changeCamera(index) {
  cameraSelectionBusy = true;
  if (els.cameraSelect) {
    els.cameraSelect.disabled = true;
  }
  if (buttons.refreshCameras) {
    buttons.refreshCameras.disabled = true;
  }
  try {
    const data = await postJSON("/api/vision/camera", { index: Number(index) });
    renderCameraCatalog(data);
  } finally {
    cameraSelectionBusy = false;
    if (els.cameraSelect) {
      els.cameraSelect.disabled = !latestCameraCatalog?.cameras?.length;
    }
    if (buttons.refreshCameras) {
      buttons.refreshCameras.disabled = false;
    }
  }
}

async function refreshCalibration() {
  const response = await fetch("/api/calibration/hand-follow", { cache: "no-store" });
  const calibration = await response.json();
  renderCalibration(calibration);
}

function renderState(snapshot) {
  latestState = snapshot;
  const vision = snapshot.vision_status || {};
  const tracking = snapshot.tracking_status || {};
  const hand = snapshot.hand_target || {};
  const pose = snapshot.robot_status?.current_pose_mm || [];
  const connected = Boolean(snapshot.robot_status?.connected);
  const handDetectorMode = vision.hand_detector_mode || window.SILVIA_SETTINGS?.hand_detector_backend || "mediapipe";
  const activeAxis = tracking.active_axis === "xy"
    ? "XY"
    : (tracking.active_axis === "y" ? "Y" : "X");
  const fixedAxis = tracking.active_axis === "xy"
    ? "ninguno"
    : (activeAxis === "Y" ? "X" : "Y");

  els.origin.textContent = `Origen: ${window.location.origin}`;
  els.detector.textContent = formatHandDetectorLabel(handDetectorMode);
  els.detector.classList.toggle("is-warning", false);
  els.detector.classList.toggle("is-ok", true);

  els.mode.textContent = snapshot.mode || "idle";
  els.cameraStatus.textContent = vision.ok ? "Webcam OK" : (vision.message || "Sin datos");
  syncCameraSelectionFromState(vision);
  els.robotStatus.textContent = connected ? "Conectado" : "Desconectado";
  els.safetyStatus.textContent = snapshot.safety_locked ? snapshot.safety_message : "Sistema listo";
  els.handStatus.textContent = hand.world_mm ? `Detectada (${formatVector(hand.world_mm)})` : "No detectada";
  els.stopReason.textContent = tracking.stop_reason || "Sin paro";

  els.robotPose.textContent = JSON.stringify({
    x_mm: pose[0] || 0,
    y_mm: pose[1] || 0,
    z_mm: pose[2] || 0,
    rx: pose[3] || 0,
    ry: pose[4] || 0,
    rz: pose[5] || 0,
  }, null, 2);

  els.trackingSummary.textContent = [
    `Mano XYZ: ${formatVector(tracking.hand_world_mm || hand.world_mm)}`,
    `Objetivo XYZ (${fixedAxis === "ninguno" ? "X/Y activos" : `${fixedAxis} fijo`}): ${formatVector(tracking.target_xyz_mm)}`,
    `Delta XYZ (${activeAxis} activo): ${formatVector(tracking.error_xyz_mm)}`,
    `TCP -> mano: ${formatNumber(tracking.distance_tcp_to_hand_mm, 3)} mm`,
    `TCP -> objetivo: ${formatNumber(tracking.distance_tcp_to_target_mm, 3)} mm`,
    `Delta distancia: ${formatNumber(tracking.distance_delta_mm, 3)} mm`,
    `Tendencia: ${formatApproachTrend(tracking.approaching_hand)}`,
    `Log activo: ${tracking.distance_log_path || "N/A"}`,
    `Velocidad: ${formatVector(tracking.velocity_mm_s)} mm/s`,
    `Edad target: ${formatNumber(tracking.target_age_s, 3)} s`,
  ].join("\n");

  if (latestCalibration) {
    renderCalibration(latestCalibration);
  }
}

async function savePlane(planeZ) {
  const data = await postJSON("/api/calibration/hand-follow", {
    plane_z_mm: planeZ,
    save: true,
  });
  renderCalibration(data.hand_follow);
}

buttons.start.addEventListener("click", () => {
  postJSON("/api/mode", { mode: "hand_follow" }).catch((error) => alert(error.message));
});

buttons.stop.addEventListener("click", () => {
  postJSON("/api/mode", { mode: "idle" }).catch((error) => alert(error.message));
});

buttons.home.addEventListener("click", () => {
  postJSON("/api/agent/tool", { action: "go_home" }).catch((error) => alert(error.message));
});

buttons.resetSafety.addEventListener("click", () => {
  postJSON("/api/safety/reset").catch((error) => alert(error.message));
});

buttons.calibratePlane.addEventListener("click", async () => {
  try {
    const z = latestState?.robot_status?.current_pose_mm?.[2];
    if (z === null || z === undefined) {
      throw new Error("No hay Z actual del robot.");
    }
    await savePlane(Number(z));
  } catch (error) {
    alert(error.message);
  }
});

buttons.savePlane.addEventListener("click", async () => {
  try {
    const planeZ = Number(els.planeZInput.value);
    if (!Number.isFinite(planeZ)) {
      throw new Error("Z plano debe ser numerico.");
    }
    await savePlane(planeZ);
  } catch (error) {
    alert(error.message);
  }
});

buttons.refreshCameras?.addEventListener("click", async () => {
  try {
    buttons.refreshCameras.disabled = true;
    await refreshCameraCatalog(true);
  } catch (error) {
    alert(error.message);
  } finally {
    buttons.refreshCameras.disabled = false;
  }
});

els.cameraSelect?.addEventListener("change", async (event) => {
  const value = event.target.value;
  if (value === "") {
    return;
  }
  try {
    await changeCamera(value);
  } catch (error) {
    alert(error.message);
    refreshCameraCatalog(true).catch(() => {});
  }
});

socket.on("state", renderState);

refreshCalibration().catch(() => {
  els.planeSummary.textContent = "No se pudo leer la calibracion.";
});
refreshCameraCatalog().catch(() => {});
setInterval(() => refreshCalibration().catch(() => {}), 3000);
