const socket = io();

const els = {
  origin: document.getElementById("demo-origin"),
  detector: document.getElementById("demo-detector"),
  cameraStatus: document.getElementById("demo-camera-status"),
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
};

let latestState = null;
let latestCalibration = null;

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
    `Plano Z: ${formatNumber(calibration.plane_z_mm, 1)} mm`,
    `Offset Z: ${formatNumber(calibration.z_offset_mm, 1)} mm`,
    `Objetivo Z: ${formatNumber(calibration.target_z_mm, 1)} mm`,
  ].join("\n");
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
  const detectorMode = vision.detector_mode || window.SILVIA_SETTINGS?.vision_detector_mode || "hand_only";

  els.origin.textContent = `Origen: ${window.location.origin}`;
  els.detector.textContent = detectorMode === "hand_only" ? "YOLO mano" : "YOLO activo";
  els.detector.classList.toggle("is-warning", false);
  els.detector.classList.toggle("is-ok", true);

  els.mode.textContent = snapshot.mode || "idle";
  els.cameraStatus.textContent = vision.rgb_ok && vision.depth_ok ? "RGB/depth OK" : (vision.message || "Sin datos");
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
    `Objetivo XYZ: ${formatVector(tracking.target_xyz_mm)}`,
    `Delta XYZ: ${formatVector(tracking.error_xyz_mm)}`,
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

socket.on("state", renderState);

refreshCalibration().catch(() => {
  els.planeSummary.textContent = "No se pudo leer la calibracion.";
});
setInterval(() => refreshCalibration().catch(() => {}), 3000);
