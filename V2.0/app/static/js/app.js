const socket = io();
const stateEls = {
  mode: document.getElementById("mode-pill"),
  robotConnection: document.getElementById("robot-connection"),
  safetyMessage: document.getElementById("safety-message"),
  visionMessage: document.getElementById("vision-message"),
  voiceStatus: document.getElementById("voice-status"),
  robotPose: document.getElementById("robot-pose"),
  conversationId: document.getElementById("conversation-id"),
  conversationLog: document.getElementById("conversation-log"),
  eventsLog: document.getElementById("events-log"),
  voiceOutput: document.getElementById("voice-output"),
};

const buttons = {
  greet: document.getElementById("greet-btn"),
  voiceToggle: document.getElementById("voice-toggle-btn"),
  resetConversation: document.getElementById("reset-conversation-btn"),
  modeHand: document.getElementById("mode-hand-btn"),
  modeObject: document.getElementById("mode-object-btn"),
  modeIdle: document.getElementById("mode-idle-btn"),
  cancel: document.getElementById("cancel-btn"),
  home: document.getElementById("home-btn"),
  release: document.getElementById("release-btn"),
  resetSafety: document.getElementById("reset-safety-btn"),
};

const chatForm = document.getElementById("chat-form");
const chatInput = document.getElementById("chat-input");

let latestState = null;
let voiceEnabled = false;
let mediaStream = null;
let recordLoopLocked = false;

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

function renderState(snapshot) {
  latestState = snapshot;
  stateEls.mode.textContent = snapshot.mode;
  stateEls.robotConnection.textContent = snapshot.robot_status.connected ? "Conectado" : "Desconectado";
  stateEls.safetyMessage.textContent = snapshot.safety_message || "Sin datos";
  stateEls.visionMessage.textContent = snapshot.vision_status.ok
    ? `${snapshot.vision_status.detections || 0} detecciones`
    : (snapshot.vision_status.message || "Sin datos");
  stateEls.voiceStatus.textContent = snapshot.voice_active ? "Escuchando" : "Inactiva";
  stateEls.conversationId.textContent = snapshot.conversation_id || "Sin sesion";

  const pose = snapshot.robot_status.current_pose_mm || [];
  stateEls.robotPose.textContent = JSON.stringify({
    x_mm: pose[0] || 0,
    y_mm: pose[1] || 0,
    z_mm: pose[2] || 0,
    rx: pose[3] || 0,
    ry: pose[4] || 0,
    rz: pose[5] || 0,
    magnet: snapshot.magnet_enabled,
    safety_locked: snapshot.safety_locked,
  }, null, 2);

  stateEls.conversationLog.innerHTML = "";
  (snapshot.messages || []).forEach((message) => {
    const node = document.createElement("div");
    node.className = `message ${message.role}`;
    node.innerHTML = `<strong>${message.role === "user" ? "Tu" : "SILVIA"}</strong><div>${message.text}</div>`;
    stateEls.conversationLog.appendChild(node);
  });

  stateEls.eventsLog.innerHTML = "";
  (snapshot.events || []).slice(-20).reverse().forEach((event) => {
    const node = document.createElement("div");
    node.className = `event ${event.kind}`;
    node.innerHTML = `<strong>${event.kind}</strong><div>${event.message}</div>`;
    stateEls.eventsLog.appendChild(node);
  });
}

socket.on("state", renderState);
socket.on("robot_log", ({ message }) => console.debug("[robot]", message));
socket.on("safety_fault", ({ message }) => console.warn("[safety]", message));

buttons.greet.addEventListener("click", () => postJSON("/api/robot/greet").catch(alert));
buttons.resetConversation.addEventListener("click", () => postJSON("/api/conversation/reset").catch(alert));
buttons.modeHand.addEventListener("click", () => postJSON("/api/mode", { mode: "hand_follow" }).catch(alert));
buttons.modeObject.addEventListener("click", () => postJSON("/api/mode", { mode: "object_pick" }).catch(alert));
buttons.modeIdle.addEventListener("click", () => postJSON("/api/mode", { mode: "idle" }).catch(alert));
buttons.cancel.addEventListener("click", () => postJSON("/api/agent/tool", { action: "cancel" }).catch(alert));
buttons.home.addEventListener("click", () => postJSON("/api/agent/tool", { action: "go_home" }).catch(alert));
buttons.release.addEventListener("click", () => postJSON("/api/agent/tool", { action: "release_object" }).catch(alert));
buttons.resetSafety.addEventListener("click", () => postJSON("/api/safety/reset").catch(alert));

chatForm.addEventListener("submit", async (event) => {
  event.preventDefault();
  const text = chatInput.value.trim();
  if (!text) return;
  chatInput.value = "";
  try {
    await postJSON("/api/conversation/message", { text });
  } catch (error) {
    alert(error.message);
  }
});

buttons.voiceToggle.addEventListener("click", async () => {
  if (voiceEnabled) {
    stopVoiceLoop();
    return;
  }
  try {
    mediaStream = await navigator.mediaDevices.getUserMedia({ audio: true });
    voiceEnabled = true;
    buttons.voiceToggle.textContent = "Detener Voz";
    processVoiceChunkLoop();
  } catch (error) {
    alert(`No se pudo iniciar el microfono: ${error.message}`);
  }
});

function stopVoiceLoop() {
  voiceEnabled = false;
  recordLoopLocked = false;
  buttons.voiceToggle.textContent = "Activar Voz";
  if (mediaStream) {
    mediaStream.getTracks().forEach((track) => track.stop());
    mediaStream = null;
  }
}

async function processVoiceChunkLoop() {
  if (!voiceEnabled || !mediaStream || recordLoopLocked) return;
  recordLoopLocked = true;

  const mimeType = MediaRecorder.isTypeSupported("audio/webm;codecs=opus")
    ? "audio/webm;codecs=opus"
    : "audio/webm";

  const recorder = new MediaRecorder(mediaStream, { mimeType });
  const parts = [];

  recorder.ondataavailable = (event) => {
    if (event.data && event.data.size > 0) {
      parts.push(event.data);
    }
  };

  recorder.onstop = async () => {
    recordLoopLocked = false;
    if (!voiceEnabled || !parts.length) {
      if (voiceEnabled) {
        setTimeout(processVoiceChunkLoop, 200);
      }
      return;
    }

    const blob = new Blob(parts, { type: mimeType });
    const formData = new FormData();
    formData.append("audio", blob, "voice-chunk.webm");

    try {
      const response = await fetch("/api/voice/chunk", {
        method: "POST",
        body: formData,
      });
      const data = await response.json();
      if (data.audio_base64) {
        stateEls.voiceOutput.src = `data:audio/mp3;base64,${data.audio_base64}`;
        stateEls.voiceOutput.play().catch(() => {});
      }
    } catch (error) {
      console.warn("Error de voz:", error.message);
    } finally {
      if (voiceEnabled) {
        setTimeout(processVoiceChunkLoop, 220);
      }
    }
  };

  recorder.start();
  setTimeout(() => {
    if (recorder.state !== "inactive") {
      recorder.stop();
    }
  }, window.SILVIA_SETTINGS?.voice_chunk_ms || 2500);
}
