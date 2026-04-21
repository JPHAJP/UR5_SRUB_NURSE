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
  robotLogDot: document.getElementById("robot-log-dot"),
  robotLogMessage: document.getElementById("robot-log-message"),
  systemEventDot: document.getElementById("system-event-dot"),
  systemEventMessage: document.getElementById("system-event-message"),
  safetyEventDot: document.getElementById("safety-event-dot"),
  safetyEventMessage: document.getElementById("safety-event-message"),
  voiceOutput: document.getElementById("voice-output"),
  networkOrigin: document.getElementById("network-origin"),
  networkVoiceHint: document.getElementById("network-voice-hint"),
  transcribeTestStatus: document.getElementById("transcribe-test-status"),
  transcribeTestOutput: document.getElementById("transcribe-test-output"),
  transcribeTestPath: document.getElementById("transcribe-test-path"),
  pushToTalkStatus: document.getElementById("push-to-talk-status"),
  pushToTalkOutput: document.getElementById("push-to-talk-output"),
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
  transcribeTest: document.getElementById("transcribe-test-btn"),
  pushToTalk: document.getElementById("push-to-talk-btn"),
};

const chatForm = document.getElementById("chat-form");
const chatInput = document.getElementById("chat-input");

let latestState = null;
let voiceEnabled = false;
let recordLoopLocked = false;
let voiceRequiresSecureOrigin = false;
let transcribeTestRunning = false;
let transcribeTestRecorder = null;
let voiceRecorder = null;
let pushToTalkRunning = false;
let pushToTalkRecorder = null;
let activeExclusiveCapture = null;
let resumeVoiceAfterExclusiveCapture = false;

function isLocalhostHostname(hostname) {
  return hostname === "localhost" || hostname === "127.0.0.1" || hostname === "::1";
}

function syncVoiceToggleButton() {
  if (voiceRequiresSecureOrigin) {
    buttons.voiceToggle.textContent = "Voz requiere HTTPS";
    return;
  }
  if (activeExclusiveCapture) {
    buttons.voiceToggle.textContent = "Voz en pausa";
    return;
  }
  if (voiceEnabled) {
    buttons.voiceToggle.textContent = "Detener Voz";
    return;
  }
  buttons.voiceToggle.textContent = "Activar Voz";
}

function renderNetworkStatus() {
  const { origin, protocol, hostname } = window.location;
  const secureContext = window.isSecureContext;
  const localhost = isLocalhostHostname(hostname);

  stateEls.networkOrigin.textContent = `Origen actual: ${origin}`;
  stateEls.networkOrigin.classList.remove("is-warning", "is-ok");
  stateEls.networkOrigin.classList.add("is-ok");

  voiceRequiresSecureOrigin = !secureContext && !localhost;
  buttons.voiceToggle.disabled = voiceRequiresSecureOrigin || Boolean(activeExclusiveCapture);
  buttons.transcribeTest.disabled = voiceRequiresSecureOrigin && !transcribeTestRunning;
  buttons.pushToTalk.disabled = voiceRequiresSecureOrigin && !pushToTalkRunning;

  stateEls.networkVoiceHint.classList.remove("is-warning", "is-ok");
  if (voiceRequiresSecureOrigin) {
    syncVoiceToggleButton();
    stateEls.networkVoiceHint.textContent =
      "Acceso LAN listo. La voz remota necesita HTTPS para que el navegador habilite el microfono.";
    stateEls.networkVoiceHint.classList.add("is-warning");
    return;
  }

  syncVoiceToggleButton();
  if (protocol === "https:") {
    stateEls.networkVoiceHint.textContent =
      "Conexion segura activa. El microfono remoto y la voz continua deberian funcionar en LAN.";
  } else {
    stateEls.networkVoiceHint.textContent =
      "Conexion local detectada. En este equipo la voz puede usarse sin HTTPS.";
  }
  stateEls.networkVoiceHint.classList.add("is-ok");
}

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
    const shownText = message.display_text || message.text;
    node.innerHTML = `<strong>${message.role === "user" ? "Tu" : "SILVIA"}</strong><div>${shownText}</div>`;
    stateEls.conversationLog.appendChild(node);
  });

  renderSignalState(snapshot.events || []);
}

function renderSignalState(events) {
  const latestRobotLog = findLatestEvent(events, ["robot_log"]);
  const latestSystemEvent = findLatestEvent(events, ["system_event"]);
  const latestSafetyEvent = findLatestEvent(events, ["safety_fault", "safety_event"]);

  setSignalCard(
    stateEls.robotLogDot,
    stateEls.robotLogMessage,
    latestRobotLog ? latestRobotLog.message : "Sin eventos",
    latestRobotLog ? "green" : "neutral",
  );
  setSignalCard(
    stateEls.systemEventDot,
    stateEls.systemEventMessage,
    latestSystemEvent ? latestSystemEvent.message : "Sin eventos",
    latestSystemEvent ? "yellow" : "neutral",
  );
  setSignalCard(
    stateEls.safetyEventDot,
    stateEls.safetyEventMessage,
    latestSafetyEvent ? latestSafetyEvent.message : "Sin eventos",
    latestSafetyEvent?.kind === "safety_fault" ? "red" : latestSafetyEvent ? "green" : "neutral",
  );
}

function findLatestEvent(events, kinds) {
  return [...events].reverse().find((event) => kinds.includes(event.kind)) || null;
}

function setSignalCard(dotEl, messageEl, message, color) {
  dotEl.classList.remove("is-green", "is-yellow", "is-red");
  if (color === "green") {
    dotEl.classList.add("is-green");
  } else if (color === "yellow") {
    dotEl.classList.add("is-yellow");
  } else if (color === "red") {
    dotEl.classList.add("is-red");
  }
  messageEl.textContent = message;
}

function stopVoiceOutput() {
  stateEls.voiceOutput.pause();
  stateEls.voiceOutput.currentTime = 0;
  stateEls.voiceOutput.removeAttribute("src");
  stateEls.voiceOutput.load();
}

function playVoiceOutput(audioBase64) {
  if (!audioBase64) return;
  stopVoiceOutput();
  stateEls.voiceOutput.src = `data:audio/mp3;base64,${audioBase64}`;
  stateEls.voiceOutput.play().catch(() => {});
}

async function playVoiceOutputAndWait(audioBase64) {
  if (!audioBase64) return;

  stopVoiceOutput();
  stateEls.voiceOutput.src = `data:audio/mp3;base64,${audioBase64}`;

  await new Promise((resolve) => {
    const cleanup = () => {
      stateEls.voiceOutput.removeEventListener("ended", onDone);
      stateEls.voiceOutput.removeEventListener("error", onDone);
      resolve();
    };
    const onDone = () => cleanup();

    stateEls.voiceOutput.addEventListener("ended", onDone, { once: true });
    stateEls.voiceOutput.addEventListener("error", onDone, { once: true });
    stateEls.voiceOutput.play().catch(() => cleanup());
  });
}

function currentChunkDurationMs() {
  const configuredMs = window.SILVIA_SETTINGS?.voice_chunk_ms || 2500;
  if (!stateEls.voiceOutput.paused) {
    return Math.min(configuredMs, 1200);
  }
  return configuredMs;
}

function wait(ms) {
  return new Promise((resolve) => {
    setTimeout(resolve, ms);
  });
}

function mergeFloat32Chunks(chunks) {
  const totalLength = chunks.reduce((sum, chunk) => sum + chunk.length, 0);
  const merged = new Float32Array(totalLength);
  let offset = 0;
  chunks.forEach((chunk) => {
    merged.set(chunk, offset);
    offset += chunk.length;
  });
  return merged;
}

function downsampleBuffer(buffer, inputSampleRate, outputSampleRate) {
  if (outputSampleRate >= inputSampleRate) {
    return buffer;
  }

  const ratio = inputSampleRate / outputSampleRate;
  const newLength = Math.round(buffer.length / ratio);
  const result = new Float32Array(newLength);
  let offsetResult = 0;
  let offsetBuffer = 0;

  while (offsetResult < result.length) {
    const nextOffsetBuffer = Math.round((offsetResult + 1) * ratio);
    let accum = 0;
    let count = 0;
    for (let i = offsetBuffer; i < nextOffsetBuffer && i < buffer.length; i += 1) {
      accum += buffer[i];
      count += 1;
    }
    result[offsetResult] = count > 0 ? accum / count : 0;
    offsetResult += 1;
    offsetBuffer = nextOffsetBuffer;
  }

  return result;
}

function encodeWavFromFloat32(samples, sampleRate) {
  const bytesPerSample = 2;
  const blockAlign = bytesPerSample;
  const byteRate = sampleRate * blockAlign;
  const dataSize = samples.length * bytesPerSample;
  const buffer = new ArrayBuffer(44 + dataSize);
  const view = new DataView(buffer);

  function writeString(offset, value) {
    for (let index = 0; index < value.length; index += 1) {
      view.setUint8(offset + index, value.charCodeAt(index));
    }
  }

  writeString(0, "RIFF");
  view.setUint32(4, 36 + dataSize, true);
  writeString(8, "WAVE");
  writeString(12, "fmt ");
  view.setUint32(16, 16, true);
  view.setUint16(20, 1, true);
  view.setUint16(22, 1, true);
  view.setUint32(24, sampleRate, true);
  view.setUint32(28, byteRate, true);
  view.setUint16(32, blockAlign, true);
  view.setUint16(34, 16, true);
  writeString(36, "data");
  view.setUint32(40, dataSize, true);

  let offset = 44;
  for (let index = 0; index < samples.length; index += 1) {
    const sample = Math.max(-1, Math.min(1, samples[index]));
    view.setInt16(offset, sample < 0 ? sample * 0x8000 : sample * 0x7fff, true);
    offset += 2;
  }

  return new Blob([buffer], { type: "audio/wav" });
}

async function createPcmRecorderSession() {
  const stream = await navigator.mediaDevices.getUserMedia({
    audio: {
      echoCancellation: true,
      noiseSuppression: true,
      autoGainControl: false,
      channelCount: 1,
    },
  });

  const audioContext = new (window.AudioContext || window.webkitAudioContext)();
  const source = audioContext.createMediaStreamSource(stream);
  const processor = audioContext.createScriptProcessor(4096, 1, 1);
  const chunks = [];

  processor.onaudioprocess = (event) => {
    const input = event.inputBuffer.getChannelData(0);
    chunks.push(new Float32Array(input));
  };

  source.connect(processor);
  processor.connect(audioContext.destination);

  return {
    stream,
    audioContext,
    source,
    processor,
    chunks,
    sampleRate: audioContext.sampleRate,
    startedAt: Date.now(),
  };
}

async function closePcmRecorderSession(recorder) {
  recorder.processor.disconnect();
  recorder.source.disconnect();
  recorder.stream.getTracks().forEach((track) => track.stop());
  await recorder.audioContext.close();
}

function flushPcmRecorderChunk(recorder) {
  const merged = mergeFloat32Chunks(recorder.chunks);
  recorder.chunks = [];
  return merged;
}

async function startTranscriptionTestRecording() {
  transcribeTestRecorder = await createPcmRecorderSession();
}

async function stopTranscriptionTestRecording() {
  if (!transcribeTestRecorder) {
    throw new Error("No habia una grabacion activa.");
  }

  const recorder = transcribeTestRecorder;
  transcribeTestRecorder = null;

  const merged = flushPcmRecorderChunk(recorder);
  await closePcmRecorderSession(recorder);
  if (!merged.length) {
    throw new Error("No se capturo audio del microfono.");
  }

  const downsampled = downsampleBuffer(merged, recorder.sampleRate, 16000);
  const audioBlob = encodeWavFromFloat32(downsampled, 16000);
  const durationMs = Date.now() - recorder.startedAt;
  return { audioBlob, durationMs };
}

function updateTranscriptionTestIdleState() {
  buttons.transcribeTest.disabled = voiceRequiresSecureOrigin;
  buttons.transcribeTest.textContent = "Iniciar Prueba";
}

function updateTranscriptionTestRecordingState() {
  buttons.transcribeTest.disabled = false;
  buttons.transcribeTest.textContent = "Detener y Transcribir";
}

async function startPushToTalkRecording() {
  pushToTalkRecorder = await createPcmRecorderSession();
}

async function stopPushToTalkRecording() {
  if (!pushToTalkRecorder) {
    throw new Error("No habia una grabacion push to talk activa.");
  }

  const recorder = pushToTalkRecorder;
  pushToTalkRecorder = null;

  const merged = flushPcmRecorderChunk(recorder);
  await closePcmRecorderSession(recorder);
  if (!merged.length) {
    throw new Error("No se capturo audio del microfono.");
  }

  const downsampled = downsampleBuffer(merged, recorder.sampleRate, 16000);
  const audioBlob = encodeWavFromFloat32(downsampled, 16000);
  const durationMs = Date.now() - recorder.startedAt;
  return { audioBlob, durationMs };
}

function updatePushToTalkIdleState() {
  buttons.pushToTalk.disabled = voiceRequiresSecureOrigin;
  buttons.pushToTalk.textContent = "Iniciar Push To Talk";
}

function updatePushToTalkRecordingState() {
  buttons.pushToTalk.disabled = false;
  buttons.pushToTalk.textContent = "Detener y Enviar";
}

async function startVoiceRecorder() {
  if (voiceRecorder) return;
  voiceRecorder = await createPcmRecorderSession();
}

async function stopVoiceRecorder() {
  if (!voiceRecorder) return;
  const recorder = voiceRecorder;
  voiceRecorder = null;
  await closePcmRecorderSession(recorder);
}

async function suspendContinuousVoiceForExclusiveCapture() {
  if (!voiceEnabled) {
    return false;
  }

  voiceEnabled = false;
  syncVoiceToggleButton();

  for (let attempt = 0; attempt < 30 && recordLoopLocked; attempt += 1) {
    await wait(120);
  }

  await stopVoiceRecorder();
  return true;
}

async function resumeContinuousVoiceAfterExclusiveCapture(shouldResume) {
  if (!shouldResume) {
    syncVoiceToggleButton();
    return;
  }

  try {
    await startVoiceRecorder();
    voiceEnabled = true;
    syncVoiceToggleButton();
    processVoiceChunkLoop();
  } catch (error) {
    voiceEnabled = false;
    syncVoiceToggleButton();
    console.warn("No se pudo reanudar la voz continua:", error.message);
  }
}

async function beginExclusiveCapture(mode) {
  if (activeExclusiveCapture && activeExclusiveCapture !== mode) {
    throw new Error("Ya hay otra captura de microfono activa.");
  }
  if (activeExclusiveCapture === mode) {
    return;
  }

  activeExclusiveCapture = mode;
  resumeVoiceAfterExclusiveCapture = await suspendContinuousVoiceForExclusiveCapture();
  renderNetworkStatus();
}

async function finishExclusiveCapture(mode) {
  if (activeExclusiveCapture !== mode) {
    return;
  }

  activeExclusiveCapture = null;
  const shouldResume = resumeVoiceAfterExclusiveCapture;
  resumeVoiceAfterExclusiveCapture = false;
  await resumeContinuousVoiceAfterExclusiveCapture(shouldResume);
  renderNetworkStatus();
}

async function buildVoiceChunkBlob(durationMs) {
  if (!voiceRecorder) {
    throw new Error("No hay grabacion de voz activa.");
  }

  await new Promise((resolve) => {
    setTimeout(resolve, durationMs);
  });

  const merged = flushPcmRecorderChunk(voiceRecorder);
  if (!merged.length) {
    return null;
  }

  const downsampled = downsampleBuffer(merged, voiceRecorder.sampleRate, 16000);
  if (!downsampled.length) {
    return null;
  }

  return encodeWavFromFloat32(downsampled, 16000);
}

async function runTranscriptionTest() {
  if (voiceRequiresSecureOrigin) {
    alert("La prueba de microfono tambien requiere HTTPS fuera de localhost.");
    return;
  }

  if (!transcribeTestRunning) {
    try {
      await beginExclusiveCapture("transcribe-test");
      buttons.transcribeTest.disabled = true;
      stateEls.transcribeTestStatus.textContent = "Abriendo microfono...";
      stateEls.transcribeTestOutput.textContent = "Esperando voz...";
      stateEls.transcribeTestPath.textContent = "Se mostrara aqui la ruta del WAV guardado.";
      await startTranscriptionTestRecording();
      transcribeTestRunning = true;
      updateTranscriptionTestRecordingState();
      stateEls.transcribeTestStatus.textContent = "Grabando WAV limpio. Cuando termines, pulsa detener.";
    } catch (error) {
      await finishExclusiveCapture("transcribe-test");
      updateTranscriptionTestIdleState();
      stateEls.transcribeTestStatus.textContent = `Error: ${error.message}`;
      stateEls.transcribeTestOutput.textContent = "(No se pudo iniciar la grabacion)";
    }
    return;
  }

  try {
    buttons.transcribeTest.disabled = true;
    stateEls.transcribeTestStatus.textContent = "Preparando WAV y enviando audio a Flask...";
    const { audioBlob, durationMs } = await stopTranscriptionTestRecording();

    const formData = new FormData();
    formData.append("audio", audioBlob, "transcribe-test.wav");

    const response = await fetch("/api/voice/transcribe-test", {
      method: "POST",
      body: formData,
    });
    const data = await response.json().catch(() => ({}));
    if (!response.ok) {
      throw new Error(data.message || "Fallo la prueba de transcripcion.");
    }

    stateEls.transcribeTestStatus.textContent =
      `${data.message || "Transcripcion completada."} Audio enviado: WAV mono 16 kHz, ${(durationMs / 1000).toFixed(1)} s.`;
    stateEls.transcribeTestOutput.textContent = data.transcript || "(Sin texto reconocido)";
    stateEls.transcribeTestPath.textContent = [
      data.saved_audio_path ? `Audio: ${data.saved_audio_path}` : null,
      data.saved_meta_path ? `Meta: ${data.saved_meta_path}` : null,
    ].filter(Boolean).join("\n");
  } catch (error) {
    stateEls.transcribeTestStatus.textContent = `Error: ${error.message}`;
    stateEls.transcribeTestOutput.textContent = "(La prueba no produjo transcripcion)";
    stateEls.transcribeTestPath.textContent = "(No se guardaron archivos en esta prueba)";
  } finally {
    transcribeTestRunning = false;
    updateTranscriptionTestIdleState();
    await finishExclusiveCapture("transcribe-test");
  }
}

async function runPushToTalk() {
  if (voiceRequiresSecureOrigin) {
    alert("Push to talk tambien requiere HTTPS fuera de localhost.");
    return;
  }

  if (!pushToTalkRunning) {
    try {
      await beginExclusiveCapture("push-to-talk");
      buttons.pushToTalk.disabled = true;
      stateEls.pushToTalkStatus.textContent = "Abriendo microfono para push to talk...";
      stateEls.pushToTalkOutput.textContent = "Esperando tu turno manual...";
      await startPushToTalkRecording();
      pushToTalkRunning = true;
      updatePushToTalkRecordingState();
      stateEls.pushToTalkStatus.textContent = "Grabando. Cuando termines, pulsa detener y enviar.";
    } catch (error) {
      await finishExclusiveCapture("push-to-talk");
      updatePushToTalkIdleState();
      stateEls.pushToTalkStatus.textContent = `Error: ${error.message}`;
      stateEls.pushToTalkOutput.textContent = "(No se pudo iniciar el push to talk)";
    }
    return;
  }

  try {
    buttons.pushToTalk.disabled = true;
    stateEls.pushToTalkStatus.textContent = "Preparando WAV y enviando turno manual...";
    const { audioBlob, durationMs } = await stopPushToTalkRecording();

    const formData = new FormData();
    formData.append("audio", audioBlob, "push-to-talk.wav");

    const response = await fetch("/api/voice/push-to-talk", {
      method: "POST",
      body: formData,
    });
    const data = await response.json().catch(() => ({}));
    if (!response.ok) {
      throw new Error(data.message || "Fallo el push to talk.");
    }

    stateEls.pushToTalkStatus.textContent =
      `${data.message || "Turno procesado."} Audio enviado: WAV mono 16 kHz, ${(durationMs / 1000).toFixed(1)} s.`;
    stateEls.pushToTalkOutput.textContent = data.raw_heard || "(Sin texto reconocido)";

    if (data.interrupt_audio) {
      stopVoiceOutput();
    }
    if (data.audio_base64) {
      await playVoiceOutputAndWait(data.audio_base64);
    }
  } catch (error) {
    stateEls.pushToTalkStatus.textContent = `Error: ${error.message}`;
    stateEls.pushToTalkOutput.textContent = "(El turno manual no produjo transcripcion)";
  } finally {
    pushToTalkRunning = false;
    updatePushToTalkIdleState();
    await finishExclusiveCapture("push-to-talk");
  }
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
buttons.transcribeTest.addEventListener("click", () => runTranscriptionTest());
buttons.pushToTalk.addEventListener("click", () => runPushToTalk());

chatForm.addEventListener("submit", async (event) => {
  event.preventDefault();
  const text = chatInput.value.trim();
  if (!text) return;
  chatInput.value = "";
  try {
    const data = await postJSON("/api/conversation/message", { text });
    playVoiceOutput(data.audio_base64);
  } catch (error) {
    alert(error.message);
  }
});

buttons.voiceToggle.addEventListener("click", async () => {
  if (voiceRequiresSecureOrigin) {
    alert("La voz continua en LAN requiere HTTPS. Activa FLASK_SSL_MODE=adhoc o configura un certificado.");
    return;
  }
  if (activeExclusiveCapture) {
    alert("La voz continua esta en pausa mientras otra captura de microfono esta activa.");
    return;
  }
  if (voiceEnabled) {
    stopVoiceLoop();
    return;
  }
  try {
    await startVoiceRecorder();
    voiceEnabled = true;
    syncVoiceToggleButton();
    processVoiceChunkLoop();
  } catch (error) {
    alert(`No se pudo iniciar el microfono: ${error.message}`);
  }
});

function stopVoiceLoop() {
  voiceEnabled = false;
  recordLoopLocked = false;
  stopVoiceRecorder().catch((error) => console.warn("No se pudo cerrar el microfono:", error.message));
  renderNetworkStatus();
}

async function processVoiceChunkLoop() {
  if (!voiceEnabled || !voiceRecorder || recordLoopLocked) return;
  recordLoopLocked = true;
  try {
    const blob = await buildVoiceChunkBlob(currentChunkDurationMs());

    if (!voiceEnabled) {
      recordLoopLocked = false;
      return;
    }

    if (!blob) {
      recordLoopLocked = false;
      setTimeout(processVoiceChunkLoop, 150);
      return;
    }

    const formData = new FormData();
    formData.append("audio", blob, "voice-chunk.wav");

    const response = await fetch("/api/voice/chunk", {
      method: "POST",
      body: formData,
    });
    const data = await response.json().catch(() => ({}));

    if (!response.ok) {
      throw new Error(data.message || "Fallo el procesamiento de voz.");
    }

    if (data.interrupt_audio) {
      stopVoiceOutput();
    }

    if (data.audio_base64) {
      await stopVoiceRecorder();
      await playVoiceOutputAndWait(data.audio_base64);
      if (voiceEnabled) {
        await startVoiceRecorder();
      }
    }
    recordLoopLocked = false;
  } catch (error) {
    recordLoopLocked = false;
    console.warn("Error de voz:", error.message);
  } finally {
    if (voiceEnabled) {
      setTimeout(processVoiceChunkLoop, 220);
    }
  }
}

renderNetworkStatus();
updateTranscriptionTestIdleState();
updatePushToTalkIdleState();
