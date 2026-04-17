from __future__ import annotations

import base64
import logging
from typing import Any, Dict

import requests

from .command_router import normalize_text

logger = logging.getLogger(__name__)


class VoiceService:
    def __init__(self, config, state, conversation_service) -> None:
        self.config = config
        self.state = state
        self.conversation_service = conversation_service

    def create_realtime_session(self) -> Dict[str, Any]:
        if not self.config.openai_api_key:
            return {"ok": False, "message": "Falta OPENAI_API_KEY."}

        try:
            response = requests.post(
                "https://api.openai.com/v1/realtime/sessions",
                headers={
                    "Authorization": f"Bearer {self.config.openai_api_key}",
                    "Content-Type": "application/json",
                },
                json={
                    "model": self.config.openai_realtime_model,
                    "voice": "alloy",
                    "instructions": (
                        "Eres SILVIA. Responde en espanol, breve y clara. "
                        "Prioriza seguridad y confirma si el usuario pide acciones del robot."
                    ),
                    "turn_detection": {"type": "server_vad"},
                },
                timeout=20,
            )
            response.raise_for_status()
            return {"ok": True, "session": response.json()}
        except Exception as error:  # pragma: no cover - network fallback
            logger.warning("No se pudo crear la sesion realtime: %s", error)
            return {
                "ok": False,
                "message": "No se pudo crear la sesion realtime. La ruta de voz por chunks sigue disponible.",
            }

    def process_browser_audio(self, uploaded_file) -> Dict[str, Any]:
        transcript = self._transcribe_file(uploaded_file)
        if not transcript:
            return {"ok": False, "heard": "", "reply": "", "voice_active": self.state.is_voice_session_active()}

        normalized = normalize_text(transcript)
        wake_word = normalize_text(self.config.wake_word)
        voice_active = self.state.is_voice_session_active()

        if wake_word in normalized:
            self.state.activate_voice_session(self.config.voice_timeout_s)
            voice_active = True
            normalized = normalized.replace(wake_word, "", 1).strip()
            transcript = transcript.replace(self.config.wake_word, "", 1).strip()
            if not transcript:
                reply = "Te escucho."
                audio_base64 = self._speech_base64(reply)
                self.state.add_message("assistant", reply, "voice")
                return {
                    "ok": True,
                    "heard": "",
                    "reply": reply,
                    "audio_base64": audio_base64,
                    "voice_active": True,
                }

        if not voice_active:
            return {
                "ok": True,
                "heard": transcript,
                "reply": "",
                "voice_active": False,
            }

        result = self.conversation_service.handle_user_text(transcript, source="voice")
        self.state.activate_voice_session(self.config.voice_timeout_s)
        return {
            "ok": bool(result.get("ok", False)),
            "heard": transcript,
            "reply": result.get("reply", ""),
            "audio_base64": self._speech_base64(result.get("reply", "")),
            "voice_active": True,
        }

    def _transcribe_file(self, uploaded_file) -> str:
        if not self.config.openai_api_key:
            return ""

        filename = getattr(uploaded_file, "filename", "audio.webm")
        content_type = getattr(uploaded_file, "content_type", "audio/webm")
        audio_bytes = uploaded_file.read()
        uploaded_file.stream.seek(0)

        try:
            response = requests.post(
                "https://api.openai.com/v1/audio/transcriptions",
                headers={"Authorization": f"Bearer {self.config.openai_api_key}"},
                files={"file": (filename, audio_bytes, content_type)},
                data={"model": self.config.openai_stt_model, "language": "es"},
                timeout=45,
            )
            response.raise_for_status()
            payload = response.json()
            return payload.get("text", "").strip()
        except Exception as error:  # pragma: no cover - network fallback
            logger.warning("No se pudo transcribir audio: %s", error)
            return ""

    def _speech_base64(self, text: str) -> str:
        if not text or not self.config.openai_api_key:
            return ""
        try:
            response = requests.post(
                "https://api.openai.com/v1/audio/speech",
                headers={
                    "Authorization": f"Bearer {self.config.openai_api_key}",
                    "Content-Type": "application/json",
                },
                json={
                    "model": self.config.openai_tts_model,
                    "voice": "alloy",
                    "input": text,
                    "response_format": "mp3",
                },
                timeout=45,
            )
            response.raise_for_status()
            return base64.b64encode(response.content).decode("ascii")
        except Exception as error:  # pragma: no cover - network fallback
            logger.warning("No se pudo sintetizar voz: %s", error)
            return ""
