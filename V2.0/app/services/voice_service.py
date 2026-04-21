from __future__ import annotations

import base64
import difflib
import json
import logging
import re
import unicodedata
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Dict
from uuid import uuid4

import requests

from .command_router import normalize_text

logger = logging.getLogger(__name__)

VOICE_FILLER_PHRASES = {
    "",
    "ah",
    "eh",
    "ey",
    "hey",
    "hola",
    "ok",
    "okay",
    "oye",
    "si",
    "aja",
    "ajam",
    "mmm",
}


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
                    "voice": self.config.openai_realtime_voice,
                    "speed": self.config.openai_voice_speed,
                    "instructions": (
                        "Eres SILVIA, un robot asistente quirurgico UR5e. "
                        "Habla siempre en espanol con tono amable, sereno, profesional y breve. "
                        "Tu voz debe sonar calida y claramente femenina. "
                        "Apoya al equipo quirurgico con seguridad, sin inventar acciones del robot."
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
        raw_transcript = transcript
        if not self._is_meaningful_transcript(raw_transcript):
            return {
                "ok": True,
                "heard": "",
                "raw_heard": "",
                "reply": "",
                "voice_active": self.state.is_voice_session_active(),
                "ignored": True,
            }

        normalized = normalize_text(raw_transcript)
        wake_word = normalize_text(self.config.wake_word)
        voice_active = self.state.is_voice_session_active()
        wake_detected = self._contains_wake_word(normalized, wake_word)

        if wake_detected:
            self.state.activate_voice_session(self.config.voice_timeout_s)
            voice_active = True
            transcript = self._strip_wake_word(raw_transcript)
            if self._should_wait_for_follow_up(transcript):
                acknowledgement = "Te escucho."
                return {
                    "ok": True,
                    "heard": "",
                    "raw_heard": raw_transcript,
                    "reply": acknowledgement,
                    "audio_base64": self.speech_base64(acknowledgement),
                    "voice_active": True,
                    "interrupt_audio": True,
                }

        if not voice_active:
            return {
                "ok": True,
                "heard": "",
                "raw_heard": "",
                "reply": "",
                "voice_active": False,
                "ignored": True,
            }

        result = self.conversation_service.handle_user_text(
            transcript,
            source="voice",
            display_text=raw_transcript,
        )
        should_end_session = self._should_end_voice_session(transcript, result.get("action", ""))
        if should_end_session:
            self.state.clear_voice_session()
        else:
            self.state.activate_voice_session(self.config.voice_timeout_s)
        return {
            "ok": bool(result.get("ok", False)),
            "heard": transcript,
            "raw_heard": raw_transcript,
            "reply": result.get("reply", ""),
            "audio_base64": self.speech_base64(result.get("reply", "")),
            "voice_active": self.state.is_voice_session_active(),
            "interrupt_audio": wake_detected,
        }

    def process_push_to_talk_audio(self, uploaded_file) -> Dict[str, Any]:
        raw_transcript = self._transcribe_file(uploaded_file)
        if not self._is_meaningful_transcript(raw_transcript):
            return {
                "ok": True,
                "heard": "",
                "raw_heard": "",
                "reply": "",
                "voice_active": self.state.is_voice_session_active(),
                "ignored": True,
                "message": "No se detecto voz util en este turno push to talk.",
                "interrupt_audio": True,
            }

        result = self.conversation_service.handle_user_text(
            raw_transcript,
            source="voice_ptt",
            display_text=raw_transcript,
        )
        if self._should_end_voice_session(raw_transcript, result.get("action", "")):
            self.state.clear_voice_session()

        return {
            "ok": bool(result.get("ok", False)),
            "heard": raw_transcript,
            "raw_heard": raw_transcript,
            "reply": result.get("reply", ""),
            "audio_base64": self.speech_base64(result.get("reply", "")),
            "voice_active": self.state.is_voice_session_active(),
            "ignored": False,
            "message": "Turno push to talk procesado.",
            "interrupt_audio": True,
        }

    def transcribe_browser_audio(self, uploaded_file) -> Dict[str, Any]:
        filename = getattr(uploaded_file, "filename", "audio.webm")
        content_type = getattr(uploaded_file, "content_type", "audio/webm")
        audio_bytes = uploaded_file.read()
        uploaded_file.stream.seek(0)
        transcript = self._transcribe_bytes(audio_bytes, filename, content_type)
        meaningful = self._is_meaningful_transcript(transcript)
        saved_audio_path, saved_meta_path = self._save_transcription_test_artifacts(
            audio_bytes=audio_bytes,
            filename=filename,
            content_type=content_type,
            transcript=transcript if meaningful else "",
            meaningful=meaningful,
        )
        return {
            "ok": True,
            "transcript": transcript if meaningful else "",
            "meaningful": meaningful,
            "message": (
                "Transcripcion completada."
                if meaningful
                else "No se detecto voz util en este fragmento."
            ),
            "saved_audio_path": str(saved_audio_path),
            "saved_meta_path": str(saved_meta_path),
        }

    def speech_base64(self, text: str) -> str:
        return self._speech_base64(text)

    def _transcribe_file(self, uploaded_file) -> str:
        if not self.config.openai_api_key:
            return ""

        filename = getattr(uploaded_file, "filename", "audio.webm")
        content_type = getattr(uploaded_file, "content_type", "audio/webm")
        audio_bytes = uploaded_file.read()
        uploaded_file.stream.seek(0)
        return self._transcribe_bytes(audio_bytes, filename, content_type)

    def _transcribe_bytes(self, audio_bytes: bytes, filename: str, content_type: str) -> str:
        if not self.config.openai_api_key:
            return ""

        try:
            data = {
                "model": self.config.openai_stt_model,
                "language": "es",
            }
            if self.config.openai_stt_prompt.strip():
                data["prompt"] = self.config.openai_stt_prompt

            response = requests.post(
                "https://api.openai.com/v1/audio/transcriptions",
                headers={"Authorization": f"Bearer {self.config.openai_api_key}"},
                files={"file": (filename, audio_bytes, content_type)},
                data=data,
                timeout=45,
            )
            response.raise_for_status()
            payload = response.json()
            transcript = payload.get("text", "").strip()
            if self._looks_like_prompt_leak(transcript):
                logger.warning("Se ignoro una transcripcion por parecer eco del prompt STT: %s", transcript)
                return ""
            return transcript
        except Exception as error:  # pragma: no cover - network fallback
            logger.warning("No se pudo transcribir audio: %s", error)
            return ""

    def _save_transcription_test_artifacts(
        self,
        audio_bytes: bytes,
        filename: str,
        content_type: str,
        transcript: str,
        meaningful: bool,
    ) -> tuple[Path, Path]:
        target_dir = self.config.log_dir / "transcribe_tests"
        target_dir.mkdir(parents=True, exist_ok=True)

        timestamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%S%fZ")
        stem = f"{timestamp}_{uuid4().hex[:8]}"
        suffix = Path(filename).suffix or ".bin"

        audio_path = target_dir / f"{stem}{suffix}"
        meta_path = target_dir / f"{stem}.json"

        audio_path.write_bytes(audio_bytes)
        metadata = {
            "saved_at_utc": timestamp,
            "filename": filename,
            "content_type": content_type,
            "meaningful": meaningful,
            "transcript": transcript,
            "audio_path": str(audio_path),
        }
        meta_path.write_text(json.dumps(metadata, ensure_ascii=True, indent=2), encoding="utf-8")
        return audio_path, meta_path

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
                    "voice": self.config.openai_tts_voice,
                    "speed": self.config.openai_voice_speed,
                    "input": text,
                    "instructions": (
                        "Habla en espanol con tono femenino, amable, sereno y profesional. "
                        "Eres SILVIA, una asistente quirurgica robotica. "
                        "Mantente breve, clara y tranquilizadora."
                    ),
                    "response_format": "mp3",
                },
                timeout=45,
            )
            response.raise_for_status()
            return base64.b64encode(response.content).decode("ascii")
        except Exception as error:  # pragma: no cover - network fallback
            logger.warning("No se pudo sintetizar voz: %s", error)
            return ""

    def _strip_wake_word(self, transcript: str) -> str:
        normalized_transcript, index_map = self._normalize_with_index_map(transcript)
        wake_word = normalize_text(self.config.wake_word)
        span = self._find_wake_word_span(normalized_transcript, wake_word)
        if span is None:
            return transcript.strip(" ,.!?:;")

        _, end = span
        if end >= len(index_map):
            return ""

        start_index = index_map[end]
        return transcript[start_index:].lstrip(" ,.!?:;-_")

    @staticmethod
    def _is_meaningful_transcript(transcript: str) -> bool:
        clean_text = (transcript or "").strip()
        if not clean_text:
            return False
        return bool(re.search(r"[a-zA-Z0-9áéíóúÁÉÍÓÚñÑ]", clean_text))

    @staticmethod
    def _should_wait_for_follow_up(transcript: str) -> bool:
        if not VoiceService._is_meaningful_transcript(transcript):
            return True
        normalized = normalize_text(transcript)
        return normalized in VOICE_FILLER_PHRASES

    def _looks_like_prompt_leak(self, transcript: str) -> bool:
        normalized_transcript = normalize_text(transcript)
        normalized_prompt = normalize_text(self.config.openai_stt_prompt)

        if not normalized_transcript or not normalized_prompt:
            return False

        if normalized_transcript == normalized_prompt:
            return True

        if len(normalized_transcript) >= 24 and normalized_transcript in normalized_prompt:
            return True

        ratio = difflib.SequenceMatcher(None, normalized_transcript, normalized_prompt).ratio()
        return ratio >= 0.72

    @staticmethod
    def _should_end_voice_session(transcript: str, action: str) -> bool:
        normalized = normalize_text(transcript)
        if action in {"cancel", "voice_exit"}:
            return True
        return normalized in {"cancelar", "cancela", "salir"}

    @staticmethod
    def _contains_wake_word(normalized_transcript: str, wake_word: str) -> bool:
        return VoiceService._find_wake_word_span(normalized_transcript, wake_word) is not None

    @staticmethod
    def _find_wake_word_span(normalized_transcript: str, wake_word: str) -> tuple[int, int] | None:
        if not normalized_transcript or not wake_word:
            return None
        pattern = re.compile(rf"(?<![a-z0-9]){re.escape(wake_word)}(?![a-z0-9])")
        match = pattern.search(normalized_transcript)
        return match.span() if match else None

    @staticmethod
    def _normalize_with_index_map(text: str) -> tuple[str, list[int]]:
        normalized_chars: list[str] = []
        index_map: list[int] = []
        previous_was_space = False

        for index, char in enumerate(text):
            decomposed = unicodedata.normalize("NFKD", char)
            ascii_chars = [piece for piece in decomposed if not unicodedata.combining(piece)]

            for piece in ascii_chars:
                lowered = piece.lower()
                if lowered.isspace():
                    if previous_was_space:
                        continue
                    normalized_chars.append(" ")
                    index_map.append(index)
                    previous_was_space = True
                    continue

                normalized_chars.append(lowered)
                index_map.append(index)
                previous_was_space = False

        return "".join(normalized_chars), index_map
