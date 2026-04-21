import io
import os
import sys
from types import SimpleNamespace

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))

from app.services.voice_service import VoiceService
from app.state import AppState


class DummyUpload:
    def __init__(self) -> None:
        self.filename = "audio.webm"
        self.content_type = "audio/webm"
        self.stream = io.BytesIO(b"audio")

    def read(self) -> bytes:
        return self.stream.read()


class FakeConversationService:
    def __init__(self) -> None:
        self.calls = []

    def handle_user_text(self, text, source="text", display_text=None):
        self.calls.append((text, source, display_text))
        return {"ok": True, "reply": f"Respuesta para: {text}"}


def build_voice_service():
    config = SimpleNamespace(
        openai_api_key="test-key",
        openai_realtime_model="gpt-realtime",
        openai_realtime_voice="shimmer",
        openai_tts_model="gpt-4o-mini-tts",
        openai_tts_voice="shimmer",
        openai_stt_model="gpt-4o-transcribe",
        openai_stt_prompt="Prompt de prueba",
        wake_word="silvia",
        voice_timeout_s=5.0,
    )
    state = AppState()
    conversation = FakeConversationService()
    service = VoiceService(config, state, conversation)
    service._speech_base64 = lambda text: f"audio::{text}" if text else ""
    return service, state, conversation


def test_voice_ignores_audio_without_wake_word():
    service, state, conversation = build_voice_service()
    service._transcribe_file = lambda uploaded_file: "ruido de fondo"

    result = service.process_browser_audio(DummyUpload())

    assert result["ok"] is True
    assert result["ignored"] is True
    assert result["reply"] == ""
    assert result["heard"] == ""
    assert result["raw_heard"] == ""
    assert state.is_voice_session_active() is False
    assert conversation.calls == []


def test_voice_wake_word_only_interrupts_audio_and_waits_for_instruction():
    service, state, conversation = build_voice_service()
    service._transcribe_file = lambda uploaded_file: "Silvia"

    result = service.process_browser_audio(DummyUpload())

    assert result["ok"] is True
    assert result["interrupt_audio"] is True
    assert result["reply"] == "Te escucho."
    assert result["audio_base64"] == "audio::Te escucho."
    assert result["voice_active"] is True
    assert result["raw_heard"] == "Silvia"
    assert state.is_voice_session_active() is True
    assert conversation.calls == []


def test_voice_wake_word_accepts_punctuation_and_same_chunk_command():
    service, state, conversation = build_voice_service()
    service._transcribe_file = lambda uploaded_file: "Silvia, pásame el bisturí"

    result = service.process_browser_audio(DummyUpload())

    assert result["ok"] is True
    assert result["heard"] == "pásame el bisturí"
    assert result["raw_heard"] == "Silvia, pásame el bisturí"
    assert result["reply"] == "Respuesta para: pásame el bisturí"
    assert result["interrupt_audio"] is True
    assert result["voice_active"] is True
    assert state.is_voice_session_active() is True
    assert conversation.calls == [("pásame el bisturí", "voice", "Silvia, pásame el bisturí")]


def test_voice_wake_word_with_prefix_only_keeps_session_open():
    service, state, conversation = build_voice_service()
    service._transcribe_file = lambda uploaded_file: "Oye Silvia"

    result = service.process_browser_audio(DummyUpload())

    assert result["ok"] is True
    assert result["heard"] == ""
    assert result["reply"] == "Te escucho."
    assert result["voice_active"] is True
    assert result["interrupt_audio"] is True
    assert result["raw_heard"] == "Oye Silvia"
    assert state.is_voice_session_active() is True
    assert conversation.calls == []


def test_voice_wake_word_with_filler_only_keeps_session_open():
    service, state, conversation = build_voice_service()
    service._transcribe_file = lambda uploaded_file: "Silvia, oye"

    result = service.process_browser_audio(DummyUpload())

    assert result["ok"] is True
    assert result["heard"] == ""
    assert result["reply"] == "Te escucho."
    assert result["voice_active"] is True
    assert result["interrupt_audio"] is True
    assert result["raw_heard"] == "Silvia, oye"
    assert state.is_voice_session_active() is True
    assert conversation.calls == []


def test_voice_wake_word_handles_accents_and_punctuation():
    service, state, conversation = build_voice_service()
    service._transcribe_file = lambda uploaded_file: "Sílvia, saluda"

    result = service.process_browser_audio(DummyUpload())

    assert result["ok"] is True
    assert result["heard"] == "saluda"
    assert result["raw_heard"] == "Sílvia, saluda"
    assert result["reply"] == "Respuesta para: saluda"
    assert result["interrupt_audio"] is True
    assert result["voice_active"] is True
    assert conversation.calls == [("saluda", "voice", "Sílvia, saluda")]


def test_voice_command_after_wake_word_keeps_session_open():
    service, state, conversation = build_voice_service()
    state.activate_voice_session(5.0)
    service._transcribe_file = lambda uploaded_file: "pásame el bisturí"

    result = service.process_browser_audio(DummyUpload())

    assert result["ok"] is True
    assert result["reply"] == "Respuesta para: pásame el bisturí"
    assert result["audio_base64"] == "audio::Respuesta para: pásame el bisturí"
    assert result["voice_active"] is True
    assert state.is_voice_session_active() is True
    assert conversation.calls == [("pásame el bisturí", "voice", "pásame el bisturí")]


def test_voice_cancel_ends_session():
    service, state, conversation = build_voice_service()
    state.activate_voice_session(5.0)
    service._transcribe_file = lambda uploaded_file: "cancelar"

    result = service.process_browser_audio(DummyUpload())

    assert result["ok"] is True
    assert result["reply"] == "Respuesta para: cancelar"
    assert result["voice_active"] is False
    assert state.is_voice_session_active() is False
    assert conversation.calls == [("cancelar", "voice", "cancelar")]


def test_voice_exit_ends_session():
    service, state, conversation = build_voice_service()
    state.activate_voice_session(5.0)
    service.conversation_service.handle_user_text = lambda text, source="text", display_text=None: {
        "ok": True,
        "reply": "Con gusto, cierro la conversacion por voz.",
        "action": "voice_exit",
    }
    service._transcribe_file = lambda uploaded_file: "salir"

    result = service.process_browser_audio(DummyUpload())

    assert result["ok"] is True
    assert result["reply"] == "Con gusto, cierro la conversacion por voz."
    assert result["voice_active"] is False
    assert state.is_voice_session_active() is False


def test_push_to_talk_processes_transcript_without_wake_word():
    service, state, conversation = build_voice_service()
    service._transcribe_file = lambda uploaded_file: "pásame el bisturí"

    result = service.process_push_to_talk_audio(DummyUpload())

    assert result["ok"] is True
    assert result["heard"] == "pásame el bisturí"
    assert result["raw_heard"] == "pásame el bisturí"
    assert result["reply"] == "Respuesta para: pásame el bisturí"
    assert result["audio_base64"] == "audio::Respuesta para: pásame el bisturí"
    assert result["ignored"] is False
    assert conversation.calls == [("pásame el bisturí", "voice_ptt", "pásame el bisturí")]


def test_push_to_talk_can_close_active_voice_session():
    service, state, conversation = build_voice_service()
    state.activate_voice_session(5.0)
    service.conversation_service.handle_user_text = lambda text, source="text", display_text=None: {
        "ok": True,
        "reply": "Con gusto, cierro la conversacion por voz.",
        "action": "voice_exit",
    }
    service._transcribe_file = lambda uploaded_file: "salir"

    result = service.process_push_to_talk_audio(DummyUpload())

    assert result["ok"] is True
    assert result["reply"] == "Con gusto, cierro la conversacion por voz."
    assert result["voice_active"] is False
    assert state.is_voice_session_active() is False


def test_transcribe_browser_audio_returns_raw_transcript():
    service, state, conversation = build_voice_service()
    service._transcribe_bytes = lambda audio_bytes, filename, content_type: "hola flask si me escuchas"
    service._save_transcription_test_artifacts = lambda **kwargs: (
        "/tmp/audio.wav",
        "/tmp/audio.json",
    )

    result = service.transcribe_browser_audio(DummyUpload())

    assert result["ok"] is True
    assert result["meaningful"] is True
    assert result["transcript"] == "hola flask si me escuchas"
    assert result["saved_audio_path"] == "/tmp/audio.wav"
    assert result["saved_meta_path"] == "/tmp/audio.json"
    assert conversation.calls == []


def test_transcribe_browser_audio_handles_empty_audio():
    service, state, conversation = build_voice_service()
    service._transcribe_bytes = lambda audio_bytes, filename, content_type: ""
    service._save_transcription_test_artifacts = lambda **kwargs: (
        "/tmp/empty.wav",
        "/tmp/empty.json",
    )

    result = service.transcribe_browser_audio(DummyUpload())

    assert result["ok"] is True
    assert result["meaningful"] is False
    assert result["transcript"] == ""
    assert result["saved_audio_path"] == "/tmp/empty.wav"
    assert result["saved_meta_path"] == "/tmp/empty.json"
    assert conversation.calls == []


def test_prompt_leak_transcript_is_ignored():
    service, state, conversation = build_voice_service()
    service.config.openai_stt_prompt = (
        "Transcripcion en espanol de un asistente quirurgico UR5 llamado SILVIA. "
        "La palabra de activacion esperada es Silvia. "
        "Vocabulario frecuente: Silvia, bisturi, pinzas, tijeras curvas, "
        "tijeras rectas, mano, recoger objetos, seguir mano, home, cancelar, "
        "UR5, electroiman, seguridad."
    )

    assert service._looks_like_prompt_leak(
        "silvia, bisturi, pinzas, tijeras curvas, tijeras rectas, mano, recoger objetos, seguir mano, home, cancelar, ur5, electroiman, seguridad"
    ) is True


def test_prompt_is_not_sent_when_empty(monkeypatch):
    service, state, conversation = build_voice_service()
    service.config.openai_stt_prompt = ""

    captured = {}

    class FakeResponse:
        def raise_for_status(self):
            return None

        def json(self):
            return {"text": "Silvia"}

    def fake_post(url, headers=None, files=None, data=None, timeout=None):
        captured["data"] = data
        return FakeResponse()

    monkeypatch.setattr("app.services.voice_service.requests.post", fake_post)

    result = service._transcribe_bytes(b"audio", "audio.wav", "audio/wav")

    assert result == "Silvia"
    assert "prompt" not in captured["data"]
