from __future__ import annotations

import logging
from typing import Any, Callable, Dict, List

import requests

logger = logging.getLogger(__name__)


class ConversationService:
    def __init__(self, config, state, router) -> None:
        self.config = config
        self.state = state
        self.router = router
        self.action_handler: Callable[[str, Dict[str, Any] | None, str], Dict[str, Any]] | None = None

    def set_action_handler(self, handler: Callable[[str, Dict[str, Any] | None, str], Dict[str, Any]]) -> None:
        self.action_handler = handler

    def handle_user_text(
        self,
        text: str,
        source: str = "text",
        display_text: str | None = None,
    ) -> Dict[str, Any]:
        clean_text = (text or "").strip()
        if not clean_text:
            return {"ok": False, "reply": "No recibi ninguna instruccion. Estoy lista para ayudarte.", "action": "none"}

        decision = self.router.route(clean_text)

        if decision.action == "new_conversation":
            if self.action_handler:
                self.action_handler(decision.action, decision.payload, source)
            self.state.add_message("assistant", decision.assistant_text, source)
            return {"ok": True, "reply": decision.assistant_text, "action": decision.action}

        if decision.action == "voice_exit":
            self.state.add_message("user", clean_text, source, display_text=display_text or clean_text)
            self.state.add_message("assistant", decision.assistant_text, source)
            return {"ok": True, "reply": decision.assistant_text, "action": decision.action}

        display_user_text = (display_text or clean_text).strip()
        self.state.add_message("user", clean_text, source, display_text=display_user_text)

        if decision.action == "chat":
            if decision.assistant_text:
                reply = decision.assistant_text
                self.state.add_message("assistant", reply, source)
                return {"ok": True, "reply": reply, "action": "chat"}
            reply = self._generate_chat_reply(clean_text)
            self.state.add_message("assistant", reply, source)
            return {"ok": True, "reply": reply, "action": "chat"}

        action_result = {"ok": False, "message": "No hay ejecutor configurado."}
        if self.action_handler:
            action_result = self.action_handler(decision.action, decision.payload, source)

        reply = decision.assistant_text or action_result.get("message", "Accion procesada.")
        self.state.add_message("assistant", reply, source)
        return {
            "ok": bool(action_result.get("ok", False)),
            "reply": reply,
            "action": decision.action,
            "payload": decision.payload,
        }

    def _generate_chat_reply(self, latest_user_text: str) -> str:
        if not self.config.openai_api_key:
            return (
                "Puedo ayudarte de forma amable con instrumental quirurgico, modos del robot, "
                "seguimiento de mano, recogida de objetos y control seguro del UR5. "
                "Configura OPENAI_API_KEY para conversacion extendida."
            )

        try:
            response = requests.post(
                "https://api.openai.com/v1/chat/completions",
                headers={
                    "Authorization": f"Bearer {self.config.openai_api_key}",
                    "Content-Type": "application/json",
                },
                json={
                    "model": self.config.openai_text_model,
                    "temperature": 0.4,
                    "messages": self._build_chat_messages(latest_user_text),
                },
                timeout=25,
            )
            response.raise_for_status()
            payload = response.json()
            return payload["choices"][0]["message"]["content"].strip()
        except Exception as error:  # pragma: no cover - network fallback
            logger.warning("Fallo respuesta OpenAI: %s", error)
            return (
                "Ahora mismo no pude contactar a OpenAI, pero sigo lista para ayudarte con "
                "comandos del robot y con el estado del sistema."
            )

    def _build_chat_messages(self, latest_user_text: str) -> List[Dict[str, str]]:
        recent_messages = list(self.state.snapshot()["messages"])[-10:]
        messages = [
            {
                "role": "system",
                "content": (
                    "Eres SILVIA, una asistente quirurgica robotica basada en UR5e. "
                    "Habla siempre en espanol con tono amable, sereno, profesional y breve. "
                    "Acompanas al equipo quirurgico con apoyo en instrumental y control seguro del robot. "
                    "Si preguntan por inventario o capacidades, menciona bisturi, pinzas, tijeras curvas, "
                    "tijeras rectas, entrega a mano, volver a casa y cancelar. "
                    "No inventes movimientos ni confirmes acciones no solicitadas o no ejecutadas."
                ),
            }
        ]
        for message in recent_messages:
            role = message["role"]
            if role not in {"user", "assistant", "system"}:
                continue
            messages.append({"role": role, "content": message["text"]})

        if not recent_messages or recent_messages[-1]["text"] != latest_user_text:
            messages.append({"role": "user", "content": latest_user_text})

        return messages
