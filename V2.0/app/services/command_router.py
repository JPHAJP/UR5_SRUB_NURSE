from __future__ import annotations

import re
import unicodedata
from dataclasses import dataclass, field
from typing import Any, Dict


OBJECT_ALIASES = {
    "bisturi": "Bisturi",
    "bisturí": "Bisturi",
    "pinzas": "Pinzas",
    "pinza": "Pinzas",
    "tijeras curvas": "Tijeras_curvas",
    "tijeras curva": "Tijeras_curvas",
    "metzenbaum": "Tijeras_curvas",
    "tijeras rectas": "Tijeras_rectas",
    "tijeras recta": "Tijeras_rectas",
    "mayo": "Tijeras_rectas",
}


@dataclass(slots=True)
class RouteDecision:
    action: str
    payload: Dict[str, Any] = field(default_factory=dict)
    assistant_text: str = ""


def normalize_text(text: str) -> str:
    normalized = unicodedata.normalize("NFKD", text)
    ascii_text = "".join(char for char in normalized if not unicodedata.combining(char))
    ascii_text = re.sub(r"\s+", " ", ascii_text.lower()).strip()
    return ascii_text


class CommandRouter:
    def __init__(self, config) -> None:
        self.config = config

    def route(self, text: str) -> RouteDecision:
        normalized = normalize_text(text)

        if "nueva conversacion" in normalized or "reinicia la conversacion" in normalized:
            return RouteDecision(
                action="new_conversation",
                assistant_text="He iniciado una nueva conversacion.",
            )

        if any(term in normalized for term in ("saluda", "saludar", "haz un saludo", "di hola")):
            return RouteDecision(action="greet", assistant_text="Voy a saludar.")

        if "seguir mano" in normalized or "modo mano" in normalized or "modo entrega" in normalized:
            return RouteDecision(
                action="set_mode",
                payload={"mode": "hand_follow"},
                assistant_text="He activado el modo seguir mano.",
            )

        if "modo recoger" in normalized or "modo objetos" in normalized or "recoger objetos" in normalized:
            return RouteDecision(
                action="set_mode",
                payload={"mode": "object_pick"},
                assistant_text="He activado el modo recoger objetos.",
            )

        if any(term in normalized for term in ("detente", "alto", "cancela", "cancela")):
            return RouteDecision(action="cancel", assistant_text="He detenido la operacion actual.")

        if any(term in normalized for term in ("casa", "home", "ve a casa", "regresa a casa")):
            return RouteDecision(action="go_home", assistant_text="Regresare a la posicion de casa.")

        if any(term in normalized for term in ("suelta", "libera", "apaga el iman", "desactiva el iman")):
            return RouteDecision(action="release_object", assistant_text="Voy a liberar el objeto.")

        if any(term in normalized for term in ("enciende el iman", "activa el iman")):
            return RouteDecision(action="magnet_on", assistant_text="Voy a activar el electroiman.")

        label = self._extract_object_label(normalized)
        if label and any(term in normalized for term in ("recoge", "busca", "ve por", "trae", "pasa", "dame", "entrega", "toma")):
            return RouteDecision(
                action="pick_object",
                payload={"label": label},
                assistant_text=f"Voy a recoger {label.replace('_', ' ').lower()}.",
            )

        return RouteDecision(action="chat", assistant_text="")

    def _extract_object_label(self, normalized_text: str) -> str | None:
        for alias, label in OBJECT_ALIASES.items():
            if alias in normalized_text:
                return label
        return None
