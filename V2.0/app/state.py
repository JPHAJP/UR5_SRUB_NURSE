from __future__ import annotations

import threading
import time
import uuid
from collections import deque
from dataclasses import dataclass, field
from typing import Any, Deque, Dict, List, Optional


@dataclass(slots=True)
class ConversationMessage:
    role: str
    text: str
    source: str
    display_text: str | None = None
    timestamp: float = field(default_factory=time.time)


@dataclass(slots=True)
class TimelineEvent:
    kind: str
    message: str
    payload: Dict[str, Any]
    timestamp: float = field(default_factory=time.time)


class AppState:
    def __init__(self) -> None:
        self._lock = threading.RLock()
        self.mode = "idle"
        self.magnet_enabled = False
        self.voice_active_until = 0.0
        self.safety_locked = False
        self.safety_message = "Sistema listo."
        self.safety_meta: Dict[str, Any] = {}
        self.robot_status: Dict[str, Any] = {}
        self.vision_status: Dict[str, Any] = {}
        self.last_detections: List[Dict[str, Any]] = []
        self.hand_target: Optional[Dict[str, Any]] = None
        self.object_target: Optional[Dict[str, Any]] = None
        self.selected_object_label: Optional[str] = None
        self.conversation_id = str(uuid.uuid4())
        self.messages: Deque[ConversationMessage] = deque(maxlen=100)
        self.events: Deque[TimelineEvent] = deque(maxlen=200)

    def add_message(
        self,
        role: str,
        text: str,
        source: str = "text",
        display_text: str | None = None,
    ) -> None:
        with self._lock:
            self.messages.append(
                ConversationMessage(
                    role=role,
                    text=text,
                    source=source,
                    display_text=display_text,
                )
            )

    def add_event(self, kind: str, message: str, payload: Dict[str, Any] | None = None) -> None:
        with self._lock:
            self.events.append(TimelineEvent(kind=kind, message=message, payload=payload or {}))

    def reset_conversation(self) -> None:
        with self._lock:
            self.conversation_id = str(uuid.uuid4())
            self.messages.clear()
            self.add_event("conversation_event", "Nueva conversacion iniciada.", {})

    def set_mode(self, mode: str) -> None:
        with self._lock:
            self.mode = mode

    def set_magnet_enabled(self, enabled: bool) -> None:
        with self._lock:
            self.magnet_enabled = enabled

    def activate_voice_session(self, timeout_s: float) -> None:
        with self._lock:
            self.voice_active_until = time.time() + timeout_s

    def clear_voice_session(self) -> None:
        with self._lock:
            self.voice_active_until = 0.0

    def is_voice_session_active(self) -> bool:
        with self._lock:
            return time.time() <= self.voice_active_until

    def lock_safety(self, message: str, meta: Dict[str, Any] | None = None) -> None:
        with self._lock:
            self.safety_locked = True
            self.safety_message = message
            self.safety_meta = meta or {}
            self.events.append(TimelineEvent("safety_fault", message, self.safety_meta))

    def unlock_safety(self, message: str = "Bloqueo manual liberado.") -> None:
        with self._lock:
            self.safety_locked = False
            self.safety_message = message
            self.safety_meta = {}
            self.events.append(TimelineEvent("safety_event", message, {}))

    def set_robot_status(self, status: Dict[str, Any]) -> None:
        with self._lock:
            self.robot_status = status

    def set_vision_status(self, status: Dict[str, Any]) -> None:
        with self._lock:
            self.vision_status = status

    def set_detections(self, detections: List[Dict[str, Any]]) -> None:
        with self._lock:
            self.last_detections = detections
            objects = [item for item in detections if item.get("type") == "object"]
            self.object_target = objects[0] if objects else None

    def set_hand_target(self, target: Dict[str, Any] | None) -> None:
        with self._lock:
            self.hand_target = target

    def set_selected_object_label(self, label: str | None) -> None:
        with self._lock:
            self.selected_object_label = label

    def snapshot(self) -> Dict[str, Any]:
        with self._lock:
            return {
                "mode": self.mode,
                "magnet_enabled": self.magnet_enabled,
                "voice_active": self.is_voice_session_active(),
                "safety_locked": self.safety_locked,
                "safety_message": self.safety_message,
                "safety_meta": self.safety_meta,
                "robot_status": self.robot_status,
                "vision_status": self.vision_status,
                "detections": list(self.last_detections),
                "hand_target": self.hand_target,
                "object_target": self.object_target,
                "selected_object_label": self.selected_object_label,
                "conversation_id": self.conversation_id,
                "messages": [
                    {
                        "role": message.role,
                        "text": message.text,
                        "display_text": message.display_text,
                        "source": message.source,
                        "timestamp": message.timestamp,
                    }
                    for message in self.messages
                ],
                "events": [
                    {
                        "kind": event.kind,
                        "message": event.message,
                        "payload": event.payload,
                        "timestamp": event.timestamp,
                    }
                    for event in self.events
                ],
            }
