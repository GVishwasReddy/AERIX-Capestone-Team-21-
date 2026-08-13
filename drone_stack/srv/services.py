"""A tiny synchronous service layer.

Nodes register named handlers (e.g. ``start_mission``, ``emergency_stop``); the
web dashboard, CLI or other nodes invoke them and get a structured response.
This mirrors ROS services without any transport machinery.
"""
from __future__ import annotations

import threading
from dataclasses import dataclass, field
from typing import Any, Callable

from drone_stack.utils.logging_setup import get_logger

_log = get_logger("services")


class ServiceError(Exception):
    """Raised when a service is missing or its handler fails."""


@dataclass
class ServiceRequest:
    name: str
    data: dict[str, Any] = field(default_factory=dict)


@dataclass
class ServiceResponse:
    success: bool = False
    message: str = ""
    data: dict[str, Any] = field(default_factory=dict)


Handler = Callable[[ServiceRequest], ServiceResponse]


class ServiceRegistry:
    """Thread-safe registry of named service handlers."""

    def __init__(self) -> None:
        self._handlers: dict[str, Handler] = {}
        self._lock = threading.RLock()

    def register(self, name: str, handler: Handler) -> None:
        with self._lock:
            if name in self._handlers:
                _log.warning("service '%s' is being overwritten", name)
            self._handlers[name] = handler
        _log.debug("registered service '%s'", name)

    def unregister(self, name: str) -> None:
        with self._lock:
            self._handlers.pop(name, None)

    def has(self, name: str) -> bool:
        with self._lock:
            return name in self._handlers

    def names(self) -> list[str]:
        with self._lock:
            return sorted(self._handlers)

    def call(self, name: str, **data: Any) -> ServiceResponse:
        """Invoke a service by name; never raises - failures are captured."""
        with self._lock:
            handler = self._handlers.get(name)
        if handler is None:
            return ServiceResponse(False, f"no such service: {name}")
        try:
            return handler(ServiceRequest(name=name, data=data))
        except Exception as exc:  # noqa: BLE001 - report instead of propagate
            _log.exception("service '%s' failed", name)
            return ServiceResponse(False, f"{type(exc).__name__}: {exc}")
