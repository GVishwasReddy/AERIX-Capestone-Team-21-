"""A minimal, thread-safe, latched publish/subscribe bus.

This is the ROS-topic replacement. Any object may be published to a string
topic; every subscriber callback registered for that topic is invoked with the
message. Key properties:

* **Thread-safe** - publish/subscribe from any node thread.
* **Latched** - the last message on each topic is retained and delivered
  immediately to new subscribers (like a ROS latched/transient-local topic), so
  a late-starting node still gets the most recent state.
* **Fault-isolated** - an exception in one subscriber is logged and never
  affects the publisher or other subscribers.

Subscriber callbacks should be cheap (they run in the publisher's thread). The
convention in this project is that a node's callback simply stores the message;
the node's own loop does the heavy work.
"""
from __future__ import annotations

import logging
import threading
from collections import defaultdict

from drone_stack.bus.topics import Topics
from typing import Any, Callable

Callback = Callable[[Any], None]

_log = logging.getLogger("drone.bus")


class Subscription:
    """Handle returned by :meth:`MessageBus.subscribe`; call :meth:`unsubscribe`."""

    __slots__ = ("_bus", "topic", "_callback", "_active")

    def __init__(self, bus: "MessageBus", topic: str, callback: Callback) -> None:
        self._bus = bus
        self.topic = topic
        self._callback = callback
        self._active = True

    def unsubscribe(self) -> None:
        if self._active:
            self._bus._remove(self.topic, self._callback)
            self._active = False

    @property
    def active(self) -> bool:
        return self._active


class MessageBus:
    """Thread-safe latched pub/sub broker."""

    def __init__(self) -> None:
        self._subscribers: dict[str, list[Callback]] = defaultdict(list)
        self._latched: dict[str, Any] = {}
        self._publish_counts: dict[str, int] = defaultdict(int)
        self._lock = threading.RLock()

    # -- publishing ----------------------------------------------------------
    def publish(self, topic: str, message: Any, latch: bool = True) -> None:
        """Publish *message* to *topic*, delivering it to all subscribers.

        Command topics (:attr:`Topics.EVENT_TOPICS`) are never latched, whatever
        *latch* says: replaying an instruction to a late subscriber makes it act
        on a request that was already carried out. State topics latch as normal.
        """
        with self._lock:
            if latch and topic not in Topics.EVENT_TOPICS:
                self._latched[topic] = message
            self._publish_counts[topic] += 1
            callbacks = tuple(self._subscribers.get(topic, ()))
        # Dispatch outside the lock so slow/reentrant subscribers can't deadlock.
        for cb in callbacks:
            try:
                cb(message)
            except Exception:  # noqa: BLE001 - isolate faulty subscribers
                _log.exception("subscriber error on topic %s", topic)

    # -- subscribing ---------------------------------------------------------
    def subscribe(
        self,
        topic: str,
        callback: Callback,
        deliver_latched: bool = True,
    ) -> Subscription:
        """Register *callback* for *topic*; returns a :class:`Subscription`.

        If *deliver_latched* and a message has already been published to the
        topic, the callback is invoked immediately with that latest message.
        """
        with self._lock:
            self._subscribers[topic].append(callback)
            latched = self._latched.get(topic, None) if deliver_latched else None
            has_latched = deliver_latched and topic in self._latched
        if has_latched:
            try:
                callback(latched)
            except Exception:  # noqa: BLE001
                _log.exception("subscriber error delivering latched %s", topic)
        return Subscription(self, topic, callback)

    def _remove(self, topic: str, callback: Callback) -> None:
        with self._lock:
            try:
                self._subscribers[topic].remove(callback)
            except (KeyError, ValueError):
                pass

    # -- introspection -------------------------------------------------------
    def latest(self, topic: str, default: Any = None) -> Any:
        """Return the last message published to *topic* (or *default*)."""
        with self._lock:
            return self._latched.get(topic, default)

    def snapshot(self) -> dict[str, Any]:
        """Return a shallow copy of the latest message on every topic."""
        with self._lock:
            return dict(self._latched)

    def topics(self) -> list[str]:
        with self._lock:
            return sorted(set(self._latched) | set(self._subscribers))

    def publish_count(self, topic: str) -> int:
        with self._lock:
            return self._publish_counts.get(topic, 0)

    def subscriber_count(self, topic: str) -> int:
        with self._lock:
            return len(self._subscribers.get(topic, ()))
