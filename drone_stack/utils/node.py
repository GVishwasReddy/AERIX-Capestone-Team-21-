"""Node framework: a self-healing threaded base class and a supervisor.

Every node runs in its own daemon thread. ``NodeBase.run`` wraps the node's
lifecycle so that **any** exception in ``on_start``/``step``/``on_stop`` is
logged and the node backs off and restarts itself automatically - a crash in one
node never takes down the process or the other nodes.

The :class:`Supervisor` owns a set of node *factories*, starts them, and runs a
watchdog that re-creates any thread that dies outright and periodically publishes
aggregated node health.
"""
from __future__ import annotations

import threading
import time
from typing import Callable

from drone_stack.bus import MessageBus, Subscription
from drone_stack.bus.topics import Topics
from drone_stack.msg import NodeHealth
from drone_stack.utils.config import Config
from drone_stack.utils.logging_setup import get_logger


class NodeBase(threading.Thread):
    """Base class for all nodes. Subclass and override the lifecycle hooks."""

    def __init__(
        self,
        name: str,
        bus: MessageBus,
        config: Config,
        rate_hz: float = 10.0,
    ) -> None:
        super().__init__(name=name, daemon=True)
        self.node_name = name
        self.bus = bus
        self.config = config
        self.log = get_logger(name)
        self._rate_hz = max(0.1, float(rate_hz))
        self._period = 1.0 / self._rate_hz
        self._stop_event = threading.Event()
        self._subscriptions: list[Subscription] = []
        self._restarts = 0
        self._last_step = 0.0
        self._running = False
        self._backoff_base = 0.5
        self._backoff_max = 10.0

    # -- lifecycle hooks (override these) ------------------------------------
    def on_start(self) -> None:
        """Called once each time the node (re)starts. Acquire resources here."""

    def step(self) -> None:
        """Called every loop tick. Do the node's work here."""

    def on_stop(self) -> None:
        """Called on shutdown and after a crash. Must be safe to call twice."""

    # -- helpers for subclasses ---------------------------------------------
    def subscribe(
        self, topic: str, callback: Callable, deliver_latched: bool = True
    ) -> Subscription:
        """Subscribe, keeping the handle so the node can unsubscribe on stop.

        Pass ``deliver_latched=False`` for *command* topics. The bus latches
        every topic and replays the last message to each new subscriber, which
        is right for state (a fresh node wants the current battery reading) and
        wrong for events: when the supervisor watchdog recreates a dead node,
        its constructor re-subscribes and would immediately re-execute the last
        command that was ever sent.
        """
        sub = self.bus.subscribe(topic, callback, deliver_latched=deliver_latched)
        self._subscriptions.append(sub)
        return sub

    def publish(self, topic: str, message) -> None:
        self.bus.publish(topic, message)

    @property
    def stopping(self) -> bool:
        return self._stop_event.is_set()

    def sleep(self, seconds: float) -> None:
        """Interruptible sleep (returns early if the node is asked to stop)."""
        self._stop_event.wait(max(0.0, seconds))

    # -- thread body ---------------------------------------------------------
    def run(self) -> None:  # noqa: D401 - Thread.run override
        while not self._stop_event.is_set():
            try:
                self.on_start()
                self.log.info("started (rate=%.1f Hz)", self._rate_hz)
                self._running = True
                self._loop()
            except Exception:  # noqa: BLE001 - resilience is the whole point
                self._restarts += 1
                backoff = min(
                    self._backoff_base * (2 ** min(self._restarts, 5)),
                    self._backoff_max,
                )
                self.log.exception(
                    "crashed (restart #%d) - restarting in %.1fs",
                    self._restarts,
                    backoff,
                )
                self._safe_on_stop()
                self._stop_event.wait(backoff)
            else:
                # _loop returned because a stop was requested: clean exit.
                self._safe_on_stop()
                break
            finally:
                self._running = False
        self._cleanup_subscriptions()
        self.log.info("stopped")

    def _loop(self) -> None:
        while not self._stop_event.is_set():
            start = time.monotonic()
            self.step()
            self._last_step = time.time()
            elapsed = time.monotonic() - start
            self._stop_event.wait(max(0.0, self._period - elapsed))

    def _safe_on_stop(self) -> None:
        try:
            self.on_stop()
        except Exception:  # noqa: BLE001
            self.log.exception("error during on_stop")

    def _cleanup_subscriptions(self) -> None:
        for sub in self._subscriptions:
            try:
                sub.unsubscribe()
            except Exception:  # noqa: BLE001
                pass
        self._subscriptions.clear()

    # -- control / introspection --------------------------------------------
    def request_stop(self) -> None:
        self._stop_event.set()

    def heartbeat_age(self) -> float:
        if self._last_step == 0.0:
            return float("inf")
        return time.time() - self._last_step

    def health(self) -> NodeHealth:
        return NodeHealth(
            name=self.node_name,
            alive=self.is_alive(),
            running=self._running,
            restarts=self._restarts,
            heartbeat_age_s=round(self.heartbeat_age(), 3)
            if self._last_step
            else 0.0,
        )


NodeFactory = Callable[[], NodeBase]


class Supervisor:
    """Starts, monitors, restarts and stops a collection of nodes."""

    def __init__(self, bus: MessageBus, watchdog_interval_s: float = 2.0) -> None:
        self.bus = bus
        self.log = get_logger("supervisor")
        self._watchdog_interval = watchdog_interval_s
        self._entries: list[list] = []  # [factory, node]
        self._stop_event = threading.Event()
        self._watchdog_thread: threading.Thread | None = None

    def add(self, factory: NodeFactory) -> NodeBase:
        """Register a node factory and construct its first instance."""
        node = factory()
        self._entries.append([factory, node])
        return node

    @property
    def nodes(self) -> list[NodeBase]:
        return [entry[1] for entry in self._entries]

    def start(self) -> None:
        for _, node in self._entries:
            node.start()
            self.log.info("launched node '%s'", node.node_name)
        self._watchdog_thread = threading.Thread(
            target=self._watchdog, name="supervisor-watchdog", daemon=True
        )
        self._watchdog_thread.start()

    def _watchdog(self) -> None:
        while not self._stop_event.wait(self._watchdog_interval):
            for entry in self._entries:
                factory, node = entry
                if not node.is_alive() and not node.stopping:
                    self.log.error(
                        "node '%s' thread died - recreating", node.node_name
                    )
                    try:
                        new_node = factory()
                        new_node.start()
                        entry[1] = new_node
                    except Exception:  # noqa: BLE001
                        self.log.exception(
                            "failed to recreate node '%s'", node.node_name
                        )
            self._publish_health()

    def _publish_health(self) -> None:
        try:
            self.bus.publish(
                Topics.DIAG_NODES, [node.health() for node in self.nodes]
            )
        except Exception:  # noqa: BLE001
            self.log.exception("failed to publish node health")

    def stop(self, timeout_s: float = 5.0) -> None:
        self.log.info("stopping %d nodes", len(self._entries))
        self._stop_event.set()
        for _, node in self._entries:
            node.request_stop()
        deadline = time.monotonic() + timeout_s
        for _, node in self._entries:
            remaining = max(0.0, deadline - time.monotonic())
            node.join(timeout=remaining)
            if node.is_alive():
                self.log.warning("node '%s' did not stop in time", node.node_name)
        if self._watchdog_thread is not None:
            self._watchdog_thread.join(timeout=1.0)
        self.log.info("all nodes stopped")
