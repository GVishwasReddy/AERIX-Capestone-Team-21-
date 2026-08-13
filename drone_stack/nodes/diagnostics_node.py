"""DiagnosticsNode - Phase 9.

Monitors host health (CPU, RAM, temperature), node health (from the supervisor's
``/diagnostics/nodes`` feed) and sensor freshness (are telemetry/scan/GPS still
arriving?), and publishes a single :class:`~drone_stack.msg.SystemDiagnostics`
message on ``/diagnostics``.
"""
from __future__ import annotations

import threading
import time

from drone_stack.bus import MessageBus
from drone_stack.bus.topics import Topics
from drone_stack.msg import (
    Diagnostic,
    DiagLevel,
    LinkQuality,
    NodeHealth,
    SystemDiagnostics,
)
from drone_stack.utils.config import Config
from drone_stack.utils.node import NodeBase

try:
    import psutil
except Exception:  # noqa: BLE001 - psutil should be present, but degrade gracefully
    psutil = None

_SEVERITY = {DiagLevel.OK: 0, DiagLevel.WARN: 1, DiagLevel.STALE: 2, DiagLevel.ERROR: 3}


class DiagnosticsNode(NodeBase):
    """Aggregates and publishes system diagnostics."""

    # sensor label -> topic it should keep arriving on
    _SENSORS = {
        "telemetry": Topics.HEARTBEAT,
        "gps": Topics.GPS,
        "lidar": Topics.SCAN,
        "fusion": Topics.FUSED_STATE,
    }

    def __init__(self, bus: MessageBus, config: Config) -> None:
        section = config.section("diagnostics")
        super().__init__("diagnostics", bus, config, rate_hz=section.get("rate_hz", 1))
        self._cpu_warn = float(section.get("cpu_warn_pct", 85.0))
        self._mem_warn = float(section.get("mem_warn_pct", 85.0))
        self._temp_warn = float(section.get("temp_warn_c", 80.0))
        self._stale = float(section.get("sensor_stale_s", 2.0))
        self._start = time.monotonic()

        self._lock = threading.Lock()
        self._last_seen: dict[str, float] = {}
        self._nodes: list[NodeHealth] = []
        self._link: LinkQuality | None = None

        for topic in set(self._SENSORS.values()):
            self.subscribe(topic, self._make_stamp(topic))
        self.subscribe(Topics.DIAG_NODES, self._on_nodes)
        self.subscribe(Topics.LINK, self._on_link)

    def _make_stamp(self, topic: str):
        def _stamp(_msg) -> None:
            with self._lock:
                self._last_seen[topic] = time.monotonic()
        return _stamp

    def _on_nodes(self, msg) -> None:
        if isinstance(msg, list):
            with self._lock:
                self._nodes = [h for h in msg if isinstance(h, NodeHealth)]

    def _on_link(self, msg) -> None:
        if isinstance(msg, LinkQuality):
            with self._lock:
                self._link = msg

    def on_start(self) -> None:
        if psutil is not None:
            try:
                psutil.cpu_percent(interval=None)  # prime the non-blocking reading
            except Exception:  # noqa: BLE001
                pass

    def step(self) -> None:
        diagnostics: list[Diagnostic] = []
        cpu, mem, temp = self._host_metrics()

        diagnostics.append(self._threshold_diag("cpu", cpu, self._cpu_warn, "%"))
        diagnostics.append(self._threshold_diag("memory", mem, self._mem_warn, "%"))
        if temp is not None:
            diagnostics.append(self._threshold_diag("temperature", temp, self._temp_warn, "C"))

        diagnostics.extend(self._sensor_diags())
        diagnostics.extend(self._node_diags())
        diagnostics.append(self._link_diag())

        level = DiagLevel.OK
        for diag in diagnostics:
            if _SEVERITY[diag.level] > _SEVERITY[level]:
                level = diag.level

        with self._lock:
            nodes = list(self._nodes)
        self.publish(
            Topics.DIAGNOSTICS,
            SystemDiagnostics(
                cpu_pct=round(cpu, 1),
                mem_pct=round(mem, 1),
                temp_c=round(temp, 1) if temp is not None else 0.0,
                uptime_s=round(time.monotonic() - self._start, 1),
                level=level,
                diagnostics=diagnostics,
                nodes=nodes,
            ),
        )

    # -- metrics -------------------------------------------------------------
    def _host_metrics(self) -> tuple[float, float, float | None]:
        if psutil is None:
            return 0.0, 0.0, self._read_temp_file()
        try:
            cpu = psutil.cpu_percent(interval=None)
            mem = psutil.virtual_memory().percent
        except Exception:  # noqa: BLE001
            cpu, mem = 0.0, 0.0
        return cpu, mem, self._read_temp()

    def _read_temp(self) -> float | None:
        if psutil is not None:
            try:
                temps = psutil.sensors_temperatures()
                for key in ("cpu_thermal", "coretemp", "cpu-thermal"):
                    if key in temps and temps[key]:
                        return float(temps[key][0].current)
            except Exception:  # noqa: BLE001
                pass
        return self._read_temp_file()

    @staticmethod
    def _read_temp_file() -> float | None:
        try:
            with open("/sys/class/thermal/thermal_zone0/temp", "r") as handle:
                return int(handle.read().strip()) / 1000.0
        except (OSError, ValueError):
            return None

    # -- diagnostic builders -------------------------------------------------
    @staticmethod
    def _threshold_diag(name: str, value: float, warn: float, unit: str) -> Diagnostic:
        level = DiagLevel.WARN if value >= warn else DiagLevel.OK
        return Diagnostic(
            name=name,
            level=level,
            message=f"{value:.1f}{unit}",
            values={"value": round(value, 1), "warn": warn},
        )

    def _sensor_diags(self) -> list[Diagnostic]:
        out: list[Diagnostic] = []
        now = time.monotonic()
        with self._lock:
            last_seen = dict(self._last_seen)
        for label, topic in self._SENSORS.items():
            seen = last_seen.get(topic)
            if seen is None:
                out.append(
                    Diagnostic(name=f"sensor.{label}", level=DiagLevel.STALE,
                               message="no data yet")
                )
                continue
            age = now - seen
            level = DiagLevel.STALE if age > self._stale else DiagLevel.OK
            out.append(
                Diagnostic(
                    name=f"sensor.{label}",
                    level=level,
                    message=f"{age:.1f}s ago",
                    values={"age_s": round(age, 2)},
                )
            )
        return out

    def _node_diags(self) -> list[Diagnostic]:
        out: list[Diagnostic] = []
        with self._lock:
            nodes = list(self._nodes)
        for health in nodes:
            if not health.alive:
                level = DiagLevel.ERROR
            elif health.restarts > 0:
                level = DiagLevel.WARN
            else:
                level = DiagLevel.OK
            out.append(
                Diagnostic(
                    name=f"node.{health.name}",
                    level=level,
                    message="alive" if health.alive else "DEAD",
                    values={"restarts": health.restarts},
                )
            )
        return out

    def _link_diag(self) -> Diagnostic:
        with self._lock:
            link = self._link
        if link is None:
            return Diagnostic(name="link", level=DiagLevel.STALE, message="unknown")
        level = DiagLevel.OK if link.connected else DiagLevel.ERROR
        return Diagnostic(
            name="link",
            level=level,
            message="connected" if link.connected else "disconnected",
            values={"drop_rate_pct": link.drop_rate_pct},
        )
