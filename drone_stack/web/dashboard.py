"""WebDashboard - Phase 8 visualisation.

A lightweight Flask server (run under werkzeug's ``make_server`` in a background
thread) that:

* serves a single-page top-down visualiser (drone, LaserScan, PointCloud,
  obstacles, path and waypoints) at ``/``,
* streams live state as Server-Sent Events at ``/stream``,
* exposes a one-shot snapshot at ``/api/snapshot``,
* lets the UI invoke navigation services (start / hold / RTL / emergency ...)
  via ``POST /api/service``.

It is a normal :class:`~drone_stack.utils.node.NodeBase`, so the supervisor
manages and restarts it like any other node.
"""
from __future__ import annotations

import json
import math
import time
from pathlib import Path

from drone_stack.bus import MessageBus
from drone_stack.bus.topics import Topics
from drone_stack.msg import to_dict
from drone_stack.srv import ServiceRegistry
from drone_stack.utils.config import Config
from drone_stack.utils.node import NodeBase

_STATIC_DIR = Path(__file__).resolve().parent / "static"


def _sanitize(obj):
    """Replace inf/nan floats (invalid JSON) with None, recursively."""
    if isinstance(obj, float):
        return obj if math.isfinite(obj) else None
    if isinstance(obj, dict):
        return {k: _sanitize(v) for k, v in obj.items()}
    if isinstance(obj, (list, tuple)):
        return [_sanitize(v) for v in obj]
    return obj


class WebDashboard(NodeBase):
    """Serves the live visualisation dashboard."""

    _TOPIC_KEYS = {
        Topics.HEARTBEAT: "heartbeat",
        Topics.GPS: "gps",
        Topics.ATTITUDE: "attitude",
        Topics.BATTERY: "battery",
        Topics.ALTITUDE: "altitude",
        Topics.VELOCITY: "velocity",
        Topics.FLIGHT_MODE: "mode",
        Topics.ARMED: "armed",
        Topics.LINK: "link",
        Topics.SCAN: "scan",
        Topics.CLOUD: "cloud",
        Topics.OBSTACLES: "obstacles",
        Topics.FUSED_STATE: "state",
        Topics.MISSION_STATE: "mission",
        Topics.MISSION_PLAN: "plan",
        Topics.DIAGNOSTICS: "diagnostics",
    }

    def __init__(
        self,
        bus: MessageBus,
        config: Config,
        services: ServiceRegistry | None = None,
    ) -> None:
        section = config.section("web")
        super().__init__("web", bus, config, rate_hz=1.0)
        self.services = services or ServiceRegistry()
        self._host = section.get("host", "0.0.0.0")
        self._port = int(section.get("port", 8090))
        self._stream_rate = float(section.get("stream_rate_hz", 10))
        self._server = None
        self._server_thread = None

    # -- payload -------------------------------------------------------------
    def _build_payload(self) -> dict:
        snapshot = self.bus.snapshot()
        payload: dict = {"t": time.time()}
        for topic, key in self._TOPIC_KEYS.items():
            if topic in snapshot:
                payload[key] = to_dict(snapshot[topic])
        return _sanitize(payload)

    # -- flask app -----------------------------------------------------------
    def _make_app(self):
        from flask import Flask, Response, jsonify, request, send_from_directory

        app = Flask("drone_stack_dashboard", static_folder=None)
        app.logger.disabled = True

        @app.route("/")
        def index():
            return send_from_directory(_STATIC_DIR, "index.html")

        @app.route("/api/snapshot")
        def snapshot():
            return jsonify(self._build_payload())

        @app.route("/api/services")
        def services_list():
            return jsonify({"services": self.services.names()})

        @app.route("/api/service", methods=["POST"])
        def call_service():
            body = request.get_json(force=True, silent=True) or {}
            name = body.get("name", "")
            params = body.get("params", {}) or {}
            response = self.services.call(name, **params)
            return jsonify(
                {"success": response.success, "message": response.message,
                 "data": response.data}
            )

        @app.route("/stream")
        def stream():
            def generate():
                period = 1.0 / max(1.0, self._stream_rate)
                while not self.stopping:
                    data = json.dumps(self._build_payload())
                    yield f"data: {data}\n\n"
                    time.sleep(period)

            return Response(generate(), mimetype="text/event-stream")

        return app

    # -- lifecycle -----------------------------------------------------------
    def on_start(self) -> None:
        from werkzeug.serving import make_server

        import threading

        app = self._make_app()
        self._server = make_server(self._host, self._port, app, threaded=True)
        self._server_thread = threading.Thread(
            target=self._server.serve_forever,
            name="web-httpd",
            daemon=True,
        )
        self._server_thread.start()
        self.log.info("dashboard on http://%s:%d", self._host, self._port)

    def step(self) -> None:
        # The HTTP server runs in its own thread; nothing to do per-tick.
        self.sleep(1.0)

    def on_stop(self) -> None:
        if self._server is not None:
            try:
                self._server.shutdown()
            except Exception:  # noqa: BLE001
                pass
            self._server = None
