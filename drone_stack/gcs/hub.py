"""GcsHub - owns the engine and produces the unified GCS state payload.

Builds the drone_stack supervisor (all nodes) on a shared bus, subscribes for a
GPS trail and a console feed, tracks publish rates, and exposes:

* :meth:`build_payload` - one JSON-safe dict with every panel's data.
* :meth:`command` - dispatch a UI command to the engine.
* record / replay of the payload stream as JSONL.
* mission .plan (QGC WPL v1) import/export.
* live SIM <-> REAL source switching.
"""
from __future__ import annotations

import itertools
import json
import logging
import math
import threading
import time
from collections import deque
from pathlib import Path
from typing import Any

from drone_stack.bus import MessageBus
from drone_stack.bus.topics import Topics
from drone_stack.gcs.cameras import CameraManager
from drone_stack.interfaces.lidar_interface import FovMask
from drone_stack.launch.bringup import build_supervisor
from drone_stack.msg import DeliveryBleResult, NavCommand, to_dict
from drone_stack.srv import ServiceRegistry
from drone_stack.utils.config import Config
from drone_stack.utils.geometry import enu_to_geodetic, geodetic_to_enu, haversine_m
from drone_stack.utils.logging_setup import get_logger

_LOG_DIR = Path("logs")


def _sanitize(obj: Any) -> Any:
    if isinstance(obj, float):
        return obj if math.isfinite(obj) else None
    if isinstance(obj, dict):
        return {k: _sanitize(v) for k, v in obj.items()}
    if isinstance(obj, (list, tuple)):
        return [_sanitize(v) for v in obj]
    return obj


class _ConsoleHandler(logging.Handler):
    """Feeds log records into the GCS console panel."""

    def __init__(self, sink) -> None:
        super().__init__()
        self._sink = sink

    def emit(self, record: logging.LogRecord) -> None:
        try:
            self._sink(record.levelname, record.name, record.getMessage())
        except Exception:  # noqa: BLE001
            pass


class GcsHub:
    def __init__(self, config: Config) -> None:
        self.config = config
        self.log = get_logger("gcs")
        self.bus = MessageBus()
        self.services = ServiceRegistry()
        self.supervisor = build_supervisor(
            config, self.bus, self.services, include_web=False
        )
        self._start = time.monotonic()
        self._lock = threading.RLock()

        # console ring buffer
        self._console: deque[dict] = deque(maxlen=400)
        self._console_id = itertools.count(1)

        # GPS trail + home
        self._trail: deque[list] = deque(maxlen=800)
        self._home: tuple[float, float] | None = None

        # publish-rate tracking
        self._rate_prev: dict[str, int] = {}
        self._rate_time = time.monotonic()
        self._rates = {"mavlink": 0.0, "lidar": 0.0, "radar": 0.0}

        # LiDAR field of view, front-referenced. Sent every frame (not only
        # when a scan arrives) so the radar can draw the 0 deg front line and
        # the ignored rear wedge even while the sensor is down.
        self._lidar_fov = FovMask(config.section("lidar")).describe()
        self._lidar_fov["front_offset_deg"] = float(
            config.get("lidar.angle_offset_deg", 0.0)
        )

        # record / replay
        self._record_fp = None
        self._record_name: str | None = None
        self._replay_frames: list[dict] | None = None
        self._replay_idx = 0

        # live cameras (USB C270 + Pi cam); lazy-start on first stream request.
        # Shares self.bus so the novelty-layer overlay processors can publish
        # PersonDetection/SegmentationFrame for DeliveryNode - see
        # cameras.py's own "Novelty layer tap" docstring section.
        self.cameras = CameraManager(
            bus=self.bus,
            enabled=bool(config.get("cameras.enabled", True)),
        )

        self._console_handler = _ConsoleHandler(self._push_console)
        logging.getLogger("drone").addHandler(self._console_handler)
        self.bus.subscribe(Topics.GPS, self._on_gps)
        self.bus.subscribe(Topics.MISSION_STATE, self._on_mission_state)

    # -- lifecycle -----------------------------------------------------------
    def start(self) -> None:
        self.supervisor.start()
        # Cameras are opt-out (cameras.enabled). CameraManager.start() is a
        # no-op when they are off, so both this and the lazy start in the
        # /api/camera stream handler are covered by the one gate.
        self.cameras.start()
        if not self.cameras.enabled:
            self._push_console("INFO", "gcs", "cameras disabled by config")
        self._push_console("INFO", "gcs", f"GCS engine started ({self.config.mode} mode)")

    def stop(self) -> None:
        logging.getLogger("drone").removeHandler(self._console_handler)
        with self._lock:
            if self._record_fp is not None:
                self._record_fp.close()
                self._record_fp = None
        self.cameras.stop()
        self.supervisor.stop()

    # -- feeds ---------------------------------------------------------------
    def _push_console(self, level: str, name: str, msg: str) -> None:
        self._console.append({
            "id": next(self._console_id),
            "t": time.time(),
            "level": level,
            "src": name.replace("drone.", ""),
            "msg": msg,
        })

    def _on_mission_state(self, msg) -> None:
        """The navigator owns home; the map just draws it."""
        try:
            if getattr(msg, "home_set", False):
                self._home = (msg.home_lat, msg.home_lon)
        except Exception:  # noqa: BLE001
            pass

    def _on_gps(self, msg) -> None:
        try:
            if getattr(msg, "fix_type", 0) < 3:
                return
            if not self._trail or haversine_m(
                self._trail[-1][0], self._trail[-1][1], msg.lat, msg.lon
            ) > 0.7:
                self._trail.append([msg.lat, msg.lon])
        except Exception:  # noqa: BLE001
            pass

    # -- rates ---------------------------------------------------------------
    def _update_rates(self) -> None:
        now = time.monotonic()
        dt = now - self._rate_time
        if dt < 0.5:
            return
        for key, topic in (
            ("mavlink", Topics.HEARTBEAT),
            ("lidar", Topics.SCAN),
            ("radar", Topics.OBSTACLES),
        ):
            count = self.bus.publish_count(topic)
            prev = self._rate_prev.get(topic, count)
            self._rates[key] = round((count - prev) / dt, 1)
            self._rate_prev[topic] = count
        self._rate_time = now

    # -- payload -------------------------------------------------------------
    def build_payload(self) -> dict:
        with self._lock:
            if self._replay_frames is not None:
                if self._replay_idx < len(self._replay_frames):
                    frame = dict(self._replay_frames[self._replay_idx])
                    self._replay_idx += 1
                    frame["replay"] = {
                        "active": True,
                        "idx": self._replay_idx,
                        "total": len(self._replay_frames),
                    }
                    return frame
                self._replay_frames = None
                self._push_console("INFO", "gcs", "replay finished")

        payload = self._live_payload()
        with self._lock:
            if self._record_fp is not None:
                try:
                    self._record_fp.write(json.dumps(payload) + "\n")
                    self._record_fp.flush()
                except Exception:  # noqa: BLE001
                    pass
        return payload

    @staticmethod
    def _rc_payload(rc) -> dict:
        """Transmitter state, so the pilot can see the link before it matters.

        The navigator hands control to whoever moves the transmitter's mode
        switch, so "is the RC link actually up?" is a pre-flight question the
        dashboard should answer, not one to discover in the air.
        """
        if rc is None:
            return {"connected": False, "channels": [], "count": 0, "rssi": 0,
                    "age_s": None}
        age = max(0.0, time.time() - float(getattr(rc, "stamp", 0.0) or 0.0))
        channels = [int(c) for c in (getattr(rc, "channels", []) or [])]
        # RC_CHANNELS keeps arriving for a moment after the transmitter is off,
        # so treat a stale frame as no link rather than a live one.
        return {
            "connected": age < 3.0 and any(c > 0 for c in channels),
            "channels": channels,
            "count": int(getattr(rc, "count", 0) or 0),
            "rssi": int(getattr(rc, "rssi", 0) or 0),
            "age_s": round(age, 1),
        }

    def _live_payload(self) -> dict:
        self._update_rates()
        snap = self.bus.snapshot()

        def latest(topic):
            return snap.get(topic)

        mode = latest(Topics.FLIGHT_MODE)
        armed = latest(Topics.ARMED)
        gps = latest(Topics.GPS)
        alt = latest(Topics.ALTITUDE)
        vel = latest(Topics.VELOCITY)
        att = latest(Topics.ATTITUDE)
        batt = latest(Topics.BATTERY)
        sys_status = latest(Topics.SYS_STATUS)
        link = latest(Topics.LINK)
        scan = latest(Topics.SCAN)
        scan_state = latest(Topics.SCAN_STATE)
        obs = latest(Topics.OBSTACLES)
        avoid = latest(Topics.AVOIDANCE)
        mission = latest(Topics.MISSION_STATE)
        plan = latest(Topics.MISSION_PLAN)
        diag = latest(Topics.DIAGNOSTICS)
        delivery = latest(Topics.DELIVERY_STATE)

        fix = getattr(gps, "fix_type", 0)
        gps_fix = {0: "none", 1: "none", 2: "2D", 3: "3D"}.get(fix, "RTK")

        telemetry = {
            "flight_mode": getattr(mode, "mode_name", "UNKNOWN"),
            "armed": bool(getattr(armed, "armed", False)),
            "gps_fix": gps_fix,
            "satellites": getattr(gps, "satellites", 0),
            "altitude": round(getattr(alt, "relative_m", 0.0), 2),
            "altitude_amsl": round(getattr(alt, "amsl_m", 0.0), 1),
            "ground_speed": round(getattr(vel, "ground_speed_ms", 0.0), 2),
            "vert_speed": round(getattr(alt, "climb_ms", 0.0), 2),
            "heading": round(getattr(vel, "heading_deg", 0.0), 1),
            "pitch": round(math.degrees(getattr(att, "pitch", 0.0)), 1),
            "roll": round(math.degrees(getattr(att, "roll", 0.0)), 1),
            "battery_v": round(getattr(batt, "voltage_v", 0.0), 2),
            "battery_pct": round(getattr(batt, "remaining_pct", 0.0), 0),
            "packets": getattr(link, "packets_received", 0),
            "cpu_ap": round(getattr(sys_status, "load_pct", 0.0), 1),
        }

        home_lat, home_lon = (self._home or (None, None))
        # Without a 3D fix the autopilot still reports a lat/lon - the EKF's
        # last known or dead-reckoned guess - and plotting it puts a confident
        # marker on the map somewhere the aircraft is not. Send null instead
        # and let the dashboard say "no fix"; a missing dot is honest, a wrong
        # dot is not.
        has_fix = fix >= 3
        position = {
            "lat": getattr(gps, "lat", 0.0) if has_fix else None,
            "lon": getattr(gps, "lon", 0.0) if has_fix else None,
            "fix": has_fix,
            "alt": telemetry["altitude"],
            "heading": telemetry["heading"],
            "home_lat": home_lat,
            "home_lon": home_lon,
        }

        scan_payload = None
        if scan is not None:
            scan_payload = {
                "angle_min": scan.angle_min,
                "angle_increment": scan.angle_increment,
                "ranges": list(scan.ranges),
                "state": getattr(scan_state, "value", "SCANNING"),
            }

        obstacles = []
        if obs is not None:
            for o in obs.obstacles:
                obstacles.append({
                    "id": o.id,
                    "cls": o.classification.value,
                    "distance": round(o.distance_m, 2),
                    "bearing": round(o.bearing_deg, 1),
                    "x": round(o.x_m, 2),
                    "y": round(o.y_m, 2),
                    "danger": o.danger,
                    "confidence": round(o.confidence, 2),
                    # Tracking. `id` above is the per-frame cluster index and
                    # changes whenever anything closer appears; track_id is the
                    # stable identity the velocities are accumulated against,
                    # so the panel can follow one object across revolutions.
                    "track_id": o.track_id,
                    "hits": o.hits,
                    "closing_ms": o.closing_ms,
                    "speed_ms": o.speed_m_s,
                    "dynamic": o.is_dynamic,
                })

        waypoints = self._mission_waypoints(plan)

        health = {
            "cpu": round(getattr(diag, "cpu_pct", 0.0), 1),
            "ram": round(getattr(diag, "mem_pct", 0.0), 1),
            "temp": round(getattr(diag, "temp_c", 0.0), 1),
            "uptime": round(time.monotonic() - self._start, 0),
            "lidar_fps": self._rates["lidar"],
            "radar_fps": self._rates["radar"],
            "mavlink_fps": self._rates["mavlink"],
        }

        payload = {
            "ts": time.time(),
            "sim": self.config.mode == "sim",
            "source": self.config.mode.upper(),
            "link": {
                "connected": bool(getattr(link, "connected", False)),
                "string": getattr(link, "connection_string", ""),
            },
            "telemetry": telemetry,
            "position": position,
            "rc": self._rc_payload(latest(Topics.RC)),
            "trail": list(self._trail),
            "scan": scan_payload,
            "lidar_fov": self._lidar_fov,
            "obstacles": obstacles,
            "avoidance": to_dict(avoid) if avoid is not None else None,
            "mission": {
                "phase": getattr(getattr(mission, "phase", None), "value", "IDLE"),
                "current_wp": getattr(mission, "current_wp", 0),
                "total_wp": getattr(mission, "total_wp", 0),
                "message": getattr(mission, "message", ""),
                "avoiding": getattr(mission, "avoiding", False),
                "pilot_override": getattr(mission, "pilot_override", False),
                "alt_ceiling_m": getattr(mission, "alt_ceiling_m", 0.0),
                "arm_refusal": getattr(mission, "arm_refusal", ""),
                "waypoints": waypoints,
            },
            "delivery": self._delivery_payload(delivery),
            "health": health,
            "cameras": self._camera_info(obstacles),
            "console": list(self._console)[-60:],
            "replay": {"active": False},
        }
        return _sanitize(payload)

    def _delivery_payload(self, state) -> dict:
        """The Firebase delivery panel's whole data source.

        Mirrors FirebaseDeliveryNode's published DeliveryState rather than
        recomputing anything, so what the operator sees on the dashboard is the
        state the node is actually acting on. Returns a fully-populated dict
        even when the node is absent, so the front-end never has to null-check.
        """
        if state is None:
            return {
                "phase": "IDLE", "order_id": "", "recipient_id": "",
                "target_lat": 0.0, "target_lon": 0.0,
                "hover_alt_m": 0.0, "hover_seconds": 0.0,
                "hover_remaining_s": 0.0, "distance_m": 0.0,
                "remaining_m": 0.0, "waypoints": 0,
                "auto_accept": False, "fc_mission_uploaded": False,
                "link": "disabled", "message": "delivery node not running",
                "last_error": "", "active": False,
                "orders": [], "recent": [], "selected_order_id": "",
            }
        phase = getattr(getattr(state, "phase", None), "value", "IDLE")
        return {
            "phase": phase,
            "order_id": getattr(state, "order_id", ""),
            "recipient_id": getattr(state, "recipient_id", ""),
            "target_lat": getattr(state, "target_lat", 0.0),
            "target_lon": getattr(state, "target_lon", 0.0),
            "hover_alt_m": getattr(state, "hover_alt_m", 0.0),
            "hover_seconds": getattr(state, "hover_seconds", 0.0),
            "hover_remaining_s": getattr(state, "hover_remaining_s", 0.0),
            "distance_m": getattr(state, "distance_m", 0.0),
            "remaining_m": getattr(state, "remaining_m", 0.0),
            "waypoints": getattr(state, "waypoints", 0),
            "auto_accept": bool(getattr(state, "auto_accept", False)),
            "fc_mission_uploaded": bool(getattr(state, "fc_mission_uploaded", False)),
            "link": getattr(state, "link", "disabled"),
            "message": getattr(state, "message", ""),
            "last_error": getattr(state, "last_error", ""),
            "active": phase in ("ACCEPTED", "ENROUTE", "HOVERING", "RETURNING"),
            # The order book: everything the app has written, so the operator
            # can see an order arrive even before anyone accepts it.
            "orders": list(getattr(state, "orders", []) or []),
            "recent": list(getattr(state, "recent", []) or []),
            "selected_order_id": getattr(state, "selected_order_id", ""),
        }

    def _mission_waypoints(self, plan) -> list[dict]:
        out: list[dict] = []
        if plan is None:
            return out
        home = self._home
        for wp in plan.waypoints:
            lat, lon = wp.lat, wp.lon
            if (not lat or not lon) and home is not None:
                lat, lon = enu_to_geodetic(wp.x_m, wp.y_m, home[0], home[1])
            out.append({
                "seq": wp.seq, "lat": lat, "lon": lon,
                "alt": wp.alt_m, "x": wp.x_m, "y": wp.y_m,
            })
        return out

    def _camera_info(self, obstacles: list[dict]) -> list[dict]:
        """Live camera tiles (name/res/fps/connected from CameraManager) plus
        detection boxes projected from the closest obstacles into image space."""
        detections = []
        for o in sorted(obstacles, key=lambda x: x["distance"])[:3]:
            if abs(o["bearing"]) > 60:
                continue
            cx = 0.5 + (o["bearing"] / 120.0)  # normalised 0..1
            size = max(0.08, min(0.5, 1.5 / max(o["distance"], 0.5)))
            detections.append({
                "label": o["cls"],
                "conf": o["confidence"],
                "x": round(cx - size / 2, 3),
                "y": round(0.5 - size / 2, 3),
                "w": round(size, 3),
                "h": round(size, 3),
            })
        # NOTE: real vision boxes are burned into each MJPEG frame by the Hailo
        # overlay (Detector/Segmenter). We deliberately do NOT project LIDAR
        # obstacles onto the camera tiles here -- doing so drew a phantom
        # "person LOCK" box straight ahead whenever the LIDAR saw a wall, even
        # in an empty scene. LIDAR obstacles are shown in the radar/scan view.
        _ = detections  # (kept for reference; intentionally unused for cameras)
        cams = []
        for info in self.cameras.infos():
            info = dict(info)
            info["detections"] = []
            cams.append(info)
        return cams

    # -- commands ------------------------------------------------------------
    def command(self, cmd: str, params: dict | None = None) -> dict:
        params = params or {}
        try:
            return self._dispatch(cmd, params)
        except Exception as exc:  # noqa: BLE001
            self.log.exception("command '%s' failed", cmd)
            return {"ok": False, "message": f"{type(exc).__name__}: {exc}"}

    def _dispatch(self, cmd: str, p: dict) -> dict:
        if cmd == "set_mode":
            self.bus.publish(Topics.MAVLINK_CMD, NavCommand("set_mode", {"mode": p.get("mode", "STABILIZE")}))
            self._push_console("INFO", "gcs", f"requested mode {p.get('mode')}")
            return {"ok": True, "message": f"mode {p.get('mode')} requested"}
        if cmd == "emergency_stop":
            # immediate motor cutoff
            self.bus.publish(Topics.MAVLINK_CMD, NavCommand("disarm", {}))
            self.services.call("emergency_stop")
            self._push_console("ERROR", "gcs", "EMERGENCY STOP - motors cut")
            return {"ok": True, "message": "emergency stop"}
        if cmd == "emergency_land":
            r = self.services.call("land")
            self._push_console("WARNING", "gcs", "EMERGENCY LAND")
            return {"ok": r.success, "message": r.message}
        if cmd == "nl":
            r = self.services.call("nl_command", text=p.get("text", ""))
            return {"ok": r.success, "message": r.message, "data": r.data}
        if cmd == "add_waypoint":
            return self._add_waypoint(p)
        if cmd == "upload_mission":
            return self._upload_mission(p.get("waypoints", []))
        if cmd == "clear_mission":
            self.services.call("load_mission", waypoints=[])
            return {"ok": True, "message": "mission cleared"}
        if cmd == "record":
            return self.record(p.get("action", "toggle"))
        if cmd == "replay":
            return self.replay(p.get("file", ""))
        if cmd == "set_source":
            return self.set_source(p.get("mode", "sim"))
        if cmd == "ble_auth_event":
            return self._on_ble_auth_event(p)
        if cmd == "ble_delivery_result":
            return self._on_ble_delivery_result(p)
        if cmd == "set_servo":
            # Payload servo on a Pixhawk AUX output (AUX1 == channel 9).
            ch = int(p.get("channel", 9))
            pwm = max(800, min(2200, int(p.get("pwm", 1500))))
            self.bus.publish(
                Topics.MAVLINK_CMD,
                NavCommand("set_servo", {"channel": ch, "pwm": pwm}),
            )
            aux = ch - 8 if ch >= 9 else ch
            self._push_console("INFO", "gcs", f"servo AUX{aux} (ch{ch}) -> {pwm}us")
            return {"ok": True, "message": f"servo ch{ch} = {pwm}us"}
        # direct service passthrough (arm, disarm, rtl, land, hold, resume,
        # start_mission, avoid_enable/disable, scan_start/stop/pause/resume, ...)
        if self.services.has(cmd):
            r = self.services.call(cmd, **p)
            return {"ok": r.success, "message": r.message, "data": r.data}
        return {"ok": False, "message": f"unknown command: {cmd}"}

    def _on_ble_auth_event(self, p: dict) -> dict:
        """Bridge for the BLE peripheral process (``ble_handshake/
        drone_ble_peripheral.py`` - a separate asyncio/D-Bus process, not a
        drone_stack node) onto the novelty layer's own bus. BLE carries NO
        release power of its own since the re-gate (project plan §9's "BLE
        release path" decision) - this only publishes a signal that
        ``DeliveryNode``'s ``DualFactorAuthenticator`` (§2.2) reads
        alongside the independent vision channel; release itself only ever
        happens through the mission FSM's own ``RELEASING`` state.

        Imported lazily so ``hub.py`` (core GCS) does not pick up an
        unconditional dependency on the optional ``drone_stack.novelty``
        package - matches the lazy-import pattern already used by
        ``perception/model_registry.py``. Publishing is safe even when
        ``novelty.enabled`` is false: with no ``DeliveryNode`` subscribed,
        this is an inert latched publish, not an error.
        """
        from drone_stack.novelty.topics import NoveltyTopics
        from drone_stack.novelty.types import BleAuthEvent

        phone_gps = p.get("phone_gps")
        event = BleAuthEvent(
            authenticated=bool(p.get("authenticated", False)),
            rssi_dbm=p.get("rssi_dbm"),
            phone_gps=tuple(phone_gps) if phone_gps else None,
        )
        self.bus.publish(NoveltyTopics.BLE_AUTH_EVENT, event)
        self._push_console(
            "INFO", "ble", f"BLE auth event: authenticated={event.authenticated}"
        )
        return {"ok": True, "message": "ble auth event published"}

    def _on_ble_delivery_result(self, p: dict) -> dict:
        """Bridge for drone_ble_peripheral.py's ``drop_characteristic``: once
        its own security gates (HMAC mutual auth, micro-geofence, DROP HMAC)
        all pass for the order being flown, it POSTs here so the CORE flight
        profile (CLAUDE.md §11 - not the optional novelty layer) can react.

        Published on ``Topics.DELIVERY_BLE_RESULT`` unconditionally (unlike
        ``_on_ble_auth_event``'s lazy novelty-only bridge above), because
        ``NavigationNode`` - the node driving the delivery hover/RTL profile -
        is always running, whether or not ``novelty.enabled`` is set. See
        ``NavigationNode._on_ble_delivery_result``.
        """
        event = DeliveryBleResult(
            order_id=str(p.get("order_id", "")),
            success=bool(p.get("success", False)),
        )
        self.bus.publish(Topics.DELIVERY_BLE_RESULT, event)
        self._push_console(
            "INFO", "ble",
            f"BLE delivery result: order={event.order_id} success={event.success}",
        )
        return {"ok": True, "message": "ble delivery result published"}

    def _home_or(self, default_lat: float, default_lon: float) -> tuple[float, float]:
        return self._home if self._home is not None else (default_lat, default_lon)

    def _add_waypoint(self, p: dict) -> dict:
        lat, lon = float(p.get("lat", 0.0)), float(p.get("lon", 0.0))
        alt = float(p.get("alt", self.config.get("navigation.cruise_altitude_m", 5.0)))
        existing = self.services.call("mission_status")
        # rebuild the mission from existing waypoints on the bus + new one
        plan = self.bus.latest(Topics.MISSION_PLAN)
        wps = []
        home = self._home_or(lat, lon)
        if plan is not None:
            for wp in plan.waypoints:
                wps.append({"x_m": wp.x_m, "y_m": wp.y_m, "alt_m": wp.alt_m,
                            "lat": wp.lat, "lon": wp.lon})
        east, north = geodetic_to_enu(lat, lon, home[0], home[1])
        wps.append({"x_m": east, "y_m": north, "alt_m": alt, "lat": lat, "lon": lon})
        self.services.call("load_mission", waypoints=wps)
        return {"ok": True, "message": f"waypoint {len(wps)} added"}

    def _upload_mission(self, items: list[dict]) -> dict:
        home = self._home_or(
            float(items[0]["lat"]) if items else 0.0,
            float(items[0]["lon"]) if items else 0.0,
        )
        wps = []
        for it in items:
            lat, lon = float(it.get("lat", 0.0)), float(it.get("lon", 0.0))
            east, north = geodetic_to_enu(lat, lon, home[0], home[1])
            wps.append({"x_m": east, "y_m": north, "lat": lat, "lon": lon,
                        "alt_m": float(it.get("alt", 5.0))})
        r = self.services.call("load_mission", waypoints=wps)
        return {"ok": r.success, "message": r.message}

    def set_source(self, mode: str) -> dict:
        target = "real" if str(mode).lower().startswith("r") else "sim"
        if target == self.config.mode:
            return {"ok": True, "message": f"already in {target} mode"}
        with self._lock:
            self._push_console("INFO", "gcs", f"switching to {target.upper()} mode")
            self.supervisor.stop()
            # Drop state tied to the previous source so e.g. the sim's default
            # home/GPS trail (Zurich) doesn't pollute the real-mode map.
            self._home = None
            self._trail.clear()
            self.config = self._load_profile(target)
            self.supervisor = build_supervisor(
                self.config, self.bus, self.services, include_web=False
            )
            self.supervisor.start()
        conn = self.config.get("mavlink.connection", "?")
        self._push_console("INFO", "gcs", f"{target.upper()} mavlink={conn}")
        return {"ok": True, "message": f"switched to {target} mode ({conn})"}

    def _load_profile(self, target: str) -> Config:
        """Reload the full profile for *target* mode.

        Switching modes must swap the whole profile (mavlink connection, lidar
        port, baud, ...), not merely flip ``mode``. Overriding only ``mode`` kept
        the previous profile's ``mavlink.connection`` - e.g. toggling the default
        sim (``udp:127.0.0.1:14550``) to real never reached ``/dev/ttyACM0``, so
        the Pixhawk link never came up ("link lost").
        """
        from drone_stack.utils.config import default_config_path

        profile = default_config_path().parent / f"{target}.yaml"
        try:
            return Config.load(profile_path=profile)
        except Exception:  # noqa: BLE001 - fall back to a mode-only override
            self.log.warning("could not load %s; overriding mode only", profile)
            return self.config.with_overrides(mode=target)

    # -- record / replay -----------------------------------------------------
    def record(self, action: str) -> dict:
        with self._lock:
            recording = self._record_fp is not None
            if action in ("stop", "off") or (action == "toggle" and recording):
                if self._record_fp is not None:
                    self._record_fp.close()
                    self._record_fp = None
                    self._push_console("INFO", "gcs", f"recording saved: {self._record_name}")
                    return {"ok": True, "message": f"saved {self._record_name}", "recording": False}
                return {"ok": True, "message": "not recording", "recording": False}
            _LOG_DIR.mkdir(exist_ok=True)
            self._record_name = time.strftime("flight_%Y%m%d_%H%M%S.jsonl")
            self._record_fp = open(_LOG_DIR / self._record_name, "w", encoding="utf-8")
            self._push_console("INFO", "gcs", f"recording to {self._record_name}")
            return {"ok": True, "message": f"recording {self._record_name}", "recording": True}

    def replay(self, filename: str) -> dict:
        name = Path(filename).name  # prevent path traversal
        path = _LOG_DIR / name
        if not path.exists():
            return {"ok": False, "message": f"no such log: {name}"}
        frames = []
        with path.open("r", encoding="utf-8") as fh:
            for line in fh:
                line = line.strip()
                if line:
                    try:
                        frames.append(json.loads(line))
                    except json.JSONDecodeError:
                        continue
        with self._lock:
            self._replay_frames = frames
            self._replay_idx = 0
        self._push_console("INFO", "gcs", f"replaying {name} ({len(frames)} frames)")
        return {"ok": True, "message": f"replaying {name}", "frames": len(frames)}

    def list_logs(self) -> list[str]:
        if not _LOG_DIR.exists():
            return []
        return sorted(p.name for p in _LOG_DIR.glob("*.jsonl"))

    # -- mission plan (QGC WPL / .plan) --------------------------------------
    def export_plan(self) -> dict:
        plan = self.bus.latest(Topics.MISSION_PLAN)
        items = []
        home = self._home
        waypoints = plan.waypoints if plan is not None else []
        for i, wp in enumerate(waypoints):
            lat, lon = wp.lat, wp.lon
            if (not lat or not lon) and home is not None:
                lat, lon = enu_to_geodetic(wp.x_m, wp.y_m, home[0], home[1])
            items.append({
                "AMSLAltAboveTerrain": None, "Altitude": wp.alt_m,
                "AltitudeMode": 1, "autoContinue": True,
                "command": 16, "doJumpId": i + 1, "frame": 3,
                "params": [0, 0, 0, None, lat, lon, wp.alt_m], "type": "SimpleItem",
            })
        return {
            "fileType": "Plan", "version": 1, "groundStation": "Drone GCS",
            "mission": {
                "version": 2, "firmwareType": 12, "vehicleType": 2,
                "cruiseSpeed": self.config.get("navigation.cruise_speed_ms", 3.0),
                "hoverSpeed": 3, "items": items,
                "plannedHomePosition": [home[0] if home else 0, home[1] if home else 0, 0],
            },
            "geoFence": {"circles": [], "polygons": [], "version": 2},
            "rallyPoints": {"points": [], "version": 2},
        }

    def import_plan(self, plan: dict) -> dict:
        items = (plan.get("mission", {}) or {}).get("items", [])
        waypoints = []
        for it in items:
            params = it.get("params", [])
            if it.get("command") == 16 and len(params) >= 7:
                lat, lon, altv = params[4], params[5], params[6]
                if lat and lon:
                    waypoints.append({"lat": lat, "lon": lon, "alt": altv or 5.0})
        if not waypoints:
            return {"ok": False, "message": "no nav waypoints found in plan"}
        return self._upload_mission(waypoints)
