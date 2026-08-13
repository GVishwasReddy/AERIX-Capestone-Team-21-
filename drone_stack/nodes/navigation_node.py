"""NavigationNode - Phase 6.

A single node that owns the whole autonomous-flight brain:

* **Mission state machine** - IDLE -> ARMING -> TAKEOFF -> NAVIGATE -> RTL -> ...
* **Waypoint navigation** - issues ``goto`` commands toward each waypoint.
* **Collision avoidance** - watches ``/obstacles`` and brakes/slows near hazards.
* **Failsafe handling** - battery, GCS link, geofence and altitude limits.
* **Emergency stop** and **Return-to-Launch** - as services and failsafe actions.

It consumes fused state + telemetry + obstacles from the bus and drives the
autopilot purely through :class:`~drone_stack.msg.NavCommand` messages published
on ``/cmd/mavlink`` - so it behaves identically in simulation and in flight.
"""
from __future__ import annotations

import math
import threading

from drone_stack.bus import MessageBus
from drone_stack.bus.topics import Topics
from drone_stack.msg import (
    ArmedStatus,
    AvoidanceStatus,
    Battery,
    FlightMode,
    FusedState,
    GpsFix,
    LinkQuality,
    Mission,
    MissionPhase,
    MissionState,
    NavCommand,
    Obstacle,
    ObstacleArray,
    Waypoint,
)
from drone_stack.srv import ServiceRegistry, ServiceRequest, ServiceResponse
from drone_stack.utils.config import Config
from drone_stack.utils.geometry import enu_to_geodetic, wrap_180
from drone_stack.utils.nl_parser import Intent, parse as parse_nl
from drone_stack.utils.node import NodeBase

# Avoidance decision levels.
CLEAR, SLOW, STOP = "clear", "slow", "stop"


class CollisionAvoider:
    """Evaluates the obstacle field ahead and recommends an action."""

    def __init__(self, config: Config) -> None:
        section = config.section("navigation")
        self.distance = float(section.get("avoidance_distance_m", 2.5))
        self.stop = float(section.get("avoidance_stop_m", 1.2))
        self.sector_deg = 50.0  # half-angle of the "ahead" cone we care about

    def evaluate(self, obstacles: ObstacleArray | None) -> tuple[str, float]:
        if obstacles is None or obstacles.count == 0:
            return CLEAR, math.inf
        ahead = [
            o
            for o in obstacles.obstacles
            if abs(wrap_180(o.bearing_deg)) <= self.sector_deg
        ]
        if not ahead:
            return CLEAR, math.inf
        nearest = min(o.distance_m for o in ahead)
        if nearest <= self.stop:
            return STOP, nearest
        if nearest <= self.distance:
            return SLOW, nearest
        return CLEAR, nearest


class NavigationNode(NodeBase):
    """Autonomous mission execution with avoidance and failsafes."""

    def __init__(
        self,
        bus: MessageBus,
        config: Config,
        services: ServiceRegistry | None = None,
    ) -> None:
        section = config.section("navigation")
        super().__init__("navigation", bus, config, rate_hz=section.get("rate_hz", 10))
        self.services = services or ServiceRegistry()
        self._safety = config.section("safety")
        self._wp_radius = float(section.get("waypoint_radius_m", 1.5))
        self._cruise_alt = float(section.get("cruise_altitude_m", 5.0))
        self._cruise_speed = float(section.get("cruise_speed_ms", 3.0))
        self._takeoff_alt = float(section.get("takeoff_altitude_m", 5.0))
        self._auto_arm = bool(section.get("auto_arm", True))
        self._is_sim = config.mode == "sim"

        # State
        self._lock = threading.RLock()
        self._phase = MissionPhase.IDLE
        self._mission = Mission()
        self._current_wp = 0
        self._home: tuple[float, float] | None = None
        self._last_goto_wp = -1
        self._avoiding = False
        self._status_message = "idle"
        self._start_requested = False
        self._auto_started = False
        self._manual_target: tuple[float, float, float] | None = None
        self._manual_desc = ""

        # Latest telemetry
        self._fused: FusedState | None = None
        self._battery: Battery | None = None
        self._link: LinkQuality | None = None
        self._obstacles: ObstacleArray | None = None
        self._gps: GpsFix | None = None
        self._armed = False
        self._mode = "UNKNOWN"

        self._avoider = CollisionAvoider(config)
        self._avoid_enabled = bool(section.get("avoidance_enabled", True))

        self._subscribe_all()
        self._register_services()

    # -- wiring --------------------------------------------------------------
    def _subscribe_all(self) -> None:
        self.subscribe(Topics.FUSED_STATE, self._set("_fused"))
        self.subscribe(Topics.BATTERY, self._set("_battery"))
        self.subscribe(Topics.LINK, self._set("_link"))
        self.subscribe(Topics.OBSTACLES, self._set("_obstacles"))
        self.subscribe(Topics.GPS, self._on_gps)
        self.subscribe(Topics.ARMED, self._on_armed)
        self.subscribe(Topics.FLIGHT_MODE, self._on_mode)
        self.subscribe(Topics.MISSION_CMD, self._on_mission_cmd)

    def _set(self, attr: str):
        def _setter(msg) -> None:
            with self._lock:
                setattr(self, attr, msg)
        return _setter

    def _on_gps(self, msg) -> None:
        if isinstance(msg, GpsFix):
            with self._lock:
                self._gps = msg
                if self._home is None and msg.has_fix:
                    self._home = (msg.lat, msg.lon)
                    self.log.info("home set to %.7f, %.7f", msg.lat, msg.lon)

    def _on_armed(self, msg) -> None:
        if isinstance(msg, ArmedStatus):
            with self._lock:
                self._armed = msg.armed

    def _on_mode(self, msg) -> None:
        if isinstance(msg, FlightMode):
            with self._lock:
                self._mode = msg.mode_name

    def _on_mission_cmd(self, msg) -> None:
        if isinstance(msg, NavCommand):
            self.services.call(msg.command, **msg.params)

    def _register_services(self) -> None:
        self.services.register("load_mission", self._svc_load_mission)
        self.services.register("start_mission", self._svc_start)
        self.services.register("hold", self._svc_hold)
        self.services.register("resume", self._svc_resume)
        self.services.register("rtl", self._svc_rtl)
        self.services.register("land", self._svc_land)
        self.services.register("emergency_stop", self._svc_emergency)
        self.services.register("clear_emergency", self._svc_clear_emergency)
        self.services.register("arm", self._svc_arm)
        self.services.register("disarm", self._svc_disarm)
        self.services.register("mission_status", self._svc_status)
        self.services.register("nl_command", self._svc_nl_command)
        self.services.register("avoid_enable", self._svc_avoid_enable)
        self.services.register("avoid_disable", self._svc_avoid_disable)
        self.services.register("set_stop_distance", self._svc_set_stop_distance)

    # -- lifecycle -----------------------------------------------------------
    def on_start(self) -> None:
        with self._lock:
            if self._is_sim and self._mission.count == 0:
                self._mission = self._demo_mission()
                self.log.info(
                    "loaded demo mission with %d waypoints", self._mission.count
                )
        self.publish(Topics.MISSION_PLAN, self._mission)

    def step(self) -> None:
        with self._lock:
            self._maybe_auto_start()
            failsafe = self._check_failsafe()
            if failsafe is not None:
                self._apply_failsafe(failsafe)
            else:
                self._run_state_machine()
            self._publish_state()
            self._publish_avoidance()

    # -- auto start (simulation convenience) ---------------------------------
    def _maybe_auto_start(self) -> None:
        if (
            self._is_sim
            and self._auto_arm
            and not self._auto_started
            and self._home is not None
            and self._mission.count > 0
            and self._phase == MissionPhase.IDLE
        ):
            self._auto_started = True
            self._start_requested = True
            self.log.info("auto-starting mission (simulation)")
        if self._start_requested and self._phase in (
            MissionPhase.IDLE,
            MissionPhase.COMPLETE,
            MissionPhase.DISARMED,
        ):
            self._start_requested = False
            self._begin_mission()

    def _begin_mission(self) -> None:
        if self._mission.count == 0:
            self._status_message = "no mission loaded"
            return
        self._current_wp = 0
        self._last_goto_wp = -1
        self._phase = MissionPhase.ARMING
        self._status_message = "arming"
        self.log.info("mission '%s' starting", self._mission.name)

    # -- failsafe ------------------------------------------------------------
    def _check_failsafe(self) -> str | None:
        # Master switch: bench testing can disable GCS-side auto-failsafes so
        # the navigator stops commanding RTL (which the FC refuses without a
        # position estimate). Re-enable for real flight.
        if not self._safety.get("failsafes_enabled", True):
            return None
        # Do not fight an in-progress emergency/landing/RTL.
        if self._phase in (
            MissionPhase.EMERGENCY,
            MissionPhase.LAND,
            MissionPhase.RTL,
            MissionPhase.IDLE,
            MissionPhase.DISARMED,
            MissionPhase.COMPLETE,
        ):
            return None

        battery = self._battery
        if battery is not None:
            if (
                battery.voltage_v
                and battery.voltage_v <= float(self._safety.get("battery_critical_voltage", 13.2))
            ) or (
                battery.remaining_pct
                and battery.remaining_pct <= float(self._safety.get("battery_critical_pct", 15))
            ):
                return "battery_critical"
            if (
                battery.voltage_v
                and battery.voltage_v <= float(self._safety.get("battery_low_voltage", 14.0))
            ) or (
                battery.remaining_pct
                and battery.remaining_pct <= float(self._safety.get("battery_low_pct", 25))
            ):
                return "battery_low"

        link = self._link
        timeout = float(self._safety.get("gcs_link_timeout_s", 3.0))
        if link is not None and (
            not link.connected or link.last_heartbeat_age_s > timeout
        ):
            return "link_lost"

        if self._fused is not None and self._home is not None:
            dist_home = math.hypot(self._fused.x, self._fused.y)
            if dist_home > float(self._safety.get("geofence_radius_m", 200.0)):
                return "geofence"
            if self._fused.alt_rel_m > float(self._safety.get("max_altitude_m", 120.0)):
                return "max_altitude"
        return None

    def _apply_failsafe(self, reason: str) -> None:
        if reason == "battery_critical":
            self._status_message = "FAILSAFE: battery critical -> LAND"
            self._enter_land()
        elif reason == "max_altitude":
            self._status_message = "FAILSAFE: altitude limit -> HOLD"
            self._phase = MissionPhase.HOLD
            self._send("brake")
        else:
            self._status_message = f"FAILSAFE: {reason} -> RTL"
            self._enter_rtl()
        self.log.warning("failsafe triggered: %s", reason)

    # -- state machine -------------------------------------------------------
    def _run_state_machine(self) -> None:
        phase = self._phase
        if phase == MissionPhase.IDLE:
            self._status_message = "idle"
        elif phase == MissionPhase.ARMING:
            self._do_arming()
        elif phase == MissionPhase.TAKEOFF:
            self._do_takeoff()
        elif phase == MissionPhase.NAVIGATE:
            self._do_navigate()
        elif phase == MissionPhase.AVOID:
            self._do_avoid()
        elif phase == MissionPhase.HOLD:
            self._send("brake")
        elif phase == MissionPhase.MANUAL:
            self._do_manual()
        elif phase == MissionPhase.RTL:
            self._do_rtl()
        elif phase == MissionPhase.LAND:
            self._do_land()
        elif phase in (MissionPhase.DISARMED, MissionPhase.COMPLETE):
            self._status_message = "mission complete"

    def _do_arming(self) -> None:
        if self._home is None:
            self._status_message = "waiting for GPS/home"
            return
        self._send("set_mode", mode="GUIDED")
        if not self._armed:
            self._send("arm")
            self._status_message = "arming"
            return
        self._phase = MissionPhase.TAKEOFF
        self._send("takeoff", altitude=self._takeoff_alt)
        self._status_message = "taking off"

    def _do_takeoff(self) -> None:
        alt = self._fused.alt_rel_m if self._fused else 0.0
        if alt >= 0.95 * self._takeoff_alt:
            self._phase = MissionPhase.NAVIGATE
            self._last_goto_wp = -1
            self._status_message = "navigating"

    def _do_navigate(self) -> None:
        decision, distance = self._avoid_decision()
        self._avoiding = decision != CLEAR
        if decision == STOP:
            self._phase = MissionPhase.AVOID
            self._status_message = f"obstacle {distance:.1f} m -> braking"
            self._send("brake")
            return

        wp = self._active_waypoint()
        if wp is None:
            self._status_message = "mission waypoints done -> RTL"
            self._enter_rtl()
            return

        dist = self._distance_to_wp(wp)
        if dist <= max(self._wp_radius, wp.radius_m):
            self._current_wp += 1
            self.log.info("reached waypoint %d", wp.seq)
            return

        if decision == SLOW:
            self._status_message = f"avoiding (slow), wp {wp.seq} in {dist:.1f} m"
            self._send(
                "velocity",
                vx=min(self._cruise_speed * 0.4, 1.0),
                vy=0.0,
                vz=0.0,
            )
            return

        if self._last_goto_wp != self._current_wp:
            self._issue_goto(wp)
            self._last_goto_wp = self._current_wp
        self._status_message = f"to waypoint {wp.seq}: {dist:.1f} m"

    def _do_avoid(self) -> None:
        decision, distance = self._avoid_decision()
        self._avoiding = True
        if decision == STOP:
            self._send("brake")
            self._status_message = f"holding for obstacle at {distance:.1f} m"
            return
        # Path ahead is clear again - resume navigation.
        self._avoiding = decision != CLEAR
        self._phase = MissionPhase.NAVIGATE
        self._last_goto_wp = -1
        self._status_message = "obstacle cleared - resuming"

    def _do_manual(self) -> None:
        decision, distance = self._avoid_decision()
        self._avoiding = decision != CLEAR
        if decision == STOP:
            self._send("brake")
            self._status_message = f"MANUAL: obstacle {distance:.1f} m - holding"
            return
        if self._manual_target is not None and self._fused is not None:
            tx, ty, talt = self._manual_target
            horiz = math.hypot(tx - self._fused.x, ty - self._fused.y)
            vert = abs(talt - self._fused.alt_rel_m)
            if horiz <= self._wp_radius and vert <= 0.5:
                self._status_message = f"MANUAL: holding ({self._manual_desc})"
            else:
                self._status_message = f"MANUAL: {self._manual_desc} ({horiz:.1f} m to go)"
        else:
            self._status_message = f"MANUAL: {self._manual_desc or 'holding'}"

    def _do_rtl(self) -> None:
        if not self._armed:
            self._phase = MissionPhase.COMPLETE
            self._status_message = "returned and disarmed"
            return
        self._status_message = "returning to launch"

    def _do_land(self) -> None:
        if not self._armed:
            self._phase = MissionPhase.DISARMED
            self._status_message = "landed and disarmed"

    # -- helpers -------------------------------------------------------------
    def _enter_rtl(self) -> None:
        self._phase = MissionPhase.RTL
        self._send("rtl")

    def _enter_land(self) -> None:
        self._phase = MissionPhase.LAND
        self._send("land")

    def _active_waypoint(self) -> Waypoint | None:
        if 0 <= self._current_wp < self._mission.count:
            return self._mission.waypoints[self._current_wp]
        return None

    def _distance_to_wp(self, wp: Waypoint) -> float:
        if self._fused is None:
            return math.inf
        return math.hypot(wp.x_m - self._fused.x, wp.y_m - self._fused.y)

    def _issue_goto(self, wp: Waypoint) -> None:
        if self._home is None:
            return
        lat, lon = enu_to_geodetic(wp.x_m, wp.y_m, self._home[0], self._home[1])
        self._send("goto", lat=lat, lon=lon, alt=wp.alt_m or self._cruise_alt)
        self.log.info("goto waypoint %d (%.1f, %.1f)", wp.seq, wp.x_m, wp.y_m)

    def _send(self, command: str, **params) -> None:
        self.publish(Topics.MAVLINK_CMD, NavCommand(command=command, params=params))

    # -- natural-language execution -----------------------------------------
    def _execute_intent(self, intent: Intent) -> tuple[bool, str]:
        """Execute one parsed intent. Returns (ok, human message)."""
        action, p = intent.action, intent.params
        if action == "takeoff":
            return self._manual_takeoff(float(p.get("altitude", self._takeoff_alt)))
        if action == "land":
            self._enter_land()
            return True, "landing"
        if action == "rtl":
            self._enter_rtl()
            return True, "returning to launch"
        if action == "set_mode":
            mode = str(p.get("mode", "STABILIZE")).upper()
            self._send("set_mode", mode=mode)
            return True, f"switching to {mode}"
        if action == "arm":
            # Arm in the current mode. Forcing GUIDED here made arming fail
            # whenever there was no GPS fix (GUIDED needs a position estimate);
            # STABILIZE/ALT_HOLD can arm without GPS.
            self._send("arm")
            return True, "arm command sent (check console for FC response)"
        if action == "disarm":
            self._send("disarm")
            self._phase = MissionPhase.DISARMED
            return True, "disarming"
        if action == "hold":
            self._phase = MissionPhase.HOLD
            self._send("brake")
            return True, "holding position"
        if action == "resume":
            if self._mission.count == 0:
                return False, "no mission to resume"
            self._phase = MissionPhase.NAVIGATE
            self._last_goto_wp = -1
            return True, "resuming mission"
        if action == "start_mission":
            if self._mission.count == 0:
                return False, "no mission loaded"
            self._start_requested = True
            return True, "starting mission"
        if action == "emergency":
            self._trigger_emergency()
            return True, "EMERGENCY STOP engaged"
        if action == "set_speed":
            speed = float(p.get("speed", self._cruise_speed))
            self._cruise_speed = speed
            self._send("set_speed", speed=speed)
            return True, f"speed set to {speed:.1f} m/s"
        if action == "yaw":
            angle = float(p.get("angle_deg", 90.0))
            direction = int(p.get("direction", 1))
            self._send("yaw", angle=angle, direction=direction)
            if self._armed:
                self._phase = MissionPhase.MANUAL
            side = "right" if direction >= 0 else "left"
            self._manual_desc = f"yaw {side} {angle:.0f} deg"
            return True, self._manual_desc
        if action == "set_altitude":
            return self._manual_goto_relative(0.0, 0.0, 0.0,
                                              absolute_alt=float(p.get("altitude", 0.0)))
        if action == "move":
            return self._manual_goto_relative(
                float(p.get("dx", 0.0)), float(p.get("dy", 0.0)), float(p.get("dz", 0.0))
            )
        if action == "unknown":
            return False, f"did not understand: '{p.get('text', '')}'"
        return False, f"unsupported action: {action}"

    def _manual_takeoff(self, altitude: float) -> tuple[bool, str]:
        self._send("set_mode", mode="GUIDED")
        self._send("arm")
        self._send("takeoff", altitude=altitude)
        self._phase = MissionPhase.MANUAL
        self._manual_target = None
        self._manual_desc = f"takeoff to {altitude:.1f} m"
        return True, self._manual_desc

    def _manual_goto_relative(
        self, dx: float, dy: float, dz: float, absolute_alt: float | None = None
    ) -> tuple[bool, str]:
        """Move relative to the body frame (dx fwd, dy left, dz up), or set an
        absolute altitude, by issuing a GUIDED goto and entering MANUAL mode."""
        if self._fused is None or self._home is None:
            return False, "no position fix yet (waiting for GPS)"
        if not self._armed:
            self._send("set_mode", mode="GUIDED")
            self._send("arm")
        yaw = self._fused.yaw
        tx = self._fused.x + dx * math.cos(yaw) - dy * math.sin(yaw)
        ty = self._fused.y + dx * math.sin(yaw) + dy * math.cos(yaw)
        if absolute_alt is not None:
            talt = max(0.3, absolute_alt)
            desc = f"go to altitude {talt:.1f} m"
        else:
            talt = max(0.3, self._fused.alt_rel_m + dz)
            parts = []
            if dx:
                parts.append(f"{'forward' if dx > 0 else 'back'} {abs(dx):.1f} m")
            if dy:
                parts.append(f"{'left' if dy > 0 else 'right'} {abs(dy):.1f} m")
            if dz:
                parts.append(f"{'up' if dz > 0 else 'down'} {abs(dz):.1f} m")
            desc = "move " + ", ".join(parts) if parts else "hold"
        lat, lon = enu_to_geodetic(tx, ty, self._home[0], self._home[1])
        self._send("goto", lat=lat, lon=lon, alt=talt)
        self._manual_target = (tx, ty, talt)
        self._manual_desc = desc
        self._phase = MissionPhase.MANUAL
        return True, desc

    def _trigger_emergency(self) -> None:
        self._phase = MissionPhase.EMERGENCY
        self._status_message = "EMERGENCY STOP"
        self._manual_target = None
        self._send("brake")
        self._send("land")
        self.log.error("EMERGENCY STOP engaged")

    def _demo_mission(self) -> Mission:
        pts = [(6.0, -3.0), (12.0, -3.0), (12.0, 6.0), (2.0, 6.0), (0.0, 0.0)]
        waypoints = [
            Waypoint(
                seq=i,
                x_m=x,
                y_m=y,
                alt_m=self._cruise_alt,
                radius_m=self._wp_radius,
            )
            for i, (x, y) in enumerate(pts)
        ]
        return Mission(name="demo", waypoints=waypoints)

    def _publish_state(self) -> None:
        wp = self._active_waypoint()
        self.publish(
            Topics.MISSION_STATE,
            MissionState(
                phase=self._phase,
                current_wp=self._current_wp,
                total_wp=self._mission.count,
                distance_to_wp_m=round(self._distance_to_wp(wp), 2) if wp else 0.0,
                armed=self._armed,
                mode=self._mode,
                avoiding=self._avoiding,
                message=self._status_message,
            ),
        )

    # -- avoidance -----------------------------------------------------------
    def _avoid_decision(self) -> tuple[str, float]:
        """Avoidance decision, forced CLEAR when avoidance is disabled."""
        if not self._avoid_enabled:
            return CLEAR, math.inf
        return self._avoider.evaluate(self._obstacles)

    def _front_obstacle(self) -> Obstacle | None:
        if self._obstacles is None or self._obstacles.count == 0:
            return None
        ahead = [
            o for o in self._obstacles.obstacles
            if abs(wrap_180(o.bearing_deg)) <= self._avoider.sector_deg
        ]
        return min(ahead, key=lambda o: o.distance_m, default=None)

    def _publish_avoidance(self) -> None:
        decision, distance = self._avoid_decision()
        front = self._front_obstacle()
        count = self._obstacles.count if self._obstacles else 0

        status = {"clear": "CLEAR", "slow": "SLOW", "stop": "BRAKE"}[decision]
        if not self._avoid_enabled:
            status = "OFF"

        # Closing speed / time-to-collision / closest-point-of-approach.
        ttc = 0.0
        cpa = 0.0
        direction = "none"
        if front is not None:
            bearing = math.radians(front.bearing_deg)
            speed = math.hypot(self._fused.vx, self._fused.vy) if self._fused else 0.0
            closing = speed * math.cos(bearing)
            if closing > 0.1:
                ttc = round(front.distance_m / closing, 2)
            cpa = round(abs(front.distance_m * math.sin(bearing)), 2)
            if abs(front.bearing_deg) < 10:
                direction = "front"
            else:
                direction = "right" if front.bearing_deg > 0 else "left"

        sending = self._avoid_enabled and decision != CLEAR
        command = {"clear": "none", "slow": "slow_down", "stop": "brake"}[decision]
        if not self._avoid_enabled:
            command = "none"

        if not self._avoid_enabled:
            reason = "avoidance disabled"
        elif front is None:
            reason = "path clear"
        elif decision == STOP:
            reason = f"{front.classification.value} at {front.distance_m:.1f} m in path - braking"
        elif decision == SLOW:
            reason = f"{front.classification.value} at {front.distance_m:.1f} m - slowing"
        else:
            reason = f"nearest {front.classification.value} at {front.distance_m:.1f} m - clear"

        self.publish(
            Topics.AVOIDANCE,
            AvoidanceStatus(
                enabled=self._avoid_enabled,
                status=status,
                sending=sending,
                direction=direction,
                count=count,
                closest_m=round(front.distance_m, 2) if front else round(distance, 2)
                if math.isfinite(distance) else 0.0,
                ttc_s=ttc,
                cpa_m=cpa,
                command=command,
                reason=reason,
            ),
        )

    # -- services ------------------------------------------------------------
    def _svc_avoid_enable(self, req: ServiceRequest) -> ServiceResponse:
        with self._lock:
            self._avoid_enabled = True
        return ServiceResponse(True, "avoidance enabled")

    def _svc_avoid_disable(self, req: ServiceRequest) -> ServiceResponse:
        with self._lock:
            self._avoid_enabled = False
        return ServiceResponse(True, "avoidance disabled")

    def _svc_set_stop_distance(self, req: ServiceRequest) -> ServiceResponse:
        """Live-tune the emergency-braking trigger distance (metres).

        Sets CollisionAvoider.stop; the slow-down band is kept at least at the
        new stop distance so SLOW never sits below STOP.
        """
        try:
            dist = float(req.data.get("distance_m"))
        except (TypeError, ValueError):
            return ServiceResponse(False, "distance_m must be a number")
        dist = max(0.3, min(8.0, dist))
        with self._lock:
            self._avoider.stop = dist
            if self._avoider.distance < dist:
                self._avoider.distance = dist
        self.log.info("emergency-brake stop distance set to %.2f m", dist)
        return ServiceResponse(
            True, f"emergency brake at {dist:.2f} m", data={"stop_m": dist}
        )

    def _svc_load_mission(self, req: ServiceRequest) -> ServiceResponse:
        raw = req.data.get("waypoints", [])
        waypoints = []
        for i, item in enumerate(raw):
            waypoints.append(
                Waypoint(
                    seq=item.get("seq", i),
                    x_m=float(item.get("x_m", 0.0)),
                    y_m=float(item.get("y_m", 0.0)),
                    alt_m=float(item.get("alt_m", self._cruise_alt)),
                    lat=float(item.get("lat", 0.0)),
                    lon=float(item.get("lon", 0.0)),
                    radius_m=float(item.get("radius_m", self._wp_radius)),
                    kind=item.get("kind", "nav"),
                )
            )
        with self._lock:
            self._mission = Mission(
                name=req.data.get("name", "mission"), waypoints=waypoints
            )
        self.publish(Topics.MISSION_PLAN, self._mission)
        return ServiceResponse(True, f"loaded {len(waypoints)} waypoints")

    def _svc_start(self, req: ServiceRequest) -> ServiceResponse:
        with self._lock:
            if self._mission.count == 0:
                return ServiceResponse(False, "no mission loaded")
            self._start_requested = True
        return ServiceResponse(True, "mission start requested")

    def _svc_hold(self, req: ServiceRequest) -> ServiceResponse:
        with self._lock:
            self._phase = MissionPhase.HOLD
            self._status_message = "holding"
        return ServiceResponse(True, "holding position")

    def _svc_resume(self, req: ServiceRequest) -> ServiceResponse:
        with self._lock:
            if self._phase == MissionPhase.HOLD:
                self._phase = MissionPhase.NAVIGATE
                self._last_goto_wp = -1
                return ServiceResponse(True, "resumed")
        return ServiceResponse(False, "not holding")

    def _svc_rtl(self, req: ServiceRequest) -> ServiceResponse:
        with self._lock:
            self._enter_rtl()
        return ServiceResponse(True, "returning to launch")

    def _svc_land(self, req: ServiceRequest) -> ServiceResponse:
        with self._lock:
            self._enter_land()
        return ServiceResponse(True, "landing")

    def _svc_emergency(self, req: ServiceRequest) -> ServiceResponse:
        with self._lock:
            self._trigger_emergency()
        return ServiceResponse(True, "emergency stop engaged")

    def _svc_nl_command(self, req: ServiceRequest) -> ServiceResponse:
        """Parse a free-text command and execute the resulting intents."""
        text = str(req.data.get("text", "")).strip()
        if not text:
            return ServiceResponse(False, "empty command")
        intents = parse_nl(text)
        if not intents:
            return ServiceResponse(False, "no command recognised")
        results = []
        all_ok = True
        with self._lock:
            for intent in intents:
                ok, message = self._execute_intent(intent)
                all_ok = all_ok and ok
                results.append(
                    {"intent": intent.describe(), "ok": ok, "message": message}
                )
        summary = "; ".join(r["message"] for r in results)
        self.log.info("NL command %r -> %s", text, summary)
        return ServiceResponse(all_ok, summary, data={"actions": results})

    def _svc_clear_emergency(self, req: ServiceRequest) -> ServiceResponse:
        with self._lock:
            if self._phase == MissionPhase.EMERGENCY:
                self._phase = MissionPhase.HOLD
                self._status_message = "emergency cleared - holding"
                return ServiceResponse(True, "emergency cleared")
        return ServiceResponse(False, "not in emergency")

    def _svc_arm(self, req: ServiceRequest) -> ServiceResponse:
        self._send("arm")
        return ServiceResponse(True, "arm command sent")

    def _svc_disarm(self, req: ServiceRequest) -> ServiceResponse:
        self._send("disarm")
        return ServiceResponse(True, "disarm command sent")

    def _svc_status(self, req: ServiceRequest) -> ServiceResponse:
        with self._lock:
            return ServiceResponse(
                True,
                self._status_message,
                data={
                    "phase": self._phase.value,
                    "current_wp": self._current_wp,
                    "total_wp": self._mission.count,
                    "armed": self._armed,
                    "mode": self._mode,
                    "avoiding": self._avoiding,
                },
            )
