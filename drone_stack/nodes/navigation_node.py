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
import time

from drone_stack.bus import MessageBus
from drone_stack.bus.topics import Topics
from drone_stack.msg import (
    ArmedStatus,
    AvoidanceStatus,
    Battery,
    DeliveryBleResult,
    FlightMode,
    FusedState,
    FcMessage,
    GpsFix,
    LinkQuality,
    Mission,
    MissionPhase,
    MissionState,
    NavCommand,
    Obstacle,
    ObstacleArray,
    RcChannels,
    Waypoint,
)
from drone_stack.srv import ServiceRegistry, ServiceRequest, ServiceResponse
from drone_stack.utils.config import Config
from drone_stack.utils.geometry import (
    enu_to_geodetic,
    geodetic_to_enu,
    haversine_m,
    wrap_180,
)
from drone_stack.utils.nl_parser import Intent, parse as parse_nl
from drone_stack.utils.node import NodeBase

# Avoidance decision levels.
CLEAR, SLOW, STOP = "clear", "slow", "stop"

#: Escape directions. TRAPPED means no side is open - and with the rear 90 deg
#: of the LiDAR masked there is no third option, so it resolves to a hold.
DODGE_LEFT, DODGE_RIGHT, DODGE_TRAPPED = "left", "right", "trapped"
DODGE_NONE = "none"


class SectorView:
    """Nearest return in each ego-centric sector, in metres (inf = nothing).

    The front/left/right split and the "turn toward whichever side has more
    room" rule are ported from the reactive avoidance node in
    Drone-Autonomy-ROS2 (``obstacle_nav_node.scan_cb`` +
    ``navigate_with_avoidance``). Three things are deliberately different
    here:

    * The side cones run out to ``side_half_deg`` - 135 deg, the real edge of
      what this LiDAR scans - instead of that code's 90 deg. The source was
      written against a 360 deg scan and only ever looked at the front half;
      taking it at 90 deg would throw away 90 deg of a 270 deg window that was
      configured specifically to be usable.

    * There is no BACK sector, and no reverse-out-of-a-dead-end escape. The
      rear 90 deg is blanked in the driver (``lidar.fov_deg``) because the
      airframe sits with permanent clutter behind it. That makes "nothing
      behind us" the absence of data, not the absence of obstacles - a
      backwards dodge would be flown blind, straight into the objects the mask
      exists to ignore. ``rear_blind`` records that the sector is unmeasured.

    * Bearings follow this stack's convention (0 = nose, positive to the
      right), so a right-hand dodge is +y in MAV_FRAME_BODY_NED. The source
      publishes a MAVROS Twist in body-ENU, where +y is LEFT; carrying its
      signs across unchanged would have dodged into the obstacle.
    """

    __slots__ = ("front", "left", "right", "rear_blind")

    def __init__(
        self,
        front: float = math.inf,
        left: float = math.inf,
        right: float = math.inf,
        rear_blind: bool = True,
    ) -> None:
        self.front = front
        self.left = left
        self.right = right
        self.rear_blind = rear_blind

    def __repr__(self) -> str:  # pragma: no cover - debugging aid
        return (
            f"SectorView(front={self.front:.2f}, left={self.left:.2f}, "
            f"right={self.right:.2f}, rear_blind={self.rear_blind})"
        )


class CollisionAvoider:
    """Evaluates the obstacle field ahead and recommends an action."""

    def __init__(self, config: Config) -> None:
        section = config.section("navigation")
        self.distance = float(section.get("avoidance_distance_m", 2.5))
        self.stop = float(section.get("avoidance_stop_m", 1.2))
        #: Clearance the FC's path planner keeps from obstacles in Guided/RTL.
        #: NOT the brake distance - it must sit well outside it, so the route
        #: bends around an obstacle long before the aircraft is close enough to
        #: need stopping. The brake is what happens when routing has failed.
        self.route_margin = float(section.get("avoidance_route_margin_m", 3.0))
        # Half-angle of the "ahead" cone that gates SLOW/STOP. Kept at this
        # airframe's tuned 50 deg rather than the source node's 15 deg: those
        # numbers are SITL defaults the port plan already flags as needing
        # re-derivation from the Tarot 650's stopping distance, and 50 deg
        # brakes earlier.
        self.sector_deg = float(section.get("avoidance_front_half_deg", 50.0))
        self.side_half_deg = float(section.get("avoidance_side_half_deg", 135.0))
        self.dodge_enabled = bool(section.get("avoidance_dodge_enabled", True))
        self.dodge_speed = float(section.get("avoidance_dodge_speed_ms", 1.0))
        self.dodge_clear = float(section.get("avoidance_dodge_clear_m", 2.5))
        self.dodge_timeout_s = float(section.get("avoidance_dodge_timeout_s", 5.0))
        #: Seconds of closing motion the stop distance is padded by. The stand-
        #: off is a *distance*, but what matters is whether we can stop before
        #: reaching it, and that depends on how fast the gap is shrinking. A
        #: person walking into the path at 1.5 m/s covers 1.5 m in the second it
        #: takes to notice and brake, so a fixed 1.7 m margin is 1.7 m against a
        #: wall and effectively 0.2 m against them. Padding by closing speed
        #: restores the same real margin in both cases.
        self.reaction_s = float(section.get("avoidance_reaction_s", 1.0))
        #: Ceiling on the padding, so a bad velocity estimate cannot inflate the
        #: brake distance without limit.
        self.reaction_max_m = float(section.get("avoidance_reaction_max_m", 2.0))
        #: How far the dodge may push us off the straight line to the waypoint
        #: before it gives up and holds. This is the "must not go off course".
        self.max_offtrack_m = float(section.get("avoidance_max_offtrack_m", 4.0))
        #: Forward speed retained while sidestepping. A pure lateral translation
        #: (the original behaviour) crabs sideways forever and never gets past
        #: the obstacle; carrying some forward motion is what turns a sidestep
        #: into going *around* something.
        self.dodge_forward_ms = float(section.get("avoidance_dodge_forward_ms", 0.4))

        #: Last computed values, kept for the GCS panel so it can show why the
        #: brake fired at the distance it did rather than at the configured one.
        self.last_stop_m = self.stop
        self.last_closing_ms = 0.0

        # The avoider must never claim clearance in a direction the sensor does
        # not look at. lidar.fov_deg is the single source of truth for how much
        # of the circle is real, so the side cones are clamped to its half
        # angle instead of being configured independently and drifting out of
        # step with it.
        fov = float(config.get("lidar.fov_deg", 360.0))
        if not bool(config.get("lidar.fov_enabled", True)):
            fov = 360.0
        self.fov_half_deg = max(0.0, min(180.0, fov / 2.0))
        self.rear_blind = self.fov_half_deg < 180.0
        self.side_clamped = self.side_half_deg > self.fov_half_deg
        self.side_half_deg = min(self.side_half_deg, self.fov_half_deg)
        self.sector_deg = min(self.sector_deg, self.side_half_deg)

    def stop_distance_for(self, closing_ms: float) -> float:
        """Brake distance in force against something closing at ``closing_ms``.

        Only *positive* closing counts. An object moving away does not shorten
        the margin, and letting it do so would drag the brake distance below the
        configured stand-off - the one number the operator set deliberately.
        """
        pad = max(0.0, closing_ms) * self.reaction_s
        return self.stop + min(pad, self.reaction_max_m)

    def evaluate(self, obstacles: ObstacleArray | None) -> tuple[str, float]:
        if obstacles is None or obstacles.count == 0:
            self.last_stop_m, self.last_closing_ms = self.stop, 0.0
            return CLEAR, math.inf
        ahead = [
            o
            for o in obstacles.obstacles
            if abs(wrap_180(o.bearing_deg)) <= self.sector_deg
        ]
        if not ahead:
            self.last_stop_m, self.last_closing_ms = self.stop, 0.0
            return CLEAR, math.inf

        # Judge each obstacle against its *own* closing speed and take the worst
        # verdict. Using only the nearest would miss the case that matters most:
        # a wall standing at 3 m while someone walks in at 4 m and 2 m/s.
        worst = CLEAR
        nearest = math.inf
        trigger_stop = self.stop
        trigger_closing = 0.0
        rank = {CLEAR: 0, SLOW: 1, STOP: 2}
        for o in ahead:
            closing = getattr(o, "closing_ms", 0.0) or 0.0
            stop_m = self.stop_distance_for(closing)
            if o.distance_m <= stop_m:
                decision = STOP
            elif o.distance_m <= self.distance:
                decision = SLOW
            else:
                decision = CLEAR
            nearest = min(nearest, o.distance_m)
            if rank[decision] > rank[worst]:
                worst, trigger_stop, trigger_closing = decision, stop_m, closing
        self.last_stop_m = trigger_stop
        self.last_closing_ms = trigger_closing
        return worst, nearest

    def sectors(self, obstacles: ObstacleArray | None) -> SectorView:
        """Nearest obstacle in the front, left and right cones."""
        front = left = right = math.inf
        if obstacles is not None:
            for o in obstacles.obstacles:
                bearing = wrap_180(o.bearing_deg)   # + = right of the nose
                offset = abs(bearing)
                if offset <= self.sector_deg:
                    front = min(front, o.distance_m)
                elif offset <= self.side_half_deg:
                    if bearing > 0:
                        right = min(right, o.distance_m)
                    else:
                        left = min(left, o.distance_m)
                # Anything past side_half_deg is in the masked rear wedge.
                # Nothing is published from there today; if the mask is ever
                # widened it still must not be read as clearance.
        return SectorView(front, left, right, rear_blind=self.rear_blind)

    def dodge(self, view: SectorView) -> str:
        """Pick an escape direction, or TRAPPED if neither side is open.

        The source compares left against right and commits to the larger,
        which will happily turn into a wall 0.6 m away as long as the other
        wall is 0.5 m away. A side has to clear ``avoidance_dodge_clear_m``
        here before it is a candidate at all.
        """
        left_ok = view.left > self.dodge_clear
        right_ok = view.right > self.dodge_clear
        if not left_ok and not right_ok:
            return DODGE_TRAPPED
        if not right_ok:
            return DODGE_LEFT
        if not left_ok:
            return DODGE_RIGHT
        return DODGE_LEFT if view.left > view.right else DODGE_RIGHT


def _m(value: float) -> str:
    """Format a sector clearance for a log line ("clear" when nothing is there)."""
    return f"{value:.1f} m" if math.isfinite(value) else "clear"


def _finite(value: float) -> float:
    """Sector clearance for the wire: 0.0 means "nothing detected".

    Matches the convention AvoidanceStatus.closest_m already uses. inf is not
    JSON-representable and the GCS renders 0.0 as "clear".
    """
    return round(value, 2) if math.isfinite(value) else 0.0


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
        # Hard altitude ceiling (metres above the launch point). Every altitude
        # this node commands is run through _clamp_alt first, so no mission,
        # NL command or delivery order can ask the aircraft to climb past it.
        # The max_altitude failsafe further down is only the backstop for a
        # climb we did not command.
        self._alt_ceiling = float(self._safety.get("max_altitude_m", 2.0))
        self._alt_margin = float(self._safety.get("altitude_margin_m", 1.0))
        self._cruise_alt = self._clamp_alt(float(section.get("cruise_altitude_m", 3.0)))
        self._cruise_speed = float(section.get("cruise_speed_ms", 3.0))
        self._takeoff_alt = self._clamp_alt(float(section.get("takeoff_altitude_m", 3.0)))
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

        # Timed hover ("position hold over the drop point"), driven by a
        # waypoint's hold_s. _hover_until is a monotonic deadline so a wall
        # clock step (NTP settling on the Pi after boot) can't cut it short
        # or strand the aircraft up there.
        delivery = config.section("delivery")
        self._hover_mode = self._safe_hover_mode(
            str(delivery.get("hover_mode", "GUIDED")).upper()
        )
        self._return_mode = str(delivery.get("return_mode", "SMART_RTL")).upper()
        self._leg_m = float(delivery.get("leg_length_m", 25.0))
        self._max_legs = int(delivery.get("max_legs", 24))
        self._default_hover_s = float(delivery.get("hover_seconds", 15.0))
        self._hover_alt = self._clamp_alt(
            float(delivery.get("hover_altitude_m", self._cruise_alt))
        )
        self._hover_until: float | None = None
        self._hover_total_s = 0.0
        self._hover_prev_mode = "GUIDED"
        # The waypoint currently being held over, plus how far the aircraft is
        # allowed to sag below its altitude before the hold is re-asserted.
        # See _guard_hover_altitude - this is the backstop for the drop-point
        # hold silently turning into a descent.
        self._hover_wp: Waypoint | None = None
        self._hover_recovering = False
        self._hover_alt_tolerance = float(delivery.get("hover_alt_tolerance_m", 1.0))
        # BLE handshake early-exit (see _on_ble_delivery_result / _do_hover):
        # once drone_ble_peripheral.py reports its drop gates passed for this
        # order, cut the hold short ble_early_rtl_wait_s later instead of
        # waiting out the full hover_seconds ceiling. hover_seconds itself
        # stays the hard timeout if the handshake never completes.
        self._ble_early_rtl_enabled = bool(delivery.get("ble_early_rtl_enabled", True))
        self._ble_early_rtl_wait_s = float(delivery.get("ble_early_rtl_wait_s", 3.0))
        self._ble_delivered_at: float | None = None

        # Post-takeoff settle. Reaching the takeoff altitude is not the same as
        # being stable at it: the climb is still bleeding off vertical rate and
        # the EKF is still settling. Departing for the waypoint on that instant
        # commits an oscillating aircraft to a translation, which reads on the
        # ground as "it lurched off sideways". Monotonic, like _hover_until.
        self._takeoff_settle_s = float(section.get("takeoff_settle_s", 2.0))
        self._settle_until: float | None = None
        self._delivery_target: tuple[float, float, float] | None = None

        # Mode arbitration. _commanded_mode is the last mode *we* asked for; if
        # the autopilot reports something else for longer than
        # pilot_override_grace_s we assume the human moved the transmitter's
        # mode switch and we stop commanding (see _check_pilot_override).
        self._commanded_mode = ""
        self._mode_mismatch_since: float | None = None
        self._pilot_override = False
        self._override_grace = float(self._safety.get("pilot_override_grace_s", 1.5))
        # Transmitter authority. The RC transmitter is the manual override and
        # must always win, so the navigator watches RC_CHANNELS directly and
        # stands down the moment the pilot moves a stick or the mode switch -
        # without waiting on a mode round-trip through the flight controller.
        self._rc: RcChannels | None = None
        self._rc_baseline: list[int] | None = None
        self._rc_deadband_us = int(self._safety.get("rc_override_deadband_us", 60))
        self._rc_stale_s = float(self._safety.get("rc_stale_s", 3.0))
        # Mode-command de-duplication. Re-sending SET_MODE every tick fights the
        # pilot (each frame yanks the aircraft back out of the mode they just
        # selected) and re-initialises the FC controller targets on every entry.
        self._last_mode_sent = ""
        self._last_mode_sent_at = 0.0
        self._mode_resend_s = float(self._safety.get("mode_resend_interval_s", 1.0))
        # Telling a refusal apart from a takeover. If the FC never leaves the
        # mode it was already in when we asked for another, it is refusing us
        # (ArduPilot rejects GUIDED outright on a thin fix or a complaining
        # EKF). Only a departure *from* a mode we actually reached is a pilot.
        self._mode_at_command = ""
        self._commanded_mode_reached = False
        self._mode_refusal_logged: tuple[str, str] | None = None
        self._rtl_requested_at: float | None = None
        self._rtl_fell_back = False
        # One-shot mirror of the configured emergency-brake distance onto the
        # FC's own margins, done once the link is up. Without it the aircraft
        # would boot with whatever AVOID_MARGIN/OA_MARGIN_MAX were left on the
        # FC from a previous session while the GCS displayed the configured
        # number - three layers, two of them disagreeing, and nothing on screen
        # to say so.
        self._brake_params_synced = False
        # Arming: the autopilot can legitimately refuse for a while (GPS still
        # converging, EKF settling), so retries are paced and bounded rather
        # than fired every step until something gives.
        self._arm_started_at = None
        self._arm_last_try = 0.0
        self._arm_attempts = 0
        self._arm_retry_s = float(self._safety.get("arm_retry_interval_s", 1.0))
        self._arm_timeout_s = float(self._safety.get("arm_timeout_s", 30.0))
        self._fc_refusal = ""          # why the FC last refused to arm
        self._fc_refusal_at = 0.0
        # Home quality gates. These live in config/default.yaml under safety
        # and were previously read by nothing at all, so home latched on the
        # first 3D fix of any quality - indoors, on five satellites, that can
        # be hundreds of metres out, and every distance measured from home
        # (geofence, delivery radius, every ENU->lat/lon conversion) inherits
        # the error.
        self._min_sats = int(self._safety.get("min_satellites", 6))
        self._min_fix_type = int(self._safety.get("min_gps_fix_type", 3))
        self._has_flown = False        # home is frozen once we commit to flight

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
        # Latched dodge: direction is chosen once per encounter and held until
        # the front clears. See _dodge_step for why it is not re-decided every
        # tick.
        self._dodge_dir: str | None = None
        self._dodge_started: float | None = None
        #: ENU position where the current dodge began, and the point it was
        #: heading for. Together they define the track the aircraft is allowed
        #: to depart from by at most avoidance_max_offtrack_m.
        self._dodge_origin: tuple[float, float] | None = None
        self._offtrack_m = 0.0
        if self._avoider.side_clamped:
            self.log.warning(
                "avoidance_side_half_deg clamped to %.1f deg by lidar.fov_deg "
                "- the side cones cannot see past the scanned window",
                self._avoider.side_half_deg,
            )
        self.log.info(
            "avoidance sectors: front +/-%.0f deg, sides to +/-%.0f deg, "
            "rear %s, dodge %s at %.1f m/s",
            self._avoider.sector_deg,
            self._avoider.side_half_deg,
            "masked (no reverse escape)" if self._avoider.rear_blind else "scanned",
            "on" if self._avoider.dodge_enabled else "off",
            self._avoider.dodge_speed,
        )

        self._subscribe_all()
        self._register_services()

    # -- wiring --------------------------------------------------------------
    def _subscribe_all(self) -> None:
        self.subscribe(Topics.FUSED_STATE, self._set("_fused"))
        self.subscribe(Topics.BATTERY, self._set("_battery"))
        self.subscribe(Topics.LINK, self._set("_link"))
        self.subscribe(Topics.OBSTACLES, self._set("_obstacles"))
        self.subscribe(Topics.GPS, self._on_gps)
        self.subscribe(Topics.FC_MESSAGE, self._on_fc_message)
        self.subscribe(Topics.ARMED, self._on_armed)
        self.subscribe(Topics.FLIGHT_MODE, self._on_mode)
        # The transmitter is the manual override, so its channels are telemetry
        # this node must actually read - not just something the dashboard shows.
        self.subscribe(Topics.RC, self._on_rc)
        # Also an event topic: a replayed 'start' would launch a mission that
        # nobody commanded (see NodeBase.subscribe).
        self.subscribe(Topics.MISSION_CMD, self._on_mission_cmd, deliver_latched=False)
        # Also an event topic: a replayed "delivered" from a past order must
        # not cut short a new order's hold (see NodeBase.subscribe).
        self.subscribe(
            Topics.DELIVERY_BLE_RESULT, self._on_ble_delivery_result, deliver_latched=False
        )

    def _clamp_alt(self, alt: float) -> float:
        """Clamp a commanded altitude to [0.5 m, ceiling].

        Single choke point for the altitude limit: anything that ends up as an
        ``alt`` on a MAVLink command goes through here, so raising the limit is
        a one-line config change and lowering it cannot be bypassed by a
        mission file, a delivery order or a natural-language command.
        """
        return max(0.5, min(float(alt), self._alt_ceiling))

    def _alt_note(self, requested: float, applied: float) -> str:
        """Explain a clamp on the status line.

        Quietly substituting 3 m for a requested 50 m makes the display look
        broken and teaches the operator to distrust it. Saying which limit
        bit, and what will actually be flown, teaches the limit instead.
        """
        try:
            req = float(requested)
        except (TypeError, ValueError):
            return ""
        if not math.isfinite(req) or abs(req - float(applied)) < 0.05:
            return ""
        return (
            f" (requested {req:.1f} m, limited to {applied:.1f} m "
            f"by the {self._alt_ceiling:.0f} m ceiling)"
        )

    def _set(self, attr: str):
        def _setter(msg) -> None:
            with self._lock:
                setattr(self, attr, msg)
        return _setter

    def _on_fc_message(self, msg) -> None:
        """Keep the autopilot's last prearm refusal.

        ArduPilot explains itself ("PreArm: GPS glitching"); without carrying
        that through, a refused arm reaches the operator as an unexplained
        abort and looks like a ground-station fault.
        """
        if isinstance(msg, FcMessage) and msg.is_prearm and msg.text:
            with self._lock:
                self._fc_refusal = msg.text
                self._fc_refusal_at = time.monotonic()

    def _fc_refusal_reason(self) -> str:
        """The refusal, if it is recent enough to still be the live reason."""
        if self._fc_refusal and (time.monotonic() - self._fc_refusal_at) < 10.0:
            return self._fc_refusal
        return ""

    def _on_ble_delivery_result(self, msg) -> None:
        """GcsHub bridges drone_ble_peripheral.py's drop gates here once they
        pass for the order being flown (see hub.py's _on_ble_delivery_result).

        Only meaningful during the delivery hold - _enter_hover resets
        ``_ble_delivered_at`` to ``None`` at the start of every hold, so a
        success recorded here only ever shortens the hold that was actually
        in progress when the phone's handshake completed. A failed/negative
        event is not tracked at all: the existing hover_seconds countdown in
        _do_hover already is the "handshake failed" path.
        """
        if not isinstance(msg, DeliveryBleResult) or not msg.success:
            return
        with self._lock:
            self._ble_delivered_at = time.monotonic()
        self.log.info(
            "BLE handshake confirmed delivery for order %s - RTL in %.0fs",
            msg.order_id, self._ble_early_rtl_wait_s,
        )

    def _on_gps(self, msg) -> None:
        if not isinstance(msg, GpsFix):
            return
        with self._lock:
            self._gps = msg

            # Home is the launch point for this power cycle. While the aircraft
            # is still on the ground and has never flown, the current fix *is*
            # the launch point, so keep taking it: a home latched on the first
            # marginal fix of the session would otherwise persist unchanged
            # after the aircraft is carried outside and the fix improves.
            #
            # It freezes the moment we first arm. Latching after takeoff would
            # put "home" wherever we happened to be flying.
            if self._armed or self._has_flown:
                return
            if msg.fix_type < self._min_fix_type or msg.satellites < self._min_sats:
                return

            previous = self._home
            self._home = (msg.lat, msg.lon)
            if previous is None:
                self.log.info(
                    "home set to %.7f, %.7f (fix %d, %d sats)",
                    msg.lat, msg.lon, msg.fix_type, msg.satellites,
                )
            else:
                moved = math.hypot(
                    (msg.lat - previous[0]) * 111320.0,
                    (msg.lon - previous[1]) * 111320.0
                    * math.cos(math.radians(msg.lat)),
                )
                if moved > 5.0:
                    self.log.info(
                        "home moved %.0f m to %.7f, %.7f (%d sats) - still on "
                        "the ground, so this is the better launch point",
                        moved, msg.lat, msg.lon, msg.satellites,
                    )

    def _on_armed(self, msg) -> None:
        if isinstance(msg, ArmedStatus):
            with self._lock:
                was_armed = self._armed
                self._armed = msg.armed
                if msg.armed and not self._has_flown:
                    # Commit: home stops tracking the GPS from here on, for the
                    # rest of this power cycle.
                    self._has_flown = True
                    if self._home is not None:
                        self.log.info(
                            "armed - home frozen at %.7f, %.7f",
                            self._home[0], self._home[1],
                        )
                if was_armed and not msg.armed:
                    # Disarmed: there is no flight left for anyone to "take
                    # over", so whatever mode the FC settles into on the
                    # ground (its own default, e.g. STABILIZE) must not be
                    # compared against a stale in-flight _commanded_mode.
                    # Without this, a failsafe-commanded "LAND" that never
                    # got updated kept mismatching against the FC's
                    # post-landing resting mode forever - RESUME cleared the
                    # latch, but the very next tick re-latched it again
                    # ("transmitter selected STABILIZE, we commanded LAND"),
                    # over and over, with no transmitter anywhere near the
                    # aircraft, until the whole process was restarted.
                    self._commanded_mode = ""
                    self._mode_at_command = ""
                    self._commanded_mode_reached = False
                    self._mode_mismatch_since = None

    def _on_mode(self, msg) -> None:
        if isinstance(msg, FlightMode):
            with self._lock:
                self._mode = msg.mode_name

    def _on_rc(self, msg) -> None:
        if isinstance(msg, RcChannels):
            with self._lock:
                self._rc = msg

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
        self.services.register("set_delivery_target", self._svc_set_delivery_target)
        self.services.register("abort_delivery", self._svc_abort_delivery)
        self.services.register("goto_gps", self._svc_goto_gps)

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
            self._sync_brake_params()
            self._maybe_auto_start()
            if self._check_pilot_override():
                # The human is flying: issue nothing, but keep the GCS warned.
                self._annunciate_while_manual()
            else:
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
            self._clear_pilot_override()
            self._begin_mission()

    def _begin_mission(self) -> None:
        if self._mission.count == 0:
            self._status_message = "no mission loaded"
            return
        self._current_wp = 0
        self._last_goto_wp = -1
        self._hover_until = None
        self._hover_wp = None
        self._hover_total_s = 0.0
        self._settle_until = None
        self._phase = MissionPhase.ARMING
        self._status_message = "arming"
        self.log.info("mission '%s' starting", self._mission.name)

    # -- pilot override ------------------------------------------------------
    #: Modes a human at the transmitter selects. If the autopilot reports one
    #: of these and we did not ask for it, the pilot has taken the aircraft
    #: and this node must get out of the way.
    _PILOT_MODES = frozenset({
        "STABILIZE", "ALT_HOLD", "ACRO", "SPORT", "DRIFT", "CIRCLE",
        "FLIP", "AUTOTUNE", "THROW", "LAND", "RTL", "SMART_RTL", "POSHOLD",
        "LOITER", "AUTO", "GUIDED", "BRAKE", "FOLLOW", "ZIGZAG",
    })

    def _check_pilot_override(self) -> bool:
        """Detect the pilot taking over. The transmitter always wins.

        Two independent detectors, either of which hands over the aircraft:

        1. ``RC_CHANNELS`` directly. The transmitter is the manual override,
           so any stick or switch movement past a deadband is a takeover,
           latched immediately. This path depends on nothing else - not on
           the FC accepting a mode change, not on our own command
           bookkeeping, and not on a grace timer - so it cannot be starved
           by this node re-issuing commands, which is exactly how a previous
           altitude-hardlock BRAKE loop locked the pilot out.
        2. The FC reporting a pilot-selectable mode we never commanded, for
           a mode we are confident we did not cause. Still useful when the
           RC channels are not forwarded to the companion computer.

        Control is handed back by the operator on the GCS (``resume`` /
        ``start_mission``), never automatically, so a deliberate takeover
        cannot be silently undone.
        """
        if self._rc_takeover():
            return True

        mode = self._mode
        if not mode or mode == "UNKNOWN":
            return self._pilot_override
        if not self._commanded_mode:
            return self._pilot_override      # we have not commanded anything yet
        if mode == self._commanded_mode:
            self._commanded_mode_reached = True
            self._mode_mismatch_since = None
            return self._pilot_override
        if mode not in self._PILOT_MODES:
            return self._pilot_override
        if not self._commanded_mode_reached and mode == self._mode_at_command:
            # The FC never left the mode it was already in when we asked for
            # something else. That is the autopilot refusing us, not a human
            # on the sticks: ArduPilot rejects GUIDED outright while the EKF
            # is complaining or the fix is thin. Reading it as a takeover
            # aborts the mission and blames a pilot who never touched it.
            self._note_mode_refusal(mode)
            return self._pilot_override
        now = time.monotonic()
        if self._mode_mismatch_since is None:
            self._mode_mismatch_since = now  # could just be command lag
            return self._pilot_override
        if (now - self._mode_mismatch_since) < self._override_grace:
            return self._pilot_override
        self._latch_override(f"transmitter selected {mode}", mode)
        return True

    def _rc_takeover(self) -> bool:
        """True once the pilot has moved something on the transmitter.

        The first live frame only establishes a baseline: a transmitter that
        is switched on and sitting still is not a takeover. Any channel that
        subsequently moves more than ``rc_override_deadband_us`` is.
        """
        rc = self._rc
        if rc is None:
            return self._pilot_override
        channels = [int(c) for c in (getattr(rc, "channels", None) or [])]
        if not channels or not any(c > 0 for c in channels):
            return self._pilot_override          # transmitter off / no link
        age = time.time() - float(getattr(rc, "stamp", 0.0) or 0.0)
        if age > self._rc_stale_s:
            # Frames keep arriving briefly after the transmitter goes off;
            # a stale frame is no link, not a live one.
            return self._pilot_override
        baseline = self._rc_baseline
        if baseline is None or len(baseline) != len(channels):
            self._rc_baseline = channels
            return self._pilot_override
        moved = [
            i for i, (now_us, base_us) in enumerate(zip(channels, baseline))
            if base_us > 0 and abs(now_us - base_us) > self._rc_deadband_us
        ]
        if not moved:
            return self._pilot_override
        where = ", ".join(str(i + 1) for i in moved)
        self._latch_override(f"transmitter moved (ch {where})", self._mode)
        return True

    def _note_mode_refusal(self, mode: str) -> None:
        """Say so, once, when the FC will not leave the mode it is in.

        This is the operator's real problem - a mode the autopilot will not
        accept - so it must not be swallowed just because it is not a
        takeover.
        """
        if self._mode_refusal_logged == (self._commanded_mode, mode):
            return
        self._mode_refusal_logged = (self._commanded_mode, mode)
        self._status_message = f"FC refused {self._commanded_mode} - still in {mode}"
        self.log.warning(
            "FC refused %s and stayed in %s - usually EKF/GPS quality. "
            "Not a pilot takeover; the mission stands.",
            self._commanded_mode, mode,
        )

    def _latch_override(self, why: str, mode: str) -> None:
        """Hand the aircraft to the human and stop commanding. Idempotent."""
        if self._pilot_override:
            return
        self._pilot_override = True
        self._hover_until = None
        self._hover_wp = None
        self._settle_until = None
        self._phase = MissionPhase.MANUAL
        self._manual_target = None
        self._manual_desc = f"pilot flying in {mode or 'UNKNOWN'}"
        self._status_message = f"PILOT OVERRIDE: {why}"
        self.log.warning(
            "pilot override: %s (FC in %s, we commanded %s) - standing down",
            why, mode or "UNKNOWN", self._commanded_mode or "nothing",
        )

    def _annunciate_while_manual(self) -> None:
        """Stand down without going quiet.

        The pilot has the aircraft and we command nothing, but the operator
        still needs a dying battery or a breached ceiling on the GCS - the
        crash this guards against was a silent hold that ran the pack flat.
        """
        warnings: list[str] = []
        battery = self._battery
        if battery is not None and battery.voltage_v:
            crit = float(self._safety.get("battery_critical_voltage", 13.2))
            low = float(self._safety.get("battery_low_voltage", 14.0))
            if battery.voltage_v <= crit:
                warnings.append(f"BATTERY CRITICAL {battery.voltage_v:.1f} V - LAND NOW")
            elif battery.voltage_v <= low:
                warnings.append(f"battery low {battery.voltage_v:.1f} V")
        if (
            self._fused is not None
            and self._home is not None
            and self._fused.alt_rel_m > (self._alt_ceiling + self._alt_margin)
        ):
            warnings.append(
                f"above ceiling: {self._fused.alt_rel_m:.1f} m > {self._alt_ceiling:.1f} m"
            )
        base = self._manual_desc or "pilot flying"
        if warnings:
            self._status_message = f"PILOT OVERRIDE ({base}) - " + "; ".join(warnings)
        else:
            self._status_message = f"PILOT OVERRIDE - {base}"

    def _clear_pilot_override(self) -> None:
        """Take control back. Operator-initiated only."""
        if self._pilot_override:
            self.log.info("pilot override cleared by operator")
        self._pilot_override = False
        self._mode_mismatch_since = None
        # Re-baseline the transmitter: wherever the sticks sit now is the new
        # neutral. Without this the next tick still sees the old deltas and
        # re-latches immediately, so the operator could never resume.
        self._rc_baseline = None
        self._last_mode_sent = ""

    # -- failsafe ------------------------------------------------------------
    def _check_failsafe(self) -> str | None:
        # ---- BATTERY CRITICAL --------------------------------------------
        # Checked first, and ungated like the hardlock below. Previously the
        # altitude hardlock returned before this, so an aircraft the hardlock
        # was holding in BRAKE could never reach the battery failsafe: it hung
        # there until the pack was flat and fell out of the sky. Nothing is
        # more urgent than a dying battery, so it is evaluated first.
        # Gated on _armed so a disarmed bench aircraft reporting 0 V (no pack
        # connected) cannot trip it.
        batt = self._battery
        if (
            self._armed
            and batt is not None
            and self._phase not in (MissionPhase.LAND, MissionPhase.EMERGENCY)
        ):
            if (
                batt.voltage_v
                and batt.voltage_v <= float(self._safety.get("battery_critical_voltage", 13.2))
            ) or (
                batt.remaining_pct
                and batt.remaining_pct <= float(self._safety.get("battery_critical_pct", 15))
            ):
                return "battery_critical"

        # ---- ALTITUDE HARDLOCK -------------------------------------------
        # Checked before the failsafes_enabled master switch, and before the
        # phase exemptions below, so it stays armed even when auto-failsafes
        # are turned off for bench testing. _clamp_alt already means we never
        # *command* a climb past the ceiling; this catches a climb we did not
        # command (baro drift, EKF altitude jump, a stale FC mission) and
        # arrests it. alt_rel_m is the autopilot's GLOBAL_POSITION_INT
        # relative_alt: metres above the home coordinates, which is exactly
        # what the ceiling is measured against.
        #
        # The response is BRAKE, not an automated descent. If the altitude
        # estimate is what went wrong, commanding "descend to 3 m" against a
        # bad estimate flies the aircraft into the ground; arresting the climb
        # and handing the operator the controls does not.
        if (
            self._fused is not None
            and self._home is not None
            and self._fused.alt_rel_m > (self._alt_ceiling + self._alt_margin)
        ):
            return "max_altitude"

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
            # (the altitude ceiling is checked at the top, ungated)
        return None

    def _apply_failsafe(self, reason: str) -> None:
        if reason == "battery_critical":
            self._status_message = "FAILSAFE: battery critical -> LAND"
            self._enter_land()
        elif reason == "max_altitude":
            alt = self._fused.alt_rel_m if self._fused else float("nan")
            self._status_message = (
                f"ALTITUDE HARDLOCK: {alt:.1f} m above home "
                f"(ceiling {self._alt_ceiling:.1f} m) -> BRAKE"
            )
            self._phase = MissionPhase.HOLD
            self._send("brake")
            self.log.error(
                "ALTITUDE HARDLOCK: %.2f m above home exceeds ceiling %.1f m "
                "+ margin %.1f m - braking. Take manual control.",
                alt, self._alt_ceiling, self._alt_margin,
            )
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
        elif phase == MissionPhase.HOVER:
            self._do_hover()
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

        if self._armed:
            self._arm_started_at = None
            self._arm_attempts = 0
            self._phase = MissionPhase.TAKEOFF
            self._send("takeoff", altitude=self._takeoff_alt)
            self._status_message = "taking off"
            return

        now = time.monotonic()
        if self._arm_started_at is None:
            self._arm_started_at = now
            self._arm_attempts = 0
            self._arm_last_try = 0.0

        # Give up rather than hammer the FC forever. A prearm check that has
        # not cleared in this long is not going to clear by being asked again.
        if (now - self._arm_started_at) > self._arm_timeout_s:
            reason = self._fc_refusal_reason() or "the autopilot refused to arm"
            self._phase = MissionPhase.IDLE
            self._arm_started_at = None
            self._status_message = f"arming failed: {reason}"
            self.log.error(
                "giving up after %.0fs and %d attempts - %s",
                self._arm_timeout_s, self._arm_attempts, reason,
            )
            return

        # One attempt per _arm_retry_s. The old code re-sent set_mode+arm on
        # every step (~10 Hz), which flooded the link with a dozen refusals a
        # second and told the operator nothing.
        if (now - self._arm_last_try) >= self._arm_retry_s:
            self._arm_last_try = now
            self._arm_attempts += 1
            self._set_mode("GUIDED")
            self._send("arm")

        reason = self._fc_refusal_reason()
        waited = now - self._arm_started_at
        self._status_message = (
            f"arming - {reason} ({waited:.0f}/{self._arm_timeout_s:.0f}s)"
            if reason
            else f"arming ({waited:.0f}/{self._arm_timeout_s:.0f}s)"
        )

    def _do_takeoff(self) -> None:
        alt = self._fused.alt_rel_m if self._fused else 0.0
        if alt < 0.95 * self._takeoff_alt:
            # Still climbing - a dip back below the gate restarts the settle.
            self._settle_until = None
            return
        now = time.monotonic()
        if self._settle_until is None:
            self._settle_until = now + self._takeoff_settle_s
            self.log.info(
                "takeoff altitude %.1f m reached - settling %.0fs before waypoint",
                self._takeoff_alt, self._takeoff_settle_s,
            )
        remaining = self._settle_until - now
        if remaining > 0:
            self._status_message = f"settling at {alt:.1f} m ({remaining:.0f}s)"
            return
        self._settle_until = None
        self._phase = MissionPhase.NAVIGATE
        self._last_goto_wp = -1
        self._status_message = "navigating"

    def _do_navigate(self) -> None:
        decision, distance = self._avoid_decision()
        self._avoiding = decision != CLEAR
        if decision == STOP:
            self._phase = MissionPhase.AVOID
            # Drop any direction left over from a previous encounter so the
            # next one is decided against a fresh scan.
            self._dodge_dir = None
            self._dodge_started = None
            self._dodge_origin = None
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
            self.log.info("reached waypoint %d", wp.seq)
            if wp.hold_s > 0:
                # Arrived over the drop point: hold position for hold_s before
                # the mission is allowed to continue (and, for the last
                # waypoint, before "waypoints done -> RTL" flies us home).
                self._enter_hover(wp)
                return
            self._current_wp += 1
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

    #: Flight modes whose *altitude* target comes from the pilot's throttle
    #: stick. Every one of them is described as "position hold", and every one
    #: of them holds altitude only for a pilot who is holding the throttle at
    #: centre. Nobody is: during an autonomous delivery the transmitter sits
    #: untouched with the throttle at RC3_MIN, and ArduPilot reads that as
    #: "descend at the full pilot rate" (PILOT_SPEED_DN, or PILOT_SPEED_UP when
    #: that is 0) the instant the mode is entered.
    #:
    #: This is not theoretical. ``hover_mode`` was POSHOLD, and on both logged
    #: delivery flights (dataflash 453 and 454) the aircraft held 3.4-3.7 m
    #: rock steady in GUIDED, then began falling within 0.35 s of the switch
    #: into POSHOLD, reached 2.3-2.5 m/s, and was still descending at that rate
    #: when it hit the ground - which is what broke the landing gear. The
    #: descent was commanded, not a failure: the logged DAlt (desired altitude)
    #: was being driven down, and the throttle channel read 999 throughout.
    #:
    #: The simulator hid it for months because ``SimWorld.set_mode`` parks the
    #: aircraft in POSHOLD/LOITER instead of modelling the throttle stick, so
    #: a hover that kills the real aircraft passes in sim.
    _STICK_ALTITUDE_MODES = frozenset({
        "STABILIZE", "ACRO", "SPORT", "DRIFT", "ALT_HOLD", "LOITER", "POSHOLD",
    })

    def _safe_hover_mode(self, mode: str) -> str:
        """Refuse a hover mode that an unattended throttle stick would fly.

        An autonomous hold must be flown by a mode that ignores the sticks -
        GUIDED (our setpoint) is the only one that also keeps us able to
        re-position. Rather than trust config to stay correct, the unsafe
        modes are rejected here, loudly, at construction.
        """
        if mode not in self._STICK_ALTITUDE_MODES:
            return mode
        self.log.error(
            "delivery.hover_mode=%s takes its altitude from the pilot's "
            "throttle stick, which rests at minimum during an autonomous "
            "delivery - entering it commands a full-rate descent into the "
            "ground. Holding in GUIDED instead.",
            mode,
        )
        return "GUIDED"

    def _enter_hover(self, wp: Waypoint) -> None:
        """Hold station over ``wp`` for ``wp.hold_s`` seconds.

        The hold is flown in ``hover_mode``, which ``_safe_hover_mode`` has
        already forced to a mode the pilot's sticks cannot drive - GUIDED by
        default, where the aircraft parks on the position setpoint we assert
        here and stays there with no further traffic from us, so a dropped
        link during the hold cannot move it.

        Do not "improve" this back to POSHOLD or LOITER for pilot
        nudgeability: see ``_STICK_ALTITUDE_MODES`` for the wreckage.
        """
        self._hover_total_s = float(wp.hold_s)
        self._hover_until = time.monotonic() + self._hover_total_s
        self._hover_prev_mode = self._mode if self._mode != "UNKNOWN" else "GUIDED"
        self._hover_wp = wp
        self._hover_recovering = False
        # Fresh hold, fresh handshake window - a signal left over from a
        # previous order (or one that arrived before this hold even started)
        # must not fire an immediate early RTL here.
        self._ble_delivered_at = None
        self._phase = MissionPhase.HOVER
        if self._hover_mode != "GUIDED":
            self._set_mode(self._hover_mode)
        else:
            # Re-assert the position target; GUIDED parks on the last setpoint.
            self._issue_goto(wp)
        self._status_message = f"hovering {self._hover_total_s:.0f}s over drop point"
        self.log.info(
            "hover started: %.0fs at %.7f, %.7f (mode %s)",
            self._hover_total_s, wp.lat, wp.lon, self._hover_mode,
        )

    def _guard_hover_altitude(self) -> bool:
        """Catch the drop-point hold turning into a descent. Returns True if it has.

        With a stick-proof ``hover_mode`` the hold should simply not sink, so
        this is the backstop for the ways it could anyway: an FC that refused
        our mode and stayed in a pilot-driven one, a setpoint that never
        arrived, an altitude estimate that walked. The response is to re-assert
        GUIDED and the position target - the same thing that was holding the
        aircraft steady moments earlier - once per breach, so this can never
        become a 10 Hz setpoint spray fighting the autopilot.
        """
        wp = self._hover_wp
        if self._fused is None or wp is None:
            return False
        target = wp.alt_m or self._hover_alt
        sag = target - self._fused.alt_rel_m
        if sag <= self._hover_alt_tolerance:
            self._hover_recovering = False
            return False
        if self._hover_recovering:
            return True                       # already re-asserted; let it work
        self._hover_recovering = True
        self.log.error(
            "HOVER SAG: %.2f m below the %.1f m hold altitude (now %.2f m, FC "
            "in %s) - re-asserting the position hold",
            sag, target, self._fused.alt_rel_m, self._mode,
        )
        self._status_message = (
            f"HOVER SAGGING {sag:.1f} m below {target:.1f} m - re-asserting hold"
        )
        self._set_mode("GUIDED")
        self._issue_goto(wp)
        return True

    @property
    def hover_remaining_s(self) -> float:
        """Seconds left on the current hold, 0 when not hovering."""
        with self._lock:
            if self._hover_until is None:
                return 0.0
            return max(0.0, self._hover_until - time.monotonic())

    def _do_hover(self) -> None:
        if self._hover_until is None:          # defensive: nothing to wait on
            self._current_wp += 1
            self._phase = MissionPhase.NAVIGATE
            self._last_goto_wp = -1
            return
        sagging = self._guard_hover_altitude()
        remaining = self._hover_until - time.monotonic()

        # BLE early exit: the handshake's drop gates already passed for this
        # order (_on_ble_delivery_result), so cut the hold short
        # ble_early_rtl_wait_s after that instead of waiting out the rest of
        # hover_seconds. If it never arrives, remaining <= 0 below is exactly
        # the pre-existing "handshake failed - RTL at the timeout" behavior.
        ble_ready = False
        if self._ble_early_rtl_enabled and self._ble_delivered_at is not None:
            ble_wait_left = self._ble_early_rtl_wait_s - (
                time.monotonic() - self._ble_delivered_at
            )
            if ble_wait_left <= 0:
                ble_ready = True
            elif not sagging:
                self._status_message = (
                    f"HOVER: BLE handshake confirmed - RTL in {ble_wait_left:.0f}s"
                )

        if remaining > 0 and not ble_ready:
            if not sagging and self._ble_delivered_at is None:
                # _guard_hover_altitude owns the status line while it is
                # recovering; do not overwrite its warning with a countdown.
                self._status_message = (
                    f"HOVER: {remaining:.0f}s remaining over drop point"
                )
            return
        # Hold complete - hand control back to the mission, either because
        # hover_seconds ran out or the post-handshake grace period did.
        self._hover_until = None
        self._hover_wp = None
        self._hover_recovering = False
        self._ble_delivered_at = None
        self._current_wp += 1
        self._last_goto_wp = -1
        if ble_ready:
            self.log.info(
                "hover cut short after %.0fs (of %.0fs) - BLE handshake confirmed",
                self._hover_total_s - max(0.0, remaining), self._hover_total_s,
            )
        else:
            self.log.info("hover complete after %.0fs", self._hover_total_s)
        if self._active_waypoint() is None:
            # That was the drop point and nothing follows it: go straight
            # home. Stepping through GUIDED first would be two mode changes
            # inside one 100 ms tick, and GUIDED is the mode the FC is most
            # likely to refuse (EKF/GPS quality). A refusal there would leave
            # us holding station over the customer until the battery failsafe
            # takes the aircraft, which is exactly the failure we already had
            # once. SMART_RTL does not need GUIDED first.
            self._status_message = "hover complete - returning to base"
            self._enter_rtl()
            return
        self._phase = MissionPhase.NAVIGATE
        if self._hover_mode != "GUIDED":
            # Back into GUIDED so our goto commands are honoured again.
            self._set_mode("GUIDED")
        self._status_message = "hover complete - resuming mission"

    def _do_avoid(self) -> None:
        decision, distance = self._avoid_decision()
        self._avoiding = True
        if decision == STOP:
            self._dodge_step(distance)
            return
        # Path ahead is clear again - resume navigation. Clearing _last_goto_wp
        # forces _do_navigate to re-issue the goto, which is what pulls the
        # aircraft back onto its original track instead of carrying on from
        # wherever the sidestep left it.
        self._dodge_dir = None
        self._dodge_started = None
        self._dodge_origin = None
        self._offtrack_m = 0.0
        self._avoiding = decision != CLEAR
        self._phase = MissionPhase.NAVIGATE
        self._last_goto_wp = -1
        self._status_message = "obstacle cleared - rejoining track"

    def _dodge_step(self, distance: float) -> None:
        """Sidestep toward the open side, or hold when there is not one.

        The reactive half of the ported logic. The direction is latched on
        entry and held until the front clears, the chosen side closes to the
        stop distance, or avoidance_dodge_timeout_s expires.

        It is latched rather than re-decided every tick because the two
        outcomes here command different flight modes - "velocity" implies
        GUIDED, "brake" implies BRAKE - and scan jitter around the openness
        threshold would otherwise flip between them at the 10 Hz loop rate.
        Re-commanding a mode at loop rate is exactly what locked the pilot out
        of the aircraft on 2026-08-22; see tests/test_transmitter_authority.py.
        """
        if not self._avoider.dodge_enabled:
            self._send("brake")
            self._status_message = f"holding for obstacle at {distance:.1f} m"
            return

        view = self._avoider.sectors(self._obstacles)
        now = time.monotonic()

        if self._dodge_dir is None:
            side = self._avoider.dodge(view)
            if side == DODGE_TRAPPED:
                # Neither side is open. The source node reverses out of a dead
                # end here; this aircraft cannot. The rear 90 deg is masked, so
                # "behind" is unmeasured rather than clear, and the 3 m ceiling
                # (safety.max_altitude_m, enforced by the altitude hardlock)
                # rules out climbing over. Hold and let the pilot or the
                # mission decide.
                self._send("brake")
                rear = ", rear masked" if view.rear_blind else ""
                self._status_message = (
                    f"trapped at {distance:.1f} m (L {_m(view.left)} / "
                    f"R {_m(view.right)}{rear}) - holding"
                )
                return
            self._dodge_dir = side
            self._dodge_started = now
            self._dodge_origin = (
                (self._fused.x, self._fused.y)
                if self._fused is not None and self._fused.valid
                else None
            )
            self.log.warning(
                "obstacle at %.1f m - dodging %s (front %s, left %s, right %s)",
                distance, side, _m(view.front), _m(view.left), _m(view.right),
            )

        chosen = view.left if self._dodge_dir == DODGE_LEFT else view.right
        expired = (
            self._dodge_started is not None
            and (now - self._dodge_started) > self._avoider.dodge_timeout_s
        )
        self._offtrack_m = self._compute_offtrack()
        strayed = self._offtrack_m > self._avoider.max_offtrack_m
        if expired or strayed or chosen <= self._avoider.stop:
            # The sidestep is not working, is about to fly us into the side we
            # picked, or has pushed us further off the planned track than the
            # mission allows. Stop. Do not swing to the other side on the same
            # encounter - that is how a reactive dodge oscillates.
            #
            # Holding is not a dead end: _do_avoid returns to NAVIGATE the
            # moment the front clears, and re-issues the goto, so the aircraft
            # rejoins its original track rather than continuing from wherever
            # the dodge left it.
            self._dodge_dir = None
            self._dodge_started = None
            self._dodge_origin = None
            self._send("brake")
            if expired:
                what = "timed out"
            elif strayed:
                what = f"{self._offtrack_m:.1f} m off track"
            else:
                what = "blocked"
            self._status_message = f"dodge {what} at {distance:.1f} m - holding"
            return

        # Forward speed is earned, not assumed. While the front is still inside
        # the brake distance this stays at zero and the dodge is a pure
        # sidestep; as the sidestep opens the front up, forward motion fades in
        # and the manoeuvre becomes going *around* the obstacle rather than
        # crabbing sideways past it forever. Flying forward at a wall 1.7 m away
        # because we happened to be dodging would defeat the whole margin.
        margin = view.front - self._avoider.last_stop_m
        vx = 0.0
        if margin > 0.0:
            vx = min(self._avoider.dodge_forward_ms, margin * 0.5)

        # MAV_FRAME_BODY_NED: +y is right of the nose, the same sign convention
        # ObstacleNode reports bearings in.
        vy = (
            self._avoider.dodge_speed
            if self._dodge_dir == DODGE_RIGHT
            else -self._avoider.dodge_speed
        )
        self._send("velocity", vx=vx, vy=vy, vz=0.0)
        self._status_message = (
            f"obstacle {distance:.1f} m -> dodging {self._dodge_dir} "
            f"(L {_m(view.left)} / R {_m(view.right)}, "
            f"fwd {vx:.1f} m/s, off-track {self._offtrack_m:.1f} m)"
        )

    def _compute_offtrack(self) -> float:
        """Perpendicular distance from the line the dodge departed from.

        The track is the straight line from where the dodge started to the
        waypoint it was heading for. Returns 0.0 when there is no fix or no
        active waypoint - an unknown excursion must not read as a large one and
        trip the cap, because that would turn "no GPS" into "never dodge".
        """
        if self._dodge_origin is None or self._fused is None or not self._fused.valid:
            return 0.0
        wp = self._active_waypoint()
        if wp is None or self._home is None:
            return 0.0
        tx, ty = wp.x_m, wp.y_m
        if wp.lat != 0.0 or wp.lon != 0.0:
            tx, ty = geodetic_to_enu(wp.lat, wp.lon, self._home[0], self._home[1])
        ox, oy = self._dodge_origin
        dx, dy = tx - ox, ty - oy
        leg = math.hypot(dx, dy)
        if leg < 1e-6:
            return 0.0
        # |cross product| / |leg| - the perpendicular offset of the current
        # position from the origin->target line.
        px, py = self._fused.x - ox, self._fused.y - oy
        return abs(px * dy - py * dx) / leg

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
            self._rtl_requested_at = None
            return
        # SMART_RTL retraces the outbound path instead of cutting a straight
        # line home, so it will not fly back through anything we already
        # dodged. The autopilot refuses it when its path buffer is empty or
        # exhausted, and a refused mode change is silent, so watch for it and
        # fall back to plain RTL rather than loitering forever.
        if (
            self._return_mode == "SMART_RTL"
            and not self._rtl_fell_back
            and self._rtl_requested_at is not None
            and self._mode != "SMART_RTL"
            and (time.monotonic() - self._rtl_requested_at) > 3.0
        ):
            self._rtl_fell_back = True
            self.log.warning(
                "SMART_RTL not accepted (FC still in %s) - falling back to RTL",
                self._mode,
            )
            self._set_mode("RTL")
            self._send("rtl")
            self._status_message = "returning to launch (RTL fallback)"
            return
        self._status_message = (
            "smart RTL: retracing path home"
            if self._mode == "SMART_RTL"
            else "returning to launch"
        )

    def _do_land(self) -> None:
        if not self._armed:
            self._phase = MissionPhase.DISARMED
            self._status_message = "landed and disarmed"

    # -- helpers -------------------------------------------------------------
    def _enter_rtl(self) -> None:
        """Come home. Prefers SMART_RTL, falls back to RTL (see _do_rtl)."""
        self._phase = MissionPhase.RTL
        self._rtl_requested_at = time.monotonic()
        self._rtl_fell_back = self._return_mode != "SMART_RTL"
        if self._return_mode == "SMART_RTL":
            self._set_mode("SMART_RTL")
        else:
            self._set_mode("RTL")
            self._send("rtl")
        self.log.info("returning home via %s", self._return_mode)

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
        """Command the autopilot to the waypoint's own coordinate.

        A delivery waypoint arrives as the customer's lat/lon; ``x_m``/``y_m``
        are derived from it for the distance maths (see Waypoint, where they
        are documented as the *optional alternative* to lat/lon). Sending the
        derived pair back through ``enu_to_geodetic`` happens to round-trip
        exactly today, because that function is the precise inverse of
        ``geodetic_to_enu`` at these ranges - but the delivered coordinate then
        silently depends on that staying true. Use the number the order
        actually carried instead. Missions expressed in local metres (the demo
        route, load_mission with no lat/lon) still need the conversion.
        """
        if self._home is None:
            return
        lat, lon = wp.lat, wp.lon
        if lat == 0.0 and lon == 0.0:
            lat, lon = enu_to_geodetic(wp.x_m, wp.y_m, self._home[0], self._home[1])
        self._send("goto", lat=lat, lon=lon, alt=wp.alt_m or self._cruise_alt)
        self.log.info(
            "goto waypoint %d -> %.7f, %.7f (%.1f, %.1f m)",
            wp.seq, lat, lon, wp.x_m, wp.y_m,
        )

    #: Commands that change flight mode, and the mode each one lands in.
    #: _send is the single choke point every command leaves by, so recording
    #: the implied mode here is what keeps _commanded_mode in step with
    #: reality. It drifting out of step is what made this node mistake its
    #: own BRAKE for a pilot takeover - and, when it had commanded nothing
    #: at all, skip the override check entirely and re-BRAKE at 10 Hz.
    _MODE_COMMANDS = {
        "brake": "BRAKE",
        "land": "LAND",
        "rtl": "RTL",
        "smart_rtl": "SMART_RTL",
        "takeoff": "GUIDED",
        "goto": "GUIDED",
        "velocity": "GUIDED",
    }

    #: Commands whose only effect is the mode change, so they are safe to drop
    #: when the FC is already in that mode. takeoff/goto/velocity carry a
    #: payload as well and must always go out.
    _MODE_ONLY_COMMANDS = frozenset({"set_mode", "brake", "land", "rtl", "smart_rtl"})

    def _set_mode(self, mode: str) -> None:
        """Ask the FC for a mode and remember that we asked.

        Delegates to _send, which records the mode and suppresses duplicate
        SET_MODE frames. It deliberately no longer clears
        _mode_mismatch_since unconditionally: doing that on every call let a
        once-per-tick re-command reset the override grace timer forever, so
        the timer could never expire and the pilot could never take over.
        """
        self._send("set_mode", mode=mode.upper())

    def _send(self, command: str, **params) -> None:
        # Last line of defence on the altitude ceiling: every altitude-bearing
        # command leaves through here, whatever built it.
        if "alt" in params:
            params["alt"] = self._clamp_alt(params["alt"])
        if "altitude" in params:
            params["altitude"] = self._clamp_alt(params["altitude"])

        implied = self._MODE_COMMANDS.get(command)
        if command == "set_mode":
            implied = str(params.get("mode", "")).upper()
        if implied:
            if self._commanded_mode != implied:
                # Only a genuine change of intent restarts the grace timer.
                self._commanded_mode = implied
                self._mode_mismatch_since = None
                self._mode_at_command = self._mode
                self._commanded_mode_reached = False
                self._mode_refusal_logged = None
            if command in self._MODE_ONLY_COMMANDS:
                now = time.monotonic()
                if self._mode == implied:
                    # Already there. Re-sending SET_MODE every tick is what
                    # locked the pilot out: each frame pulled the aircraft
                    # back out of the mode they had just selected.
                    self._last_mode_sent = implied
                    self._last_mode_sent_at = now
                    return
                if (
                    self._last_mode_sent == implied
                    and (now - self._last_mode_sent_at) < self._mode_resend_s
                ):
                    return                      # retry, but paced
                self._last_mode_sent = implied
                self._last_mode_sent_at = now

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
            self._set_mode(mode)
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
            outcome = self._resume_locked()
            if outcome is not None:
                return outcome
            if self._mission.count == 0:
                return False, "no mission to resume"
            self._phase = MissionPhase.NAVIGATE
            self._last_goto_wp = -1
            return True, "resuming mission"
        if action == "start_mission":
            if self._pilot_override:
                return False, "pilot override active - press RESUME to take control back"
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
        requested = altitude
        altitude = self._clamp_alt(altitude)
        self._set_mode("GUIDED")
        self._send("arm")
        self._send("takeoff", altitude=altitude)
        self._phase = MissionPhase.MANUAL
        self._manual_target = None
        self._manual_desc = (
            f"takeoff to {altitude:.1f} m" + self._alt_note(requested, altitude)
        )
        return True, self._manual_desc

    def _manual_goto_relative(
        self, dx: float, dy: float, dz: float, absolute_alt: float | None = None
    ) -> tuple[bool, str]:
        """Move relative to the body frame (dx fwd, dy left, dz up), or set an
        absolute altitude, by issuing a GUIDED goto and entering MANUAL mode."""
        if self._fused is None or self._home is None:
            return False, "no position fix yet (waiting for GPS)"
        if not self._armed:
            self._set_mode("GUIDED")
            self._send("arm")
        yaw = self._fused.yaw
        tx = self._fused.x + dx * math.cos(yaw) - dy * math.sin(yaw)
        ty = self._fused.y + dx * math.sin(yaw) + dy * math.cos(yaw)
        if absolute_alt is not None:
            talt = self._clamp_alt(absolute_alt)
            desc = f"go to altitude {talt:.1f} m" + self._alt_note(absolute_alt, talt)
        else:
            talt = self._clamp_alt(self._fused.alt_rel_m + dz)
            # Describe the climb we will actually fly, not the one requested:
            # "move up 999 m" while the clamp sends us to 3 m is the same lie
            # as on takeoff. dz is the one part of a move that can be refused,
            # so it is reported in the delta the operator asked in, with the
            # requested figure kept alongside it.
            flown_dz = talt - self._fused.alt_rel_m
            parts = []
            if dx:
                parts.append(f"{'forward' if dx > 0 else 'back'} {abs(dx):.1f} m")
            if dy:
                parts.append(f"{'left' if dy > 0 else 'right'} {abs(dy):.1f} m")
            if dz:
                if abs(flown_dz) < 0.05:
                    parts.append("holding altitude")
                else:
                    parts.append(
                        f"{'up' if flown_dz > 0 else 'down'} {abs(flown_dz):.1f} m"
                    )
            desc = "move " + ", ".join(parts) if parts else "hold"
            if dz and abs(flown_dz - dz) >= 0.05:
                desc += (
                    f" (requested {abs(dz):.1f} m {'up' if dz > 0 else 'down'}, "
                    f"limited to {talt:.1f} m by the "
                    f"{self._alt_ceiling:.0f} m ceiling)"
                )
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
                pilot_override=self._pilot_override,
                alt_ceiling_m=self._alt_ceiling,
                arm_refusal=self._fc_refusal_reason(),
                home_lat=self._home[0] if self._home else 0.0,
                home_lon=self._home[1] if self._home else 0.0,
                home_set=self._home is not None,
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
        view = self._avoider.sectors(self._obstacles)
        # What the dodge WOULD do, so the panel shows the escape route before
        # the aircraft needs it - not only once it is already sidestepping.
        if not self._avoid_enabled or not self._avoider.dodge_enabled:
            dodge = DODGE_NONE
        elif self._dodge_dir is not None:
            dodge = self._dodge_dir
        elif decision == STOP:
            dodge = self._avoider.dodge(view)
        else:
            dodge = DODGE_NONE

        status = {"clear": "CLEAR", "slow": "SLOW", "stop": "BRAKE"}[decision]
        if not self._avoid_enabled:
            status = "OFF"

        # Closing speed / time-to-collision / closest-point-of-approach.
        #
        # Closing speed comes from the obstacle's own track when it has one.
        # The old estimate - our ground speed projected onto the bearing -
        # assumed the obstacle was nailed to the ground, so a person walking
        # into the path while the aircraft hovered read as zero closing and
        # infinite time-to-collision. Tracking measures the gap shrinking
        # regardless of which of the two is moving.
        ttc = 0.0
        cpa = 0.0
        closing = 0.0
        direction = "none"
        if front is not None:
            bearing = math.radians(front.bearing_deg)
            tracked = getattr(front, "closing_ms", 0.0) or 0.0
            if front.hits >= 2:
                closing = tracked
            else:
                speed = math.hypot(self._fused.vx, self._fused.vy) if self._fused else 0.0
                closing = speed * math.cos(bearing)
            if closing > 0.1:
                ttc = round(front.distance_m / closing, 2)
            cpa = round(abs(front.distance_m * math.sin(bearing)), 2)
            if abs(front.bearing_deg) < 10:
                direction = "front"
            else:
                direction = "right" if front.bearing_deg > 0 else "left"

        dynamic_count = 0
        if self._obstacles is not None:
            dynamic_count = sum(
                1 for o in self._obstacles.obstacles if getattr(o, "is_dynamic", False)
            )

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
                front_m=_finite(view.front),
                left_m=_finite(view.left),
                right_m=_finite(view.right),
                dodge=dodge,
                rear_blind=view.rear_blind,
                front_half_deg=round(self._avoider.sector_deg, 1),
                side_half_deg=round(self._avoider.side_half_deg, 1),
                closing_ms=round(closing, 2),
                dynamic_count=dynamic_count,
                stop_distance_m=round(self._avoider.last_stop_m, 2),
                offtrack_m=round(self._offtrack_m, 2),
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

    #: Autopilot parameter for the HARD BRAKE - the last-resort stop. Simple
    #: avoidance (AltHold/Loiter only) is the system that enforces it, and it is
    #: the one the GCS emergency-brake control sets.
    #:
    #: OA_MARGIN_MAX is deliberately NOT in here. It looks like the same idea
    #: and is not: it is the clearance the *planned route* keeps in Guided/RTL,
    #: not the distance at which the aircraft panics. Driving both from one
    #: control made the planner shave its path to exactly the brake distance,
    #: so any drift fired the brake - the opposite of what the brake is for.
    #: The route margin comes from avoidance_route_margin_m instead and is
    #: held strictly wider than the brake, so routing happens first and the
    #: brake stays the thing that never normally triggers.
    _BRAKE_DISTANCE_PARAMS = ("AVOID_MARGIN",)

    #: Path-planning clearance in Guided/RTL. Separate concept, separate knob.
    _ROUTE_MARGIN_PARAM = "OA_MARGIN_MAX"

    def _sync_brake_params(self) -> None:
        """Push the brake distance and route margin to the FC after link-up.

        Called from step() rather than on_start() because on_start runs before
        MAVLink has a link, and a PARAM_SET sent into a closed link is simply
        dropped - the FC would then keep whatever margin a previous session
        left on it.
        """
        if self._brake_params_synced:
            return
        if self._link is None or not getattr(self._link, "connected", False):
            return
        self._brake_params_synced = True
        self._push_brake_distance(self._avoider.stop)
        self._push_route_margin()
        self.log.info(
            "brake %.2f m (%s) / route clearance %.2f m (%s) mirrored to FC",
            self._avoider.stop, "/".join(self._BRAKE_DISTANCE_PARAMS),
            self._route_margin(), self._ROUTE_MARGIN_PARAM,
        )

    def _route_margin(self) -> float:
        """Clearance the planner keeps, always strictly wider than the brake.

        Clamped rather than trusted: a route margin at or below the brake
        distance means the planner is aiming for the exact line at which the
        aircraft slams on the brakes, and every plan would end in a hard stop.
        """
        return max(self._avoider.route_margin, self._avoider.stop + 0.5)

    def _push_brake_distance(self, dist: float) -> None:
        """Mirror the emergency-brake distance onto the flight controller.

        The GCS slider is the single source of truth for the hard stop, so it
        must reach the FC as well as this node's own reactive layer - the Pi is
        not what stops the aircraft in Loiter.
        """
        for name in self._BRAKE_DISTANCE_PARAMS:
            self._send("set_param", name=name, value=round(dist, 2))

    def _push_route_margin(self) -> None:
        self._send(
            "set_param",
            name=self._ROUTE_MARGIN_PARAM,
            value=round(self._route_margin(), 2),
        )

    def _svc_set_stop_distance(self, req: ServiceRequest) -> ServiceResponse:
        """Live-tune the emergency-braking trigger distance (metres).

        Sets CollisionAvoider.stop, keeps the slow-down band at or above it so
        SLOW never sits below STOP, and mirrors the value onto AVOID_MARGIN.

        The route margin is re-pushed too: it is clamped to stay strictly wider
        than the brake, so raising the brake past it has to widen it as well -
        otherwise the planner would be aiming for the line the aircraft brakes
        at, and every route would end in a hard stop.
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
        self._push_brake_distance(dist)
        self._push_route_margin()
        self.log.info(
            "emergency-brake stop distance set to %.2f m (GCS + %s), "
            "route clearance now %.2f m",
            dist, "/".join(self._BRAKE_DISTANCE_PARAMS), self._route_margin(),
        )
        return ServiceResponse(
            True, f"emergency brake at {dist:.2f} m", data={"stop_m": dist}
        )

    # -- delivery ------------------------------------------------------------
    def _expand_delivery_mission(
        self, lat: float, lon: float, alt_m: float, hover_s: float
    ) -> list[Waypoint]:
        """Turn one customer GPS coordinate into a flyable waypoint list.

        The customer gives us a single point. The autopilot wants a *route*.
        This splits the straight line from home to that point into legs of
        ``delivery.leg_length_m`` (capped at ``delivery.max_legs``) and puts a
        hold-for-``hover_s`` waypoint at the end.

        Splitting matters for two reasons: the navigator issues one ``goto``
        per waypoint, so a single 400 m hop would fly the whole leg before it
        next re-evaluated the LiDAR obstacle field; and the GCS map draws the
        waypoint list, so the intermediate legs are what make the intended
        track visible before you commit to it. Home must be known (GPS fix) -
        every waypoint carries both lat/lon and the local ENU metres the
        navigator actually measures distance in.
        """
        home_lat, home_lon = self._home
        total_m = haversine_m(home_lat, home_lon, lat, lon)
        legs = int(math.ceil(total_m / max(1.0, self._leg_m))) if total_m > 0 else 1
        legs = max(1, min(legs, self._max_legs))

        waypoints: list[Waypoint] = []
        for i in range(1, legs + 1):
            frac = i / legs
            final = i == legs
            wlat = lat if final else home_lat + (lat - home_lat) * frac
            wlon = lon if final else home_lon + (lon - home_lon) * frac
            x_m, y_m = geodetic_to_enu(wlat, wlon, home_lat, home_lon)
            waypoints.append(
                Waypoint(
                    seq=i - 1,
                    lat=wlat,
                    lon=wlon,
                    alt_m=self._clamp_alt(alt_m if final else self._cruise_alt),
                    x_m=x_m, y_m=y_m,
                    radius_m=self._wp_radius,
                    hold_s=hover_s if final else 0.0,
                    kind="hover" if final else "nav",
                )
            )
        return waypoints

    def _svc_set_delivery_target(self, req: ServiceRequest) -> ServiceResponse:
        """Load a delivery mission built from one customer GPS coordinate.

        Called by FirebaseDeliveryNode once an order has been accepted. Loads
        the mission but deliberately does **not** launch: the caller decides
        when to ``start_mission``, so accepting an order and committing the
        aircraft to the air stay two separate decisions.
        """
        try:
            lat = float(req.data["lat"])
            lon = float(req.data["lon"])
        except (KeyError, TypeError, ValueError):
            return ServiceResponse(False, "lat and lon are required")
        alt_m = self._clamp_alt(float(req.data.get("alt_m", self._hover_alt)))
        hover_s = float(req.data.get("hover_s", self._default_hover_s))
        # "name" is reserved by ServiceRegistry.call(name, **data), so callers
        # pass mission_name; the old key stays accepted for direct callers.
        name = str(req.data.get("mission_name") or req.data.get("name") or "delivery")

        with self._lock:
            if self._home is None:
                return ServiceResponse(
                    False, "no home position yet (waiting for GPS fix)"
                )
            waypoints = self._expand_delivery_mission(lat, lon, alt_m, hover_s)
            self._mission = Mission(name=name, waypoints=waypoints)
            self._current_wp = 0
            self._last_goto_wp = -1
            self._hover_until = None
            self._delivery_target = (lat, lon, alt_m)
            distance = haversine_m(self._home[0], self._home[1], lat, lon)
        self.publish(Topics.MISSION_PLAN, self._mission)
        self.log.info(
            "delivery mission %r: %d waypoint(s) over %.0f m to %.7f, %.7f "
            "(hover %.0fs at %.1f m)",
            name, len(waypoints), distance, lat, lon, hover_s, alt_m,
        )
        return ServiceResponse(
            True,
            f"{len(waypoints)} waypoints, {distance:.0f} m to target",
            data={
                "waypoints": len(waypoints),
                "distance_m": round(distance, 1),
                "hover_s": hover_s,
                "alt_m": alt_m,
                "plan": [
                    {"seq": w.seq, "lat": w.lat, "lon": w.lon,
                     "alt_m": w.alt_m, "hold_s": w.hold_s, "kind": w.kind}
                    for w in waypoints
                ],
            },
        )

    def _svc_abort_delivery(self, req: ServiceRequest) -> ServiceResponse:
        """Stop the delivery wherever it is and come home."""
        with self._lock:
            self._hover_until = None
            self._delivery_target = None
            self._mission = Mission(name="aborted", waypoints=[])
            self._current_wp = 0
            self._last_goto_wp = -1
            if self._armed:
                self._enter_rtl()
                message = "delivery aborted - returning to launch"
            else:
                self._phase = MissionPhase.IDLE
                message = "delivery aborted (aircraft was not armed)"
            self._status_message = message
        self.publish(Topics.MISSION_PLAN, self._mission)
        self.log.warning("%s", message)
        return ServiceResponse(True, message)

    def _svc_goto_gps(self, req: ServiceRequest) -> ServiceResponse:
        """Fly to one absolute lat/lon now, without loading a mission.

        Used for a manual reposition from the GCS map (reuses MANUAL so
        avoidance and status reporting keep working en route), and by
        drone_ble_peripheral.py's autonomous "follow-me" relocation
        (request_follow_me) when the phone isn't within the micro-geofence
        yet. During a delivery HOVER, follow-me must reposition the hold
        itself rather than abandon it into MANUAL: _do_manual never returns to
        HOVER on its own (it just parks at the new target once reached), which
        would silently strand both the BLE early-RTL countdown and the plain
        hover_seconds timeout/RTL forever - the aircraft would just sit there.
        """
        try:
            lat = float(req.data["lat"])
            lon = float(req.data["lon"])
        except (KeyError, TypeError, ValueError):
            return ServiceResponse(False, "lat and lon are required")
        alt = self._clamp_alt(float(req.data.get("alt", self._cruise_alt)))
        with self._lock:
            if self._home is None:
                return ServiceResponse(False, "no position fix yet (waiting for GPS)")
            if self._phase == MissionPhase.HOVER and self._hover_wp is not None:
                self._hover_wp.lat = lat
                self._hover_wp.lon = lon
                self._issue_goto(self._hover_wp)
                self.log.info(
                    "goto_gps during delivery hover: relocating hold to %.7f, %.7f",
                    lat, lon,
                )
                return ServiceResponse(True, f"hover relocated to {lat:.6f},{lon:.6f}")
            if not self._armed:
                self._set_mode("GUIDED")
                self._send("arm")
            tx, ty = geodetic_to_enu(lat, lon, self._home[0], self._home[1])
            self._send("goto", lat=lat, lon=lon, alt=alt)
            self._manual_target = (tx, ty, alt)
            self._manual_desc = f"goto {lat:.6f},{lon:.6f}"
            self._phase = MissionPhase.MANUAL
        self.log.info("goto_gps: %.7f, %.7f at %.1f m", lat, lon, alt)
        return ServiceResponse(True, self._manual_desc, data={"x_m": tx, "y_m": ty})

    def _svc_load_mission(self, req: ServiceRequest) -> ServiceResponse:
        raw = req.data.get("waypoints", [])
        waypoints = []
        for i, item in enumerate(raw):
            waypoints.append(
                Waypoint(
                    seq=item.get("seq", i),
                    x_m=float(item.get("x_m", 0.0)),
                    y_m=float(item.get("y_m", 0.0)),
                    alt_m=self._clamp_alt(float(item.get("alt_m", self._cruise_alt))),
                    lat=float(item.get("lat", 0.0)),
                    lon=float(item.get("lon", 0.0)),
                    radius_m=float(item.get("radius_m", self._wp_radius)),
                    hold_s=float(item.get("hold_s", 0.0)),
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
            if self._pilot_override:
                return ServiceResponse(False, "pilot override active - press RESUME to take control back")
            if self._mission.count == 0:
                return ServiceResponse(False, "no mission loaded")
            self._start_requested = True
        return ServiceResponse(True, "mission start requested")

    def _svc_hold(self, req: ServiceRequest) -> ServiceResponse:
        with self._lock:
            self._phase = MissionPhase.HOLD
            self._status_message = "holding"
        return ServiceResponse(True, "holding position")

    def _resume_locked(self) -> tuple[bool, str] | None:
        """Operator-initiated hand-back. The caller must hold ``_lock``.

        Every way of asking to resume has to land here. The ``resume`` service
        and the natural-language ``resume`` intent used to carry separate
        copies of this, and the NL copy - the one the GCS button actually goes
        through - only set the phase. That left ``_pilot_override`` latched, so
        ``step`` went on standing down, ``_start_requested`` was never consumed
        and every accepted delivery aborted a tick later while resume itself
        cheerfully reported success.

        Returns ``None`` when there is nothing to hand back, so each caller
        keeps its own wording for that case.
        """
        if self._pilot_override:
            # Nothing clears a transmitter takeover automatically.
            self._clear_pilot_override()
            if self._mission.count and self._current_wp < self._mission.count:
                self._set_mode("GUIDED")
                self._phase = MissionPhase.NAVIGATE
                self._last_goto_wp = -1
                return True, "control resumed - continuing mission"
            self._phase = MissionPhase.IDLE
            return True, "control resumed"
        if self._phase == MissionPhase.HOLD:
            self._phase = MissionPhase.NAVIGATE
            self._last_goto_wp = -1
            return True, "resumed"
        return None

    def _svc_resume(self, req: ServiceRequest) -> ServiceResponse:
        with self._lock:
            outcome = self._resume_locked()
            if outcome is not None:
                return ServiceResponse(*outcome)
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
                    "hover_remaining_s": round(self.hover_remaining_s, 1),
                    "hover_total_s": self._hover_total_s,
                    "mission_name": self._mission.name,
                    "arm_refusal": self._fc_refusal_reason(),
                    "home": list(self._home) if self._home else None,
                    "alt_ceiling_m": self._alt_ceiling,
                    "hover_mode": self._hover_mode,
                    "return_mode": self._return_mode,
                    "pilot_override": self._pilot_override,
                    "commanded_mode": self._commanded_mode,
                },
            )
