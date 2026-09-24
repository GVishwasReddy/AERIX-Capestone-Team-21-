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

import csv
import math
import os
import threading
import time
from collections import deque

from drone_stack.bus import MessageBus
from drone_stack.bus.topics import Topics
from drone_stack.msg import (
    ArmedStatus,
    AvoidanceStatus,
    Battery,
    BlePhoneSignal,
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
    PersonLockState,
    PhoneHint,
    RcChannels,
    Waypoint,
)
from drone_stack.nodes.obstacle_tracker import enu_to_body
from drone_stack.nodes.phone_locator import (
    PhoneFix,
    PhoneLocator,
    body_to_enu,
    image_to_ground,
)
from drone_stack.srv import ServiceRegistry, ServiceRequest, ServiceResponse
from drone_stack.utils.config import Config
from drone_stack.utils.geometry import (
    enu_to_geodetic,
    geodetic_to_enu,
    haversine_m,
    wrap_180,
    wrap_pi,
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
        # Hysteresis on releasing the brake. evaluate() flips between STOP and
        # SLOW on scan jitter around the stop distance, and those two command
        # different flight modes, so releasing on the first non-STOP tick
        # chatters the mode at the 10 Hz loop rate - the exact thing the
        # 2026-08-22 lockout rule forbids. The obstacle has to open up by this
        # much beyond the stop distance before the hold is handed back.
        self.release_m = float(section.get("avoidance_release_m", 0.4))
        # Look before you go. A waypoint whose bearing falls in the masked rear
        # has never been scanned, and unmeasured sectors go to the FC as 65535
        # = unknown, which its proximity database reads as CLEAR. Flying at it
        # would let BendyRuler route confidently around obstacles the LiDAR
        # never had a chance to see. Turn the nose onto the path first.
        self.yaw_before_move = bool(section.get("avoidance_yaw_before_move", True))
        # Proceed once the target is this close to the nose. Well inside
        # fov_half_deg so the gate cannot chatter on its own threshold, and far
        # enough in that the path is covered by the dense middle of the window
        # rather than its extreme edge.
        self.yaw_release_deg = float(section.get("avoidance_yaw_release_deg", 60.0))
        self.yaw_rate_deg_s = float(section.get("avoidance_yaw_rate_deg_s", 25.0))
        self.yaw_timeout_s = float(section.get("avoidance_yaw_timeout_s", 20.0))
        # Face home before handing the aircraft over to the FC's RTL.
        #
        # yaw_before_move above does NOT cover the return leg: once RTL is
        # commanded the FLIGHT CONTROLLER flies it, and this node issues no
        # more gotos to gate. The only lever there is WP_YAW_BEHAVIOR, an FC
        # parameter this node cannot read - and at its old value of 2 ("face
        # next waypoint EXCEPT RTL") the aircraft translated home at whatever
        # heading the delivery ended on, scan window pointing wherever. Turning
        # before the handover makes the return leg's coverage depend on this
        # node instead of on a parameter nobody can see from here.
        self.yaw_before_rtl = bool(section.get("avoidance_yaw_before_rtl", True))
        # Deliberately much shorter than yaw_timeout_s (20 s): a 180 deg turn at
        # yaw_rate_deg_s takes ~7 s, so this is "the turn should have finished
        # by now", not "wait and see".
        #
        # It also bounds a real side effect. The phase is already RTL while the
        # turn runs, and _check_failsafe() deliberately does not fight an
        # in-progress RTL - so the GCS-side failsafes (battery, link, geofence)
        # are suppressed for exactly this long. That is survivable at 8 s and
        # would not be at 20 s. The FC's own battery failsafe is unaffected.
        self.rtl_yaw_timeout_s = float(section.get("avoidance_rtl_yaw_timeout_s", 8.0))
        # Stage 1 of the pre-RTL turn: a full 180 deg about-face the moment the
        # job is done, before yaw_before_rtl above is even consulted.
        #
        # Unlike yaw_before_rtl this is UNCONDITIONAL - it is not asking a
        # question about where home is. The aircraft finishes a delivery nose-on
        # to the customer, having flown in forwards, so "turn around" and "point
        # back down the inbound track" are the same instruction, and doing it
        # always means the behaviour is the same every flight instead of
        # depending on a bearing nobody watched. It also sweeps the LiDAR's
        # masked rear 110 deg through the airspace behind before anything moves
        # into it, which is the sector the return leg is about to fly into.
        self.rtl_about_face = bool(section.get("avoidance_rtl_about_face", True))
        # How short of a true 180 counts as turned. The turn is MEASURED off the
        # attitude estimate (see _about_face_step), and demanding an exact 180
        # from a yaw controller that settles with overshoot would just burn the
        # timeout every time. 15 deg is inside yaw_release_deg (60), so a
        # completed about-face never leaves stage 2 with work to do.
        self.about_face_tol_deg = float(
            section.get("avoidance_rtl_about_face_tol_deg", 15.0))
        # ONE ceiling over BOTH turn stages, not one each.
        #
        # The phase is already RTL while either stage runs and _check_failsafe()
        # deliberately does not fight an in-progress RTL, so this is exactly how
        # long the GCS-side failsafes (battery, link, geofence) stay suppressed.
        # Letting about-face and turn-onto-home time out independently would
        # stack to 8 + 8 s, and rtl_yaw_timeout_s's own comment says 8 s is
        # survivable and 20 s is not. So the stages share a budget instead.
        #
        # 12 s is the honest floor, not a preference: 180 deg at
        # yaw_rate_deg_s (25 deg/s) is 7.2 s of unavoidable turning, plus a
        # handover margin. Raising yaw_rate_deg_s is the way to shrink it.
        # The FC's own battery failsafe is unaffected throughout.
        self.rtl_yaw_total_s = float(
            section.get("avoidance_rtl_yaw_total_s", 12.0))
        # How long the Pi may hold an FC-flown return braked before handing the
        # return back whatever the LiDAR still says.
        #
        # Bounded for the same reason the turn budget above is, and it is the
        # same risk written twice: while this holds, the aircraft is not coming
        # home. It is also the 2026-08-22 lockout in miniature - the Pi seizing
        # a mode the FC is flying - so the brake has to be a moment, not a
        # state. Ten seconds is long enough for a person to walk out of the
        # path and short enough that a false return cannot strand the aircraft.
        self.rtl_brake_max_s = float(
            section.get("avoidance_rtl_brake_max_s", 10.0))
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

        # ── cruise-band steering ─────────────────────────────────────────
        # The dodge above is an EMERGENCY manoeuvre: it runs at the brake
        # distance, rations forward speed, and its job is to not hit the thing
        # that is already too close. It is the wrong tool for normal flight,
        # and until 2026-09-19 it was the only tool - between
        # avoidance_distance_m and avoidance_stop_m the node commanded nothing
        # at all, deferring to an FC path planner that OA_TYPE=0 meant was
        # never running. So the aircraft flew at cruise speed straight to the
        # brake distance and stopped dead.
        #
        # These drive the band above it: a continuous course correction that
        # starts at avoidance_distance_m, grows as the obstacle closes, and
        # keeps the aircraft at speed the whole way. Slower slew than the
        # dodge on purpose - this is a cruise-band nudge, not an escape.
        self.steer_rate_deg_s = float(
            section.get("avoidance_steer_rate_deg_s", 15.0))
        #: Authority at the OUTER edge of the band, so the lean begins the
        #: instant an obstacle enters it rather than one tick later.
        #:
        #: 1 - (1 - frac)**3 is still exactly 0.0 at frac = 0. The ease-out fix
        #: of 2026-09-19 corrected the SHAPE of the curve but left its left
        #: endpoint pinned at zero, so the aircraft still arrived at
        #: avoidance_distance_m commanding no deflection at all and spent the
        #: first second of the encounter playing catch-up against the rate
        #: limiter - measured as a slew pinned at its 15 deg/s cap for the whole
        #: manoeuvre, which is the signature of a turn that started late rather
        #: than one that started gently.
        #:
        #: A floor is cheap where the band is wide. At 10 m a 0.25 authority on
        #: a 20 deg gap is 5 deg of lean - about 9 cm of lateral travel per
        #: second, i.e. nothing if the cluster turns out to be a single-return
        #: artefact, and a full second of head start if it is real.
        self.steer_floor = float(
            section.get("avoidance_steer_authority_floor", 0.25))
        self.steer_floor = max(0.0, min(1.0, self.steer_floor))

        # ── nose-on-waypoint tracking ────────────────────────────────────
        #: Hold the nose on the active waypoint at all times, including
        #: throughout an avoidance manoeuvre. See _hold_nose_on_waypoint.
        self.nose_track = bool(section.get("avoidance_nose_track", True))
        #: How far the nose may drift off the waypoint before it is
        #: re-commanded. Not a precision target: CONDITION_YAW settles with
        #: overshoot, the attitude estimate has its own noise, and a deadband
        #: tighter than that spends the flight issuing turns that cancel. 8 deg
        #: is well inside the +/-50 deg front cone that gates SLOW/STOP, so the
        #: path stays in the dense middle of the scan at all times.
        self.nose_deadband_deg = float(
            section.get("avoidance_nose_deadband_deg", 8.0))
        #: Pacing on the re-command. CONDITION_YAW is a discrete command, so
        #: re-sending it restarts the turn - at 10 Hz it would never land. 2 s
        #: is the interval already proven on the pre-RTL turn, and at
        #: yaw_rate_deg_s (25) it is 50 deg of authority per command, far more
        #: than the deadband can accumulate.
        self.nose_recmd_s = float(
            section.get("avoidance_nose_recommand_s", 2.0))
        # Who routes during the FC-flown return. See _push_oa_type.
        self.rtl_oa_handoff = bool(
            section.get("avoidance_rtl_oa_handoff", True))
        self.rtl_oa_type = int(section.get("avoidance_rtl_oa_type", 1))
        # "pi" = fly the return with this node's own measured steerer.
        # "fc" = hand the return to the flight controller. See _pi_return_wanted.
        self.rtl_router = str(section.get("avoidance_rtl_router", "pi")).lower()
        self.rtl_pi_budget_s = float(
            section.get("avoidance_rtl_pi_budget_s", 240.0))
        #: Speed held while bending. 0.0 means "use navigation.cruise_speed_ms",
        #: which is the intent: the correction is applied to the COURSE, never
        #: to the speed. Slowing down is what produced the stop-and-wait.
        self.steer_speed_ms = float(section.get("avoidance_steer_speed_ms", 0.0))

        # ── VFH+ steering ────────────────────────────────────────────────
        # The three-cone dodge above answers "left or right?". That is the
        # wrong question when the way past something is a gap 20 deg off the
        # nose: front/left/right collapses the whole scan into three numbers,
        # so a doorway and a solid wall with a dent in it look identical.
        #
        # VFH+ (Ulrich & Borenstein) keeps the angular detail: bin the scan
        # into a polar histogram, widen each obstacle by the room the airframe
        # actually needs, then steer at the best surviving GAP. Values are the
        # ones flown in Drone-Autonomy-ROS2' mission_avoidance_node.
        self.vfh_enabled = bool(section.get("avoidance_vfh_enabled", True))
        self.vfh_sector_deg = float(section.get("avoidance_vfh_sector_deg", 5.0))
        #: Half-width the airframe needs, used for angular enlargement. An
        #: obstacle does not block one bearing, it blocks every bearing that
        #: would fly us within this of it - and that arc grows as it gets
        #: closer, which is the entire point of the enlargement step.
        self.vfh_safety_radius = float(
            section.get("avoidance_vfh_safety_radius_m", 1.0))
        #: A gap narrower than this is not flyable, whatever the histogram
        #: says. Without it the planner happily aims at a one-bin slot between
        #: two obstacles that the drone cannot fit through.
        self.vfh_min_valley_deg = float(
            section.get("avoidance_vfh_min_valley_deg", 18.0))
        self.vfh_goal_weight = float(section.get("avoidance_vfh_goal_weight", 1.0))
        #: Pull toward the heading already being flown. Raised well off zero on
        #: purpose: a memoryless cost function flip-flops between two similarly
        #: good gaps every scan, and that oscillation is what upsets the EKF.
        self.vfh_hysteresis_weight = float(
            section.get("avoidance_vfh_hysteresis_weight", 0.8))
        #: deg/s cap on how fast the commanded course may slew. Obstacles do
        #: not move faster than this; anything quicker is self-inflicted.
        self.vfh_max_heading_rate = float(
            section.get("avoidance_vfh_max_heading_rate_deg_s", 45.0))
        #: Only obstacles inside this steer. Something 30 m away is not a
        #: reason to bend the course now, and letting it vote makes the
        #: histogram twitch on distant clutter.
        self.vfh_range_m = float(section.get("avoidance_vfh_range_m", 10.0))

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

    def stop_distance_for(
        self, closing_ms: float, own_speed_ms: float = 0.0,
        is_dynamic: bool = False,
    ) -> float:
        """Brake distance in force against this obstacle.

        ``avoidance_stop_m`` is the CRITICAL distance - the range at which the
        aircraft gives up on steering, stops dead and looks for a new route. It
        is the one number the operator sets deliberately, and against a
        stationary obstacle it is now honoured exactly.

        It did not used to be. The pad was derived from ``closing_ms``, which is
        body-frame range rate and therefore *includes our own forward motion*
        ("a wall we are flying at closes exactly as dangerously as a car driving
        at us"). Flying at a fence post at cruise_speed_ms (1.0) made closing
        1.0, pad 1.0, and the hard brake fire at 2.70 m against a configured
        1.70 - the operator asked for one distance and got another, and the
        cruise-band steering lost the bottom metre of its runway to a brake it
        could not see coming.

        The pad exists for a real reason and is kept: a person walking into the
        path at 1.5 m/s covers 1.5 m in the second it takes to notice and brake,
        so a fixed stand-off is the full margin against a wall and almost none
        against them. What changed is WHICH speed feeds it. ``is_dynamic`` and
        ``speed_m_s`` come off the tracker with our own translation AND yaw rate
        already removed, so they answer the question the pad is actually asking -
        "is this thing coming at us under its own power?" - instead of
        conflating it with "are we moving?".

        Net effect: a static obstacle brakes at exactly ``avoidance_stop_m``; a
        mover still buys its reaction margin.
        """
        if not is_dynamic or closing_ms <= 0.0:
            # Either ours is the only motion in play, or the gap is not
            # shrinking at all. The stand-off is the answer, and the cruise band
            # above it is what keeps us from ever reaching it.
            return self.stop
        # BOTH terms are needed, and each supplies what the other loses.
        #
        # closing_ms carries DIRECTION but not ownership: it knows the gap is
        # shrinking, but cannot tell our 1.0 m/s cruise from a car. speed_m_s
        # carries OWNERSHIP but not direction: the tracker has removed our
        # translation and yaw rate, but it is a magnitude, so on its own it pads
        # the brake for something walking AWAY from us exactly as hard as for
        # something walking at us - measured, and a straight regression of the
        # old behaviour's one correct property.
        #
        # The minimum of the two is the object's own approach rate bounded by
        # the rate the gap is really closing: receding -> no pad, crossing ->
        # almost none, walking in at 1.5 m/s -> the full 1.5 m of reaction
        # margin it needs.
        own_closing = min(max(0.0, own_speed_ms), closing_ms)
        pad = own_closing * self.reaction_s
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
            stop_m = self.stop_distance_for(
                closing,
                own_speed_ms=getattr(o, "speed_m_s", 0.0) or 0.0,
                is_dynamic=bool(getattr(o, "is_dynamic", False)),
            )
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

    # ── VFH+ ──────────────────────────────────────────────────────────────
    def _bin_centres(self) -> list[float]:
        """Bearing at the centre of each histogram bin, nose-relative.

        The histogram spans ONLY the scanned window (+/- fov_half_deg). The
        masked rear is not represented at all, which is deliberate and is the
        single most important property of this implementation: a bearing that
        has no bin cannot be found free, so no gap search can ever steer the
        aircraft into the 110 deg it cannot see. "Nothing behind us" stays the
        absence of data rather than the absence of obstacles.
        """
        span = 2.0 * self.fov_half_deg
        count = max(1, int(math.ceil(span / self.vfh_sector_deg)))
        step = span / count
        return [-self.fov_half_deg + (i + 0.5) * step for i in range(count)]

    def build_histogram(self, obstacles: ObstacleArray | None) -> list[bool]:
        """Polar occupancy over the scanned window. True = blocked.

        Each obstacle blocks its own angular width PLUS an enlargement of
        asin(safety_radius / distance). That term is what makes this different
        from a nearest-distance cone: the same object blocks a wider arc the
        closer it gets, so the histogram tightens as the encounter develops
        and the chosen gap moves away from it on its own.

        An obstacle nearer than the safety radius blocks +/-90 deg around
        itself - asin saturates - which is the correct answer. At that range
        there is no heading anywhere near it that is safe to fly.
        """
        centres = self._bin_centres()
        blocked = [False] * len(centres)
        if obstacles is None or obstacles.count == 0:
            return blocked
        for o in obstacles.obstacles:
            distance = float(o.distance_m)
            if not math.isfinite(distance) or distance <= 0.0:
                continue
            if distance > self.vfh_range_m:
                continue
            bearing = wrap_180(o.bearing_deg)
            if abs(bearing) > self.fov_half_deg:
                continue    # in the masked rear; not ours to reason about
            ratio = min(1.0, self.vfh_safety_radius / distance)
            half = abs(float(o.angular_width_deg)) / 2.0 + math.degrees(
                math.asin(ratio))
            for i, centre in enumerate(centres):
                if not blocked[i] and abs(wrap_180(centre - bearing)) <= half:
                    blocked[i] = True
        return blocked

    def valleys(self, blocked: list[bool]) -> list[tuple[float, float]]:
        """Contiguous free runs, as (start_deg, end_deg) inclusive of edges.

        Runs are NOT wrapped around the circle: the window has two hard ends
        (the mask boundaries) and joining them would invent a gap straight
        through the aircraft's blind side.
        """
        centres = self._bin_centres()
        if not centres:
            return []
        half_step = (centres[1] - centres[0]) / 2.0 if len(centres) > 1 else (
            self.vfh_sector_deg / 2.0)
        out: list[tuple[float, float]] = []
        start: int | None = None
        for i, is_blocked in enumerate(blocked):
            if not is_blocked and start is None:
                start = i
            elif is_blocked and start is not None:
                out.append((centres[start] - half_step, centres[i - 1] + half_step))
                start = None
        if start is not None:
            out.append((centres[start] - half_step, centres[-1] + half_step))
        return out

    def choose_heading(
        self,
        obstacles: ObstacleArray | None,
        goal_bearing_deg: float = 0.0,
        previous_deg: float | None = None,
    ) -> float | None:
        """Best flyable bearing, or None when nothing is wide enough.

        Candidates are the point in each valley closest to the goal while
        still keeping half a minimum-valley-width from either edge, so the
        aircraft aims through the middle of a gap rather than shaving its lip.

        Cost is goal attraction plus hysteresis toward the heading already
        being flown. Returning None means TRAPPED, and the caller must brake -
        never reverse, because the rear is unmeasured (see SectorView).
        """
        room = self.vfh_min_valley_deg / 2.0
        best: float | None = None
        best_cost = math.inf
        for lo, hi in self.valleys(self.build_histogram(obstacles)):
            if (hi - lo) < self.vfh_min_valley_deg:
                continue
            candidate = min(max(goal_bearing_deg, lo + room), hi - room)
            cost = self.vfh_goal_weight * abs(wrap_180(candidate - goal_bearing_deg))
            if previous_deg is not None:
                cost += self.vfh_hysteresis_weight * abs(
                    wrap_180(candidate - previous_deg))
            if cost < best_cost:
                best, best_cost = candidate, cost
        return best

    def steer_authority(self, distance_m: float) -> float:
        """How hard to bend the course at this obstacle distance, 0.0 to 1.0.

        0.0 flies the goal bearing untouched; 1.0 flies the VFH+ gap bearing
        outright. ``_steer_step`` blends between the two by this factor, so
        this function alone decides the SHAPE of every avoidance manoeuvre the
        aircraft makes in normal flight - how early it starts leaning away,
        and how much of the turn is left to do late.

        The band runs from ``self.distance`` (avoidance_distance_m, the outer
        edge where the obstacle first counts as SLOW) inward to
        ``self.last_stop_m`` (the closing-speed-padded brake distance). Outside
        the band there is nothing to avoid; at or inside the brake distance the
        dodge and then the brake take over.

        Ease-OUT with a band-edge FLOOR:

            floor + (1 - floor) * (1 - (1 - frac)**3)

        It was frac**2 (ease-in) until 2026-09-19, then 1 - (1 - frac)**2 with
        no floor, and reached its present form on 2026-09-21. Both flips are
        recorded in full because both were the difference between clearing a
        pole and parking in front of one.

        The angle the geometry actually demands is asin(vfh_safety_radius /
        distance). That is a hyperbola: cheap far out, ruinous close in. On the
        band as flown (10.0 m -> 1.50 m, safety radius 2.5 m) it runs 14.5 deg
        at 10 m, 18.2 at 8 m, 24.6 at 6 m, 38.7 at 4 m, and 90 at 2.5 m, where
        the obstacle is exactly one safety radius away and nothing but a full
        turn clears it. Expressed as a fraction of the VFH+ gap bearing, the
        authority the aircraft NEEDS is ~0.60 at the band edge rising to ~0.87
        at 2.5 m - it starts high and is nearly flat. Any curve leaving the
        edge at zero therefore starts behind; all that matters is how fast it
        catches up.

        frac**2 never did. It commanded 0.0 deg where 7.2 was needed, 1.9
        where 9.6 was needed, 9.7 where 14.5 was needed - under-deflected on
        every tick, accumulating clearance debt the whole way in and repaying
        it only at the bottom. A closed-loop 10 Hz approach against a 0.3 m
        pole ended in the brake band at 1.69 m with vx collapsed to 0.00:
        exactly the stop-and-wait this band exists to prevent.

        WHY THE FLOOR, added 2026-09-21. Ease-out fixed the catch-up but is
        still exactly 0.0 at frac=0, and 0.0 at the band edge means the lean
        cannot BEGIN where the LiDAR first sees the obstacle - the aircraft
        coasts into the band and then has to turn harder lower down. Measured
        on the 10 m band before the floor: authority 0.000 and a commanded
        0.0 deg where +18.2 was geometrically needed, with slew_steer pinned
        at its cap for the whole manoeuvre. A rate pinned at the cap
        throughout IS the signature of a late turn, not a gentle one. The
        floor (avoidance_steer_authority_floor, 0.25 as flown) is what makes
        the operator's "avoid from long distance" mean anything.

        Measured against the requirement, band edge inward: the curve is
        behind by 8.5 deg at 10 m and 4.5 deg at 9 m, crosses ahead at 8 m,
        and leads with growing margin all the way in (23.1 vs 20.9 needed at
        7 m, 48.1 vs 38.7 at 4 m). Being behind in the top 2 m is harmless and
        deliberate: that is where the geometry is cheapest and where there is
        the most distance left to repay it in.

        What was given up: the old ease-in argued that near-zero authority at
        avoidance_distance_m protects against the C1's sparsest returns, where
        a cluster on a handful of samples can appear and vanish between
        revolutions. That concern is real and the floor spends more against it
        than ease-out alone did - 0.25 of a 24 deg gap is 6.0 deg of lean at
        10 m. Rate-limited by slew_steer that is about 6 cm of lateral travel
        if the cluster turns out to be noise, against arriving at a 1.5 m
        critical distance with the turn still to do. Still a fair trade, but
        it is a trade: if flight shows the C1 twitching at the band edge, the
        floor is the knob to lower, NOT the band.

        Smoothness is NOT this function's job. Smoothness is a property of
        turn RATE, and slew_steer's steer_rate_deg_s (10 deg/s as flown, down
        from 15 on 2026-09-21) is what guarantees it. Shaping heading here as
        well was a second brake on the same quantity, and it was the one that
        read zero exactly where the manoeuvre had to begin. Start early, turn
        slowly - do not wait and then turn hard.

        The fraction is clamped BEFORE the curve is applied. An obstacle
        outside the band produces a negative fraction, and (1 - frac)**3 on a
        negative would hand back MORE than full authority. Note that clamped
        no longer means zero: outside the band this returns the floor, which
        is why the clamp test asserts "never more than at the band edge"
        rather than "== 0.0".
        """
        span = self.distance - self.last_stop_m
        if span <= 0.0:
            # The brake distance has swallowed the whole band. last_stop_m
            # grows with closing speed, so a fast mover can do this. There is
            # no room left to ease into, and the honest answer is full
            # authority now - the dodge and the brake are what follow.
            return 1.0
        frac = min(1.0, max(0.0, (self.distance - distance_m) / span))
        eased = 1.0 - (1.0 - frac) ** 3
        return self.steer_floor + (1.0 - self.steer_floor) * eased

    def slew_steer(
        self, want_deg: float, previous_deg: float | None, dt_s: float
    ) -> float:
        """Rate-limit the cruise-band course at steer_rate_deg_s.

        Same rate limiter as slew_heading, against the gentler cruise-band
        rate. Kept separate so retuning the nudge cannot silently slow the
        emergency dodge, which needs every degree per second it has.
        """
        if previous_deg is None or dt_s <= 0.0:
            return want_deg
        delta = wrap_180(want_deg - previous_deg)
        limit = self.steer_rate_deg_s * dt_s
        if abs(delta) > limit:
            delta = limit if delta > 0 else -limit
        return wrap_180(previous_deg + delta)

    def slew_heading(
        self, want_deg: float, previous_deg: float | None, dt_s: float
    ) -> float:
        """Rate-limit the commanded course so it ramps instead of stepping."""
        if previous_deg is None or dt_s <= 0.0:
            return want_deg
        delta = wrap_180(want_deg - previous_deg)
        limit = self.vfh_max_heading_rate * dt_s
        if abs(delta) > limit:
            delta = limit if delta > 0 else -limit
        return wrap_180(previous_deg + delta)


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
        self._init_person_scan(delivery)
        # Camera tilt servo (AUX6 MG90S, config block `aux2_servo`): DOWN on
        # the approach to the drop point, UP when the return starts, DOWN
        # again for the landing, UP once on the ground. See
        # _camera_tilt_on_approach, _enter_rtl and _camera_tilt_for_landing.
        # Angles and envelope come
        # from the same block GcsHub ships to the DOWN/UP buttons, so the
        # automatic and manual commands can never disagree about where "down"
        # is.
        cam = config.section("aux2_servo") or {}
        self._cam_auto = bool(cam.get("auto_tilt", True))
        self._cam_down_m = float(cam.get("down_before_drop_m", 3.0))
        self._cam_ch = int(cam.get("out_channel", 14))
        self._cam_min_us = int(cam.get("min_us", 500))
        self._cam_max_us = int(cam.get("max_us", 2500))
        if self._cam_min_us >= self._cam_max_us:   # same guard as GcsHub
            self._cam_min_us, self._cam_max_us = 500, 2500
        self._cam_span = float(cam.get("deg_span", 180.0)) or 180.0
        self._cam_down_deg = float(cam.get("down_deg", 165.0))
        self._cam_up_deg = float(cam.get("up_deg", 90.0))
        #: What THIS node last commanded: None (never), "down" or "up". It is a
        #: latch, not a readback - an operator pressing the GCS buttons in
        #: between is deliberately not overridden.
        self._cam_tilt: str | None = None
        #: Set by _on_armed on armed -> disarmed, consumed by
        #: _camera_tilt_for_landing on the next tick (bus callbacks record,
        #: step() commands).
        self._cam_disarm_edge = False
        # Arrival -> LAND (see _rtl_arrived_home). RTL on its own only lands
        # because RTL_ALT_FINAL happens to be 0 on this FC; that is a parameter
        # nobody can see from here, and until 2026-09-19 nothing in this node
        # ever left MissionPhase.RTL except a disarm. The navigator now decides
        # when the aircraft is home and commands LAND itself, so the phase the
        # GCS shows and the mode the aircraft is in agree.
        self._land_radius_m = float(delivery.get("land_radius_m", 2.0))
        self._land_arm_delay_s = float(delivery.get("land_arm_delay_s", 5.0))
        self._land_rtl_timeout_s = float(delivery.get("land_rtl_timeout_s", 120.0))
        self._land_sent_at = 0.0
        self._land_reason = ""

        # Post-takeoff settle. Reaching the takeoff altitude is not the same as
        # being stable at it: the climb is still bleeding off vertical rate and
        # the EKF is still settling. Departing for the waypoint on that instant
        # commits an oscillating aircraft to a translation, which reads on the
        # ground as "it lurched off sideways". Monotonic, like _hover_until.
        self._takeoff_settle_s = float(section.get("takeoff_settle_s", 2.0))
        self._settle_until: float | None = None
        self._delivery_target: tuple[float, float, float] | None = None

        # Yaw gate: set while turning the LiDAR onto a path that starts out in
        # the masked rear. None = not gating.
        self._alt_hardlock_logged = False
        self._yaw_gate_since: float | None = None
        self._yaw_gate_sent_at = 0.0
        self._yaw_gate_stuck = False

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
        # Pre-RTL turn. None whenever no turn is in progress, which is the
        # common case: the turn only engages when home is behind the aircraft.
        self._rtl_turn_since: float | None = None
        self._rtl_turn_sent_at = 0.0
        self._rtl_turn_warned = False
        # Stage 1, the post-delivery about-face. _about_face_since is the "in
        # progress" flag, mirroring _rtl_turn_since above.
        self._about_face_since: float | None = None
        self._about_face_sent_at = 0.0
        self._about_face_warned = False
        # Absolute heading (rad) the about-face is aiming at, latched once when
        # the stage starts. THIS, not an accumulator, is what completion is
        # measured against - see _about_face_step for why the accumulator was
        # wrong. None whenever no about-face is in progress.
        self._about_face_target: float | None = None
        # Last yaw sample (rad) and how far the aircraft has turned since the
        # stage began (deg, SIGNED in the commanded direction). Progress only:
        # it drives the status line and the log, never the completion test.
        self._about_face_yaw: float | None = None
        self._about_face_turned = 0.0
        self._about_face_direction = 1  # +1 = clockwise, -1 = counter-clockwise
        # When the FIRST stage started. Both stages expire against this.
        self._rtl_yaw_started: float | None = None
        # Pi-side emergency brake during the FC-flown return. None whenever the
        # path is clear, which is the normal case - see _rtl_avoid_step.
        self._rtl_braking_since: float | None = None
        self._rtl_brake_warned = False
        # One-shot mirror of the configured emergency-brake distance onto the
        # FC's own margins, done once the link is up. Without it the aircraft
        # would boot with whatever AVOID_MARGIN/OA_MARGIN_MAX were left on the
        # FC from a previous session while the GCS displayed the configured
        # number - three layers, two of them disagreeing, and nothing on screen
        # to say so.
        self._brake_params_synced = False
        #: Last OA_TYPE value pushed, so the handoff is not re-sent at 10 Hz.
        #: None means "never pushed", which is distinct from "pushed 0".
        self._oa_type_pushed: int | None = None
        #: When the Pi-flown return leg started, or None. Also the one-shot
        #: latch: a return leg is offered once per flight and never re-entered.
        self._pi_return_since: float | None = None
        #: Set once the Pi-flown return has been offered, so arriving home does
        #: not fire a second about-face or a second return leg.
        self._pi_return_flown = False
        #: True when the return came from a finished job rather than a failsafe
        #: or an operator press. Only the former may be flown by the Pi.
        self._rtl_job_finished = False
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
        # Takeoff gate (2026-09-23). Avoidance stays in STANDBY - evaluated for
        # the panel, never acted on - until the aircraft is genuinely airborne,
        # then latches ACTIVE for the rest of the flight. Lifting off, the
        # LiDAR plane sits at ground-clutter height: the legs, the operator,
        # the car the aircraft launched beside. Braking or steering off those
        # mid-climb fights the takeoff. Latched rather than re-tested each tick
        # because the logs show in-flight dips to ~1.5 m, and avoidance must
        # not blink off in the moment the aircraft is lowest. Reset on disarm.
        # 0 disables the gate (sim and the unit tests keep the old behaviour).
        self._avoid_min_alt = float(section.get("avoidance_min_alt_m", 0.0))
        #: Deceleration used when the cruise band finds no flyable gap, m/s^2.
        #: 0.4 takes 1.0 m/s cruise to rest over 1.25 m - gentle enough that
        #: the pitch-back is not felt, short enough to start well inside the
        #: band.
        self._trapped_decel = max(
            0.05, float(section.get("avoidance_trapped_decel_ms2", 0.4)))
        self._avoid_airborne = False
        # Latched dodge: direction is chosen once per encounter and held until
        # the front clears. See _dodge_step for why it is not re-decided every
        # tick.
        self._dodge_dir: str | None = None
        self._dodge_started: float | None = None
        #: ENU position where the current dodge began, and the point it was
        #: heading for. Together they define the track the aircraft is allowed
        #: to depart from by at most avoidance_max_offtrack_m.
        self._dodge_origin: tuple[float, float] | None = None
        #: Course VFH+ is steering, nose-relative degrees (+ right). Held
        #: across ticks so choose_heading can be pulled toward it (hysteresis)
        #: and slew_heading can rate-limit the change.
        self._dodge_heading: float | None = None
        self._dodge_heading_t: float | None = None
        self._offtrack_m = 0.0
        # Cruise-band steering state. Separate from the dodge's: the two
        # run in different bands with different rate limits, and sharing
        # one heading would seed an emergency dodge with a course solved
        # under a fraction of full authority.
        self._steer_heading: float | None = None
        self._steer_heading_t: float | None = None
        self._steer_origin: tuple[float, float] | None = None
        #: Nose-on-waypoint tracking. CONDITION_YAW is a discrete command, not
        #: a setpoint, so it is paced rather than streamed - see
        #: _hold_nose_on_waypoint.
        self._nose_yaw_sent_at = 0.0
        #: Last attitude yaw seen, radians. _steer_step counter-rotates its
        #: body-frame course by the delta so the rate limiter and the yaw
        #: controller do not fight each other.
        self._prev_yaw_rad: float | None = None
        # Hysteresis for the SLOW→CLEAR transition. Without this, LiDAR noise
        # around avoidance_distance_m causes _do_navigate to flip between
        # velocity steering and goto at 10 Hz, producing the vigorous wobble
        # seen on 2026-09-19. The steering is only dropped after the path has
        # been CLEAR for _STEER_CLEAR_HOLDOFF_S consecutive seconds.
        self._steer_clear_since: float | None = None
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
        # Drop-point person scan. Phone samples are events (a replayed RSSI
        # sample from a past order is noise); the lock is state.
        self.subscribe(Topics.BLE_PHONE, self._on_ble_phone, deliver_latched=False)
        self.subscribe(Topics.PERSON_LOCK, self._on_person_lock)

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
            holding = (self._phase == MissionPhase.HOVER
                       and self._hover_until is not None)
            overridden = self._pilot_override
            airborne = self._armed and self._has_flown

        if holding and self._scan_enabled and self._scan_stage != "locked":
            # drone_ble_peripheral.py refuses the DROP until handshake_open,
            # so this means a peripheral that did not gate (an old build, or
            # the flag unreadable). The parcel has gone either way: leave.
            self.log.warning(
                "BLE drop reported for order %s during the person scan's %s "
                "stage, before any lock - the peripheral did not gate the "
                "release. Returning home.", msg.order_id, self._scan_stage or "?",
            )
            return
        if holding:
            # _do_hover owns the countdown and the RTL that follows it.
            self.log.info(
                "BLE handshake confirmed delivery for order %s - RTL in %.0fs",
                msg.order_id, self._ble_early_rtl_wait_s,
            )
            return

        # Not in the hold. _ble_delivered_at is only ever read by _do_hover, so
        # on 2026-08-31 this logged "RTL in 3s" after the mission had already
        # aborted and then nothing happened at all - the aircraft sat where it
        # was until the battery would have decided for it. Say what is actually
        # going to happen, and where this node still has the authority to act,
        # act.
        if overridden:
            self.log.warning(
                "BLE handshake confirmed delivery for order %s, but the "
                "navigator has stood down (pilot override) - NO RTL will be "
                "commanded. Fly home manually, or press RESUME to hand control "
                "back and then RTL.", msg.order_id,
            )
            self._status_message = (
                "handshake OK - pilot has control, RTL NOT commanded"
            )
            return

        if airborne:
            self.log.warning(
                "BLE handshake confirmed delivery for order %s outside the "
                "delivery hold (phase %s) - returning home now.",
                msg.order_id, self._phase.value,
            )
            self._status_message = "handshake confirmed - returning to base"
            self._enter_rtl(turn_first=True)
            return

        self.log.info(
            "BLE handshake confirmed delivery for order %s while on the ground "
            "(phase %s) - nothing to do.", msg.order_id, self._phase.value,
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
                    self._cam_disarm_edge = True    # touchdown: camera UP

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
            self._record_pose()
            self._update_avoid_airborne()
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
            self._camera_tilt_for_landing()
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
        self._scan_stage = ""
        self._phone_fix = None
        self._locator.reset()
        # New flight, new approach: the previous one's DOWN must not suppress
        # this one's.
        self._cam_tilt = None
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
        self._yaw_gate_clear()
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
        # Gated on _armed for the same reason the battery check above is: a
        # disarmed aircraft on the bench reports whatever its barometer has
        # drifted to since the datum was set, and relative_alt only re-zeros
        # when the FC arms. Sitting on the ground it read 5.08 m against a
        # 2.0 m ceiling and tripped this 1637 times in three minutes, filling
        # the journal and burying every other error in it. An altitude ceiling
        # means nothing on an aircraft that cannot fly.
        if (
            self._armed
            and self._fused is not None
            and self._home is not None
            and self._fused.alt_rel_m > (self._alt_ceiling + self._alt_margin)
        ):
            return "max_altitude"
        if self._alt_hardlock_logged:
            self._alt_hardlock_logged = False
            self.log.info("altitude back inside the ceiling - hardlock cleared")

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
            self._send("brake")     # _send paces mode-only commands itself
            # Once per breach, not once per tick. The brake keeps being applied
            # at loop rate - only the log is rate limited - because an ERROR
            # repeated 10 times a second is how a real fault goes unnoticed.
            if not self._alt_hardlock_logged:
                self._alt_hardlock_logged = True
                self.log.error(
                    "ALTITUDE HARDLOCK: %.2f m above home exceeds ceiling %.1f m "
                    "+ margin %.1f m - braking. Take manual control.",
                    alt, self._alt_ceiling, self._alt_margin,
                )
            return
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
            # next one is decided against a fresh scan. The VFH+ course is part
            # of that state: carried over, it seeds the next encounter's
            # hysteresis with a bearing solved against obstacles that have
            # since gone.
            self._dodge_dir = None
            self._dodge_started = None
            self._dodge_origin = None
            self._dodge_heading = None
            self._dodge_heading_t = None
            self._steer_heading = None
            self._steer_heading_t = None
            self._steer_origin = None
            self._status_message = f"obstacle {distance:.1f} m -> braking"
            self._send("brake")
            return

        wp = self._active_waypoint()
        if wp is None:
            self._status_message = "mission waypoints done -> RTL"
            self._enter_rtl(turn_first=True)
            return

        dist = self._distance_to_wp(wp)
        # Before the arrival check, so a drop point reached in one tick still
        # gets the camera down before the hold starts.
        self._camera_tilt_on_approach(wp, dist)
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

        # Everything below commands motion. After an avoidance brake the FC is
        # in BRAKE, where a goto is silently ignored - see _ensure_guided.
        if not self._ensure_guided():
            self._status_message = (
                f"leaving BRAKE for waypoint {wp.seq} ({dist:.1f} m)"
            )
            return

        # Do not translate into a bearing the LiDAR has never scanned.
        if self._pi_return_overdue():
            return

        if not self._yaw_gate_ok():
            return

        # Nose onto the waypoint, every tick, whatever the obstacle field is
        # doing. Deliberately AFTER the yaw gate: that gate owns the large
        # recovery turn when the waypoint is behind the masked rear, and two
        # things commanding yaw at once is the same mistake as two things
        # commanding a route. Once it releases, this holds what it achieved.
        self._hold_nose_on_waypoint()

        if decision == SLOW:
            # Reset the clear hold-off whenever we see SLOW again, so the
            # 0.5 s timer restarts from scratch.
            self._steer_clear_since = None
            # Bend the course; do not wait. This branch used to command nothing
            # and defer to the FC's BendyRuler - but OA_TYPE was MEASURED as 0
            # on 2026-09-19, and OA_BR_TYPE/OA_BR_LOOKAHEAD do not even exist
            # on the FC, which is how ArduPilot reports a path-planning backend
            # that was never instantiated at boot. Nothing was routing. The
            # aircraft flew at cruise speed from avoidance_distance_m to
            # avoidance_stop_m and slammed on the brakes, which is exactly the
            # "stops in front of the obstacle and sits there" that was
            # reported.
            #
            # Steering here is safe in a way the 2026-08-31 deadlock was not:
            # _ensure_guided() above has already secured GUIDED, so these
            # setpoints land in a mode that honours them. And with OA_TYPE=0
            # there is no FC route for a velocity setpoint to tear up - the
            # "two routers is worse than one" objection in real.yaml is
            # satisfied by this aircraft's measured configuration, not
            # overridden.
            self._steer_step(distance, wp, dist)
            return

        # CLEAR. If we were bending, stop bending and rejoin the ORIGINAL
        # track. Dropping _last_goto_wp forces the goto below to be re-issued,
        # which pulls the aircraft back onto the straight line to the waypoint
        # rather than letting it carry on down the deviated course.
        #
        # Hysteresis: LiDAR noise around avoidance_distance_m flickers the
        # decision between SLOW and CLEAR at 10 Hz. Without a hold-off, each
        # CLEAR tick reissues a goto (the FC pitches forward) and each SLOW
        # tick issues a velocity correction (the FC pitches back). That is
        # the vigorous wobble. Require 0.5 s of consecutive CLEAR before
        # committing to the release.
        if self._steer_heading is not None:
            now_m = time.monotonic()
            if self._steer_clear_since is None:
                self._steer_clear_since = now_m
            if (now_m - self._steer_clear_since) < 0.5:
                # Still in the hold-off window: keep the current steering
                # active but decay authority toward the goal bearing so the
                # transition is smooth rather than a hard snap.
                self._steer_step(self._avoider.distance, wp, dist)
                return
            self.log.info(
                "obstacle cleared - rejoining track to waypoint %d", wp.seq)
            self._steer_heading = None
            self._steer_heading_t = None
            self._steer_origin = None
            self._steer_clear_since = None
            self._offtrack_m = 0.0
            self._last_goto_wp = -1
        self._status_message = f"to waypoint {wp.seq}: {dist:.1f} m"

        if self._last_goto_wp != self._current_wp:
            self._issue_goto(wp)
            self._last_goto_wp = self._current_wp

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
        if self._scan_enabled:
            self._enter_scan(wp)
            return
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
        if self._scan_enabled and self._scan_stage:
            self._do_scan()
            return
        avoiding = self._hover_avoid_step()
        sagging = avoiding or self._guard_hover_altitude()
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
            self._enter_rtl(turn_first=True)
            return
        self._phase = MissionPhase.NAVIGATE
        if self._hover_mode != "GUIDED":
            # Back into GUIDED so our goto commands are honoured again.
            self._set_mode("GUIDED")
        self._status_message = "hover complete - resuming mission"

    # -- drop-point person scan (2026-09-24) ---------------------------------
    # Requirement, in the operator's words: at the drop point go down to 3 m,
    # scan with the camera facing down, and only once a human is found and
    # locked accept the phone's handshake. No human in 25 s -> RTL. Locked but
    # no handshake in 25 s -> RTL. Circle slowly while searching in case the
    # recipient is just outside the footprint, and keep avoiding obstacles the
    # whole time. With two or more people below, use the phone's BLE signal to
    # pick the right one.
    #
    # Stages (``_scan_stage``), all inside MissionPhase.HOVER:
    #   descend  goto the drop point at scan_altitude_m
    #   search   orbit it; accept a fresh camera lock (see _scan_lock_accepted)
    #   locked   hold where we are; the handshake window is open
    #
    # The handshake gate itself is enforced by drone_ble_peripheral.py, which
    # reads ``handshake_open`` from MissionState via /api/state and answers
    # NOT_READY to a DROP write until it is True - identity (auth) and the
    # phone's GPS are accepted before the lock, only the release is gated.

    def _init_person_scan(self, delivery) -> None:
        self._scan_enabled = bool(delivery.get("person_scan_enabled", False))
        self._scan_alt = self._clamp_alt(float(delivery.get("scan_altitude_m", 3.0)))
        self._scan_search_s = float(delivery.get("person_search_s", 25.0))
        self._scan_window_s = float(delivery.get("handshake_window_s", 25.0))
        self._scan_orbit_r = max(0.0, float(delivery.get("scan_orbit_radius_m", 2.0)))
        self._scan_orbit_n = max(3, int(delivery.get("scan_orbit_points", 8)))
        self._scan_step_s = max(0.5, float(delivery.get("scan_orbit_step_s", 3.0)))
        self._scan_descend_s = float(delivery.get("scan_descend_timeout_s", 8.0))
        self._scan_hold_m = float(delivery.get("scan_obstacle_hold_m", 3.0))
        self._scan_max_s = float(delivery.get("scan_max_total_s", 90.0))
        self._scan_lock_fresh_s = float(delivery.get("scan_lock_fresh_s", 1.0))
        self._scan_leg_yaw_s = float(delivery.get("scan_leg_yaw_timeout_s", 6.0))
        # Someone in view mid-orbit: stop there and give the camera time to
        # lock, instead of flying on and carrying them out of the frame.
        self._scan_pause_on = bool(delivery.get("scan_pause_on_person", True))
        #: A plain detection must score this to stop for (acquire/lock/hold
        #: from the camera always does). Above the HEF's 0.20 floor so one
        #: flickering false positive cannot freeze the whole search.
        self._scan_pause_conf = float(delivery.get("scan_pause_conf", 0.30))
        #: Stopped this long with no lock: move on - another angle may help.
        self._scan_pause_max_s = float(delivery.get("scan_pause_max_s", 6.0))
        #: Person out of view this long before the orbit resumes.
        self._scan_resume_s = float(delivery.get("scan_resume_s", 1.5))
        #: Stopped on someone when the search time runs out: this much extra
        #: for the lock to land, instead of leaving mid-acquire.
        self._scan_found_grace_s = max(0.0, float(delivery.get("scan_found_grace_s", 3.0)))
        self._scan_paused_at: float | None = None
        self._scan_seen_at = 0.0
        self._scan_pause_spent = False
        #: Height of a detection's box centre above the ground (torso), used to
        #: project it; the phone itself is modelled at locator_phone_height_m.
        self._scan_person_h = float(delivery.get("scan_person_height_m", 1.0))
        pitch = delivery.get("scan_cam_pitch_from_nadir_deg")
        self._scan_cam_pitch = None if pitch is None else float(pitch)

        self._loc_enabled = bool(delivery.get("locator_enabled", True))
        self._loc_pick_prob = float(delivery.get("locator_pick_prob", 0.8))
        self._loc_steer_std = float(delivery.get("locator_steer_std_m", 3.0))
        self._loc_recentre_m = float(delivery.get("locator_recentre_m", 1.5))
        self._loc_step_m = float(delivery.get("locator_step_m", 2.0))
        self._loc_max_offset = float(delivery.get("locator_max_offset_m", 8.0))
        self._loc_recentre_s = float(delivery.get("locator_recentre_every_s", 6.0))
        self._locator = PhoneLocator(
            grid_radius_m=float(delivery.get("locator_grid_radius_m", 12.0)),
            grid_step_m=float(delivery.get("locator_grid_step_m", 0.5)),
            path_loss_n=float(delivery.get("locator_path_loss_n", 2.0)),
            rssi_sigma_db=float(delivery.get("locator_rssi_sigma_db", 6.0)),
            shadow_sigma_db=float(delivery.get("locator_shadow_sigma_db", 4.5)),
            gps_sigma_m=float(delivery.get("locator_gps_sigma_m", 5.0)),
            prior_sigma_m=float(delivery.get("locator_prior_sigma_m", 8.0)),
            phone_height_m=float(delivery.get("locator_phone_height_m", 1.2)),
            bin_m=float(delivery.get("locator_bin_m", 0.75)),
            ref_dbm=float(delivery.get("locator_ref_dbm", -58.0)),
            ref_sigma_db=float(delivery.get("locator_ref_sigma_db", 10.0)),
            use_rssi=bool(delivery.get("locator_use_rssi", True)),
        )
        log_dir = str(delivery.get("locator_log_dir", "") or "")
        self._loc_log_dir = log_dir or None

        self._scan_stage = ""
        self._scan_seq = 0
        self._scan_started = 0.0
        self._scan_stage_since = 0.0
        self._scan_search_since = 0.0
        self._scan_drop: tuple[float, float] = (0.0, 0.0)
        self._scan_centre: tuple[float, float] = (0.0, 0.0)
        self._scan_target: tuple[float, float] | None = None
        self._scan_pending: tuple[float, float] | None = None
        self._scan_pending_since = 0.0
        self._scan_orbit_i = 0
        self._scan_orbit_a0 = 0.0
        self._scan_next_step = 0.0
        self._scan_recentred_at = 0.0
        self._scan_note = ""
        self._hover_avoid = ""
        self._person: PersonLockState | None = None
        self._person_rx = 0.0
        self._phone_fix: PhoneFix | None = None
        self._loc_last = 0.0
        self._hint_seq = 0
        self._hint_sent: tuple | None = None
        self._hint_at = 0.0
        self._relock_at = 0.0
        #: (wall time, east, north, alt) at the loop rate, so an RSSI sample is
        #: paired with where the aircraft was when the controller measured it.
        self._pose_hist: deque = deque(maxlen=600)

    # -- inputs --------------------------------------------------------------
    def _on_ble_phone(self, msg) -> None:
        if not isinstance(msg, BlePhoneSignal):
            return
        with self._lock:
            if self._home is None:
                return
            if msg.kind == "gps":
                if msg.lat == 0.0 and msg.lon == 0.0:
                    return
                e, n = geodetic_to_enu(msg.lat, msg.lon, self._home[0], self._home[1])
                self._locator.add_gps(e, n, msg.t or time.time())
                return
            pose = self._pose_at(msg.t)
            if pose is None:
                return
            self._locator.add_rssi(msg.rssi_dbm, pose[0], pose[1], pose[2], msg.t)
            self._log_rssi(msg, pose)

    def _on_person_lock(self, msg) -> None:
        if isinstance(msg, PersonLockState):
            with self._lock:
                self._person = msg
                self._person_rx = time.monotonic()

    def _record_pose(self) -> None:
        f = self._fused
        if f is not None and f.valid:
            self._pose_hist.append((time.time(), f.x, f.y, f.alt_rel_m))

    def _pose_at(self, t: float) -> tuple[float, float, float] | None:
        """Aircraft (east, north, alt) nearest wall time ``t``, within 0.5 s."""
        best, best_dt = None, 0.5
        for pt, e, n, a in reversed(self._pose_hist):
            dt = abs(pt - t)
            if dt < best_dt:
                best, best_dt = (e, n, a), dt
            elif pt < t - 0.5:
                break
        return best

    def _log_rssi(self, msg: BlePhoneSignal, pose) -> None:
        """Raw samples + pose to CSV, for calibrating ref_dbm / path_loss_n
        against real flights instead of trusting the defaults."""
        if not self._loc_log_dir:
            return
        try:
            os.makedirs(self._loc_log_dir, exist_ok=True)
            path = os.path.join(self._loc_log_dir,
                                time.strftime("rssi-%Y%m%d.csv", time.localtime(msg.t)))
            new = not os.path.exists(path)
            with open(path, "a", newline="") as fh:
                w = csv.writer(fh)
                if new:
                    w.writerow(["t", "order_id", "rssi_dbm", "east_m", "north_m",
                                "alt_m", "yaw_rad", "phase", "scan_stage"])
                yaw = self._fused.yaw if self._fused is not None else 0.0
                w.writerow([f"{msg.t:.3f}", msg.order_id, f"{msg.rssi_dbm:.0f}",
                            f"{pose[0]:.2f}", f"{pose[1]:.2f}", f"{pose[2]:.2f}",
                            f"{yaw:.3f}", self._phase.value, self._scan_stage])
        except OSError as exc:                       # never let logging fly
            self.log.warning("rssi log disabled: %s", exc)
            self._loc_log_dir = None

    # -- the gate ------------------------------------------------------------
    def _handshake_open(self) -> bool:
        """May the BLE peripheral release the parcel right now?

        Scan disabled: always (the pre-scan behaviour). Disarmed: always -
        nothing is flying, and the bench tests the handshake on the ground.
        Otherwise only inside the locked stage's window.
        """
        if not self._scan_enabled or not self._armed:
            return True
        return (
            self._phase == MissionPhase.HOVER
            and self._scan_stage == "locked"
            and self._hover_until is not None
            and time.monotonic() < self._hover_until
        )

    # -- stages --------------------------------------------------------------
    def _enter_scan(self, wp: Waypoint) -> None:
        now = time.monotonic()
        self._hover_total_s = self._scan_search_s
        self._hover_prev_mode = self._mode if self._mode != "UNKNOWN" else "GUIDED"
        self._hover_recovering = False
        self._ble_delivered_at = None
        self._hover_avoid = ""
        self._phase = MissionPhase.HOVER
        self._scan_seq = wp.seq
        drop = (wp.x_m, wp.y_m)
        if drop == (0.0, 0.0) and (wp.lat or wp.lon) and self._home is not None:
            drop = geodetic_to_enu(wp.lat, wp.lon, self._home[0], self._home[1])
        self._scan_drop = drop
        self._scan_centre = self._scan_drop
        self._scan_started = now
        self._scan_stage = "descend"
        self._scan_stage_since = now
        self._scan_note = ""
        self._scan_pending = None
        self._person = None                  # nothing seen before this counts
        self._hover_until = now + self._scan_descend_s + self._scan_search_s
        self._publish_hint(None)
        self._camera_tilt_down("person scan")
        self._scan_goto(*self._scan_drop)
        self._status_message = f"SCAN: descending to {self._scan_alt:.1f} m"
        self.log.info(
            "person scan: descending to %.1f m over the drop point, then %.0fs "
            "search (orbit r=%.1f m) and a %.0fs handshake window after a lock",
            self._scan_alt, self._scan_search_s, self._scan_orbit_r, self._scan_window_s,
        )

    def _scan_goto(self, east: float, north: float) -> None:
        """Hold/goto an ENU point at scan altitude. ``_hover_wp`` follows it,
        so the sag guard re-asserts THIS target, not the original pin."""
        if self._home is None:
            return
        lat, lon = enu_to_geodetic(east, north, self._home[0], self._home[1])
        self._hover_wp = Waypoint(seq=self._scan_seq, lat=lat, lon=lon,
                                  alt_m=self._scan_alt, x_m=east, y_m=north,
                                  kind="scan")
        self._scan_target = (east, north)
        self._send("goto", lat=lat, lon=lon, alt=self._scan_alt)

    def _hold_here(self) -> None:
        f = self._fused
        if f is None or not f.valid:
            return
        if self._scan_stage:
            self._scan_goto(f.x, f.y)
            return
        # Legacy hold: re-anchor on the spot, keep the hold altitude.
        alt = (self._hover_wp.alt_m if self._hover_wp else 0.0) or self._hover_alt
        if self._home is None:
            return
        lat, lon = enu_to_geodetic(f.x, f.y, self._home[0], self._home[1])
        seq = self._hover_wp.seq if self._hover_wp else 0
        self._hover_wp = Waypoint(seq=seq, lat=lat, lon=lon, alt_m=alt, x_m=f.x, y_m=f.y)
        self._send("goto", lat=lat, lon=lon, alt=alt)

    def _do_scan(self) -> None:
        now = time.monotonic()
        if now - self._scan_started > self._scan_max_s:
            self._scan_finish(f"scan exceeded its {self._scan_max_s:.0f}s cap")
            return
        avoiding = self._hover_avoid_step()
        sagging = False if avoiding else self._guard_hover_altitude()

        # A confirmed drop means the parcel has left, whatever stage we are
        # in - the peripheral is what gates the release, so staying would
        # only hover over the customer for nothing.
        if self._ble_early_rtl_enabled and self._ble_delivered_at is not None:
            left = self._ble_early_rtl_wait_s - (now - self._ble_delivered_at)
            if left <= 0:
                self._scan_finish("BLE handshake confirmed delivery")
                return
            if not sagging and not avoiding:
                self._status_message = f"HOVER: BLE handshake confirmed - RTL in {left:.0f}s"
            return

        stage = self._scan_stage
        if stage == "descend":
            alt = self._fused.alt_rel_m if self._fused is not None else math.inf
            if alt <= self._scan_alt + 0.3 or now - self._scan_stage_since >= self._scan_descend_s:
                self._scan_begin_search(now)
            elif not sagging and not avoiding:
                self._status_message = f"SCAN: descending to {self._scan_alt:.1f} m ({alt:.1f} m)"
            return

        if stage == "search":
            self._scan_update_locator(now)
            self._scan_update_hint(now)
            if self._scan_lock_accepted(now):
                self._scan_begin_locked(now)
                return
            paused = self._scan_pause_step(now)
            if now >= self._hover_until and not (
                    paused and now < self._hover_until + self._scan_found_grace_s):
                self._scan_finish(f"no person found in {self._scan_search_s:.0f}s")
                return
            if not avoiding and not sagging:
                people = self._person.people if self._scan_person_fresh(now) else 0
                if paused:
                    self._status_message = (
                        f"SCAN: person in view - holding for a lock "
                        f"({max(0.0, self._hover_until - now):.0f}s){self._phone_note()}"
                    )
                else:
                    self._scan_orbit_step(now)
                    self._status_message = (
                        f"SCAN: searching {self._hover_until - now:.0f}s "
                        f"({people} in view){self._phone_note()}"
                    )
            return

        if stage == "locked":
            left = self._hover_until - now
            if left <= 0:
                self._scan_finish(
                    f"no handshake within {self._scan_window_s:.0f}s of the lock")
                return
            if not avoiding and not sagging:
                self._status_message = f"LOCKED on recipient - handshake open {left:.0f}s"

    def _scan_begin_search(self, now: float) -> None:
        self._scan_stage = "search"
        self._scan_stage_since = self._scan_search_since = now
        self._hover_until = now + self._scan_search_s
        self._scan_paused_at = None
        self._scan_pause_spent = False
        self._scan_orbit_i = 0
        # Start the circle at the point dead ahead, so the first leg needs no
        # turn and the next ones are small ones.
        yaw = self._fused.yaw if self._fused is not None else 0.0
        self._scan_orbit_a0 = yaw
        self._scan_next_step = now + self._scan_step_s
        self._scan_goto(*self._scan_centre)
        self.log.info("person scan: at %.1f m - searching for %.0fs",
                      self._fused.alt_rel_m if self._fused else float("nan"),
                      self._scan_search_s)

    def _scan_begin_locked(self, now: float) -> None:
        self._scan_stage = "locked"
        self._scan_stage_since = now
        self._hover_until = now + self._scan_window_s
        # Stay put rather than centring over the person: a hovering aircraft
        # directly over someone's head is where a released parcel lands.
        # Already stopped on them (the pause) -> keep that spot, don't creep.
        if self._scan_paused_at is None:
            self._scan_stop_here()
        self._scan_paused_at = None
        p = self._person
        self.log.info(
            "person LOCKED (id %s, score %.2f, %d in view%s) - handshake window "
            "open for %.0fs", p.lock_id if p else "?", p.score if p else 0.0,
            p.people if p else 0, f"; {self._scan_note}" if self._scan_note else "",
            self._scan_window_s,
        )
        self._status_message = f"LOCKED on recipient - handshake open {self._scan_window_s:.0f}s"

    def _scan_finish(self, why: str) -> None:
        self.log.info("person scan finished: %s", why)
        self._scan_stage = ""
        self._hover_until = None
        self._hover_wp = None
        self._hover_recovering = False
        self._hover_avoid = ""
        self._ble_delivered_at = None
        self._scan_pending = None
        self._scan_paused_at = None
        self._publish_hint(None)
        self._current_wp += 1
        self._last_goto_wp = -1
        if self._active_waypoint() is None:
            self._status_message = f"{why} - returning to base"
            self._enter_rtl(turn_first=True)
            return
        self._phase = MissionPhase.NAVIGATE
        self._status_message = f"{why} - resuming mission"

    # -- search: the orbit ---------------------------------------------------
    def _orbit_point(self, k: int) -> tuple[float, float]:
        a = self._scan_orbit_a0 + 2.0 * math.pi * k / self._scan_orbit_n
        cx, cy = self._scan_centre
        return cx + self._scan_orbit_r * math.sin(a), cy + self._scan_orbit_r * math.cos(a)

    def _scan_orbit_step(self, now: float) -> None:
        # A target that has come within the hold distance of an obstacle is
        # abandoned on the spot - the orbit never flies at something it sees.
        if self._scan_target is not None and self._near_obstacle(self._scan_target) \
                and self._scan_pending is None:
            f = self._fused
            if f is not None and math.hypot(self._scan_target[0] - f.x,
                                             self._scan_target[1] - f.y) > 0.3:
                self._hold_here()
                self._scan_next_step = now + self._scan_step_s
                self.log.info("scan: orbit target within %.1f m of an obstacle - holding",
                              self._scan_hold_m)
        if self._scan_pending is not None:
            if self._scan_leg_ready(self._scan_pending):
                self._scan_goto(*self._scan_pending)
                self._scan_pending = None
                self._scan_next_step = now + self._scan_step_s
            elif now - self._scan_pending_since > self._scan_leg_yaw_s:
                self.log.info("scan: nose never came round to the next leg - skipping it")
                self._scan_pending = None
                self._scan_next_step = now
            return
        if self._scan_orbit_r <= 0.0 or now < self._scan_next_step:
            return
        for _ in range(self._scan_orbit_n):
            self._scan_orbit_i += 1
            pt = self._orbit_point(self._scan_orbit_i)
            if not self._near_obstacle(pt):
                break
        else:
            self._scan_next_step = now + self._scan_step_s
            self._status_message = "SCAN: every orbit point is near an obstacle - holding"
            return
        self._scan_pending = pt
        self._scan_pending_since = now

    # -- search: stop for a person ---------------------------------------------
    def _scan_stop_here(self) -> None:
        """Abandon the next orbit leg and hold on the spot. A leg that was
        waiting for the nose to come round has a CONDITION_YAW in flight;
        a relative 0 deg yaw pins the heading where it is now, so the camera
        footprint stops swinging off the person too."""
        if self._scan_pending is not None:
            self._send("yaw", angle=0.0, direction=1, rate=self._avoider.yaw_rate_deg_s)
        self._scan_pending = None
        self._hold_here()

    def _scan_candidate(self, now: float) -> bool:
        """Someone worth stopping the orbit for: the camera acquiring or
        holding a target, or a detection strong enough that a lock may follow."""
        if not self._scan_person_fresh(now):
            return False
        p = self._person
        if p.state in ("acquire", "lock", "hold"):
            return True
        for q in p.people_xy or []:
            try:
                if float(q[2]) >= self._scan_pause_conf:
                    return True
            except (TypeError, ValueError, IndexError):
                continue
        return False

    def _scan_pause_step(self, now: float) -> bool:
        """True while the orbit is stopped on a person, waiting for the lock.

        Stops the first time someone is in view; resumes once they have been
        out of view for scan_resume_s (a one-frame dropout is not leaving),
        or after scan_pause_max_s with no lock - then it does not stop for
        that same sighting again until the person has left the frame."""
        if not self._scan_pause_on:
            return False
        if self._scan_candidate(now):
            self._scan_seen_at = now
            if self._scan_paused_at is None:
                if self._scan_pause_spent:
                    return False
                self._scan_paused_at = now
                self._scan_stop_here()
                p = self._person
                self.log.info("scan: person in view (%s, %d in frame) - stopping the "
                              "orbit here for the lock", p.state, p.people)
                return True
            if now - self._scan_paused_at > self._scan_pause_max_s:
                self._scan_paused_at = None
                self._scan_pause_spent = True
                self._scan_next_step = now
                self.log.info("scan: no lock after %.0fs stopped on a person - "
                              "resuming the orbit for another angle", self._scan_pause_max_s)
                return False
            return True
        if now - self._scan_seen_at < self._scan_resume_s:
            return self._scan_paused_at is not None     # brief dropout: keep holding
        self._scan_pause_spent = False
        if self._scan_paused_at is not None:
            self._scan_paused_at = None
            self._scan_next_step = now
            self.log.info("scan: person out of view for %.1fs - resuming the orbit",
                          self._scan_resume_s)
        return False

    def _scan_leg_ready(self, pt: tuple[float, float]) -> bool:
        """Only translate toward a point the LiDAR can see. The rear 110 deg
        is unscanned, so a leg that starts behind the nose is preceded by a
        (paced, relative) yaw onto it - the same rule the cruise leg follows."""
        f = self._fused
        if f is None or not f.valid:
            return False
        de, dn = pt[0] - f.x, pt[1] - f.y
        if math.hypot(de, dn) < 0.5:
            return True
        rel = wrap_180(math.degrees(math.atan2(de, dn) - f.yaw))
        if abs(rel) <= min(60.0, self._avoider.yaw_release_deg):
            return True
        now = time.monotonic()
        if now - self._nose_yaw_sent_at >= self._avoider.nose_recmd_s:
            self._nose_yaw_sent_at = now
            self._send("yaw", angle=abs(rel), direction=1 if rel > 0 else -1,
                       rate=self._avoider.yaw_rate_deg_s)
        return False

    def _obstacles_enu(self, within_m: float) -> list[tuple[float, float]]:
        f, obs = self._fused, self._obstacles
        if f is None or not f.valid or obs is None:
            return []
        out = []
        for o in obs.obstacles:
            if o.distance_m <= within_m:
                de, dn = body_to_enu(o.x_m, -o.y_m, f.yaw)   # y_m is +left
                out.append((f.x + de, f.y + dn))
        return out

    def _near_obstacle(self, pt: tuple[float, float]) -> bool:
        reach = self._scan_hold_m + 2.0 * self._scan_orbit_r + 1.0
        return any(math.hypot(pt[0] - e, pt[1] - n) < self._scan_hold_m
                   for e, n in self._obstacles_enu(reach))

    # -- search: phone locator -----------------------------------------------
    def _scan_update_locator(self, now: float) -> None:
        """Once a second: refresh the phone estimate, and walk the search
        circle toward it when it is confident and far enough off-centre.

        The estimate's BEARING is good and its RANGE is not (the posterior
        is long along the bearing - see PhoneLocator.estimate), so the
        centre moves at most locator_step_m per move and re-orbits there,
        which re-measures from the new spot instead of trusting the range."""
        if now - self._loc_last < 1.0:
            return
        self._loc_last = now
        fix = self._locator.estimate(self._scan_drop[0], self._scan_drop[1], time.time())
        self._phone_fix = fix
        if (not self._loc_enabled or fix is None or fix.rssi_bins < 3
                or fix.std_m > self._loc_steer_std
                or now - self._scan_recentred_at < self._loc_recentre_s):
            return
        cx, cy = self._scan_centre
        de, dn = fix.east - cx, fix.north - cy
        d = math.hypot(de, dn)
        if d < self._loc_recentre_m:
            return
        step = min(d, self._loc_step_m)
        self._scan_set_centre(cx + de * step / d, cy + dn * step / d,
                              f"phone estimate {d:.1f} m off (+/-{fix.std_m:.1f} m)")

    def _scan_set_centre(self, east: float, north: float, why: str) -> None:
        de, dn = east - self._scan_drop[0], north - self._scan_drop[1]
        d = math.hypot(de, dn)
        if d > self._loc_max_offset:              # never wander off the order
            east = self._scan_drop[0] + de * self._loc_max_offset / d
            north = self._scan_drop[1] + dn * self._loc_max_offset / d
        self._scan_centre = (east, north)
        self._scan_recentred_at = time.monotonic()
        self._scan_orbit_i = 0
        self._scan_pending = self._scan_centre
        self._scan_pending_since = time.monotonic()
        self.log.info("scan: search centre -> %.1f, %.1f (%s)", east, north, why)

    def _phone_note(self) -> str:
        fix = self._phone_fix
        if fix is None or self._fused is None:
            return ""
        d = math.hypot(fix.east - self._fused.x, fix.north - self._fused.y)
        return f", phone ~{d:.0f} m +/-{fix.std_m:.0f}"

    # -- search: which person? ------------------------------------------------
    def _scan_person_fresh(self, now: float) -> bool:
        return (self._person is not None
                and self._person_rx >= self._scan_search_since
                and now - self._person_rx <= self._scan_lock_fresh_s)

    def _cam_pitch_from_nadir(self) -> float:
        if self._scan_cam_pitch is not None:
            return self._scan_cam_pitch
        # up_deg looks at the horizon (90 deg from nadir); every servo degree
        # past it pitches the camera one degree further down.
        return max(0.0, 90.0 - (self._cam_down_deg - self._cam_up_deg))

    def _people_enu(self, pl: PersonLockState) -> list[tuple[int, float, float]]:
        """(index into people_xy, east, north) for every person in view."""
        f = self._fused
        if f is None or not f.valid or self._cam_tilt != "down":
            return []                        # tilt unknown: projection is a guess
        pitch = self._cam_pitch_from_nadir()
        h = max(0.5, f.alt_rel_m - self._scan_person_h)
        out = []
        for i, p in enumerate(pl.people_xy or []):
            try:
                g = image_to_ground(float(p[0]), float(p[1]), h, pitch)
            except (TypeError, ValueError, IndexError):
                continue
            if g is None:
                continue
            de, dn = body_to_enu(g[0], g[1], f.yaw)
            out.append((i, f.x + de, f.y + dn))
        return out

    def _rssi_pick(self, pl: PersonLockState) -> tuple[int, float, int] | None:
        """(people_xy index, probability, candidates) of the person the phone
        evidence favours, or None when there is nobody to choose between."""
        if not self._loc_enabled:
            return None
        cands = self._people_enu(pl)
        if len(cands) < 2:
            return None
        probs = self._locator.score_candidates(
            [(e, n) for _, e, n in cands], self._scan_drop[0], self._scan_drop[1],
            time.time())
        if not probs:
            return None
        best = max(range(len(probs)), key=probs.__getitem__)
        return cands[best][0], probs[best], len(cands)

    def _scan_update_hint(self, now: float) -> None:
        """Tell the camera which person to prefer, before it locks."""
        if now - self._hint_at < 0.25:
            return
        self._hint_at = now
        pick = self._rssi_pick(self._person) if self._scan_person_fresh(now) else None
        if pick is None or pick[1] < self._loc_pick_prob:
            self._publish_hint(None)
            return
        p = self._person.people_xy[pick[0]]
        self._publish_hint((float(p[0]), float(p[1])))

    def _publish_hint(self, nxy: tuple[float, float] | None, relock: bool = False) -> None:
        if relock:
            self._hint_seq += 1
        key = None if nxy is None else (round(nxy[0], 2), round(nxy[1], 2), self._hint_seq)
        if key == self._hint_sent and not relock:
            return
        self._hint_sent = key
        if nxy is None:
            self.publish(Topics.PHONE_HINT, PhoneHint(valid=False, seq=self._hint_seq))
        else:
            self.publish(Topics.PHONE_HINT, PhoneHint(
                valid=True, nx=nxy[0], ny=nxy[1], sigma=0.15,
                relock=relock, seq=self._hint_seq))

    def _scan_lock_accepted(self, now: float) -> bool:
        """A fresh camera lock, on the right person if we can tell.

        One person in view, or RSSI that cannot separate them: take the
        camera's lock - the HMAC handshake is what actually authenticates the
        recipient, the lock only decides where to wait. RSSI confident that
        the phone is on SOMEONE ELSE: do not open the window; ask the camera
        to re-lock onto them instead."""
        if not self._scan_person_fresh(now) or self._person.state != "lock":
            return False
        pl = self._person
        self._scan_note = ""
        pick = self._rssi_pick(pl)
        if pick is None:
            return True
        idx, prob, n = pick
        if prob < self._loc_pick_prob:
            self._scan_note = (f"RSSI cannot separate {n} people (best p={prob:.2f}) "
                               "- camera's pick")
            return True
        locked = min(range(len(pl.people_xy)),
                     key=lambda i: (pl.people_xy[i][0] - pl.nx) ** 2
                     + (pl.people_xy[i][1] - pl.ny) ** 2)
        if locked == idx:
            self._scan_note = f"RSSI picked this person of {n} (p={prob:.2f})"
            return True
        if now - self._relock_at >= 2.0:
            self._relock_at = now
            p = pl.people_xy[idx]
            self._publish_hint((float(p[0]), float(p[1])), relock=True)
            self.log.info("scan: camera locked person %d of %d but the phone is on "
                          "person %d (p=%.2f) - asking for a re-lock", locked + 1, n,
                          idx + 1, prob)
        return False

    # -- hover obstacle avoidance --------------------------------------------
    def _hover_avoid_step(self) -> bool:
        """Obstacle response while holding over the drop point. True when it
        commanded motion this tick (the stage must not issue its own).

        The cruise avoider only looks down the flight path (the +/-50 deg
        front cone), and a hover has no flight path - which is why nothing
        reacted to an obstacle during the hold before this. Here every scanned
        bearing counts, each against its own stop distance (a person walking
        in buys their reaction pad), and the response is to move AWAY at dodge
        speed: straight away when that is inside the scanned window, sideways
        toward the clearer side when "away" would be the blind rear, because a
        reverse into the 110 deg the LiDAR cannot see is never commanded.
        """
        if not self._avoid_enabled or self._avoid_standby() or self._obstacles is None:
            return self._hover_avoid_release()
        worst, worst_margin = None, math.inf
        for o in self._obstacles.obstacles:
            stop_m = self._avoider.stop_distance_for(
                getattr(o, "closing_ms", 0.0) or 0.0,
                own_speed_ms=getattr(o, "speed_m_s", 0.0) or 0.0,
                is_dynamic=bool(getattr(o, "is_dynamic", False)),
            )
            margin = o.distance_m - stop_m
            if margin < worst_margin:
                worst, worst_margin = o, margin
        release = self._avoider.release_m if self._hover_avoid == "back" else 0.0
        if worst is None or worst_margin > release:
            return self._hover_avoid_release()

        self._avoiding = True
        if not self._ensure_guided():
            self._status_message = "HOVER: leaving BRAKE to back off an obstacle"
            return True
        v = self._avoider.dodge_speed
        away = wrap_180(worst.bearing_deg + 180.0)
        rad = math.radians(away)
        vx, vy = v * math.cos(rad), v * math.sin(rad)
        how = "backing off"
        if vx < 0.0:
            # "Away" is behind us: slide sideways only. Nearly dead ahead there
            # is no sideways component to speak of, so pick the clearer side.
            view = self._avoider.sectors(self._obstacles)
            if abs(math.sin(rad)) >= 0.35:
                right = vy > 0.0
            else:
                right = view.right > view.left
            side_room = view.right if right else view.left
            if side_room <= self._avoider.stop:
                self._send("brake")
                self._hover_avoid = "back"
                self._status_message = (
                    f"HOVER: obstacle {worst.distance_m:.1f} m, no room to move - holding")
                return True
            vx, vy = 0.0, v if right else -v
            how = "sliding " + ("right" if right else "left")
        if self._hover_avoid != "back":
            self.log.warning("hover: obstacle %.1f m at %+.0f deg - %s",
                             worst.distance_m, worst.bearing_deg, how)
        self._hover_avoid = "back"
        self._scan_pending = None
        self._send("velocity", vx=vx, vy=vy, vz=0.0)
        self._status_message = f"HOVER: obstacle {worst.distance_m:.1f} m - {how}"
        return True

    def _hover_avoid_release(self) -> bool:
        if self._hover_avoid == "back":
            # Clear. Re-anchor where we are rather than fly back at it.
            self._hover_avoid = ""
            self._avoiding = False
            self._hold_here()
            self.log.info("hover: obstacle clear - holding here")
        return False

    def _do_avoid(self) -> None:
        decision, distance = self._avoid_decision()
        self._avoiding = True
        if decision == STOP:
            self._dodge_step(distance)
            return
        # Hysteresis before releasing the hold. STOP and everything else now
        # command different flight modes (BRAKE against a goto in GUIDED), so
        # handing back on the first non-STOP tick lets scan jitter around the
        # stop distance chatter the mode at 10 Hz. Require the gap to have
        # opened by release_m first. CLEAR is exempt: nothing is in the cone at
        # all, so there is no boundary to jitter across.
        if (decision != CLEAR
                and distance < self._avoider.last_stop_m + self._avoider.release_m):
            self._dodge_step(distance)
            self._status_message = (
                f"holding at {distance:.1f} m - needs "
                f"{self._avoider.last_stop_m + self._avoider.release_m:.1f} m to release"
            )
            return
        # Path ahead is clear again - resume navigation. Clearing _last_goto_wp
        # forces _do_navigate to re-issue the goto, which is what pulls the
        # aircraft back onto its original track instead of carrying on from
        # wherever the sidestep left it.
        self._dodge_dir = None
        self._dodge_started = None
        self._dodge_origin = None
        self._dodge_heading = None
        self._dodge_heading_t = None
        self._offtrack_m = 0.0
        self._avoiding = decision != CLEAR
        self._phase = MissionPhase.NAVIGATE
        self._last_goto_wp = -1
        self._status_message = "obstacle cleared - rejoining track"

    def _hold_nose_on_waypoint(self) -> None:
        """Keep the nose pointed at the active waypoint at all times.

        Until now the stack never commanded yaw. Velocity setpoints go out in
        MAV_FRAME_BODY_NED with the yaw bit masked off, so ArduPilot STRAFES:
        during an avoidance manoeuvre the aircraft slid sideways with the nose
        wherever the last nav command happened to leave it. _yaw_gate_ok only
        guarantees the waypoint is within yaw_release_deg (60 deg) before
        translating - it does not hold it there.

        Two reasons this is worth commanding, beyond the operator's preference
        for an aircraft that looks where it is going:

        * The LiDAR window is 250 deg front-referenced, so the 110 deg it CANNOT
          see is always directly behind the nose. Pinning the nose to the
          waypoint puts the dense middle of the scan on the path being flown,
          and - because _steer_step and _dodge_step both clamp vx >= 0 - every
          commanded translation is then within +/-90 deg of the nose, i.e.
          inside the scanned window by a 35 deg margin. The masked rear becomes
          unreachable by construction rather than by luck.
        * A body-frame VFH+ solution is only as meaningful as the body frame is
          stable. Holding a known heading makes the gap bearings comparable
          across ticks, which is what the hysteresis term assumes.

        Paced, not streamed: CONDITION_YAW is a discrete command and re-sending
        it at the 10 Hz loop rate restarts the turn every tick so it never
        lands. This is the same trap documented on the pre-RTL turn, and the
        same 2 s pacing answers it.

        Sent RELATIVE (the "yaw" command hardcodes param4=1), so the magnitude
        is the nose-relative bearing straight out of _goal_bearing_deg and the
        sign is the direction.
        """
        if not self._avoider.nose_track:
            return
        if self._fused is None or not self._fused.valid or self._home is None:
            return                      # bearing unknown; a guess is worse
        if self._active_waypoint() is None:
            return
        bearing = self._goal_bearing_deg()
        if abs(bearing) <= self._avoider.nose_deadband_deg:
            return                      # close enough; do not chase noise
        now = time.monotonic()
        if (now - self._nose_yaw_sent_at) < self._avoider.nose_recmd_s:
            return
        self._nose_yaw_sent_at = now
        self._send(
            "yaw",
            angle=abs(bearing),
            direction=1 if bearing > 0.0 else -1,
            rate=self._avoider.yaw_rate_deg_s,
        )

    def _steer_step(self, distance: float, wp: Waypoint, dist_to_wp: float) -> None:
        """Bend the course around an obstacle without losing way.

        The cruise-band counterpart to ``_dodge_step``. The dodge is an
        emergency manoeuvre flown at the brake distance with forward speed
        rationed out; this runs the whole band above it and holds cruise speed
        throughout. The correction is applied to the COURSE, never the speed -
        slowing down is what turned an obstacle into a full stop.

        The course flown is the goal bearing blended toward the VFH+ gap
        bearing by ``steer_authority(distance)``. That ramp does three jobs at
        once: the deviation is gradual rather than a step, the manoeuvre is
        finished before the brake distance is reached, and the return to track
        is automatic - as the obstacle leaves the front cone the authority
        decays and the course relaxes back onto the goal bearing on its own,
        with the re-issued goto in ``_do_navigate`` closing the last of it.
        """
        now = time.monotonic()
        goal = self._goal_bearing_deg()
        want = self._avoider.choose_heading(
            self._obstacles, goal, self._steer_heading)
        if want is None:
            # Nothing wide enough to fly through anywhere in the scanned
            # window. Do not invent a course, and let the STOP band own the
            # outcome - that is where TRAPPED is handled properly. But do not
            # arrive there at cruise speed either (2026-09-23, "no hard
            # braking"): holding the goto used to carry full speed right up to
            # the brake distance and then slam the FC into BRAKE. Instead fly
            # the goal bearing on a constant-deceleration profile,
            # v = sqrt(2 a room), so the aircraft reaches the brake distance
            # already close to a standstill and BRAKE has almost nothing left
            # to take out. If a gap opens meanwhile, choose_heading returns it
            # next tick and the steer below resumes from this slower speed.
            speed = self._avoider.steer_speed_ms or self._cruise_speed
            room = max(0.0, distance - self._avoider.last_stop_m)
            v = min(speed, math.sqrt(2.0 * self._trapped_decel * room))
            rad = math.radians(goal)
            self._send("velocity", vx=max(0.0, v * math.cos(rad)),
                       vy=v * math.sin(rad), vz=0.0)
            # No goto is in force any more; make CLEAR re-issue it rather than
            # leave the FC coasting out a stale velocity setpoint.
            self._last_goto_wp = -1
            self._status_message = (
                f"obstacle {distance:.1f} m - no flyable gap, easing to "
                f"{v:.1f} m/s towards waypoint {wp.seq}"
            )
            return

        if self._steer_origin is None and self._fused is not None \
                and self._fused.valid:
            # Where the deviation began, so off-track is measured from the
            # track we actually left rather than from the leg's start.
            self._steer_origin = (self._fused.x, self._fused.y)
            self.log.info(
                "obstacle at %.1f m - easing around it (gap %+.0f deg)",
                distance, want)

        authority = self._avoider.steer_authority(distance)
        if authority is None:
            # steer_authority() is not implemented yet. Fail to the PREVIOUS
            # behaviour - hold the goto - rather than to a TypeError at 10 Hz.
            # aerix-gcs runs Restart=always, so whatever is on disk becomes the
            # flying code the moment anything restarts the service; a
            # half-finished edit must degrade, not throw.
            self.log.error(
                "steer_authority() returned None - cruise-band steering is "
                "not implemented, holding course. Obstacle at %.1f m.",
                distance)
            if self._last_goto_wp != self._current_wp:
                self._issue_goto(wp)
                self._last_goto_wp = self._current_wp
            return
        authority = max(0.0, min(1.0, float(authority)))
        target = wrap_180(goal + authority * wrap_180(want - goal))

        # The stored course is a BODY-frame bearing and the airframe is now
        # being yawed onto the waypoint underneath it. Counter-rotate by the
        # measured yaw delta so _steer_heading keeps pointing at the same patch
        # of ground: without this the rate limiter reads the aircraft's own
        # rotation as a course change it must resist, and the yaw controller and
        # slew_steer spend the manoeuvre cancelling each other out.
        prev = self._steer_heading
        if prev is not None and self._prev_yaw_rad is not None \
                and self._fused is not None and self._fused.valid:
            dyaw = math.degrees(wrap_pi(self._fused.yaw - self._prev_yaw_rad))
            prev = wrap_180(prev - dyaw)

        if self._fused is not None and self._fused.valid:
            self._prev_yaw_rad = self._fused.yaw

        dt = (now - self._steer_heading_t) if self._steer_heading_t else 0.0
        heading = self._avoider.slew_steer(target, prev, dt)
        self._steer_heading = heading
        self._steer_heading_t = now

        # Cruise speed along the bent course. cos() is clamped at zero for the
        # same reason it is in _dodge_step: choose_heading may legitimately
        # return a rear-quadrant bearing, and passing that through unclamped
        # commands vx < 0 - a reverse into the 110 deg the LiDAR cannot see.
        # A rear-quadrant course becomes a pure lateral slide, never a back-up.
        speed = self._avoider.steer_speed_ms or self._cruise_speed
        rad = math.radians(heading)
        vx = max(0.0, speed * math.cos(rad))
        vy = speed * math.sin(rad)
        self._offtrack_m = self._compute_offtrack(self._steer_origin)
        self._send("velocity", vx=vx, vy=vy, vz=0.0)
        self._status_message = (
            f"obstacle {distance:.1f} m -> easing "
            f"{wrap_180(heading - goal):+.0f} deg off course "
            f"({authority * 100:.0f}% authority, {speed:.1f} m/s, off-track "
            f"{self._offtrack_m:.1f} m, waypoint {wp.seq} {dist_to_wp:.1f} m)"
        )

    def _vfh_latch(self, view: SectorView, now: float) -> str:
        """Commit to a course using VFH+, or report TRAPPED.

        Returns one of the DODGE_* constants so the latch, timeout, off-track
        and status-message machinery below is untouched - the side is derived
        from the sign of the chosen bearing, and the bearing itself is what
        actually gets flown.
        """
        heading = self._avoider.choose_heading(
            self._obstacles, self._goal_bearing_deg(), None)
        if heading is None:
            return DODGE_TRAPPED
        self._dodge_heading = heading
        self._dodge_heading_t = now
        return DODGE_RIGHT if heading > 0.0 else DODGE_LEFT

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
            side = self._vfh_latch(view, now) if self._avoider.vfh_enabled \
                else self._avoider.dodge(view)
            if side == DODGE_TRAPPED:
                # Neither side is open. The source node reverses out of a dead
                # end here; this aircraft cannot. The rear 110 deg is masked
                # (250 deg scanned of 360), so "behind" is unmeasured rather
                # than clear, and the operating ceiling (safety.max_altitude_m,
                # enforced by the altitude hardlock) rules out climbing over.
                # Hold and let the pilot or the mission decide.
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

        # Re-solve the course every tick. choose_heading is pulled toward the
        # heading already being flown (hysteresis) and the result is rate
        # limited, so this refines a committed course rather than re-deciding
        # it - the latch above still owns whether we are dodging at all, which
        # is what keeps the commanded MODE stable at 10 Hz.
        heading = self._dodge_heading
        if self._avoider.vfh_enabled:
            want = self._avoider.choose_heading(
                self._obstacles, self._goal_bearing_deg(), self._dodge_heading)
            if want is None:
                # The gap we committed to has closed. Same answer as a blocked
                # side: stop. Never reverse - the rear is unmeasured.
                self._dodge_dir = None
                self._dodge_started = None
                self._dodge_origin = None
                self._dodge_heading = None
                self._dodge_heading_t = None
                self._send("brake")
                self._status_message = (
                    f"no flyable gap at {distance:.1f} m - holding"
                )
                return
            dt = (now - self._dodge_heading_t) if self._dodge_heading_t else 0.0
            heading = self._avoider.slew_heading(want, self._dodge_heading, dt)
            self._dodge_heading = heading
            self._dodge_heading_t = now
            # Keep the latched side pointing at the course actually being
            # flown. The heading is re-solved every tick and crosses the nose
            # whenever the goal bearing moves; leaving _dodge_dir on the side
            # picked at entry makes the "chosen side closed in" guard below
            # read the clearance of a side we are no longer flying toward -
            # which clears us on a blocked course, or brakes us on a clear one.
            # This only relabels which cone that guard reads: both branches
            # still command "velocity", so the commanded MODE does not move and
            # the 10 Hz mode-stability rule above is untouched.
            self._dodge_dir = DODGE_RIGHT if heading > 0.0 else DODGE_LEFT

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
            self._dodge_heading = None
            self._dodge_heading_t = None
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
        # THE BRAKE THAT PUT US HERE LEFT THE FC IN BRAKE, where velocity
        # setpoints are silently discarded. Ask for GUIDED back before
        # commanding any motion.
        #
        # Without this the dodge solved a perfect VFH+ course, called
        # _send("velocity", ...) at 10 Hz, and commanded NOTHING - "velocity"
        # records GUIDED in _commanded_mode but is not a _MODE_ONLY_COMMAND, so
        # no SET_MODE frame was ever emitted. The aircraft braked at
        # avoidance_stop_m, sat there for avoidance_dodge_timeout_s while the
        # status line reported a heading it was not flying, then gave up and
        # held. That is the whole of "avoidance does not work": not the
        # geometry, which was right, but the mode it was written into.
        #
        # _ensure_guided's own docstring described this exact failure for the
        # NAVIGATE path on 2026-08-31. The fix was never applied here.
        if not self._ensure_guided():
            self._status_message = (
                f"obstacle {distance:.1f} m - leaving BRAKE to steer "
                f"{self._dodge_dir}"
            )
            return

        margin = view.front - self._avoider.last_stop_m
        vx = 0.0
        if margin > 0.0:
            vx = min(self._avoider.dodge_forward_ms, margin * 0.5)

        # MAV_FRAME_BODY_NED: +y is right of the nose, the same sign convention
        # ObstacleNode reports bearings in.
        if self._avoider.vfh_enabled and heading is not None:
            # Fly the chosen course. The forward component is still EARNED the
            # same way as the pure sidestep above - capped by dodge_forward_ms
            # and by half the front margin - so switching to VFH+ can never
            # command more forward speed at an obstacle than the three-cone
            # dodge already would. All the extra freedom is lateral.
            rad = math.radians(heading)
            vy = self._avoider.dodge_speed * math.sin(rad)
            # cos() turns NEGATIVE past +/-90 deg, and choose_heading may
            # legitimately return up to +/-(fov_half_deg - min_valley/2) =
            # +/-116 deg - which happens any time the waypoint sits behind the
            # wing, as an overshooting dodge or an RTL leg both produce.
            # Passing that through unclamped commands vx < 0: a reverse into
            # the 110 deg the LiDAR cannot see. That is the one manoeuvre this
            # design forbids everywhere else - see SectorView, _bin_centres and
            # the TRAPPED branch - so it is clamped here too. A rear-quadrant
            # course becomes a pure lateral slide, never a back-up.
            forward = max(0.0, self._avoider.dodge_speed * math.cos(rad))
            vx = min(vx, forward)
            self._send("velocity", vx=vx, vy=vy, vz=0.0)
            self._status_message = (
                f"obstacle {distance:.1f} m -> steering {heading:+.0f} deg "
                f"(L {_m(view.left)} / R {_m(view.right)}, "
                f"fwd {vx:.1f} m/s, off-track {self._offtrack_m:.1f} m)"
            )
            return
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

    def _goal_bearing_deg(self) -> float:
        """Bearing to the active waypoint, nose-relative (+ right).

        0.0 whenever the answer is not actually known - no fix, no home, no
        waypoint. That degrades VFH+ to "take the gap nearest straight ahead",
        which is the correct fallback: a guessed goal bearing would pull the
        aircraft toward a heading nothing supports.

        Uses the tracker's enu_to_body rather than a fresh rotation, so this
        cannot drift out of step with the frame conventions the rest of the
        stack already agrees on (body x forward / y left, yaw 0 = North
        clockwise, ENU x East / y North).
        """
        if self._fused is None or not self._fused.valid or self._home is None:
            return 0.0
        wp = self._active_waypoint()
        if wp is None:
            return 0.0
        tx, ty = wp.x_m, wp.y_m
        if wp.lat != 0.0 or wp.lon != 0.0:
            tx, ty = geodetic_to_enu(wp.lat, wp.lon, self._home[0], self._home[1])
        east, north = tx - self._fused.x, ty - self._fused.y
        if math.hypot(east, north) < 1e-6:
            return 0.0
        forward, left = enu_to_body(east, north, self._fused.yaw)
        return wrap_180(math.degrees(math.atan2(-left, forward)))

    def _compute_offtrack(
        self, origin: tuple[float, float] | None = None
    ) -> float:
        """Perpendicular distance from the line the manoeuvre departed from.

        The track is the straight line from where the manoeuvre started to the
        waypoint it was heading for. Returns 0.0 when there is no fix or no
        active waypoint - an unknown excursion must not read as a large one and
        trip the cap, because that would turn "no GPS" into "never dodge".

        ``origin`` defaults to the dodge's, so existing callers are unchanged;
        the cruise-band steering passes its own, which is where IT left the
        track rather than where a later dodge would.
        """
        origin = origin if origin is not None else self._dodge_origin
        if origin is None or self._fused is None or not self._fused.valid:
            return 0.0
        wp = self._active_waypoint()
        if wp is None or self._home is None:
            return 0.0
        tx, ty = wp.x_m, wp.y_m
        if wp.lat != 0.0 or wp.lon != 0.0:
            tx, ty = geodetic_to_enu(wp.lat, wp.lon, self._home[0], self._home[1])
        ox, oy = origin
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
            self._rtl_turn_since = None
            self._about_face_since = None
            self._about_face_target = None
            self._rtl_yaw_started = None
            return
        # Two-stage pre-RTL turn, in order: turn around, then point at home.
        # Nothing below has been commanded yet while either is running.
        #
        # Stage 2 usually finds nothing to do once stage 1 has run - an
        # about-face from a delivery flown in forwards already leaves home
        # inside the scan window - but it is still consulted rather than
        # assumed, because the inbound leg may have dodged.
        if self._about_face_since is not None:
            if not self._about_face_step():
                return
            self._about_face_since = None
            if self._rtl_turn_wanted():
                return
            self._commit_rtl()
        if self._rtl_turn_since is not None:
            if not self._rtl_turn_step():
                return
            self._commit_rtl()
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
        # The return leg is flown by the FC, but the Pi's picture of the
        # obstacle field must not go dark while it is - see _rtl_avoid_step.
        if self._rtl_avoid_step():
            return

        # Home at last: put it on the ground rather than leave it hovering.
        if self._rtl_arrived_home():
            self._status_message = f"over home - landing ({self._land_reason})"
            self._enter_land()
            return

        self._status_message = (
            "smart RTL: retracing path home"
            if self._mode == "SMART_RTL"
            else "returning to launch"
        )

    def _rtl_arrived_home(self) -> bool:
        """Is the aircraft over the pad (or out of time) and ready to land?

        Only ever consulted once ``_commit_rtl`` has actually commanded the
        return - ``_rtl_requested_at`` is None through both turn stages, which
        is exactly the window in which the aircraft is still parked over the
        customer and would otherwise satisfy the radius test immediately on a
        short flight.
        """
        if self._rtl_requested_at is None:
            return False
        since = time.monotonic() - self._rtl_requested_at
        if since < self._land_arm_delay_s:
            return False

        fused = self._fused
        if fused is not None and fused.valid and self._home is not None:
            # fused.x/y are ENU metres from home, so the distance home is just
            # the magnitude - no geodetic round trip (same as _home_bearing_deg).
            dist = math.hypot(fused.x, fused.y)
            if dist <= self._land_radius_m:
                self._land_reason = f"{dist:.1f} m from home"
                self.log.info(
                    "RTL arrived: %.1f m from home after %.0fs at %.1f m - "
                    "commanding LAND", dist, since, fused.alt_rel_m,
                )
                return True

        if since > self._land_rtl_timeout_s:
            # Not a normal exit. Either the return never made progress or the
            # position estimate went away mid-leg. Landing where it is beats
            # orbiting until the battery failsafe decides for us, and LAND is
            # a descent the FC flies on its own sensors.
            self._land_reason = f"RTL timed out after {since:.0f}s"
            self.log.error(
                "RTL has not reached home within %.0fs - landing where the "
                "aircraft is. Check RTL_ALT against the altitude ceiling and "
                "that the FC accepted %s.",
                self._land_rtl_timeout_s, self._return_mode,
            )
            return True
        return False

    def _rtl_avoid_step(self) -> bool:
        """Keep the Pi's avoidance layer alive through the FC-flown return.

        Until 2026-09-19 ``_do_rtl`` never called ``_avoid_decision`` at all.
        Two things followed from that, and only one of them was intended.

        The intended one: the Pi does not steer the return. ArduPilot's
        BendyRuler does (OA_TYPE=1, fed by the OBSTACLE_DISTANCE stream
        ProximityNode publishes regardless of phase), and two routers arguing
        over one aircraft is the failure mode ``avoidance_dodge_enabled`` is
        switched off for.

        The unintended one: ``self._avoiding`` was never updated during RTL, so
        the GCS avoidance panel went quiet for the entire return leg no matter
        what was in front of the aircraft. From the operator's seat that is
        indistinguishable from avoidance being off - which is exactly how it
        was reported. Watching is not steering, and this method watches.

        Returns True if it has taken over the tick; the caller must then issue
        nothing else.
        """
        decision, distance = self._avoid_decision()
        self._avoiding = decision != CLEAR

        if decision != STOP:
            self._rtl_brake_release()
            return False

        now = time.monotonic()
        if self._rtl_braking_since is None:
            self._rtl_braking_since = now
            self.log.error(
                "obstacle %.1f m ahead during %s - inside the %.1f m hard "
                "brake. BendyRuler routed into something it could not see or "
                "could not fit past.",
                distance, self._return_mode, self._avoider.stop,
            )
        held = now - self._rtl_braking_since

        if held > self._avoider.rtl_brake_max_s:
            # The budget is spent. As with the pre-RTL turn stages, the answer
            # is to go home rather than to keep holding: an aircraft parked
            # mid-return burns battery until a human notices, and the FC has
            # its own margins for the last few metres.
            if not self._rtl_brake_warned:
                self._rtl_brake_warned = True
                self.log.error(
                    "held the return for %.0fs against an obstacle at %.1f m "
                    "and it has not cleared - handing the return back to the "
                    "FC. TAKE MANUAL CONTROL IF YOU CAN SEE THE AIRCRAFT.",
                    held, distance,
                )
            self._rtl_brake_release()
            return False

        return self._rtl_obstacle_response(distance, held)

    def _rtl_obstacle_response(self, distance: float, held: float) -> bool:
        """What the Pi does about an obstacle inside the hard-brake distance
        while the flight controller is flying the return leg.

        Called only when the obstacle is inside ``avoidance_stop_m`` and the
        ``rtl_brake_max_s`` budget still has room; ``_rtl_avoid_step`` owns
        both of those tests and the release when the path clears again.

        ``held`` is how long this obstacle has been blocking us, in seconds.

        Return True to claim the tick (nothing else will be commanded), or
        False to let the flight controller carry on flying the return.

        The Pi brakes and reports here; it never issues a setpoint. The comment
        below is why that is a contract rather than a gap to be filled in.
        """
        # The Pi does NOT steer the return. Until 2026-09-19 this branch
        # computed a VFH+ heading and streamed a velocity setpoint at 10 Hz -
        # the same mistake the NAVIGATE phase made and unwound on 2026-08-31,
        # see the comment above the SLOW branch in _do_navigate.
        #
        # In RTL the flight controller DISCARDS SET_POSITION_TARGET_LOCAL_NED.
        # Those setpoints are honoured in GUIDED and nowhere else. So the dodge
        # commanded nothing, returned True, and wrote "steering +45 deg" to the
        # status line. A no-op that reports success is worse than an error: it
        # spends the rtl_brake_max_s budget while the operator reads that the
        # aircraft is handling it, and so does not reach for the sticks.
        #
        # Making it real would mean _ensure_guided() first - and then the Pi
        # owns a mode the FC was flying, at a 2 m ceiling, with an unscanned
        # rear 110 deg it cannot reverse into. That is the 2026-08-22 lockout
        # shape. So: brake, which is a real command in every mode, and say so.
        view = self._avoider.sectors(self._obstacles)
        self._send("brake")

        # Which way is clear, as ADVICE for whoever has the sticks. Computing a
        # heading costs nothing and an operator who can see the aircraft needs
        # it. Issuing it as a setpoint is the part that must not happen.
        hint = ""
        if self._avoider.vfh_enabled:
            home_bearing = self._home_bearing_deg()
            heading = self._avoider.choose_heading(
                self._obstacles,
                home_bearing if home_bearing is not None else 0.0,
                self._dodge_heading,
            )
            if heading is not None:
                hint = f", clear {heading:+.0f} deg"

        self._status_message = (
            f"RTL obstacle {distance:.1f} m - HOLDING{hint} "
            f"(L {_m(view.left)} / R {_m(view.right)}) "
            f"{held:.0f}s of {self._avoider.rtl_brake_max_s:.0f}s"
        )
        return True

    def _rtl_brake_release(self) -> None:
        """Hand the return back to the FC after a Pi-side brake. Idempotent.

        Re-commanding the mode is not optional. If the brake ever put the
        aircraft in BRAKE, the FC is no longer flying the return, and simply
        ceasing to command anything would leave it holding position forever -
        which is the shape of the 2026-08-31 deadlock. Whatever released the
        brake, the return has to be asked for again.
        """
        if self._rtl_braking_since is None:
            return
        self._rtl_braking_since = None
        self._rtl_brake_warned = False
        self.log.info(
            "path clear again - resuming %s (FC in %s)",
            self._return_mode, self._mode,
        )
        if self._return_mode == "SMART_RTL":
            self._set_mode("SMART_RTL")
        else:
            self._set_mode("RTL")
            self._send("rtl")

    def _do_land(self) -> None:
        if not self._armed:
            self._phase = MissionPhase.DISARMED
            self._status_message = "landed and disarmed"
            self._land_sent_at = 0.0
            return
        # LAND is a mode change like any other, and the FC can refuse one. Until
        # 2026-09-19 this method did nothing at all while armed, so a refused
        # LAND left the aircraft hovering with the GCS reporting LAND - the
        # worst kind of silent failure. Re-assert it, paced, until the mode
        # sticks. _send already rate-limits mode-only commands.
        now = time.monotonic()
        if self._mode != "LAND" and (now - self._land_sent_at) > 2.0:
            self._land_sent_at = now
            self._send("land")
            self.log.warning(
                "commanded LAND but the FC is still in %s - re-asserting",
                self._mode,
            )
        alt = self._fused.alt_rel_m if self._fused is not None else float("nan")
        self._status_message = (
            f"LANDING: {alt:.1f} m above home"
            if self._mode == "LAND"
            else f"LANDING requested - FC still in {self._mode}"
        )

    # -- helpers -------------------------------------------------------------
    # -- camera tilt (AUX6) ---------------------------------------------------
    def _camera_tilt_on_approach(self, wp: Waypoint, dist: float) -> None:
        """Point the camera DOWN once the aircraft is inside
        ``down_before_drop_m`` of the drop point.

        Only a waypoint with a hold counts - that is the customer's pin, which
        _expand_delivery_mission ends every delivery route with. Intermediate
        route legs have no hold, and the Pi-flown home leg is kind "rtl", so
        neither can tilt the camera just because the aircraft passed near one.
        """
        if wp.hold_s <= 0 or wp.kind == "rtl":
            return
        if dist > self._cam_down_m:
            return
        self._camera_tilt_down(f"{dist:.1f} m from drop point")

    def _camera_tilt_for_landing(self) -> None:
        """DOWN while landing, UP once on the ground. Runs every tick from
        step(), OUTSIDE the pilot-override branch: it follows what the
        aircraft is doing, whoever is flying it, so a pilot's RC LAND or a
        failsafe LAND tilts the camera exactly like the navigator's own.

        "Landing" is the FC actually being in LAND, or this node having
        commanded it (the FC confirms a tick or two later). Both require
        ARMED, and that is load-bearing: ArduPilot stays in LAND after it
        disarms on touchdown, so without it the camera would be driven back
        down on the ground straight after the UP below.

        "On the ground" is the disarm edge latched by _on_armed. There is no
        landed-state message on this bus, and LAND disarms on touchdown
        detection, so the edge is the touchdown.
        """
        if self._cam_disarm_edge:
            self._cam_disarm_edge = False
            self._camera_tilt_up("landed")
            return
        if self._armed and (
            self._mode == "LAND" or self._phase == MissionPhase.LAND
        ):
            self._camera_tilt_down("landing")

    # The two latched commands. Each fires once per change of state, so a
    # per-tick caller sends one DO_SET_SERVO, not ten a second - and an
    # operator who presses the GCS button the other way is not overridden
    # until the flight actually moves on to its next stage.
    def _camera_tilt_down(self, why: str) -> None:
        if self._cam_tilt != "down":
            self._command_camera("down", self._cam_down_deg, why)

    def _camera_tilt_up(self, why: str) -> None:
        if self._cam_tilt != "up":
            self._command_camera("up", self._cam_up_deg, why)

    def _command_camera(self, state: str, deg: float, why: str) -> None:
        if not self._cam_auto:
            return
        # Same mapping as the GCS buttons (app.js aux2DegToUs), including
        # Math.round's round-half-up, then clamped to the configured envelope:
        # this command goes straight onto MAVLINK_CMD and never passes through
        # GcsHub's per-channel clamp.
        span = self._cam_max_us - self._cam_min_us
        us = int(math.floor(self._cam_min_us + deg * span / self._cam_span + 0.5))
        us = max(self._cam_min_us, min(self._cam_max_us, us))
        self._cam_tilt = state
        self._send("set_servo", channel=self._cam_ch, pwm=us)
        self.log.info("camera %s -> %.0f deg (ch%d %dus): %s",
                      state.upper(), deg, self._cam_ch, us, why)

    def _enter_rtl(self, turn_first: bool = False) -> None:
        """Come home. Prefers SMART_RTL, falls back to RTL (see _do_rtl).

        ``turn_first`` asks for the aircraft to be turned before the return is
        commanded: a full 180 deg about-face (:meth:`_about_face_wanted`) and
        then, if home is still behind, onto home (:meth:`_rtl_turn_wanted`). It
        is passed only by the three callers where the aircraft has just finished
        its job and a few seconds cost nothing: the BLE handshake outside the
        hold, waypoints exhausted, and hover complete.

        It is deliberately NOT passed by the failsafe path or by any of the
        three operator-commanded returns. A failsafe RTL fires on a low battery
        or a geofence breach, and delaying that to turn is wrong in exactly the
        situation where delay costs most; an operator pressing RTL is usually
        reacting to something and means *now*. Those four go home immediately,
        as they always have.
        """
        # Which of the two RTL families this is. _pi_return_wanted needs it at
        # _commit_rtl time, which may be several ticks later once a pre-RTL
        # turn is in progress, so it is latched rather than passed down.
        self._rtl_job_finished = bool(turn_first)
        self._phase = MissionPhase.RTL
        # Camera back up for the trip home - on EVERY return, not only the
        # job-finished ones. A failsafe RTL mid-approach has the camera down
        # and the pilot needs the forward view most then; and a return where
        # this node never commanded the servo leaves it wherever someone last
        # parked it, so commanding UP makes its position known. Arrival
        # re-entry after a Pi-flown return is a no-op through the latch.
        # Here, before the about-face, rather than in _commit_rtl: the turn
        # gates the commit for several seconds and the camera should already
        # face forward when the aircraft starts moving.
        self._camera_tilt_up("returning home")
        if self._pi_return_flown:
            # Arriving home at the end of a Pi-flown return re-enters here,
            # because the home leg is an ordinary waypoint and running out of
            # waypoints is what triggers the return. _about_face_wanted is
            # UNCONDITIONAL, so without this the aircraft would spin a second
            # 180 deg over home before landing. The about-face that requirement
            # asks for has already been flown - it happened before the return
            # leg, which is exactly where it belongs.
            turn_first = False
        if turn_first and self._about_face_wanted():
            return
        if turn_first and self._rtl_turn_wanted():
            return
        self._commit_rtl()

    def _pi_return_wanted(self) -> bool:
        """Fly the return with this node's steerer instead of handing it over.

        True if the return leg has been taken over; the caller must then
        command nothing.

        WHY. "Make sure avoidance works in RTL also" had no good FC-side
        answer on this airframe. _rtl_avoid_step watches and hard-brakes but
        deliberately never steers, and the flight controller could not steer
        either: OA_TYPE measured 0, so BendyRuler has never run here, and the
        two alternatives are worse rather than better - Dijkstra (OA_TYPE 2)
        plans against FENCE polygons only and FENCE_ENABLE measured 0 with
        none defined, so it cannot see a LiDAR return at all, and OA_TYPE 3 is
        Dijkstra plus BendyRuler, where the Dijkstra half contributes nothing.
        BendyRuler is the only FC-side planner that can use this sensor, and
        it is entirely unmeasured.

        Meanwhile the Pi's own cruise-band steerer IS measured, on this
        aircraft's own geometry: leaning from the 10 m band edge, 2.44 m
        closest approach to a 0.3 m pole against a 1.5 m critical distance,
        never entering the brake, peak slew exactly at the 10 deg/s cap, nose
        held on the target throughout. So the better router is not a different
        FC planner - it is the one already flying the outbound legs.

        HOW, and why it is this small. The return is flown as an ORDINARY
        WAYPOINT at home. That is the entire mechanism. Nothing about the
        avoidance path is re-implemented or re-tuned for RTL: the cruise band,
        the authority floor, the slew limiter, nose-track, the clear hold-off,
        the goto re-issue and the 1.5 m brake are the same code on the way
        home as on the way out, so they are covered by the same measurements
        and the same tests. Running out of waypoints then re-enters
        _enter_rtl, where _pi_return_flown routes it to the flight controller
        for the descent and landing.

        WHAT IS GIVEN UP, because it is not nothing. The flight controller's
        RTL owns the climb to RTL_ALT and its own failsafes; a Pi-flown return
        at the 2.0 m ceiling has neither. So this is offered ONLY on the three
        job-finished paths (``turn_first``) and never on a failsafe or an
        operator-commanded return - a low-battery or geofence RTL still goes
        straight to the FC, unchanged, which is the split _enter_rtl already
        documents. And it is budgeted: rtl_pi_budget_s bounds how long the Pi
        may own the return before handing it back, so a steerer that cannot
        solve its way home cannot strand the aircraft at 2 m.
        """
        if self._avoider.rtl_router != "pi":
            return False
        if not self._rtl_job_finished:
            return False                # failsafe or operator RTL: FC's job
        if self._pi_return_flown:
            return False                # one-shot; already offered
        if self._home is None or self._fused is None or not self._fused.valid:
            # No home or no fix means no waypoint to build. The FC's RTL has
            # its own idea of home and does not need ours.
            self.log.warning(
                "no home or no fix - handing the return to the FC")
            return False

        self._pi_return_flown = True
        self._pi_return_since = time.monotonic()
        alt = max(0.5, self._fused.alt_rel_m)
        home_wp = Waypoint(
            seq=self._mission.count, lat=self._home[0], lon=self._home[1],
            alt_m=alt, kind="rtl", radius_m=max(self._wp_radius, 2.0),
        )
        self._mission.waypoints.append(home_wp)
        self._current_wp = self._mission.count - 1
        # Straight back into the machinery that flew the outbound legs.
        self._phase = MissionPhase.NAVIGATE
        self._last_goto_wp = -1
        self._steer_heading = None
        self._steer_heading_t = None
        self._steer_origin = None
        self._steer_clear_since = None
        self._offtrack_m = 0.0
        self._ensure_guided()
        self._status_message = "returning home under Pi avoidance"
        self.log.info(
            "flying the return as waypoint %d (home) with the Pi's own "
            "avoidance - budget %.0f s", home_wp.seq,
            self._avoider.rtl_pi_budget_s)
        return True

    def _pi_return_overdue(self) -> bool:
        """Has the Pi-flown return spent its budget? Hands back if so.

        An aircraft that cannot solve its way home must not keep trying at a
        2 m ceiling until the battery failsafe decides for it. Handing back is
        always safe: the FC's RTL is what would have flown this leg anyway.
        """
        if self._pi_return_since is None:
            return False
        held = time.monotonic() - self._pi_return_since
        if held < self._avoider.rtl_pi_budget_s:
            return False
        self.log.error(
            "the Pi-flown return has used its %.0f s budget without reaching "
            "home - handing the return to the FC.",
            self._avoider.rtl_pi_budget_s)
        self._pi_return_since = None
        self._rtl_job_finished = False      # so _commit_rtl really commits
        self._phase = MissionPhase.RTL
        self._commit_rtl()
        return True

    def _commit_rtl(self) -> None:
        """Actually command the return.

        ``_rtl_requested_at`` is stamped HERE, not when a pre-RTL turn starts.
        _do_rtl's SMART_RTL fallback measures from it ("if the FC is not in
        SMART_RTL 3 s after we asked, fall back"), so starting that clock at the
        top of the turn would fire the fallback against a mode nobody has asked
        for yet, mid-turn.
        """
        if self._pi_return_wanted():
            return                          # the Pi is flying this one
        self._pi_return_since = None
        self._rtl_turn_since = None
        self._about_face_since = None
        self._about_face_target = None
        self._rtl_yaw_started = None
        self._rtl_braking_since = None
        self._rtl_brake_warned = False
        self._rtl_requested_at = time.monotonic()
        self._rtl_fell_back = self._return_mode != "SMART_RTL"
        # The Pi stops routing here and the FC starts. This is the whole
        # handoff, and it is deliberately the LAST thing before the mode
        # change: see _push_oa_type.
        self._push_oa_type(self._avoider.rtl_oa_type)
        if self._return_mode == "SMART_RTL":
            self._set_mode("SMART_RTL")
        else:
            self._set_mode("RTL")
            self._send("rtl")
        self.log.info("returning home via %s", self._return_mode)

    def _home_bearing_deg(self) -> float | None:
        """Bearing to home, nose-relative (+ right), or None if not knowable.

        Home is the ENU origin the fused estimate is expressed in, so the vector
        to it is simply the negated position - no geodetic round trip.

        None rather than 0.0 (which is what _goal_bearing_deg returns when it
        cannot answer) because the two mean different things here: 0.0 reads as
        "already facing home, no turn needed" and would silently skip the turn,
        while None says the question could not be answered and lets the caller
        log why.
        """
        if self._fused is None or not self._fused.valid or self._home is None:
            return None
        east, north = -self._fused.x, -self._fused.y
        if math.hypot(east, north) < 1e-6:
            return None                 # sitting on home; no bearing to face
        forward, left = enu_to_body(east, north, self._fused.yaw)
        return wrap_180(math.degrees(math.atan2(-left, forward)))

    def _rtl_turn_wanted(self) -> bool:
        """Start a pre-RTL turn if home is outside the scanned window.

        Engages on the same criterion as _yaw_gate_ok - beyond fov_half_deg
        (125 deg), i.e. home is in the masked rear the LiDAR never sees - and
        reuses that gate's tuned constants, because it is the same physical
        question asked about a different leg.

        **Usually this does nothing.** An aircraft that finishes a delivery
        already pointed roughly homeward returns exactly as it did before, with
        no added delay. It only fires when home is genuinely behind.

        Returns True if a turn was started (the caller must not command the
        return yet).
        """
        if not self._avoider.yaw_before_rtl:
            return False
        if self._rtl_yaw_spent(time.monotonic()):
            # Stage 1 already spent the whole window. Starting here would send a
            # yaw command, log a turn, and then bail on the very next tick when
            # _rtl_turn_step asks the same question - all while the GCS-side
            # failsafes stay suppressed past the figure that bounds them.
            self.log.warning(
                "pre-RTL turning budget (%.0fs) is already spent - returning "
                "without turning onto home",
                self._avoider.rtl_yaw_total_s,
            )
            return False
        bearing = self._home_bearing_deg()
        if bearing is None:
            self.log.info(
                "no usable home bearing (fix/home unknown) - returning without "
                "turning first"
            )
            return False
        if abs(bearing) <= self._avoider.fov_half_deg:
            return False                # already inside the scan window

        self._rtl_turn_since = time.monotonic()
        if self._rtl_yaw_started is None:
            self._rtl_yaw_started = self._rtl_turn_since
        self._rtl_turn_sent_at = 0.0
        self._rtl_turn_warned = False
        self._rtl_requested_at = None   # nothing commanded yet - see _commit_rtl
        self.log.warning(
            "home is %.0f deg off the nose, behind the %.0f deg LiDAR window - "
            "turning onto it before handing over to %s",
            bearing, self._avoider.fov_half_deg * 2.0, self._return_mode,
        )
        return True

    def _rtl_turn_step(self) -> bool:
        """One tick of the pre-RTL turn. True when the return may be commanded.

        Holding costs no command: the aircraft is already in GUIDED at a
        waypoint it has reached, so simply not commanding anything leaves it
        station-keeping while it turns - the same trick _yaw_gate_ok uses. No
        mode change is added by this stage, which matters at the hover-complete
        caller, whose comment warns that two mode changes in one tick risk a
        refused GUIDED stranding the aircraft over the customer.

        Pilot override is not handled here: step() checks it before dispatching
        the state machine, so this never runs once the human has the aircraft.
        """
        now = time.monotonic()
        bearing = self._home_bearing_deg()

        if bearing is None:
            # The fix went away mid-turn. Do not keep turning against an answer
            # we no longer have.
            self.log.warning("lost the home bearing mid-turn - returning now")
            return True

        if abs(bearing) <= self._avoider.yaw_release_deg:
            self.log.info(
                "nose is on home (%.0f deg off after %.1fs) - returning",
                bearing, now - self._rtl_turn_since,
            )
            return True

        if (now - self._rtl_turn_since) > self._avoider.rtl_yaw_timeout_s \
                or self._rtl_yaw_spent(now):
            # Unlike _yaw_gate_ok, this does NOT hold. That gate is deciding
            # whether to fly at a waypoint, where holding is the safe answer.
            # Here the aircraft is going home, and refusing to start would keep
            # it airborne burning battery until a human noticed. Coming home
            # unturned is worse than coming home turned, and far better than
            # not coming home.
            if not self._rtl_turn_warned:
                self._rtl_turn_warned = True
                self.log.error(
                    "STILL %.0f deg off home after %.0fs - the aircraft is not "
                    "turning. RETURNING ANYWAY, with the unscanned rear leading. "
                    "Check WP_YAW_BEHAVIOR is 1 so the FC turns on the way.",
                    bearing, self._avoider.rtl_yaw_timeout_s,
                )
            return True

        # CONDITION_YAW is a discrete command, not a setpoint: re-sending it at
        # the 10 Hz loop rate restarts the turn every tick and it never lands.
        if (now - self._rtl_turn_sent_at) > 2.0:
            self._rtl_turn_sent_at = now
            self._send(
                "yaw",
                angle=abs(bearing),
                direction=1 if bearing > 0.0 else -1,
                rate=self._avoider.yaw_rate_deg_s,
            )
        self._status_message = (
            f"turning onto home before RTL: {bearing:+.0f} deg to go"
        )
        return False

    def _rtl_yaw_spent(self, now: float) -> bool:
        """Has the shared pre-RTL turning budget run out?

        Both turn stages ask this. See ``rtl_yaw_total_s`` for why the budget is
        shared rather than one timeout each: it is the failsafe-suppression
        window, and suppression does not care which stage is using it.
        """
        started = self._rtl_yaw_started
        return started is not None and \
            (now - started) > self._avoider.rtl_yaw_total_s

    def _about_face_wanted(self) -> bool:
        """Start the post-delivery about-face. True if one was started.

        Unconditional where ``_rtl_turn_wanted`` is conditional: there is no
        bearing test here to pass or fail, so this returns False only when the
        feature is switched off or the aircraft has no attitude to measure the
        turn against.
        """
        if not self._avoider.rtl_about_face:
            return False
        fused = self._fused
        if fused is None or not fused.valid:
            # Refusing here is not a safety call, it is an honesty one. The turn
            # is verified against the yaw estimate and nothing else, so with no
            # estimate the aircraft could sit commanding a turn it has no way to
            # confirm until the budget expires. Go home instead.
            self.log.warning(
                "no valid attitude estimate - skipping the post-delivery "
                "about-face and returning now"
            )
            return False

        now = time.monotonic()
        self._about_face_since = now
        self._rtl_yaw_started = now
        self._about_face_sent_at = 0.0
        self._about_face_warned = False
        self._about_face_yaw = fused.yaw
        self._about_face_turned = 0.0
        # Pick the shorter path toward home so the about-face doesn't
        # spin AWAY from it (the 360-deg spin seen on 2026-09-13).
        bearing = self._home_bearing_deg()
        if bearing is not None:
            self._about_face_direction = 1 if bearing > 0.0 else -1
        else:
            self._about_face_direction = 1  # no GPS; pick one
        # Latch WHERE the nose has to end up, once, here. _about_face_step then
        # flies an error to this fixed heading instead of counting how far the
        # aircraft has moved, which is what makes the turn repeatable.
        self._about_face_target = wrap_pi(
            fused.yaw + self._about_face_direction * math.pi
        )
        self._rtl_requested_at = None   # nothing commanded yet - see _commit_rtl
        self.log.info(
            "delivery complete - turning 180 deg about-face (%s) at %.0f deg/s: "
            "heading %.0f -> %.0f deg, then handing over to %s",
            "CW" if self._about_face_direction == 1 else "CCW",
            self._avoider.yaw_rate_deg_s,
            math.degrees(fused.yaw) % 360.0,
            math.degrees(self._about_face_target) % 360.0,
            self._return_mode,
        )
        return True

    def _about_face_step(self) -> bool:
        """One tick of the about-face. True when the next stage may proceed.

        Completion is measured as the ERROR TO A FIXED TARGET HEADING latched
        by ``_about_face_wanted``, not as an integral of how far the aircraft
        has moved. That distinction is the whole fix for the turn landing
        somewhere different every flight, and it is worth spelling out.

        The obvious implementation - and what this did until 2026-09-19 - is to
        accumulate ``abs(yaw - last_yaw)`` each tick and stop at 180. Taking the
        absolute value RECTIFIES the estimator's noise: jitter adds to the total
        whichever way it jitters, so at the 10 Hz loop rate the counter runs
        fast by an amount that depends only on how noisy the yaw estimate
        happened to be. The turn therefore stopped short, by a different margin
        on every flight. Worse, the 2 s resend recomputed ``remaining`` from
        that inflated counter and re-aimed the flight controller short as well,
        so the error compounded instead of washing out.

        An error to a latched heading has no such term. It is the difference of
        two absolute angles, so noise shows up as noise (bounded, zero-mean)
        rather than as drift, and every resend re-aims at the same fixed target
        - a late or lost sample costs nothing, where the accumulator lost that
        movement permanently. It also bounds OVERSHOOT, which the accumulator
        could not detect at all: past the target the error simply changes sign.

        This is the same closed-loop shape ``_rtl_turn_step`` already uses to
        seek the home bearing. Stage 1 and stage 2 now differ only in what they
        are aiming at.

        Holding costs no command, as in ``_rtl_turn_step``: the aircraft is
        already in GUIDED at a waypoint it has reached, so issuing nothing
        leaves it station-keeping while it turns.
        """
        now = time.monotonic()
        fused = self._fused

        if fused is None or not fused.valid or self._about_face_target is None:
            # Same reasoning as _rtl_turn_step losing the home bearing: do not
            # keep turning against an answer we no longer have.
            self.log.warning(
                "lost the attitude estimate mid about-face (%.0f deg turned) - "
                "returning now", self._about_face_turned,
            )
            return True

        # How far the nose still has to go, as the shortest rotation onto the
        # target heading. Positive = the target is clockwise of us (yaw is
        # CW-positive from North here - see enu_to_body).
        error_deg = wrap_180(
            math.degrees(wrap_pi(self._about_face_target - fused.yaw))
        )

        # Progress, for the log and the status line only. SIGNED in the
        # commanded direction, so estimator noise cancels instead of
        # accumulating, and a turn going the wrong way reads negative.
        self._about_face_turned += self._about_face_direction * math.degrees(
            wrap_pi(fused.yaw - self._about_face_yaw)
        )
        self._about_face_yaw = fused.yaw

        if abs(error_deg) <= self._avoider.about_face_tol_deg:
            self.log.info(
                "about-face complete: %.0f deg off target after %.1fs "
                "(%.0f deg turned)",
                error_deg, now - self._about_face_since,
                self._about_face_turned,
            )
            return True

        if self._rtl_yaw_spent(now):
            # As in _rtl_turn_step, this does NOT hold. The aircraft is going
            # home; refusing to start would keep it airborne burning battery
            # until a human noticed. Coming home half-turned is worse than
            # coming home turned, and far better than not coming home.
            if not self._about_face_warned:
                self._about_face_warned = True
                self.log.error(
                    "about-face stalled %.0f deg off target after %.0fs "
                    "(%.0f of 180 deg turned) - the aircraft is not turning. "
                    "RETURNING ANYWAY. Check WP_YAW_BEHAVIOR is 1 so the FC "
                    "turns on the way.",
                    error_deg, self._avoider.rtl_yaw_total_s,
                    self._about_face_turned,
                )
            return True

        # CONDITION_YAW is sent RELATIVE (param4=1), so a resend commands a
        # whole new turn from wherever the nose is now. Sending the live error
        # is exactly right for that: each resend re-aims at the latched target
        # from the current heading, so the command is self-correcting rather
        # than additive.
        if (now - self._about_face_sent_at) > 2.0:
            self._about_face_sent_at = now
            # While the target is still roughly opposite the nose the SIGN of
            # error_deg is decided by noise - at exactly 180 deg either way
            # round is the same distance - and following it would let the
            # aircraft reverse direction between resends and sit there. Hold
            # the direction picked at the start (the short way toward home)
            # until the turn is more than half done, then track the error so
            # an overshoot is corrected rather than chased the long way round.
            direction = (
                self._about_face_direction if abs(error_deg) > 90.0
                else (1 if error_deg > 0.0 else -1)
            )
            self._send(
                "yaw",
                angle=abs(error_deg),
                direction=direction,
                rate=self._avoider.yaw_rate_deg_s,
            )
        self._status_message = (
            f"about-face before RTL: {abs(error_deg):.0f} deg to go"
        )
        return False

    def _enter_land(self) -> None:
        """Put the aircraft down where it is, slowly.

        The descent rate is the FC's LAND_SPEED (30 cm/s, measured), not
        anything this node sets: LAND is the one autonomous descent ArduPilot
        flies entirely on its own sensors, which is the point of using it
        rather than commanding a goto at a lower altitude against an altitude
        estimate that may be what went wrong.
        """
        self._phase = MissionPhase.LAND
        self._land_sent_at = time.monotonic()
        self._send("land")
        self.log.info(
            "LAND commanded%s", f" ({self._land_reason})" if self._land_reason else ""
        )

    def _active_waypoint(self) -> Waypoint | None:
        if 0 <= self._current_wp < self._mission.count:
            return self._mission.waypoints[self._current_wp]
        return None

    def _distance_to_wp(self, wp: Waypoint) -> float:
        if self._fused is None:
            return math.inf
        return math.hypot(wp.x_m - self._fused.x, wp.y_m - self._fused.y)

    def _yaw_gate_clear(self) -> None:
        self._yaw_gate_since = None
        self._yaw_gate_sent_at = 0.0
        self._yaw_gate_stuck = False

    def _yaw_gate_ok(self) -> bool:
        """Turn the LiDAR onto the path before flying it.

        The window is 250 deg front-referenced, so bearings beyond
        +/-fov_half_deg (125 deg) are in the masked rear - never scanned. Those
        sectors are streamed to the FC as 65535 (unknown), and ArduPilot's
        proximity database treats unknown as *clear*: BendyRuler would route
        into them with complete confidence. The aircraft also cannot reverse
        out of trouble there, and at a 2 m ceiling it cannot climb over.

        So when the target is behind, hold and yaw onto it first. Holding needs
        no command: simply not issuing the goto leaves the aircraft on its last
        one, which it has already reached, so it station-keeps in GUIDED while
        it turns.

        Hysteresis is wide on purpose - it engages beyond 125 deg and releases
        inside yaw_release_deg (60 deg) - so the gate cannot oscillate on its
        own threshold, and the path ends up covered by the dense middle of the
        window rather than its extreme edge.

        Returns True when it is safe to translate.
        """
        if not self._avoider.yaw_before_move:
            return True
        if self._fused is None or not self._fused.valid or self._home is None:
            return True                 # bearing unknown; gating on a guess is worse
        if self._active_waypoint() is None:
            return True

        bearing = self._goal_bearing_deg()
        now = time.monotonic()

        if self._yaw_gate_since is None:
            if abs(bearing) <= self._avoider.fov_half_deg:
                return True             # already looking at it
            self._yaw_gate_since = now
            self._yaw_gate_sent_at = 0.0
            self._yaw_gate_stuck = False
            self.log.warning(
                "waypoint %.0f deg off the nose is behind the %.0f deg LiDAR "
                "window - holding to turn onto it before flying",
                bearing, self._avoider.fov_half_deg * 2.0,
            )

        if abs(bearing) <= self._avoider.yaw_release_deg:
            self.log.info(
                "turned onto the path (%.0f deg off the nose after %.1fs) - "
                "LiDAR now covers it, resuming",
                bearing, now - self._yaw_gate_since,
            )
            self._yaw_gate_clear()
            return True

        if (now - self._yaw_gate_since) > self._avoider.yaw_timeout_s:
            # Deliberately does NOT give up and fly blind: defeating a safety
            # gate on a timer defeats the gate. It holds and says so, loudly
            # and once, because the failure that actually hurts is a SILENT
            # hold - see the 2026-08-31 BRAKE deadlock. The pilot has the
            # transmitter, and avoidance_yaw_before_move turns this off.
            if not self._yaw_gate_stuck:
                self._yaw_gate_stuck = True
                self.log.error(
                    "STILL %.0f deg off the nose after %.0fs - the aircraft is "
                    "not turning. HOLDING rather than flying into the unscanned "
                    "rear. Take manual control, or set "
                    "avoidance_yaw_before_move=false to disable this gate.",
                    bearing, self._avoider.yaw_timeout_s,
                )
            self._status_message = (
                f"HELD: cannot turn onto the path ({bearing:+.0f} deg) - "
                f"rear is unscanned"
            )
            return False

        # Re-command the turn only occasionally. CONDITION_YAW is a discrete
        # command, not a setpoint; re-sending it at the 10 Hz loop rate would
        # restart the turn every tick and it would never finish.
        if (now - self._yaw_gate_sent_at) > 2.0:
            self._yaw_gate_sent_at = now
            self._send(
                "yaw",
                angle=abs(bearing),
                direction=1 if bearing > 0.0 else -1,
                rate=self._avoider.yaw_rate_deg_s,
            )
        self._status_message = (
            f"turning onto the path: {bearing:+.0f} deg to go "
            f"(rear {360.0 - self._avoider.fov_half_deg * 2.0:.0f} deg unscanned)"
        )
        return False

    def _ensure_guided(self) -> bool:
        """Put the FC back in GUIDED before commanding motion.

        ``goto`` and ``velocity`` record GUIDED in ``_commanded_mode`` but,
        not being ``_MODE_ONLY_COMMANDS``, they never emit a SET_MODE frame.
        So once avoidance had braked, *nothing* in the navigate path ever asked
        for GUIDED again: the FC sat in BRAKE ignoring a 10 Hz stream of
        setpoints while ``_note_mode_refusal`` reported "FC refused GUIDED and
        stayed in BRAKE" once a second, until a human took the aircraft. That
        is the 2026-08-31 delivery flight, and it made every avoidance brake a
        one-way trip.

        Only BRAKE is recovered from, and only because BRAKE is the mode this
        node put the aircraft in itself - a pilot-selected mode latches
        ``_pilot_override`` long before this is reached, and AUTO/RTL are
        someone else's to own.

        Returns False *only* while sitting in BRAKE waiting for the switch.
        Every other mode is passed through exactly as before: gating on
        ``mode == "GUIDED"`` instead looked tidier but silently stopped the
        mission dead whenever the mode was merely unknown - no heartbeat yet,
        or an FC-run mode - which tests/test_delivery.py caught immediately.
        Narrow the fix to the mode that actually swallows the setpoints.

        ``_set_mode`` is self-pacing (``_MODE_ONLY_COMMANDS`` plus
        ``_mode_resend_s``) and a no-op when the FC is already there, so
        calling this every tick does **not** re-command the mode at loop rate.
        """
        if self._mode == "BRAKE":
            self._set_mode("GUIDED")
            return False
        return True

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
                **self._scan_state_fields(),
            ),
        )

    def _scan_state_fields(self) -> dict:
        stage = self._scan_stage if self._phase == MissionPhase.HOVER else ""
        left = 0.0
        if stage and self._hover_until is not None:
            left = max(0.0, self._hover_until - time.monotonic())
        out = {"scan_stage": stage, "scan_left_s": round(left, 1),
               "handshake_open": self._handshake_open()}
        fix = self._phone_fix
        if stage and fix is not None and self._home is not None:
            lat, lon = enu_to_geodetic(fix.east, fix.north, self._home[0], self._home[1])
            out.update(phone_lat=lat, phone_lon=lon, phone_std_m=round(fix.std_m, 2))
        return out

    # -- avoidance -----------------------------------------------------------
    def _avoid_decision(self) -> tuple[str, float]:
        """Avoidance decision, forced CLEAR when disabled or in STANDBY.

        Every consumer - NAVIGATE, AVOID, MANUAL brake, the RTL return leg -
        goes through here, so the takeoff gate cannot be bypassed by a path
        that reads the avoider directly.
        """
        if not self._avoid_enabled or self._avoid_standby():
            return CLEAR, math.inf
        return self._avoider.evaluate(self._obstacles)

    def _avoid_standby(self) -> bool:
        """True while the takeoff gate is holding avoidance off."""
        return self._avoid_min_alt > 0.0 and not self._avoid_airborne

    def _update_avoid_airborne(self) -> None:
        """Maintain the latched takeoff gate. Called first thing every tick."""
        if self._avoid_min_alt <= 0.0:
            return
        if not self._armed:
            if self._avoid_airborne:
                self.log.info("disarmed - avoidance back to standby")
            self._avoid_airborne = False
            return
        if not self._avoid_airborne and self._airborne_for_avoidance():
            self._avoid_airborne = True
            self.log.info(
                "airborne at %.1f m (%s) - avoidance ACTIVE",
                self._fused.alt_rel_m if self._fused is not None else float("nan"),
                self._phase.name,
            )

    def _airborne_for_avoidance(self) -> bool:
        """Has the aircraft left the ground far enough to hand it to avoidance?

        Called only while ARMED and not yet latched; returning True latches
        avoidance ACTIVE until disarm. Available: ``self._fused`` (may be None;
        ``.valid``, ``.alt_rel_m`` metres above home), ``self._phase``
        (MissionPhase), ``self._avoid_min_alt`` (avoidance_min_alt_m).
        """
        fused = self._fused
        if fused is None or not fused.valid:
            return False                  # never latch blind
        if self._phase in (MissionPhase.ARMING, MissionPhase.TAKEOFF):
            return False                  # let the climb and settle finish first
        return fused.alt_rel_m >= self._avoid_min_alt

    def _front_obstacle(self) -> Obstacle | None:
        if self._obstacles is None or self._obstacles.count == 0:
            return None
        ahead = [
            o for o in self._obstacles.obstacles
            if abs(wrap_180(o.bearing_deg)) <= self._avoider.sector_deg
        ]
        return min(ahead, key=lambda o: o.distance_m, default=None)

    def _predicted_dodge(self, view: SectorView) -> str:
        """The escape route the panel should show, from the live router.

        _publish_avoidance draws this before the aircraft is dodging, so it has
        to ask whichever router would actually fly it. Asking the three-cone
        dodge while VFH+ is enabled paints a side the aircraft will not take -
        and paints one at all in the case VFH+ would report TRAPPED.
        """
        if not self._avoider.vfh_enabled:
            return self._avoider.dodge(view)
        heading = self._avoider.choose_heading(
            self._obstacles, self._goal_bearing_deg(), self._dodge_heading)
        if heading is None:
            return DODGE_TRAPPED
        return DODGE_RIGHT if heading > 0.0 else DODGE_LEFT

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
            dodge = self._predicted_dodge(view)
        else:
            dodge = DODGE_NONE

        status = {"clear": "CLEAR", "slow": "SLOW", "stop": "BRAKE"}[decision]
        standby = self._avoid_enabled and self._avoid_standby()
        if not self._avoid_enabled:
            status = "OFF"
        elif standby:
            status = "STANDBY"

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
        elif standby:
            reason = f"standby until airborne (>= {self._avoid_min_alt:.1f} m)"
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

    #: Which path planner the FLIGHT CONTROLLER runs. 0 = none, 1 = BendyRuler.
    _OA_TYPE_PARAM = "OA_TYPE"

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
        # The Pi routes everything it flies itself, so the FC's planner starts
        # switched off and is handed the job at _commit_rtl. See _push_oa_type.
        self._push_oa_type(0)
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

    def _push_oa_type(self, oa_type: int) -> None:
        """Hand FC-side path planning on or off, so exactly one router steers.

        The operator's requirement was "make sure avoidance works in RTL
        also". Until 2026-09-21 it did not, in the only sense that matters:
        _rtl_avoid_step WATCHES the obstacle field and hard-brakes at
        avoidance_stop_m, but nothing STEERS the return. The Pi deliberately
        does not (see _rtl_obstacle_response - in RTL the FC discards
        SET_POSITION_TARGET_LOCAL_NED, and owning the mode is the 2026-08-22
        lockout shape), and the FC could not, because OA_TYPE measured 0.

        So the return leg had a brake and no steering. The aircraft would stop
        dead in front of an obstacle, hold for rtl_brake_max_s, and hand back.

        WHY A HANDOFF RATHER THAN JUST SETTING OA_TYPE=1. ArduPilot has no
        per-mode object-avoidance switch: OA_TYPE applies to AUTO, GUIDED and
        RTL alike. Setting it to 1 and leaving it there puts BendyRuler and
        this node's cruise band on the same aircraft at the same time, which
        is the two-routers-arguing failure the project has warned about since
        the OA_TYPE measurement was first taken. So ownership is switched
        explicitly: 0 while the Pi flies (pushed at link-up), rtl_oa_type on
        the way into the return (pushed by _commit_rtl).

        That split is also robust to the one thing here not established by
        measurement. Whether BendyRuler would engage against a GUIDED
        *velocity* setpoint, as opposed to a position target, is a question
        about ArduPilot internals this project has not measured. It does not
        need to be answered: during NAVIGATE the planner is off either way, so
        there is exactly one router under either reading.

        A push that never lands leaves the aircraft exactly where it is today
        - Pi routing in NAVIGATE, brake-only in RTL - which is the safe
        degradation aerix-gcs's Restart=always demands. OA_TYPE only
        instantiates its backend AT BOOT, so this can enable planning on an
        FC that was booted with OA_TYPE=1 and can only ever disable it on one
        that was not. That one-time enable plus reboot is an operator action,
        not something a node should do to an aircraft.
        """
        if not self._avoider.rtl_oa_handoff:
            return
        if self._oa_type_pushed == oa_type:
            return                          # do not re-push at 10 Hz
        self._oa_type_pushed = oa_type
        self._send("set_param", name=self._OA_TYPE_PARAM, value=int(oa_type))
        self.log.info(
            "%s = %d - %s now routes", self._OA_TYPE_PARAM, oa_type,
            "the flight controller" if oa_type else "the Pi")

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
            if self._phase == MissionPhase.HOVER and self._scan_stage:
                # Follow-me during a person scan: the phone's GPS moves the
                # SEARCH, it does not abandon it. Ignored once locked - the
                # handshake window is latched on the person in view.
                e, n = geodetic_to_enu(lat, lon, self._home[0], self._home[1])
                self._locator.add_gps(e, n, time.time())
                if self._scan_stage == "search":
                    self._scan_set_centre(e, n, "follow-me (phone GPS)")
                    return ServiceResponse(True, f"search re-centred on {lat:.6f},{lon:.6f}")
                return ServiceResponse(
                    True, f"person scan {self._scan_stage}: holding, phone fix noted")
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
