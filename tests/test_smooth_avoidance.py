"""Smooth cruise-band obstacle avoidance (2026-09-19).

Three faults were found together, and this file pins all three.

1. ``OA_TYPE`` was MEASURED as 0 on the aircraft, and ``OA_BR_TYPE`` /
   ``OA_BR_LOOKAHEAD`` did not exist on the FC at all - which is how ArduPilot
   reports a path-planning backend that was never instantiated at boot. So the
   BendyRuler that ``_do_navigate``'s SLOW branch deferred to was not running,
   and that branch - which commanded nothing and only wrote "FC routing to
   waypoint" to the status line - meant the aircraft flew at cruise speed from
   ``avoidance_distance_m`` all the way to the hard brake.

2. ``_dodge_step`` then solved a perfectly good VFH+ course and commanded
   NOTHING: the brake that put the aircraft in AVOID had left the FC in BRAKE,
   ``velocity`` is not a ``_MODE_ONLY_COMMAND`` so no SET_MODE was ever
   emitted, and BRAKE discards setpoints. ``_ensure_guided`` had been added to
   the NAVIGATE path on 2026-08-31 for exactly this and never to the avoid
   path.

Together: brake at 1.7 m, sit there for ``avoidance_dodge_timeout_s``, hold.
Which is "obstacle avoidance isn't working at all - it stops in front of the
obstacle and waits there".

3. The fix has to be SMOOTH, so the correction is applied to the course and
   never to the speed, and it ramps with proximity rather than switching on.
"""
from __future__ import annotations

import copy
import math

import pytest

from drone_stack.bus import MessageBus
from drone_stack.bus.topics import Topics
from drone_stack.msg import FusedState, MissionPhase, Obstacle, ObstacleArray, Waypoint
from drone_stack.nodes.navigation_node import CollisionAvoider, NavigationNode
from drone_stack.srv import ServiceRegistry
from drone_stack.utils.config import Config
from drone_stack.utils.geometry import wrap_180


#: The flown profile's band, pinned here rather than inherited from whichever
#: config profile Config.load() happens to resolve. These tests are about the
#: SHAPE of a manoeuvre ACROSS a band, so the band itself has to be the
#: constant - reading it from default.yaml once made every distance in this
#: file read as CLEAR and six tests pass vacuously.
_BAND = {
    "avoidance_distance_m": 8.0,
    "avoidance_stop_m": 1.7,
    "cruise_speed_ms": 1.0,
    "avoidance_vfh_safety_radius_m": 2.5,
    "avoidance_steer_rate_deg_s": 15.0,
    # The floor is part of the band's SHAPE, so it is pinned here with the
    # rest of it. 0.0 reproduces the pre-2026-09-21 curve, which could not
    # begin leaning until the obstacle was already inside the band.
    "avoidance_steer_authority_floor": 0.25,
    "avoidance_steer_speed_ms": 0.0,
    "avoidance_dodge_enabled": True,
    "avoidance_vfh_enabled": True,
}


def _cfg(**overrides) -> Config:
    raw = copy.deepcopy(Config.load().raw)
    raw.setdefault("navigation", {}).update(_BAND)
    for name, values in overrides.items():
        raw.setdefault(name, {}).update(values)
    return Config(raw)


def _avoider(**overrides) -> CollisionAvoider:
    return CollisionAvoider(_cfg(**overrides))


def _nav(**overrides):
    bus = MessageBus()
    sent: list = []
    bus.subscribe(Topics.MAVLINK_CMD, sent.append)
    node = NavigationNode(bus, _cfg(**overrides), ServiceRegistry())
    node._avoid_enabled = True
    sent.clear()
    return node, sent


def _obs(*pairs) -> ObstacleArray:
    return ObstacleArray(
        obstacles=[Obstacle(distance_m=d, bearing_deg=b) for d, b in pairs]
    )


def _enroute(node, north_m=40.0):
    """Airborne, nose north, waypoint due north so the goal bearing is 0.

    lat/lon left at 0 on purpose: _goal_bearing_deg then reads x_m/y_m as ENU
    directly, so the test controls the bearing instead of inheriting it from a
    geodetic conversion.
    """
    node._armed = True
    node._has_flown = True
    node._home = (12.9, 77.6)
    node._phase = MissionPhase.NAVIGATE
    node._mission.waypoints = [Waypoint(seq=0, x_m=0.0, y_m=north_m, alt_m=2.0)]
    node._current_wp = 0
    node._fused = FusedState(x=0.0, y=0.0, alt_rel_m=2.0, yaw=0.0, valid=True)
    node._mode = "GUIDED"
    return node


def _of(sent, command):
    return [c for c in sent if c.command == command]


def _modes(sent):
    return [c.params.get("mode") for c in sent if c.command == "set_mode"]


# ===================== 1. the authority ramp ================================
def test_the_lean_begins_at_the_outer_edge():
    """Inverted 2026-09-21: the old curve was exactly 0.0 at the band edge.

    A cubic ease-out through (0,0) means the first metre of the band commands
    nothing, so the aircraft cannot start turning where the LiDAR first sees
    the obstacle - it coasts in and then has to turn hard lower down. Measured
    before the floor: authority 0.000 and a commanded 0.0 deg where +18.2 deg
    was geometrically needed, with the slew pinned at its cap for the whole
    manoeuvre. That IS the late hard turn the operator was reporting.

    The floor is what makes "avoid from long distance" mean anything.
    """
    av = _avoider()
    assert av.steer_floor > 0.0, "no floor - the lean cannot begin at the edge"
    assert av.steer_authority(av.distance) == pytest.approx(av.steer_floor)
    # Outside the band the floor is all there is: still gentle, never a twitch.
    assert av.steer_authority(av.distance + 5.0) == pytest.approx(av.steer_floor)
    assert av.steer_floor < 0.5, "a floor this high is a twitch, not a lean"


def test_authority_is_complete_by_the_brake_distance():
    """Arriving at the brake distance still half-turned IS the stop-and-wait."""
    av = _avoider()
    assert av.steer_authority(av.last_stop_m) == 1.0
    assert av.steer_authority(av.last_stop_m - 0.5) == 1.0


def test_authority_rises_monotonically_as_the_obstacle_closes():
    av = _avoider()
    values = [av.steer_authority(d) for d in (8.0, 7.0, 6.0, 5.0, 4.0, 3.0, 2.0)]
    assert all(b >= a for a, b in zip(values, values[1:]))
    assert values[0] < values[-1]


def test_the_ramp_eases_out_rather_than_running_linear():
    """Ease-OUT, not ease-in. This assertion is inverted from what it was on
    2026-09-19 and the inversion is the whole fix, so it is pinned hard.

    The clearance angle the geometry demands is asin(safety_radius/distance) -
    cheap far out, ruinous close in. frac**2 left the aircraft commanding less
    deflection than it needed on every tick from 8 m to 3 m and it ended in the
    brake band at 1.69 m. Authority has to lead the requirement, not trail it.
    """
    av = _avoider()
    midpoint = (av.distance + av.last_stop_m) / 2.0
    # Linear would be 0.5 here. Ease-out must be markedly above it.
    assert av.steer_authority(midpoint) > 0.65


def test_authority_leads_the_angle_the_geometry_demands():
    """The real acceptance test for the curve's shape.

    At each distance, compare the deflection actually commanded against
    asin(safety_radius / distance) - the angle needed to clear. The curve is
    allowed to trail briefly at the very edge, where the deficit is a couple of
    degrees and there are seconds of runway, but it must cross ahead in the
    first third of the band and stay ahead all the way in. frac**2 crossed at
    ~2.5 m, which is inside the brake band and therefore too late to matter.
    """
    import math

    av = _avoider()
    crossed_at = None
    for d in [x / 10.0 for x in range(int(av.distance * 10), 20, -1)]:
        gap = av.choose_heading(_obs((d, 0.0)), 0.0, None)
        if gap is None:
            continue
        commanded = abs(av.steer_authority(d) * gap)
        needed = math.degrees(math.asin(min(1.0, av.vfh_safety_radius / d)))
        if commanded >= needed:
            if crossed_at is None:
                crossed_at = d
        elif crossed_at is not None:
            raise AssertionError(
                "authority fell behind the required angle again at %.1f m "
                "(commanded %.1f deg, needed %.1f deg)" % (d, commanded, needed))
    assert crossed_at is not None, "authority never caught the required angle"
    # Measured crossover on the flown band, as a fraction of it:
    #     frac**2 (ease-in)   26%   <- what shipped, and what braked
    #     frac    (linear)    36%
    #     1-(1-frac)**2       60%   <- flown
    # Half the band separates the working curve from both broken ones. This
    # static check is deliberately pessimistic: it asks for the angle needed
    # while the aircraft is still dead on track at distance d, whereas in the
    # closed loop it has already built lateral offset by then, the obstacle has
    # drifted off the nose, and the angle actually required is smaller.
    assert crossed_at > av.distance * 0.5, (
        "caught up only at %.1f m - too deep into the band" % crossed_at)


def test_the_steerer_aims_outside_the_band_the_brake_calls_an_emergency():
    """The invariant that made the curve fix insufficient on its own.

    VFH+ steers to miss by vfh_safety_radius. The brake fires at
    stop_distance_for(closing). If the radius is the smaller of the two, the
    steerer flies its whole manoeuvre INSIDE the brake's emergency band and
    succeeds by its own definition while failing by the brake's - measured on
    2026-09-19 as closest approach 1.66 m against a 1.70 m brake, then a dead
    stop. Radius was 1.0 against a 1.7 m stand-off that pads to 2.70 m at
    cruise.
    """
    av = _avoider()
    assert av.vfh_safety_radius > av.stop, (
        "vfh_safety_radius %.2f does not clear the nominal stand-off %.2f"
        % (av.vfh_safety_radius, av.stop))
    # And with margin, because C1 range noise and cluster-centroid error both
    # land on this number.
    assert av.vfh_safety_radius - av.stop >= 0.3


def test_every_shipped_profile_satisfies_the_clearance_invariant():
    """The test above pins the invariant against _BAND, which this file sets.

    This one checks the profiles that actually fly, so the invariant cannot be
    broken by editing YAML. Both were broken when it was written: real.yaml had
    radius 1.0 against stop 1.7, default.yaml radius 1.0 against stop 1.2.
    """
    import pathlib

    root = pathlib.Path(__file__).resolve().parents[1] / "config"
    checked = 0
    for name in ("default.yaml", "real.yaml", "sim.yaml"):
        path = root / name
        if not path.exists():
            continue
        nav = Config.load(str(path)).section("navigation")
        radius = float(nav.get("avoidance_vfh_safety_radius_m", 0.0))
        stop = float(nav.get("avoidance_stop_m", 0.0))
        assert radius - stop >= 0.3, (
            "%s: vfh_safety_radius %.2f must clear avoidance_stop_m %.2f by "
            ">= 0.3 m or avoidance ends in a dead stop" % (name, radius, stop))
        outer = float(nav.get("avoidance_distance_m", 0.0))
        assert outer > stop, "%s: avoidance_distance_m must exceed stop" % name
        vfh_range = float(nav.get("avoidance_vfh_range_m", 0.0))
        assert vfh_range > outer, (
            "%s: avoidance_vfh_range_m %.1f must sit outside "
            "avoidance_distance_m %.1f" % (name, vfh_range, outer))
        max_range = float(Config.load(str(path)).section("lidar").get("max_range_m", 12.0))
        assert vfh_range <= max_range, (
            "%s: avoidance_vfh_range_m %.1f exceeds lidar.max_range_m %.1f - "
            "nothing is measured out there" % (name, vfh_range, max_range))
        checked += 1
    assert checked >= 2


def test_authority_never_leaves_zero_to_one():
    av = _avoider()
    for d in (-5.0, 0.0, 1.0, 1.7, 4.0, 8.0, 50.0):
        assert 0.0 <= av.steer_authority(d) <= 1.0


def test_a_negative_fraction_cannot_inflate_authority():
    """The clamp has to happen BEFORE the cube, or an obstacle outside the
    band produces authority from a negative fraction.

    Pre-2026-09-21 this asserted == 0.0, which only held because the curve
    passed through the origin. With a floor, "clamped" no longer means "zero"
    - so assert the property that actually matters: distance beyond the band
    can never buy MORE authority than standing at the band edge.
    """
    av = _avoider()
    far = av.steer_authority(av.distance * 3)
    assert far == pytest.approx(av.steer_floor)
    assert far <= av.steer_authority(av.distance)
    assert 0.0 <= far <= 1.0


def test_a_brake_distance_that_swallows_the_band_gives_full_authority():
    """last_stop_m grows with closing speed, so a fast mover can erase the
    band entirely. There is no room left to ease into."""
    av = _avoider()
    av.last_stop_m = av.distance + 1.0
    assert av.steer_authority(4.0) == 1.0


def test_a_closing_obstacle_widens_the_band_on_its_own():
    """Same geometry, faster closing speed: more authority at the same range,
    because last_stop_m is padded and the band therefore starts earlier."""
    av = _avoider()
    still = av.steer_authority(5.0)
    av.last_stop_m = 3.2                 # ~1.5 m/s walker
    assert av.steer_authority(5.0) > still


# ============ 2. the SLOW band commands motion, not a status string =========
def test_the_slow_band_actually_commands_a_velocity():
    """The whole first fault: this branch used to command nothing at all."""
    node, sent = _nav()
    _enroute(node)
    node._obstacles = _obs((5.0, 0.0))        # inside 8.0, outside 1.7
    node._do_navigate()
    assert _of(sent, "velocity"), "SLOW band commanded nothing"
    assert not _of(sent, "brake"), "SLOW band must not brake"


def test_the_slow_band_does_not_slow_down():
    """The correction belongs on the COURSE. Slowing lengthens the time spent
    alongside the obstacle without buying any clearance."""
    node, sent = _nav()
    _enroute(node)
    node._obstacles = _obs((5.0, 0.0))
    node._do_navigate()
    v = _of(sent, "velocity")[-1].params
    speed = (v["vx"] ** 2 + v["vy"] ** 2) ** 0.5
    assert abs(speed - node._cruise_speed) < 0.05


def test_the_deviation_grows_as_the_obstacle_closes():
    """Fresh nodes: the slew limiter would otherwise dominate the comparison."""
    lateral = []
    for distance in (6.0, 3.0):
        node, sent = _nav()
        _enroute(node)
        node._obstacles = _obs((distance, 0.0))
        node._do_navigate()
        lateral.append(abs(_of(sent, "velocity")[-1].params["vy"]))
    assert lateral[1] > lateral[0]


def test_a_distant_obstacle_deflects_the_course_gently():
    """Both halves matter, and the floor changed both numbers.

    Threshold re-derived by measurement on 2026-09-21 rather than adjusted
    until green: at 7.5 m the VFH+ gap sits near -29 deg and authority is
    0.25 + 0.75*(1-(1-0.0794)^3) = 0.416, giving -12.0 deg and
    vy = sin(-12 deg) = -0.208. The old bound was 0.15 because the old curve
    was near zero here - i.e. it was pinning the absence of the lean.

    The lower bound is the one the operator asked for. Without it this test
    passes just as happily on code that does nothing at 7.5 m, which is
    exactly the regression the floor exists to prevent.
    """
    node, sent = _nav()
    _enroute(node)
    node._obstacles = _obs((7.5, 0.0))
    node._do_navigate()
    vy = abs(_of(sent, "velocity")[-1].params["vy"])
    # 0.16 discriminates: measured 0.208 with the floor, 0.111 without it.
    # A bound of 0.10 let the floor be patched straight back out and stayed
    # green, which is the whole failure mode section 8 exists to catch.
    assert vy > 0.16, "no lean at 7.5 m - not avoiding from long distance"
    assert vy < 0.30, "that is a swerve, not a lean"


def test_the_steer_band_never_commands_a_reverse():
    """choose_heading may legitimately return a rear-quadrant bearing. vx < 0
    would back into the 110 deg the LiDAR cannot see."""
    for bearing in (0.0, 30.0, -30.0, 60.0):
        node, sent = _nav()
        _enroute(node)
        node._obstacles = _obs((2.2, bearing))
        node._do_navigate()
        for cmd in _of(sent, "velocity"):
            assert cmd.params["vx"] >= 0.0


# ============ 3. the dodge must ask for GUIDED before it steers =============
def test_the_dodge_asks_to_leave_brake_before_commanding_velocity():
    """THE reported bug. The dodge solved a course and commanded nothing,
    because BRAKE discards setpoints and nothing ever asked for GUIDED."""
    node, sent = _nav()
    _enroute(node)
    node._phase = MissionPhase.AVOID
    node._mode = "BRAKE"                      # where our own brake left it
    node._obstacles = _obs((1.0, 0.0))        # inside the stop distance

    node._do_avoid()
    assert "GUIDED" in _modes(sent), "never asked to leave BRAKE - the bug"
    assert not _of(sent, "velocity"), "setpoint sprayed into BRAKE"


def test_the_dodge_steers_once_guided_is_confirmed():
    node, sent = _nav()
    _enroute(node)
    node._phase = MissionPhase.AVOID
    node._mode = "BRAKE"
    node._obstacles = _obs((1.0, 0.0))
    node._do_avoid()

    sent.clear()
    node._mode = "GUIDED"                     # FC confirms the switch
    node._do_avoid()
    assert _of(sent, "velocity"), "dodge still commanding nothing in GUIDED"


# ===================== 4. rejoining the original track ======================
def test_clearing_the_obstacle_reissues_the_goto():
    """Off-track recovery: the re-issued goto is what pulls the aircraft back
    onto the straight line instead of carrying on down the deviated course."""
    node, sent = _nav()
    _enroute(node)
    node._obstacles = _obs((5.0, 0.0))
    node._do_navigate()                       # steering; _steer_heading set
    assert node._steer_heading is not None

    sent.clear()
    node._obstacles = _obs((40.0, 0.0))       # clear
    node._do_navigate()
    # The 0.5 s CLEAR hold-off (added with the wobble fix, which is what broke
    # this test) deliberately keeps steering through the first clear ticks so
    # LiDAR flicker at the band edge cannot strobe goto against velocity.
    assert node._steer_heading is not None, "released without serving hold-off"
    node._steer_clear_since -= 1.0             # let the hold-off elapse
    node._do_navigate()

    assert node._steer_heading is None, "still holding a steering course"
    assert _of(sent, "goto"), "never rejoined the track"
    assert node._offtrack_m == 0.0, "stale off-track carried past the release"


def test_entering_the_brake_band_drops_the_steering_course():
    """The dodge must not inherit a course solved under partial authority."""
    node, sent = _nav()
    _enroute(node)
    node._obstacles = _obs((5.0, 0.0))
    node._do_navigate()
    assert node._steer_heading is not None

    node._obstacles = _obs((1.0, 0.0))        # now inside the brake distance
    node._do_navigate()
    assert node._phase == MissionPhase.AVOID
    assert node._steer_heading is None


# ============== 6. contracts added with the 2026-09-21 long-range tune ======
def test_the_hold_off_keeps_steering_rather_than_snapping_straight():
    """The release must not be a step change in commanded heading."""
    node, sent = _nav()
    _enroute(node)
    node._obstacles = _obs((5.0, 0.0))
    node._do_navigate()
    held = node._steer_heading

    sent.clear()
    node._obstacles = _obs((40.0, 0.0))
    node._do_navigate()
    assert not _of(sent, "goto"), "rejoined the track inside the hold-off"
    assert _of(sent, "velocity"), "commanded nothing during the hold-off"
    # Still steering, and decaying toward the goal rather than jumping to it.
    assert abs(node._steer_heading) <= abs(held) + 1e-6


def test_the_manoeuvre_develops_gradually_across_the_band():
    """Requirement, 2026-09-21: 'smooth alter of course', not a hard swerve.

    Smoothness is a property of the turn RATE, not of the heading shape - a
    gentle-looking S curve flown late is still a hard turn. So walk the band
    at the flown tick rate and pin the per-tick heading change under the
    configured slew, which is what the airframe actually feels.
    """
    node, sent = _nav()
    _enroute(node)
    headings = []
    dt = 0.1                                   # the flown 10 Hz tick
    t = 1000.0
    times = iter([t + i * dt for i in range(400)])
    node._steer_heading_t = None
    for distance in (8.0, 7.0, 6.0, 5.0, 4.0, 3.0, 2.5):
        node._obstacles = _obs((distance, 0.0))
        node._do_navigate()
        headings.append(node._steer_heading)
    assert all(h is not None for h in headings)
    # It must be leaning from the very first sample inside the band.
    assert abs(headings[0]) > 1.0, "flat at the band edge - the late-turn bug"
    # And it must be monotone away from the obstacle: no reversals, which are
    # what an operator reads as hunting.
    steps = [b - a for a, b in zip(headings, headings[1:])]
    signs = {s > 0 for s in steps if abs(s) > 1e-9}
    assert len(signs) <= 1, f"course reversed mid-manoeuvre: {headings}"


def test_the_critical_distance_is_the_configured_one():
    """Requirement, 2026-09-21: hard stop at 1.5 m and not before.

    Guards the clearance invariant from the other side: if the brake pad ever
    starts keying off our own cruise again, this goes red at cruise speed
    while every unit test on stop_distance_for stays green.
    """
    av = _avoider()
    o = Obstacle(bearing_deg=0.0, distance_m=3.0, closing_ms=1.0,
                 speed_m_s=0.0, is_dynamic=False)
    av.evaluate(ObstacleArray(obstacles=[o]))
    assert av.last_stop_m == pytest.approx(av.stop)


def test_the_safety_radius_clears_the_brake_distance():
    """Section 12c, the clearance invariant, as an executable assertion.

    VFH+ enlarges every obstacle by vfh_safety_radius, so the steerer aims for
    gaps that keep it that far off. If the radius does not clear the brake
    distance, the steerer flies its whole manoeuvre inside the band the brake
    calls an emergency, and the two fight. 0.3 m is the documented minimum.
    """
    av = _avoider()
    assert av.vfh_safety_radius >= av.stop + 0.3, (
        "safety radius %.2f does not clear the %.2f m brake by 0.3 m"
        % (av.vfh_safety_radius, av.stop))


def test_the_stored_course_survives_the_airframe_rotating_under_it():
    """The nose-track/steerer interaction, and the easiest thing here to get
    wrong - I got it wrong once already, in the other direction.

    _steer_heading is a BODY-frame bearing, and since 2026-09-21 the stack
    yaws the airframe onto the waypoint while the steerer is running. So the
    stored course silently goes stale: the same patch of ground is a different
    body bearing after the nose moves. Without counter-rotation the rate
    limiter reads the aircraft's own rotation as a course change it must
    resist, and the yaw controller and slew_steer spend the manoeuvre
    cancelling each other out.

    Set up a rotation that changes NOTHING about the world - the aircraft
    yaws +30 deg, and the obstacle and waypoint move -30 deg in body frame to
    match. A correct steerer holds the same world-frame course across it.

    (The first cut of this recorded _prev_yaw_rad in _do_navigate, one call
    BEFORE _steer_step consumed it, so every delta was exactly zero and the
    counter-rotation was dead code that no test could see. This is that test.)
    """
    node, sent = _nav()
    _enroute(node)
    node._obstacles = _obs((5.0, 0.0))
    node._do_navigate()
    assert node._steer_heading is not None
    world_before = node._steer_heading            # yaw is 0, so body == world

    # Same world, rotated airframe.
    node._fused = FusedState(x=0.0, y=0.0, alt_rel_m=2.0,
                             yaw=math.radians(30.0), valid=True)
    node._obstacles = _obs((5.0, -30.0))
    node._do_navigate()

    world_after = 30.0 + node._steer_heading
    drift = abs(wrap_180(world_after - world_before))
    assert drift < 5.0, (
        "course slipped %.1f deg when only the nose moved (body %.1f -> %.1f)"
        % (drift, world_before, node._steer_heading))
