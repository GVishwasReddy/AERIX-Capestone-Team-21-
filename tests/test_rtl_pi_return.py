"""The Pi-flown return: coming home under this node's own avoidance.

Why this exists. The operator's requirement was "make sure avoidance works in
rtl also", and on this airframe there was no good FC-side answer.
``_rtl_avoid_step`` watches the obstacle field and hard-brakes at
``avoidance_stop_m``, but nothing STEERS the return: the Pi deliberately does
not (in RTL the flight controller discards SET_POSITION_TARGET_LOCAL_NED, and
owning the mode is the 2026-08-22 lockout shape), and the FC could not either,
because ``OA_TYPE`` measured **0** - BendyRuler has never run here.

The alternatives are worse, not better. Dijkstra (``OA_TYPE`` 2) plans against
FENCE polygons only, and ``FENCE_ENABLE`` measured 0 with none defined, so it
cannot see a LiDAR return at all; ``OA_TYPE`` 3 is Dijkstra plus BendyRuler,
where the Dijkstra half contributes nothing here. BendyRuler is the only
FC-side planner this sensor can feed, and it is entirely unmeasured.

So the return is flown as an ORDINARY WAYPOINT AT HOME. That is the whole
mechanism, and the reason it is worth trusting: the cruise band, the authority
floor, the slew limiter, nose-track, the clear hold-off, the goto re-issue and
the 1.5 m critical brake are the SAME code coming home as going out, so they
are covered by the same measurements and the same tests rather than by a
second implementation nobody has flown.

What this file has to pin is therefore not the avoidance - that is
test_smooth_avoidance.py's job - but the HANDOVER RULES around it, which are
where the safety lives:

  * only a finished job may be flown by the Pi; a failsafe never is
  * the 180 deg about-face still happens first, and only once
  * arriving home gives the descent back to the FC
  * a return that cannot be solved hands back rather than hovering at 2 m
"""
from __future__ import annotations

import copy
import math
import time

import pytest

from drone_stack.bus import MessageBus
from drone_stack.bus.topics import Topics
from drone_stack.msg import (
    FusedState, MissionPhase, Obstacle, ObstacleArray, Waypoint,
)
from drone_stack.nodes.navigation_node import NavigationNode
from drone_stack.srv import ServiceRegistry
from drone_stack.utils.config import Config


def _cfg(**overrides) -> Config:
    raw = copy.deepcopy(Config.load().raw)
    nav = raw.setdefault("navigation", {})
    # Pinned, not inherited - this file is about the Pi-flown path.
    nav["avoidance_rtl_router"] = "pi"
    # The band as flown, for the same reason test_smooth_avoidance.py pins it:
    # whichever profile Config.load() resolves decides whether 5 m reads as
    # SLOW or CLEAR, and a test that passes because everything read CLEAR is
    # worse than no test.
    nav["avoidance_distance_m"] = 10.0
    nav["avoidance_stop_m"] = 1.5
    nav["avoidance_dodge_enabled"] = True
    nav["avoidance_vfh_enabled"] = True
    for name, values in overrides.items():
        raw.setdefault(name, {}).update(values)
    return Config(raw)


def _nav(**overrides):
    bus = MessageBus()
    sent: list = []
    bus.subscribe(Topics.MAVLINK_CMD, sent.append)
    node = NavigationNode(bus, _cfg(**overrides), ServiceRegistry())
    node._avoid_enabled = True
    node._armed = True
    node._has_flown = True
    node._home = (12.9, 77.6)
    node._mode = "GUIDED"
    node._phase = MissionPhase.NAVIGATE
    # 200 m out, nose north, delivery done. lat/lon home with x/y fused means
    # _goal_bearing_deg reads the geodetic conversion, same as in flight.
    node._fused = FusedState(x=120.0, y=160.0, alt_rel_m=2.0, yaw=0.0,
                             valid=True)
    node._mission.waypoints = [Waypoint(seq=0, x_m=120.0, y_m=160.0, alt_m=2.0)]
    node._current_wp = 0
    sent.clear()
    return node, sent


def _of(sent, command):
    return [c for c in sent if c.command == command]


def _modes(sent):
    return [c.params.get("mode") for c in sent if c.command == "set_mode"]


def _finish_the_about_face(node):
    """Drive stage 1 to completion so _commit_rtl is actually reached."""
    for _ in range(200):
        if node._about_face_since is None:
            return
        if node._about_face_target is not None:
            node._fused = FusedState(
                x=node._fused.x, y=node._fused.y, alt_rel_m=2.0,
                yaw=node._about_face_target, valid=True)
        node._do_rtl()


# ===================== 1. who gets to fly the return ========================
def test_a_finished_job_is_flown_home_by_the_pi():
    node, sent = _nav()
    node._enter_rtl(turn_first=True)
    _finish_the_about_face(node)

    assert "RTL" not in _modes(sent), "handed the return to the FC"
    assert "SMART_RTL" not in _modes(sent)
    assert node._phase == MissionPhase.NAVIGATE, \
        "not back in the phase that owns the avoidance machinery"
    wp = node._mission.waypoints[node._current_wp]
    assert wp.kind == "rtl", "the return leg is not a home waypoint"
    assert (wp.lat, wp.lon) == node._home


def test_a_failsafe_return_still_goes_straight_to_the_fc():
    """THE safety line in this feature.

    A low-battery or geofence RTL fires in exactly the situation where delay
    costs most, and a Pi-flown return at the 2 m ceiling has neither the
    climb to RTL_ALT nor the FC's own failsafes. _enter_rtl already draws
    this line for the pre-RTL turns; the Pi router has to respect the same one.
    """
    node, sent = _nav()
    node._enter_rtl()                      # no turn_first: the failsafe path
    assert _modes(sent), "commanded no return at all"
    assert node._phase == MissionPhase.RTL
    assert node._pi_return_since is None, "the Pi took a failsafe return"


def test_the_config_can_hand_the_return_back_to_the_fc():
    node, sent = _nav(navigation={"avoidance_rtl_router": "fc"})
    node._enter_rtl(turn_first=True)
    _finish_the_about_face(node)
    assert _modes(sent), "router 'fc' did not command a return"
    assert node._phase == MissionPhase.RTL


def test_no_fix_hands_the_return_to_the_fc():
    """No fix means no home waypoint to build. The FC's RTL has its own idea
    of home and does not need ours."""
    node, sent = _nav()
    node._enter_rtl(turn_first=True)
    _finish_the_about_face(node)
    # Re-run the decision with the fix pulled, as a fresh node.
    node, sent = _nav()
    node._fused = None
    node._enter_rtl(turn_first=True)
    assert _modes(sent), "commanded nothing with no fix"


# =============== 2. the about-face still happens, and only once =============
def test_the_about_face_fires_before_the_return_leg():
    """Requirement, 2026-09-21: "turn 180 degree after reaching waypoint
    before rtl gets triggered". The Pi router must not swallow it."""
    node, sent = _nav()
    node._enter_rtl(turn_first=True)
    assert node._about_face_since is not None, "the about-face never started"

    # _enter_rtl only STARTS stage 1 - _about_face_step sends the
    # CONDITION_YAW on the next tick, because the command is paced (relative
    # yaw restarts the turn every time it is re-sent, see the yaw gate).
    node._do_rtl()
    yaws = _of(sent, "yaw")
    assert yaws, "no about-face was commanded"
    assert yaws[0].params["angle"] == pytest.approx(180.0, abs=1.0)
    # And the return leg has NOT started yet - the turn gates it.
    assert node._pi_return_since is None, "started home mid-turn"

    _finish_the_about_face(node)
    assert node._pi_return_since is not None, "never started the return"


def test_arriving_home_does_not_spin_a_second_about_face():
    """_about_face_wanted is UNCONDITIONAL, and running out of waypoints is
    what triggers a return - so the home leg re-enters _enter_rtl. Without
    the _pi_return_flown guard the aircraft spins 180 deg over home before
    landing."""
    node, sent = _nav()
    node._enter_rtl(turn_first=True)
    _finish_the_about_face(node)
    assert node._pi_return_since is not None

    # Home reached: the mission runs out of waypoints.
    sent.clear()
    node._current_wp = node._mission.count
    node._enter_rtl(turn_first=True)

    assert not _of(sent, "yaw"), "spun a second about-face over home"
    assert _modes(sent), "never handed the descent to the FC"
    assert node._phase == MissionPhase.RTL


def test_the_return_leg_is_offered_only_once():
    node, sent = _nav()
    node._enter_rtl(turn_first=True)
    _finish_the_about_face(node)
    before = node._mission.count
    node._enter_rtl(turn_first=True)
    assert node._mission.count == before, "appended a second home waypoint"


# ===================== 3. it actually avoids on the way home ================
def test_an_obstacle_on_the_return_leg_is_steered_around():
    """The point of the whole feature. Before this, an obstacle on the return
    produced a hard brake and nothing else."""
    node, sent = _nav()
    node._enter_rtl(turn_first=True)
    _finish_the_about_face(node)
    assert node._phase == MissionPhase.NAVIGATE

    sent.clear()
    node._obstacles = ObstacleArray(
        obstacles=[Obstacle(bearing_deg=0.0, distance_m=6.0, hits=5)])
    node._do_navigate()

    vel = _of(sent, "velocity")
    assert vel, "commanded nothing against an obstacle on the return"
    assert abs(vel[-1].params["vy"]) > 0.05, "no lateral deflection"
    assert vel[-1].params["vx"] > 0.0, "gave up forward speed"
    assert node._steer_heading is not None


def test_the_return_leg_brakes_at_the_critical_distance():
    node, sent = _nav()
    node._enter_rtl(turn_first=True)
    _finish_the_about_face(node)

    sent.clear()
    node._obstacles = ObstacleArray(
        obstacles=[Obstacle(bearing_deg=0.0, distance_m=1.0, hits=5)])
    node._do_navigate()
    assert _of(sent, "brake") or node._phase == MissionPhase.AVOID, \
        "flew on through the critical distance"


def test_the_nose_stays_on_home_during_the_return():
    """Requirement 4 applies to the return leg too - and it matters more
    there, because the 250 deg scanned window points where the nose does."""
    node, sent = _nav()
    node._enter_rtl(turn_first=True)
    _finish_the_about_face(node)

    sent.clear()
    node._do_navigate()
    yaws = _of(sent, "yaw")
    assert yaws, "nose not brought onto home for the return"


# ===================== 4. it cannot strand the aircraft =====================
def test_a_return_that_overruns_its_budget_hands_back_to_the_fc():
    """An aircraft that cannot solve its way home must not keep trying at a
    2 m ceiling until the battery failsafe decides for it."""
    node, sent = _nav(navigation={"avoidance_rtl_pi_budget_s": 30.0})
    node._enter_rtl(turn_first=True)
    _finish_the_about_face(node)
    assert node._pi_return_since is not None

    sent.clear()
    node._pi_return_since -= 31.0          # spend the budget
    node._do_navigate()

    assert _modes(sent), "burned the budget and commanded nothing"
    assert node._phase == MissionPhase.RTL
    assert node._pi_return_since is None


def test_a_return_inside_its_budget_is_left_alone():
    node, sent = _nav(navigation={"avoidance_rtl_pi_budget_s": 30.0})
    node._enter_rtl(turn_first=True)
    _finish_the_about_face(node)

    sent.clear()
    node._pi_return_since -= 5.0
    node._do_navigate()
    assert "RTL" not in _modes(sent), "handed back well inside the budget"
    assert node._phase == MissionPhase.NAVIGATE
