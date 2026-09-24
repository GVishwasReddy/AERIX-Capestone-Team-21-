"""The rear 110 deg is never scanned - so never fly into it without looking.

Those sectors are streamed to the FC as 65535 = unknown, and ArduPilot's
proximity database treats unknown as CLEAR. Left alone, BendyRuler would route
into ground the LiDAR has never seen, with complete confidence. The aircraft
also cannot reverse out of the rear (that is what the mask is for) and at a 2 m
ceiling it cannot climb over.

So a waypoint behind the aircraft is not flown at: hold station, yaw the nose
onto it, then proceed under the normal avoidance.
"""
from __future__ import annotations

import copy
import math

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
    for name, values in overrides.items():
        raw.setdefault(name, {}).update(values)
    return Config(raw)


def _nav(**overrides):
    bus = MessageBus()
    sent: list = []
    bus.subscribe(Topics.MAVLINK_CMD, sent.append)
    node = NavigationNode(bus, _cfg(**overrides), ServiceRegistry())
    node._avoid_enabled = True
    sent.clear()
    return node, sent


def _of(sent, command):
    return [c for c in sent if c.command == command]


def _at(node, bearing_deg: float, dist_m: float = 40.0):
    """Put a waypoint *bearing_deg* off the nose (+ right), aircraft facing N."""
    node._armed = True
    node._has_flown = True
    node._home = (0.0, 0.0)
    node._phase = MissionPhase.NAVIGATE
    node._mode = "GUIDED"
    # yaw 0 = North; body x forward = North, body y left = West.
    rad = math.radians(bearing_deg)
    east = dist_m * math.sin(rad)
    north = dist_m * math.cos(rad)
    node._fused = FusedState(x=0.0, y=0.0, alt_rel_m=2.0, yaw=0.0, valid=True)
    node._mission.waypoints = [Waypoint(seq=0, x_m=east, y_m=north, alt_m=2.0)]
    node._current_wp = 0
    node._obstacles = None            # clear field: only the gate is under test
    return node


def _yaws(sent):
    return [c for c in sent if c.command == "yaw"]


def _gotos(sent):
    return [c for c in sent if c.command == "goto"]


# -- the gate ----------------------------------------------------------------
@pytest.mark.parametrize("bearing", [180.0, -180.0, 150.0, -150.0, 130.0, -130.0])
def test_a_waypoint_in_the_masked_rear_is_not_flown_at(bearing):
    node, sent = _nav()
    _at(node, bearing)
    node._do_navigate()

    assert not _gotos(sent), f"translated toward an unscanned {bearing} deg bearing"
    assert _yaws(sent), "did not turn to look first"


@pytest.mark.parametrize("bearing", [0.0, 45.0, -45.0, 100.0, -100.0, 124.0])
def test_a_waypoint_inside_the_window_is_flown_at_once(bearing):
    """The gate must not tax the normal case - anything the LiDAR can already
    see is flown immediately, WITHOUT WAITING for a turn.

    The "with no turn" half was dropped on 2026-09-21. It was never the gate's
    contract - it was an artefact of nothing else in the stack commanding yaw.
    Nose-track now does, so what this test must pin is the gate's own property:
    inside the scanned window, the goto goes out on THIS tick. Whether the nose
    is also being brought round is a different feature's business, and
    test_nose_track_and_the_gate_are_orthogonal is where that lives.
    """
    node, sent = _nav()
    _at(node, bearing)
    node._do_navigate()

    assert _gotos(sent), f"held for {bearing} deg, which is inside the window"


def test_the_turn_goes_the_short_way_round():
    node, sent = _nav()
    _at(node, 170.0)                       # to the right, behind
    node._do_navigate()
    yaw = _yaws(sent)[0]
    assert yaw.params["direction"] == 1
    assert yaw.params["angle"] == pytest.approx(170.0, abs=1.0)

    node, sent = _nav()
    _at(node, -170.0)                      # to the left, behind
    node._do_navigate()
    yaw = _yaws(sent)[0]
    assert yaw.params["direction"] == -1
    assert yaw.params["angle"] == pytest.approx(170.0, abs=1.0)


def test_it_flies_once_the_nose_has_come_round():
    node, sent = _nav()
    _at(node, 180.0)
    node._do_navigate()
    assert not _gotos(sent)

    # the aircraft has turned to face it
    sent.clear()
    _at(node, 10.0)
    node._do_navigate()
    assert _gotos(sent), "still holding after the nose came round"
    assert node._yaw_gate_since is None


def test_the_gate_holds_through_the_hysteresis_band():
    """Engages past 125 deg, releases inside 60. Between the two it must keep
    holding, or it would let go at the very edge of the window - the sparsest,
    least reliable part of the scan - and chatter on its own threshold."""
    node, sent = _nav()
    _at(node, 180.0)
    node._do_navigate()
    assert node._yaw_gate_since is not None

    sent.clear()
    _at(node, 100.0)                       # inside the window, outside release
    node._do_navigate()
    assert not _gotos(sent), "released at the edge of the window"
    assert node._yaw_gate_since is not None

    sent.clear()
    _at(node, 55.0)                        # inside release
    node._do_navigate()
    assert _gotos(sent)


def test_the_turn_is_not_re_commanded_at_loop_rate():
    """CONDITION_YAW is a discrete command, not a setpoint. Re-sending it every
    tick restarts the turn and it never completes."""
    node, sent = _nav()
    _at(node, 180.0)
    for _ in range(30):
        node._do_navigate()
    assert len(_yaws(sent)) == 1, f"{len(_yaws(sent))} yaw commands in 30 ticks"


def test_a_stuck_turn_holds_rather_than_flying_blind():
    """Defeating a safety gate on a timer defeats the gate. It holds - but it
    says so loudly, because the failure that hurts is a SILENT hold."""
    node, sent = _nav(navigation={"avoidance_yaw_timeout_s": 0.0})
    _at(node, 180.0)
    node._do_navigate()
    node._do_navigate()

    assert not _gotos(sent), "flew into the unscanned rear after giving up"
    assert node._yaw_gate_stuck
    assert "HELD" in node._status_message


def test_the_gate_can_be_switched_off():
    """Off means "do not HOLD" - it never meant "do not yaw".

    Both flags have to be off to get a silent nose, and that is the point of
    keeping them separate: an operator who disables the safety gate has not
    asked to stop pointing the camera at the waypoint.
    """
    node, sent = _nav(navigation={"avoidance_yaw_before_move": False,
                                  "avoidance_nose_track": False})
    _at(node, 180.0)
    node._do_navigate()
    assert _gotos(sent)
    assert not _yaws(sent)


def test_switching_the_gate_off_still_points_the_nose():
    node, sent = _nav(navigation={"avoidance_yaw_before_move": False})
    _at(node, 180.0)
    node._do_navigate()
    assert _gotos(sent), "held with the gate disabled"
    assert _yaws(sent), "gate off silenced nose-track as well"


# -- it must not interfere with anything else --------------------------------
def test_the_gate_is_dropped_when_the_pilot_takes_over():
    node, _ = _nav()
    _at(node, 180.0)
    node._do_navigate()
    assert node._yaw_gate_since is not None

    node._latch_override("transmitter moved", "BRAKE")
    assert node._yaw_gate_since is None, "a stale gate would resume on RESUME"


def test_no_fix_means_no_gate():
    """The bearing is unknown, and gating on a guess is worse than not gating:
    it would hold the aircraft over a customer for a bearing it invented."""
    node, sent = _nav()
    _at(node, 180.0)
    node._fused = None
    node._do_navigate()
    assert _gotos(sent) or not _yaws(sent)


def test_a_braking_obstacle_still_wins_over_the_gate():
    """Ordering: STOP is decided before the gate is consulted, so an obstacle
    in front still brakes even while the aircraft wants to turn around."""
    from drone_stack.msg import Obstacle, ObstacleArray
    node, sent = _nav()
    _at(node, 180.0)
    node._obstacles = ObstacleArray(
        obstacles=[Obstacle(distance_m=0.5, bearing_deg=0.0)])
    node._do_navigate()

    assert node._phase == MissionPhase.AVOID
    assert [c for c in sent if c.command == "brake"]


# ===== nose-on-waypoint, added 2026-09-21 ==================================
# Requirement: "drone must always point front towards the waypoint always".
#
# This is a SEPARATE feature from the yaw gate, and the distinction is the
# thing most likely to be lost by a later reader:
#
#   avoidance_yaw_before_move  - a SAFETY gate. Refuses to translate toward a
#                                bearing the LiDAR has never scanned. It is
#                                about WAITING.
#   avoidance_nose_track       - a POINTING policy. Keeps the nose (and so the
#                                camera and the 250 deg scanned window) on the
#                                waypoint while flying. It is about YAW.
#
# Velocity setpoints go out in MAV_FRAME_BODY_NED with IGNORE_YAW set, so
# ArduPilot STRAFES during a dodge and the nose stays wherever it was left.
# That is why pointing has to be commanded on its own.
def test_the_nose_is_brought_round_to_the_waypoint():
    node, sent = _nav()
    _at(node, 60.0)
    node._do_navigate()
    yaws = _yaws(sent)
    assert yaws, "nose left off the waypoint"
    assert yaws[0].params["direction"] == 1
    assert yaws[0].params["angle"] == pytest.approx(60.0, abs=1.0)


def test_the_nose_turns_the_short_way_for_a_left_waypoint():
    node, sent = _nav()
    _at(node, -60.0)
    node._do_navigate()
    yaw = _yaws(sent)[0]
    assert yaw.params["direction"] == -1
    assert yaw.params["angle"] == pytest.approx(60.0, abs=1.0)


def test_a_waypoint_already_on_the_nose_is_not_chased():
    """The deadband is not a nicety. CONDITION_YAW is RELATIVE (param4=1), so
    every command restarts the turn from wherever the nose is now - chasing
    heading noise means the turn never lands."""
    node, sent = _nav()
    _at(node, 3.0)                          # inside avoidance_nose_deadband_deg
    node._do_navigate()
    assert not _yaws(sent), "chased 3 deg of noise"
    assert _gotos(sent)


def test_the_nose_command_is_paced():
    """Same reason. Re-commanding at the 10 Hz tick rate restarts the relative
    turn ten times a second, so the nose creeps and never arrives. Measured on
    the yaw gate first; the same trap applies here."""
    node, sent = _nav()
    _at(node, 90.0)
    for _ in range(30):                     # 3 s of ticks
        node._do_navigate()
    assert len(_yaws(sent)) == 1, f"{len(_yaws(sent))} yaws in 30 ticks"


def test_nose_track_can_be_switched_off():
    node, sent = _nav(navigation={"avoidance_nose_track": False})
    _at(node, 60.0)
    node._do_navigate()
    assert not _yaws(sent)
    assert _gotos(sent)


def test_nose_track_and_the_gate_are_orthogonal():
    """Four combinations, and each has to behave as its own feature."""
    # gate ON, nose ON: rear waypoint -> hold AND turn (the gate's turn).
    node, sent = _nav()
    _at(node, 180.0)
    node._do_navigate()
    assert _yaws(sent) and not _gotos(sent)

    # gate ON, nose OFF: rear waypoint still holds and still turns to look.
    node, sent = _nav(navigation={"avoidance_nose_track": False})
    _at(node, 180.0)
    node._do_navigate()
    assert _yaws(sent), "the safety gate stopped turning to look"
    assert not _gotos(sent)

    # gate OFF, nose ON: flies at once, and points.
    node, sent = _nav(navigation={"avoidance_yaw_before_move": False})
    _at(node, 180.0)
    node._do_navigate()
    assert _yaws(sent) and _gotos(sent)


def test_the_nose_is_not_commanded_without_a_position_fix():
    """A bearing needs a fix. Guessing one points the only scanned window in
    the wrong direction, which is worse than leaving the nose alone."""
    node, sent = _nav()
    _at(node, 60.0)
    node._fused = None
    node._do_navigate()
    assert not _yaws(sent)


def test_the_nose_holds_during_an_avoidance_dodge():
    """The dodge strafes (IGNORE_YAW), so this is the tick where pointing is
    most easily lost - and the tick where it matters most, because the LiDAR
    needs the obstacle inside its scanned window to keep tracking it."""
    # The band is pinned rather than inherited: whichever profile Config.load()
    # resolves decides whether 5.0 m reads as SLOW or CLEAR, and a test that
    # passes vacuously because everything read CLEAR has bitten this suite
    # before (see the _BAND note in test_smooth_avoidance.py).
    node, sent = _nav(navigation={"avoidance_distance_m": 10.0,
                                  "avoidance_stop_m": 1.5,
                                  "avoidance_dodge_enabled": True,
                                  "avoidance_vfh_enabled": True})
    _at(node, 60.0)
    node._obstacles = ObstacleArray(
        obstacles=[Obstacle(bearing_deg=0.0, distance_m=5.0)])
    node._do_navigate()
    assert _of(sent, "velocity"), "not steering - wrong tick under test"
    assert _yaws(sent), "nose abandoned mid-dodge"
