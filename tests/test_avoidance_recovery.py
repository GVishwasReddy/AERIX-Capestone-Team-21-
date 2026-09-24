"""Regression tests for the 2026-08-31 delivery flight.

What happened, from the log: avoidance braked 2 s after takeoff, and the
aircraft never moved again. ``_do_navigate``'s SLOW branch streamed a
``velocity`` setpoint at 10 Hz; ``velocity`` records GUIDED in
``_commanded_mode`` but, not being a ``_MODE_ONLY_COMMAND``, never emits a
SET_MODE frame. So the FC sat in BRAKE ignoring every setpoint while
``_note_mode_refusal`` logged "FC refused GUIDED and stayed in BRAKE" once a
second, for ten seconds, until the pilot moved the sticks and the mission
aborted. Then the BLE handshake completed and logged "RTL in 3s" - and nothing
happened at all, because ``_ble_delivered_at`` is only ever read by
``_do_hover`` and the mission was no longer holding.

Three separate faults, three groups of tests below.
"""
from __future__ import annotations

import copy
import time

from drone_stack.bus import MessageBus
from drone_stack.bus.topics import Topics
from drone_stack.msg import (
    DeliveryBleResult,
    FusedState,
    MissionPhase,
    Obstacle,
    ObstacleArray,
    Waypoint,
)
from drone_stack.nodes.navigation_node import NavigationNode
from drone_stack.srv import ServiceRegistry
from drone_stack.utils.config import Config


def _cfg(**overrides) -> Config:
    raw = copy.deepcopy(Config.load().raw)
    # This file is about the FC-OWNED return: the pre-RTL turn stages and the
    # handover to the flight controller. Since 2026-09-21 the shipped default
    # is avoidance_rtl_router: pi, which flies the return as a home waypoint
    # under the Pi's own avoidance and so never reaches _commit_rtl's mode
    # change - correct behaviour, and not what these tests assert about. Pin
    # the router here rather than inheriting it, for the same reason
    # test_smooth_avoidance.py pins _BAND: a test must pin what it asserts
    # about. The Pi-flown path has its own file, test_rtl_pi_return.py.
    raw.setdefault("navigation", {})["avoidance_rtl_router"] = "fc"
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


def _obs(*pairs) -> ObstacleArray:
    return ObstacleArray(
        obstacles=[Obstacle(distance_m=d, bearing_deg=b) for d, b in pairs]
    )


def _enroute(node, dist_m=40.0):
    """A node airborne and navigating toward a waypoint well out of range."""
    node._armed = True
    node._has_flown = True
    node._home = (12.9, 77.6)
    node._phase = MissionPhase.NAVIGATE
    node._mission.waypoints = [Waypoint(seq=0, x_m=dist_m, y_m=0.0, alt_m=2.0,
                                        lat=12.9005, lon=77.6576)]
    node._current_wp = 0
    node._fused = FusedState(x=0.0, y=0.0, alt_rel_m=2.0, valid=True)
    return node


def _modes(sent):
    return [c.params.get("mode") for c in sent if c.command == "set_mode"]


# ===================== 1. the BRAKE deadlock ================================
def test_a_brake_is_released_back_into_guided_when_the_obstacle_clears():
    """The core failure. Braked in BRAKE, obstacle gone: the node must ask for
    GUIDED, because a goto alone is silently ignored in BRAKE."""
    node, sent = _nav()
    _enroute(node)
    node._mode = "BRAKE"                      # where our own brake left it
    node._obstacles = _obs((40.0, 0.0))       # nothing in the way any more

    node._do_navigate()
    assert "GUIDED" in _modes(sent), "never asked to leave BRAKE"

    # ...and once the FC confirms GUIDED, the goto goes out.
    sent.clear()
    node._mode = "GUIDED"
    node._do_navigate()
    assert [c for c in sent if c.command == "goto"], "no goto after regaining GUIDED"


def test_the_slow_branch_never_streams_velocity():
    """The Pi does not steer: OA_TYPE=1 BendyRuler routes and this node's only
    output is the brake. The velocity stream both broke that and, implying
    GUIDED without commanding it, is what wedged the aircraft."""
    node, sent = _nav()
    _enroute(node)
    node._mode = "GUIDED"
    node._obstacles = _obs((3.0, 0.0))        # inside avoidance_distance_m, past stop

    for _ in range(20):
        node._do_navigate()

    assert not [c for c in sent if c.command == "velocity"], \
        "velocity setpoints are the Pi steering; that is the FC's job"
    assert [c for c in sent if c.command == "goto"], \
        "the goto must stand so the FC can route around it"


def test_a_brake_does_not_re_command_the_mode_at_loop_rate():
    """The 2026-08-22 rule. Twenty ticks stuck in BRAKE must not produce twenty
    SET_MODE frames - that is what locked the pilot out of the aircraft."""
    node, sent = _nav()
    _enroute(node)
    node._mode = "BRAKE"
    node._obstacles = _obs((40.0, 0.0))

    for _ in range(20):
        node._do_navigate()

    assert len(_modes(sent)) <= 2, f"{len(_modes(sent))} SET_MODE frames in 20 ticks"


def test_navigation_is_not_gated_when_the_mode_is_merely_unknown():
    """Only BRAKE swallows setpoints. Gating on == GUIDED instead stopped the
    mission dead before a first heartbeat arrived."""
    node, sent = _nav()
    _enroute(node)
    node._mode = ""                           # no heartbeat yet
    node._obstacles = _obs((40.0, 0.0))
    node._do_navigate()
    assert [c for c in sent if c.command == "goto"]


# ===================== 2. release hysteresis ================================
def test_the_hold_is_not_released_on_the_first_non_stop_tick():
    """STOP and SLOW now command different modes, so releasing on scan jitter
    around the stop distance would chatter BRAKE against GUIDED at 10 Hz."""
    node, _ = _nav()
    _enroute(node)
    node._mode = "BRAKE"
    node._phase = MissionPhase.AVOID
    # just past the stop distance, but inside the release margin
    edge = node._avoider.stop + node._avoider.release_m * 0.5
    node._obstacles = _obs((edge, 0.0))

    node._do_avoid()
    assert node._phase == MissionPhase.AVOID, "released while still inside the margin"


def test_the_hold_releases_once_the_gap_opens_past_the_margin():
    node, _ = _nav()
    _enroute(node)
    node._mode = "BRAKE"
    node._phase = MissionPhase.AVOID
    node._obstacles = _obs((node._avoider.stop + node._avoider.release_m + 0.5, 0.0))

    node._do_avoid()
    assert node._phase == MissionPhase.NAVIGATE
    assert node._last_goto_wp == -1, "must re-issue the goto to rejoin the track"


def test_a_completely_clear_field_releases_immediately():
    """CLEAR is exempt from the hysteresis: nothing is in the cone at all, so
    there is no boundary to jitter across and waiting would just add lag."""
    node, _ = _nav()
    _enroute(node)
    node._mode = "BRAKE"
    node._phase = MissionPhase.AVOID
    node._obstacles = _obs((40.0, 0.0))

    node._do_avoid()
    assert node._phase == MissionPhase.NAVIGATE


# ===================== 3. the handshake that promised an RTL =================
def _handshake(node, order_id="order-1"):
    node._on_ble_delivery_result(
        DeliveryBleResult(order_id=order_id, success=True))


def test_a_handshake_outside_the_hold_actually_returns_home():
    """The reported bug: handshake done, aircraft just sat there. It used to
    log 'RTL in 3s' and set a field only _do_hover ever reads."""
    node, sent = _nav()
    _enroute(node)
    node._phase = MissionPhase.NAVIGATE       # aborted mission, not holding
    node._mode = "GUIDED"

    _handshake(node)

    assert node._phase == MissionPhase.RTL
    # Since the post-delivery about-face (test_rtl_about_face.py) this path
    # turns 180 deg before handing over, so the return is not commanded in the
    # same tick any more. The bug this test guards was the aircraft doing
    # NOTHING - setting a field only _do_hover ever read - so what matters is
    # that something went out on the bus and that the return actually lands.
    if node._about_face_since is not None:
        # The turn is started here but FLOWN by _do_rtl, which the main loop
        # runs every tick - and that is precisely the property the original bug
        # lacked, so tick it rather than trusting the field. The budget is
        # expired first so this asserts the return lands even when the aircraft
        # never turns (see TestTheSharedBudget); a turn that DOES complete is
        # covered in test_rtl_about_face.py.
        node._rtl_yaw_started = time.monotonic() - 99.0
        node._do_rtl()

    # whichever return_mode is configured - default.yaml says SMART_RTL,
    # config/real.yaml says RTL; _enter_rtl owns that choice, not this handler
    assert ({"RTL", "SMART_RTL"} & set(_modes(sent))
            or [c for c in sent if c.command in ("rtl", "smart_rtl")]), \
        f"no return commanded, only {[c.command for c in sent]}"


def test_a_handshake_under_pilot_override_commands_nothing():
    """Transmitter authority outranks the delivery. The pilot is flying it."""
    node, sent = _nav()
    _enroute(node)
    node._latch_override("transmitter moved", "BRAKE")
    sent.clear()

    _handshake(node)

    assert not sent, f"commanded {[c.command for c in sent]} while the pilot was flying"
    assert node._phase == MissionPhase.MANUAL
    assert "RTL NOT commanded" in node._status_message


def test_a_handshake_during_the_hold_still_shortens_it():
    """The path that already worked must keep working: _do_hover owns the
    countdown, so the handler must not pre-empt it with its own RTL."""
    node, sent = _nav()
    _enroute(node)
    node._phase = MissionPhase.HOVER
    node._hover_until = 1e18                  # holding
    node._mode = "GUIDED"

    _handshake(node)

    assert node._phase == MissionPhase.HOVER, "handler must leave the hold to _do_hover"
    assert node._ble_delivered_at is not None
    assert not [c for c in sent if c.command in ("rtl", "smart_rtl")]


def test_a_handshake_on_the_ground_does_nothing():
    node, sent = _nav()
    node._armed = False
    node._has_flown = False
    node._phase = MissionPhase.IDLE

    _handshake(node)

    assert not sent
    assert node._phase == MissionPhase.IDLE


def test_a_failed_handshake_is_still_ignored():
    node, sent = _nav()
    _enroute(node)
    node._on_ble_delivery_result(
        DeliveryBleResult(order_id="order-1", success=False))
    assert not sent
    assert node._ble_delivered_at is None
