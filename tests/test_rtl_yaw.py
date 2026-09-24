"""The pre-RTL turn: point the nose home before handing over to the FC.

Why this exists at all. ``_yaw_gate_ok`` only gates gotos that NavigationNode
issues; the moment RTL is commanded the FLIGHT CONTROLLER flies the leg and this
node issues nothing more to gate. The only lever there is ``WP_YAW_BEHAVIOR``,
an FC parameter this node cannot read - and at its old value of 2 ("face next
waypoint EXCEPT RTL") the aircraft translated home at whatever heading the
delivery ended on, with the unscanned rear 110 deg potentially leading the whole
way. Turning before the handover makes the return leg's coverage depend on this
node instead of on a parameter nobody can see from here.

The turn is deliberately NOT universal: see TestOnlyTheCalmPathsTurn.

Scope: this file covers STAGE 2 of the pre-RTL turn - the conditional turn onto
home - in isolation, so ``_nav`` below switches stage 1 off. Stage 1 (the
unconditional post-delivery 180 deg about-face) and the interaction between the
two live in test_rtl_about_face.py.
"""
from __future__ import annotations

import copy
import math
import time

import pytest

from drone_stack.bus import MessageBus
from drone_stack.bus.topics import Topics
from drone_stack.msg import FusedState, MissionPhase, Waypoint
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
    """A node with the about-face (stage 1) OFF, so stage 2 is reached directly.

    Left on, every test here would assert against stage 1 instead: the
    about-face is unconditional and runs first, so ``_enter_rtl(turn_first=True)``
    would set ``_about_face_since`` and stage 2 would not be consulted until it
    finished. A caller can pass ``navigation={"avoidance_rtl_about_face": True}``
    to get the composed behaviour back.
    """
    nav = {"avoidance_rtl_about_face": False}
    nav.update(overrides.pop("navigation", {}))
    overrides["navigation"] = nav
    bus = MessageBus()
    sent: list = []
    bus.subscribe(Topics.MAVLINK_CMD, sent.append)
    node = NavigationNode(bus, _cfg(**overrides), ServiceRegistry())
    node._armed = True
    node._has_flown = True
    node._home = (12.9, 77.6)
    node._mode = "GUIDED"
    node._phase = MissionPhase.NAVIGATE
    sent.clear()
    return node, sent


def _place(node, east: float, north: float, yaw_rad: float = 0.0):
    """Put the aircraft at an ENU offset from home, facing ``yaw_rad``.

    yaw 0 = nose North (see obstacle_tracker.enu_to_body).
    """
    node._fused = FusedState(x=east, y=north, alt_rel_m=2.0, yaw=yaw_rad,
                             valid=True)
    return node


#: 50 m from home on a bearing that leaves home 150 deg off the nose at yaw 0 -
#: well outside the 125 deg window, and deliberately off the +/-180 knife edge.
_HOME_BEHIND = (25.0, 43.3)


def _returns(sent) -> list:
    return [c for c in sent
            if c.command in ("rtl", "smart_rtl")
            or (c.command == "set_mode"
                and c.params.get("mode") in ("RTL", "SMART_RTL"))]


def _yaws(sent) -> list:
    return [c for c in sent if c.command == "yaw"]


class TestTheTurnEngages:
    def test_home_behind_turns_before_commanding_the_return(self):
        node, sent = _nav()
        _place(node, *_HOME_BEHIND)

        node._enter_rtl(turn_first=True)

        assert node._phase == MissionPhase.RTL      # honest: it IS returning
        assert node._rtl_turn_since is not None
        assert not _returns(sent), (
            f"handed over before turning: {[c.command for c in sent]}")

    def test_the_turn_is_commanded_toward_home(self):
        node, sent = _nav()
        _place(node, *_HOME_BEHIND)
        node._enter_rtl(turn_first=True)
        node._do_rtl()

        yaws = _yaws(sent)
        assert len(yaws) == 1
        # ~150 deg off to the LEFT of the nose, so turn left (direction -1).
        # approx: _HOME_BEHIND is a rounded offset, not an exact 150 deg.
        assert yaws[0].params["angle"] == pytest.approx(150.0, abs=0.01)
        assert yaws[0].params["direction"] == -1
        assert yaws[0].params["rate"] == node._avoider.yaw_rate_deg_s

    def test_turning_commands_no_mode_change(self):
        """The hover-complete caller warns that two mode changes in one tick
        risk a refused GUIDED stranding the aircraft over the customer. This
        stage must add none: the aircraft is already in GUIDED and holds
        station simply by not being given a new goto."""
        node, sent = _nav()
        _place(node, *_HOME_BEHIND)
        node._enter_rtl(turn_first=True)
        for _ in range(10):
            node._do_rtl()

        assert [c for c in sent if c.command == "set_mode"] == []

    def test_yaw_is_not_resent_every_tick(self):
        """CONDITION_YAW is a discrete command, not a setpoint. Re-sending it
        at the 10 Hz loop rate restarts the turn forever and it never lands."""
        node, sent = _nav()
        _place(node, *_HOME_BEHIND)
        node._enter_rtl(turn_first=True)
        for _ in range(30):          # 3 s of ticks
            node._do_rtl()

        assert len(_yaws(sent)) == 1

    def test_status_line_says_what_it_is_doing(self):
        node, _ = _nav()
        _place(node, *_HOME_BEHIND)
        node._enter_rtl(turn_first=True)
        node._do_rtl()

        assert "turning onto home" in node._status_message


class TestTheTurnFinishes:
    def test_facing_home_commands_the_return(self):
        node, sent = _nav()
        _place(node, *_HOME_BEHIND)
        node._enter_rtl(turn_first=True)
        node._do_rtl()
        sent.clear()

        # The aircraft has now turned: same position, nose swung onto home.
        _place(node, *_HOME_BEHIND, yaw_rad=math.radians(-150.0))
        node._do_rtl()

        assert _returns(sent), "turned onto home but never handed over"
        assert node._rtl_turn_since is None

    def test_timeout_returns_anyway_rather_than_holding(self):
        """Unlike _yaw_gate_ok, this must NOT hold. That gate decides whether
        to fly at a waypoint, where holding is safe. Refusing to start an RTL
        keeps the aircraft up burning battery until a human notices."""
        node, sent = _nav()
        _place(node, *_HOME_BEHIND)
        node._enter_rtl(turn_first=True)
        # Aircraft never turns; wind the clock past the timeout.
        node._rtl_turn_since -= node._avoider.rtl_yaw_timeout_s + 0.1
        node._do_rtl()

        assert _returns(sent), "timed out and never went home"
        assert node._rtl_turn_since is None

    def test_losing_the_fix_mid_turn_returns_rather_than_turning_blind(self):
        node, sent = _nav()
        _place(node, *_HOME_BEHIND)
        node._enter_rtl(turn_first=True)
        node._do_rtl()
        sent.clear()

        node._fused = FusedState(x=25.0, y=43.3, valid=False)
        node._do_rtl()

        assert _returns(sent)


class TestTheTurnIsSkipped:
    """Usually this feature does nothing, which is the point."""

    def test_home_already_ahead_returns_immediately(self):
        node, sent = _nav()
        _place(node, 0.0, 50.0, yaw_rad=math.pi)   # nose already on home

        node._enter_rtl(turn_first=True)

        assert node._rtl_turn_since is None
        assert _returns(sent), "delayed a return that needed no turn"
        assert not _yaws(sent)

    def test_home_just_inside_the_window_does_not_turn(self):
        """Engages on the same criterion as the waypoint gate: beyond
        fov_half_deg, i.e. genuinely in the masked rear."""
        node, sent = _nav()
        half = node._avoider.fov_half_deg
        # 120 deg off the nose with a 125 deg half-window: still scanned.
        angle = math.radians(120.0)
        _place(node, -50.0 * math.sin(angle), -50.0 * math.cos(angle))
        assert abs(node._home_bearing_deg()) < half

        node._enter_rtl(turn_first=True)
        assert node._rtl_turn_since is None
        assert _returns(sent)

    def test_no_home_bearing_returns_immediately(self):
        node, sent = _nav()
        _place(node, 0.0, 0.0)          # sitting on home; no bearing to face
        node._enter_rtl(turn_first=True)

        assert node._rtl_turn_since is None
        assert _returns(sent)

    def test_invalid_fix_returns_immediately(self):
        node, sent = _nav()
        node._fused = FusedState(x=25.0, y=43.3, valid=False)
        node._enter_rtl(turn_first=True)

        assert node._rtl_turn_since is None
        assert _returns(sent)

    def test_config_can_disable_it(self):
        node, sent = _nav(navigation={"avoidance_yaw_before_rtl": False})
        _place(node, *_HOME_BEHIND)
        node._enter_rtl(turn_first=True)

        assert node._rtl_turn_since is None
        assert _returns(sent)


class TestOnlyTheCalmPathsTurn:
    """A failsafe RTL fires on a low battery or a geofence breach, and an
    operator pressing RTL is usually reacting to something. Neither may be
    delayed to turn - only the three "job finished" returns are."""

    @pytest.mark.parametrize("reason", ["battery_low", "link_lost", "geofence"])
    def test_failsafe_rtl_goes_home_immediately(self, reason):
        """Every failsafe that RTLs. (battery_critical LANDs and max_altitude
        BRAKEs, so neither reaches _enter_rtl at all.)"""
        node, sent = _nav()
        _place(node, *_HOME_BEHIND)
        node._apply_failsafe(reason)

        assert node._rtl_turn_since is None
        assert _returns(sent), f"a {reason} failsafe RTL was delayed by a turn"

    def test_operator_rtl_goes_home_immediately(self):
        node, sent = _nav()
        _place(node, *_HOME_BEHIND)
        node._svc_rtl(None)

        assert node._rtl_turn_since is None
        assert _returns(sent)

    def test_operator_abort_goes_home_immediately(self):
        node, sent = _nav()
        _place(node, *_HOME_BEHIND)
        node._svc_abort_delivery(None)

        assert node._rtl_turn_since is None
        assert _returns(sent)

    def test_the_default_is_not_to_turn(self):
        """_enter_rtl() with no argument must behave exactly as before, so a
        caller added later does not silently inherit the delay."""
        node, sent = _nav()
        _place(node, *_HOME_BEHIND)
        node._enter_rtl()

        assert node._rtl_turn_since is None
        assert _returns(sent)


class TestTheSmartRtlFallbackTrap:
    """_do_rtl falls back to plain RTL if the FC is not in SMART_RTL 3 s after
    we asked, measured from _rtl_requested_at. Stamping that at the START of
    the turn would fire the fallback against a mode nobody has asked for yet."""

    def test_the_fallback_clock_does_not_run_during_the_turn(self):
        node, sent = _nav()
        node._return_mode = "SMART_RTL"
        _place(node, *_HOME_BEHIND)
        node._enter_rtl(turn_first=True)

        assert node._rtl_requested_at is None
        # Four seconds of turning - past the 3 s fallback threshold.
        node._rtl_turn_since -= 4.0
        node._rtl_turn_sent_at -= 4.0
        node._do_rtl()

        assert not node._rtl_fell_back, "fell back to RTL mid-turn"
        assert "RTL" not in [c.params.get("mode")
                             for c in sent if c.command == "set_mode"]

    def test_the_clock_starts_when_the_mode_is_actually_commanded(self):
        node, _ = _nav()
        node._return_mode = "SMART_RTL"
        _place(node, *_HOME_BEHIND)
        node._enter_rtl(turn_first=True)
        before = time.monotonic()

        _place(node, *_HOME_BEHIND, yaw_rad=math.radians(-150.0))
        node._do_rtl()

        assert node._rtl_requested_at is not None
        assert node._rtl_requested_at >= before


def test_disarming_mid_turn_clears_the_turn():
    node, _ = _nav()
    _place(node, *_HOME_BEHIND)
    node._enter_rtl(turn_first=True)
    node._armed = False
    node._do_rtl()

    assert node._phase == MissionPhase.COMPLETE
    assert node._rtl_turn_since is None


class TestTheRealDeliveryPath:
    """End-to-end through _do_hover, not by calling _enter_rtl directly.

    This is the scenario the feature exists for: the drop is done, the customer
    is behind the aircraft's tail because it flew in nose-first, and the next
    thing that happens is a return leg the FC flies.
    """

    def _dropped(self, node):
        node._phase = MissionPhase.HOVER
        node._mission.waypoints = [
            Waypoint(seq=0, x_m=25.0, y_m=43.3, alt_m=2.0, lat=12.9005, lon=77.6576)
        ]
        node._current_wp = 0            # +=1 in _do_hover exhausts the mission
        node._hover_until = time.monotonic() - 1.0      # hold already expired
        return node

    def test_hover_complete_with_home_behind_turns_first(self):
        node, sent = _nav()
        _place(node, *_HOME_BEHIND)
        self._dropped(node)

        node._do_hover()

        assert node._phase == MissionPhase.RTL
        assert node._rtl_turn_since is not None
        assert not _returns(sent), "handed over to the FC without turning"

        node._do_rtl()
        assert len(_yaws(sent)) == 1

        # Nose comes onto home; now it may hand over.
        _place(node, *_HOME_BEHIND, yaw_rad=math.radians(-150.0))
        node._do_rtl()
        assert _returns(sent)

    def test_hover_complete_pointed_home_is_unchanged(self):
        """The common case must not regress: no turn, no delay, hands over on
        the same tick exactly as it did before this feature existed."""
        node, sent = _nav()
        _place(node, 0.0, 50.0, yaw_rad=math.pi)
        self._dropped(node)

        node._do_hover()

        assert node._phase == MissionPhase.RTL
        assert node._rtl_turn_since is None
        assert _returns(sent)
        assert not _yaws(sent)
