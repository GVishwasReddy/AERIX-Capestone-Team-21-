"""The end of the flight: RTL arrives, and the aircraft LANDS.

Until 2026-09-19 nothing in NavigationNode ever left ``MissionPhase.RTL``
except a disarm. The aircraft did usually come down, but only because
``RTL_ALT_FINAL`` happened to be 0 on this flight controller - a parameter this
node cannot read, on hardware the GCS cannot see. The phase the operator was
shown said RTL until the motors stopped, and if the FC had ever refused the
return, or finished it at altitude, nothing would have noticed.

So the navigator now decides for itself when the return is over and commands
LAND. The descent rate is still the FC's (LAND_SPEED, 30 cm/s measured) -
LAND is the one autonomous descent ArduPilot flies entirely on its own sensors,
which is the point of using it rather than commanding a goto at a lower
altitude against an altitude estimate that may be the thing that went wrong.

The measured reason the old arrangement failed in the field is NOT in this file
because it is not in this repo: ``RTL_ALT`` was 300 cm against a 2.0 m ceiling
with a 1.0 m hardlock margin, so RTL's first act - climbing to RTL_ALT - tripped
``_check_failsafe``'s altitude hardlock, which is checked ahead of both the
``failsafes_enabled`` master switch and the "do not fight an in-progress RTL"
phase exemption. The aircraft was braked into MissionPhase.HOLD before the
return leg ever developed. See scripts/set_rtl_alt.py.
"""
from __future__ import annotations

import copy
import math
import time

import pytest

from drone_stack.bus import MessageBus
from drone_stack.bus.topics import Topics
from drone_stack.msg import (
    FusedState,
    MissionPhase,
    Obstacle,
    ObstacleArray,
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
    node._armed = True
    node._has_flown = True
    node._home = (12.9, 77.6)
    node._mode = "RTL"
    node._phase = MissionPhase.RTL
    sent.clear()
    return node, sent


def _at(node, east: float, north: float, alt: float = 2.0, yaw: float = 0.0):
    node._fused = FusedState(x=east, y=north, alt_rel_m=alt, yaw=yaw, valid=True)
    return node


def _returning(node, ago_s: float):
    """Put the node in the state _commit_rtl leaves: return commanded ago_s ago."""
    node._phase = MissionPhase.RTL
    node._rtl_requested_at = time.monotonic() - ago_s
    node._rtl_fell_back = True          # suppress the SMART_RTL fallback branch
    node._about_face_since = None
    node._rtl_turn_since = None


def _lands(sent) -> list:
    return [c for c in sent
            if c.command == "land"
            or (c.command == "set_mode" and c.params.get("mode") == "LAND")]


class TestItLandsWhenItGetsHome:
    def test_arriving_over_the_pad_commands_land(self):
        node, sent = _nav()
        _at(node, 0.5, 0.5)                     # 0.7 m from home
        _returning(node, ago_s=30.0)

        node._do_rtl()

        assert node._phase is MissionPhase.LAND
        assert _lands(sent), "over home and still not landing"

    def test_still_out_on_the_return_keeps_returning(self):
        node, sent = _nav()
        _at(node, 30.0, 40.0)                   # 50 m out
        _returning(node, ago_s=30.0)

        node._do_rtl()

        assert node._phase is MissionPhase.RTL
        assert not _lands(sent)

    def test_the_radius_is_land_radius_not_home_radius(self):
        """home_radius_m (15 m) is a REPORTING threshold sized for GPS scatter.
        Landing there would put the aircraft 15 m from the ground station."""
        node, sent = _nav()
        _at(node, 10.0, 0.0)                    # inside home_radius, outside land
        _returning(node, ago_s=30.0)

        node._do_rtl()

        assert node._phase is MissionPhase.RTL, (
            "landed at home_radius_m - that is the reporting threshold, not "
            "the pad")


class TestItDoesNotLandOnTheCustomer:
    def test_the_arm_delay_covers_the_moment_rtl_is_commanded(self):
        """For the first seconds of RTL the aircraft is still over the drop
        point. On a short test flight that point is inside land_radius_m of
        home, and without the delay the return would end before it began."""
        node, sent = _nav()
        _at(node, 0.5, 0.5)
        _returning(node, ago_s=0.0)

        node._do_rtl()

        assert node._phase is MissionPhase.RTL
        assert not _lands(sent)

    def test_the_turn_stages_are_not_an_arrival(self):
        """_rtl_requested_at is None through both pre-RTL turn stages - nothing
        has been commanded yet - so the arrival test must not fire there."""
        node, sent = _nav()
        _at(node, 0.5, 0.5)
        node._phase = MissionPhase.RTL
        node._rtl_requested_at = None
        node._about_face_since = time.monotonic()
        node._about_face_target = math.pi
        node._about_face_yaw = 0.0

        node._do_rtl()

        assert not _lands(sent), "landed mid about-face"


class TestTheReturnIsBounded:
    def test_a_return_that_never_arrives_lands_anyway(self):
        """Orbiting until the battery failsafe decides is not an outcome."""
        node, sent = _nav(delivery={"land_rtl_timeout_s": 60.0})
        _at(node, 80.0, 80.0)                   # still 113 m out
        _returning(node, ago_s=61.0)

        node._do_rtl()

        assert node._phase is MissionPhase.LAND
        assert _lands(sent)
        assert "timed out" in node._land_reason

    def test_losing_the_fix_mid_return_still_ends_in_a_landing(self):
        node, sent = _nav(delivery={"land_rtl_timeout_s": 60.0})
        node._fused = FusedState(x=0.0, y=0.0, alt_rel_m=2.0, yaw=0.0,
                                 valid=False)
        _returning(node, ago_s=61.0)

        node._do_rtl()

        assert node._phase is MissionPhase.LAND
        assert _lands(sent)


class TestLandIsAsserted:
    """A commanded mode is not a mode the FC accepted - the servoN_raw lesson."""

    def test_a_refused_land_is_re_asserted(self):
        node, sent = _nav()
        _at(node, 0.0, 0.0, alt=2.0)
        node._phase = MissionPhase.LAND
        node._mode = "GUIDED"                   # FC never took LAND
        node._land_sent_at = 0.0

        node._do_land()

        assert _lands(sent), "FC refused LAND and nothing re-asserted it"

    def test_a_refused_land_says_so_on_the_status_line(self):
        node, _ = _nav()
        _at(node, 0.0, 0.0)
        node._phase = MissionPhase.LAND
        node._mode = "GUIDED"
        node._land_sent_at = 0.0

        node._do_land()

        assert "GUIDED" in node._status_message

    def test_it_is_not_respammed_every_tick(self):
        node, sent = _nav()
        _at(node, 0.0, 0.0)
        node._phase = MissionPhase.LAND
        node._mode = "GUIDED"
        node._land_sent_at = 0.0

        for _ in range(30):                     # 3 s of 10 Hz ticks
            node._do_land()

        assert len(_lands(sent)) == 1

    def test_a_landing_in_progress_is_left_alone(self):
        node, sent = _nav()
        _at(node, 0.0, 0.0, alt=1.2)
        node._phase = MissionPhase.LAND
        node._mode = "LAND"
        node._land_sent_at = 0.0

        node._do_land()

        assert not _lands(sent), "re-commanded LAND at an aircraft already landing"
        assert "1.2" in node._status_message

    def test_the_disarm_ends_the_flight(self):
        node, _ = _nav()
        _at(node, 0.0, 0.0, alt=0.0)
        node._phase = MissionPhase.LAND
        node._armed = False

        node._do_land()

        assert node._phase is MissionPhase.DISARMED


class TestAvoidanceStaysAwakeDuringTheReturn:
    """The Pi does not STEER the return - BendyRuler does - but it must not go
    blind either. _avoiding drives the GCS avoidance panel, and a panel that
    reads clear for the whole return leg is indistinguishable from avoidance
    being switched off. That is exactly how this was reported from the field.
    """

    def test_the_avoidance_state_is_evaluated_on_the_return_leg(self, monkeypatch):
        node, _ = _nav()
        _at(node, 30.0, 40.0)
        _returning(node, ago_s=30.0)

        seen = []
        real = node._avoid_decision
        monkeypatch.setattr(node, "_avoid_decision",
                            lambda: (seen.append(1), real())[1])

        node._do_rtl()

        assert seen, "_do_rtl never asked the obstacle field anything"

    def test_a_clear_path_reports_clear_rather_than_stale(self):
        node, _ = _nav()
        _at(node, 30.0, 40.0)
        _returning(node, ago_s=30.0)
        node._avoiding = True                   # stale from the outbound leg

        node._do_rtl()

        assert node._avoiding is False


def _obs(*pairs: tuple[float, float]) -> ObstacleArray:
    """ObstacleArray from (distance_m, bearing_deg) pairs. + bearing = right."""
    return ObstacleArray(
        obstacles=[Obstacle(distance_m=d, bearing_deg=b) for d, b in pairs]
    )


def _blocked(node):
    """Something inside the hard-brake distance, dead ahead and to the left."""
    stop = node._avoider.stop
    node._obstacles = _obs((stop * 0.5, 0.0), (stop * 0.6, -30.0))
    node._avoid_enabled = True
    return node


def _velocities(sent) -> list:
    return [c for c in sent if c.command == "velocity"]


def _brakes(sent) -> list:
    return [c for c in sent
            if c.command == "brake"
            or (c.command == "set_mode" and c.params.get("mode") == "BRAKE")]


class TestThePiDoesNotSteerTheReturn:
    """RTL is flown by the flight controller. The Pi may stop it. It may not
    drive it.

    ArduPilot honours SET_POSITION_TARGET_LOCAL_NED in GUIDED and nowhere
    else, so a velocity setpoint issued during RTL is silently discarded. The
    danger is not the discarded packet - it is that the code around it carries
    on as though the aircraft dodged: it claims the tick, spends the
    rtl_brake_max_s budget, and writes "steering +45 deg" to the status line an
    operator is reading instead of reaching for the sticks.

    These tests exist so that a future "the RTL dodge does nothing, let me make
    it active" cannot quietly reintroduce that without _ensure_guided() and a
    deliberate decision to take the return away from the flight controller.
    """

    def test_an_obstacle_inside_the_hard_stop_brakes(self):
        node, sent = _nav()
        _at(node, 30.0, 40.0)
        _returning(node, ago_s=30.0)
        _blocked(node)

        node._do_rtl()

        assert _brakes(sent), (
            "something is inside the hard stop and nothing braked")
        assert node._avoiding is True

    def test_no_velocity_setpoint_is_issued_while_the_fc_flies_the_return(self):
        node, sent = _nav()
        _at(node, 30.0, 40.0)
        _returning(node, ago_s=30.0)
        _blocked(node)

        for _ in range(20):                     # 2 s of 10 Hz ticks
            node._do_rtl()

        assert not _velocities(sent), (
            "streamed a velocity setpoint during RTL - the FC discards those "
            "outside GUIDED, so this commands nothing while reporting success")

    def test_the_status_line_does_not_claim_to_be_steering(self):
        node, _ = _nav()
        _at(node, 30.0, 40.0)
        _returning(node, ago_s=30.0)
        _blocked(node)

        node._do_rtl()

        assert "steering" not in node._status_message.lower(), (
            f"status line says {node._status_message!r} - this node is not "
            "steering the aircraft")
        assert "HOLDING" in node._status_message

    def test_the_clear_bearing_is_offered_as_advice(self):
        """An operator who can see the aircraft needs to know which way is
        open. Reporting a heading is not the same as flying one."""
        node, sent = _nav()
        _at(node, 30.0, 40.0)
        _returning(node, ago_s=30.0)
        _blocked(node)

        node._do_rtl()

        assert "clear" in node._status_message
        assert not _velocities(sent)

    def test_a_spent_budget_hands_the_return_back(self):
        node, sent = _nav()
        _at(node, 30.0, 40.0)
        _returning(node, ago_s=30.0)
        _blocked(node)

        node._do_rtl()                          # first STOP stamps the clock
        node._mode = "BRAKE"                    # the FC took the brake
        node._rtl_braking_since = (
            time.monotonic() - node._avoider.rtl_brake_max_s - 1.0)
        sent.clear()

        node._do_rtl()

        assert node._rtl_braking_since is None, "still holding past the budget"
        assert any(c.command == "rtl"
                   or (c.command == "set_mode"
                       and c.params.get("mode") in ("RTL", "SMART_RTL"))
                   for c in sent), (
            "released the brake without re-commanding the return - that leaves "
            "the FC holding in BRAKE, which is the 2026-08-31 deadlock")

    def test_the_brake_is_not_respammed_once_the_fc_holds(self):
        """_send suppresses a mode-only command the FC is already in. Worth
        pinning here: re-sending SET_MODE every tick is what locked the pilot
        out before, and this path runs at 10 Hz for up to rtl_brake_max_s."""
        node, sent = _nav()
        _at(node, 30.0, 40.0)
        _returning(node, ago_s=30.0)
        _blocked(node)

        node._do_rtl()
        node._mode = "BRAKE"                    # the FC took it
        sent.clear()

        for _ in range(30):                     # 3 s of ticks
            node._do_rtl()

        assert not _brakes(sent), (
            "kept commanding BRAKE at an aircraft already braked")
        assert "HOLDING" in node._status_message

    def test_a_clear_path_never_brakes(self):
        node, sent = _nav()
        _at(node, 30.0, 40.0)
        _returning(node, ago_s=30.0)
        node._obstacles = _obs((40.0, 0.0))
        node._avoid_enabled = True

        node._do_rtl()

        assert not _brakes(sent)
        assert not _velocities(sent)
        assert node._avoiding is False
