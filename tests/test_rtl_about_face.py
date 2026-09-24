"""Stage 1 of the pre-RTL turn: the post-delivery 180 deg about-face.

Why a second stage at all, when test_rtl_yaw.py already turns before RTL. That
turn is CONDITIONAL and BEARING-SEEKING: it fires only when home is outside the
scanned window, servos onto ``_home_bearing_deg()``, and releases inside
``yaw_release_deg``. On a delivery that ends roughly pointed homeward it does
nothing at all - which is correct for its purpose and wrong for this one.

This stage is unconditional and has no EXTERNAL target bearing. The aircraft
flew in forwards and finishes nose-on to the customer, so "turn around" is a
fixed 180 deg of displacement rather than a landmark to seek.

It is still flown closed-loop, against a target heading latched from the yaw
estimate when the stage starts (``_about_face_target``). Until 2026-09-19 it was
not: it integrated ``abs(yaw - last_yaw)`` per tick and stopped at 180. That
absolute value RECTIFIES estimator noise - jitter adds to the total whichever
way it jitters - so at 10 Hz the counter ran fast by however noisy the estimate
happened to be that flight, and the turn stopped short by a different margin
every time. It also could not see an overshoot at all: past 180 the count only
grows. Both failure modes are pinned in TestTheTurnIsNotAnIntegral.

The two stages compose: about-face first, then the home-bearing test is still
asked (an inbound leg that dodged may not leave home dead ahead after 180 deg).

Ordering trap this file guards: ``MAV_CMD_CONDITION_YAW`` goes out RELATIVE
(param4=1, see mavlink_interface). Stage 2 can resend its command freely because
a bearing-seeking turn re-aims at the same place. A resent RELATIVE 180 commands
a WHOLE NEW 180 from the current heading, so this stage must resend what is
LEFT. See TestTheResendIsNotAdditive.
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


#: 50 m out on a bearing that leaves home 150 deg off the nose at yaw 0 - the
#: same fixture test_rtl_yaw.py uses, so the two files describe one aircraft.
#: After a clean 180 this becomes -30 deg: inside the window, stage 2 no-ops.
_HOME_BEHIND = (25.0, 43.3)


def _place(node, east: float, north: float, yaw_rad: float = 0.0):
    node._fused = FusedState(x=east, y=north, alt_rel_m=2.0, yaw=yaw_rad,
                             valid=True)
    return node


def _set_yaw(node, yaw_rad: float):
    """Rotate the aircraft, keeping position. Yaw is wrapped as a real EKF's is."""
    f = node._fused
    node._fused = FusedState(x=f.x, y=f.y, alt_rel_m=f.alt_rel_m,
                             yaw=math.atan2(math.sin(yaw_rad),
                                            math.cos(yaw_rad)),
                             valid=True)


def _fly_turn(node, total_deg: float, steps: int = 36, start_deg: float = 0.0):
    """Tick _do_rtl while rotating the aircraft, as the FC would.

    Feeds the yaw estimate in small increments and wraps it at +/-pi, which is
    the case a start-vs-now comparison gets wrong.
    """
    for i in range(1, steps + 1):
        _set_yaw(node, math.radians(start_deg + total_deg * i / steps))
        node._do_rtl()


def _returns(sent) -> list:
    return [c for c in sent
            if c.command in ("rtl", "smart_rtl")
            or (c.command == "set_mode"
                and c.params.get("mode") in ("RTL", "SMART_RTL"))]


def _yaws(sent) -> list:
    return [c for c in sent if c.command == "yaw"]


class TestItAlwaysEngages:
    """The defining difference from stage 2: no bearing test to pass."""

    @pytest.mark.parametrize("east,north", [
        _HOME_BEHIND,
        (0.0, 50.0),        # home dead astern-ish / nose already on home
        (0.0, -50.0),       # home dead ahead
        (50.0, 0.0),        # home abeam
    ])
    def test_it_turns_regardless_of_where_home_is(self, east, north):
        node, sent = _nav()
        _place(node, east, north)

        node._enter_rtl(turn_first=True)

        assert node._about_face_since is not None
        assert not _returns(sent), (
            f"handed over without turning: {[c.command for c in sent]}")

    def test_it_runs_before_the_home_bearing_stage(self):
        node, _ = _nav()
        _place(node, *_HOME_BEHIND)     # stage 2 would also want this one

        node._enter_rtl(turn_first=True)

        assert node._about_face_since is not None
        assert node._rtl_turn_since is None, "stage 2 ran first"

    def test_the_phase_is_honest_about_returning(self):
        node, _ = _nav()
        _place(node, *_HOME_BEHIND)
        node._enter_rtl(turn_first=True)
        assert node._phase == MissionPhase.RTL

    def test_the_first_command_asks_for_a_full_180(self):
        node, sent = _nav()
        _place(node, *_HOME_BEHIND)
        node._enter_rtl(turn_first=True)
        node._do_rtl()

        yaws = _yaws(sent)
        assert len(yaws) == 1
        assert yaws[0].params["angle"] == pytest.approx(180.0)
        assert yaws[0].params["direction"] in (1, -1)
        assert yaws[0].params["rate"] == node._avoider.yaw_rate_deg_s

    def test_turning_commands_no_mode_change(self):
        """Same constraint stage 2 is under: the hover-complete caller warns
        that two mode changes in one tick risk a refused GUIDED stranding the
        aircraft over the customer. Holding station costs no command."""
        node, sent = _nav()
        _place(node, *_HOME_BEHIND)
        node._enter_rtl(turn_first=True)
        for _ in range(10):
            node._do_rtl()

        assert [c for c in sent if c.command == "set_mode"] == []

    def test_status_line_says_what_it_is_doing(self):
        node, _ = _nav()
        _place(node, *_HOME_BEHIND)
        node._enter_rtl(turn_first=True)
        node._do_rtl()
        assert "about-face" in node._status_message.lower()


class TestTheResendIsNotAdditive:
    """A RELATIVE yaw resend commands a whole new turn. Send what is LEFT."""

    def test_it_is_not_resent_every_tick(self):
        node, sent = _nav()
        _place(node, *_HOME_BEHIND)
        node._enter_rtl(turn_first=True)
        for _ in range(30):             # 3 s of 10 Hz ticks
            node._do_rtl()

        assert len(_yaws(sent)) == 1

    def test_a_resend_asks_for_the_remainder_not_another_180(self):
        node, sent = _nav()
        _place(node, *_HOME_BEHIND)
        node._enter_rtl(turn_first=True)
        node._do_rtl()                                  # first send: 180

        _set_yaw(node, math.radians(90.0))              # half way round
        node._do_rtl()
        node._about_face_sent_at = 0.0                  # force the resend
        node._do_rtl()

        yaws = _yaws(sent)
        assert len(yaws) == 2
        assert yaws[1].params["angle"] == pytest.approx(90.0, abs=20.0), (
            "a resent RELATIVE 180 would turn a further 180 - it must carry "
            f"the remainder, got {yaws[1].params['angle']:.0f}")


class TestItFinishes:
    def test_a_measured_180_hands_over_to_the_return(self):
        node, sent = _nav()
        _place(node, *_HOME_BEHIND)
        node._enter_rtl(turn_first=True)
        _fly_turn(node, 180.0)

        assert node._about_face_since is None
        assert _returns(sent), "turned 180 and never went home"

    def test_the_turn_is_measured_not_timed(self):
        """An aircraft that never moves must NOT be reported as turned. This is
        the servoN_raw lesson in another costume: a command that was accepted is
        not a thing that happened."""
        node, sent = _nav()
        _place(node, *_HOME_BEHIND)
        node._enter_rtl(turn_first=True)
        for _ in range(40):             # 4 s of ticks, yaw never changes
            node._do_rtl()

        assert node._about_face_since is not None
        assert not _returns(sent)

    def test_wrapping_past_180_still_counts_as_turned(self):
        """Yaw wraps at +/-pi. Comparing start against now is the obvious
        implementation and it breaks here: a 180 deg turn leaves the two
        readings a wrapped 180 apart, which is the same number the knife edge
        reports either side of. Integrating per-tick deltas does not care."""
        node, sent = _nav()
        _place(node, *_HOME_BEHIND, yaw_rad=math.radians(120.0))
        node._enter_rtl(turn_first=True)
        _fly_turn(node, 180.0, start_deg=120.0)         # 120 -> 300, wraps

        assert node._about_face_since is None
        assert _returns(sent)

    def test_a_turn_that_stops_short_does_not_count(self):
        node, sent = _nav()
        _place(node, *_HOME_BEHIND)
        node._enter_rtl(turn_first=True)
        _fly_turn(node, 120.0)          # well outside about_face_tol_deg

        assert node._about_face_since is not None
        assert not _returns(sent)

    def test_losing_the_estimate_mid_turn_returns_rather_than_turning_blind(self):
        node, sent = _nav()
        _place(node, *_HOME_BEHIND)
        node._enter_rtl(turn_first=True)
        node._do_rtl()
        node._fused = FusedState(x=25.0, y=43.3, alt_rel_m=2.0, yaw=0.0,
                                 valid=False)
        node._do_rtl()

        assert _returns(sent), "kept turning against an estimate it lost"


class TestTheSharedBudget:
    """Both stages expire against ONE clock, because that clock is the window
    in which _check_failsafe is suppressed - and suppression does not care
    which stage is spending it."""

    def test_a_stalled_turn_returns_anyway_rather_than_holding(self):
        node, sent = _nav()
        _place(node, *_HOME_BEHIND)
        node._enter_rtl(turn_first=True)
        node._rtl_yaw_started = time.monotonic() - 99.0
        node._do_rtl()

        assert _returns(sent), "held instead of coming home"
        assert node._about_face_since is None

    def test_stage_two_inherits_the_clock_rather_than_restarting_it(self):
        """The regression this guards: 8 s of about-face followed by a fresh 8 s
        of turn-onto-home is 16 s of suppressed battery/link/geofence failsafes,
        and rtl_yaw_timeout_s's own comment says 8 s is survivable and 20 s is
        not."""
        node, sent = _nav()
        _place(node, *_HOME_BEHIND)
        node._enter_rtl(turn_first=True)
        started = node._rtl_yaw_started
        assert started is not None

        # Finish stage 1 having already spent most of the budget.
        node._rtl_yaw_started = time.monotonic() - (
            node._avoider.rtl_yaw_total_s - 0.5)
        _fly_turn(node, 180.0)

        if node._rtl_turn_since is not None:
            assert node._rtl_yaw_started == pytest.approx(
                node._rtl_yaw_started), "stage 2 restarted the shared clock"
            node._do_rtl()
            assert _returns(sent), "stage 2 spent a second full budget"

    def test_the_budget_covers_a_real_180(self):
        """A floor, not a preference: the turn physically takes this long."""
        node, _ = _nav()
        turn_s = 180.0 / node._avoider.yaw_rate_deg_s
        assert node._avoider.rtl_yaw_total_s > turn_s, (
            f"budget {node._avoider.rtl_yaw_total_s}s cannot fit a 180 at "
            f"{node._avoider.yaw_rate_deg_s} deg/s ({turn_s:.1f}s)")


class TestTheStagesCompose:
    def test_after_the_about_face_the_home_bearing_is_still_asked(self):
        """180 deg from _HOME_BEHIND puts home 30 deg off the nose - inside the
        window - so stage 2 correctly finds nothing to do and hands over."""
        node, sent = _nav()
        _place(node, *_HOME_BEHIND)
        node._enter_rtl(turn_first=True)
        _fly_turn(node, 180.0)

        assert node._rtl_turn_since is None
        assert _returns(sent)

    def test_home_still_behind_after_the_about_face_turns_again(self):
        """An inbound leg that dodged can leave home behind even after 180 deg.
        Stage 2 is consulted, not assumed.

        Home starts ~30 deg off the nose, so the about-face swings it to ~150 -
        outside the 125 deg window - and stage 2 then has real work to do. This
        is the case that makes "just do the 180 and hand over" wrong.
        """
        node, sent = _nav()
        # _home_bearing_deg negates position (home IS the ENU origin), so a
        # SOUTH-east offset is what leaves home ~30 deg off a north-facing nose.
        _place(node, 25.0, -43.3)
        before = node._home_bearing_deg()
        assert abs(before) <= node._avoider.fov_half_deg, (
            f"fixture wrong: home already outside the window at {before:.0f}")

        node._enter_rtl(turn_first=True)
        _fly_turn(node, 180.0)

        assert node._about_face_since is None, "still about-facing after 180"
        after = node._home_bearing_deg()
        assert abs(after) > node._avoider.fov_half_deg
        assert node._rtl_turn_since is not None, (
            f"home is {after:.0f} deg off and stage 2 did not engage")


class TestOnlyTheCalmPathsTurn:
    """Unchanged contract from stage 2: a failsafe or an operator means NOW."""

    @pytest.mark.parametrize("reason", ["battery_low", "link_lost", "geofence"])
    def test_failsafe_rtl_goes_home_immediately(self, reason):
        node, sent = _nav()
        _place(node, *_HOME_BEHIND)
        node._enter_rtl()               # no turn_first - the failsafe path
        assert node._about_face_since is None
        assert _returns(sent)

    def test_operator_rtl_goes_home_immediately(self):
        node, sent = _nav()
        _place(node, *_HOME_BEHIND)
        node._enter_rtl()
        assert node._about_face_since is None
        assert _returns(sent)


class TestItCanBeTurnedOff:
    def test_config_disables_it_and_stage_two_behaves_as_before(self):
        node, sent = _nav(navigation={"avoidance_rtl_about_face": False})
        _place(node, *_HOME_BEHIND)
        node._enter_rtl(turn_first=True)

        assert node._about_face_since is None
        assert node._rtl_turn_since is not None, "stage 2 did not take over"

    def test_no_attitude_estimate_skips_it_rather_than_turning_unverifiably(self):
        node, sent = _nav()
        node._fused = None
        node._enter_rtl(turn_first=True)

        assert node._about_face_since is None
        assert _returns(sent)


def test_disarming_mid_turn_clears_the_turn():
    node, _ = _nav()
    _place(node, *_HOME_BEHIND)
    node._enter_rtl(turn_first=True)
    assert node._about_face_since is not None

    node._armed = False
    node._do_rtl()

    assert node._about_face_since is None
    assert node._rtl_yaw_started is None
    assert node._phase == MissionPhase.COMPLETE


class TestTheSmartRtlFallbackTrap:
    def test_the_fallback_clock_does_not_run_during_the_about_face(self):
        """_do_rtl's SMART_RTL fallback measures from _rtl_requested_at ("if the
        FC is not in SMART_RTL 3 s after we asked, fall back"). Starting that
        clock when the TURN starts would fire the fallback mid-turn against a
        mode nobody has asked for yet."""
        node, sent = _nav()
        _place(node, *_HOME_BEHIND)
        node._enter_rtl(turn_first=True)

        assert node._rtl_requested_at is None
        for _ in range(50):             # 5 s of ticks, not turning
            node._do_rtl()
        assert not _returns(sent), "fell back to RTL during the about-face"


class TestTheRealDeliveryPath:
    def test_hover_complete_at_the_drop_point_about_faces(self):
        """The path the user actually flies: hover over the customer expires,
        nothing follows it, so the aircraft goes home - turning around first."""
        node, sent = _nav()
        _place(node, *_HOME_BEHIND)
        node._phase = MissionPhase.HOVER
        node._mission.waypoints = [
            Waypoint(seq=0, x_m=25.0, y_m=43.3, alt_m=2.0,
                     lat=12.9005, lon=77.6576)
        ]
        node._current_wp = 0            # += 1 in _do_hover exhausts the mission
        node._hover_until = time.monotonic() - 1.0      # hold already expired
        sent.clear()

        node._do_hover()

        assert node._phase == MissionPhase.RTL
        assert node._about_face_since is not None
        assert not _returns(sent), "went home without turning around"


class TestTheTurnIsNotAnIntegral:
    """The 2026-09-19 fix: completion is an error to a latched heading, not a
    running total of movement. These two tests are the difference."""

    def test_jitter_in_place_is_never_mistaken_for_a_turn(self):
        """An aircraft whose yaw estimate twitches but whose nose never moves
        has not turned, however long you watch it.

        This is what made the turn random. Accumulating abs(delta) adds ~2 deg
        per tick of +/-1 deg jitter, so ~90 ticks - nine seconds at the 10 Hz
        loop rate - "completes" a 180 the aircraft never flew. How short the
        real turn ended up depended only on how noisy that flight's estimate
        was, which is exactly the reported symptom.
        """
        node, sent = _nav()
        _place(node, *_HOME_BEHIND)
        node._enter_rtl(turn_first=True)

        for i in range(400):                    # 40 s of ticks at 10 Hz
            _set_yaw(node, math.radians(1.0 if i % 2 else -1.0))
            node._do_rtl()

        assert node._about_face_since is not None, (
            "reported a 180 from a stationary aircraft - the completion test "
            "is integrating noise, not measuring heading")
        assert not _returns(sent)

    def test_an_overshoot_is_corrected_rather_than_declared_finished(self):
        """Past the target the error changes sign; a running total cannot.

        The old implementation's `remaining = 180 - turned` only ever fell, so
        200 deg of turn read as finished-and-then-some and the aircraft handed
        over to the FC pointing 20 deg off. The error to a latched heading says
        20 deg the other way, and the resend asks for it back.
        """
        node, sent = _nav()
        _place(node, *_HOME_BEHIND)
        node._enter_rtl(turn_first=True)
        node._do_rtl()                                  # first send

        # Arrive PAST the target between two samples - a fast slew, or a tick
        # the estimate did not update on. A turn that merely passes through the
        # target is caught inside about_face_tol_deg and finishes there, which
        # is correct; this is the case where it is not caught.
        _set_yaw(node, math.radians(200.0))             # 20 deg too far
        node._do_rtl()
        assert node._about_face_since is not None, (
            "declared the about-face finished 20 deg past the target")

        node._about_face_sent_at = 0.0                  # force the resend
        node._do_rtl()

        last = _yaws(sent)[-1]
        assert last.params["angle"] == pytest.approx(20.0, abs=8.0), (
            f"asked for {last.params['angle']:.0f} deg to correct a 20 deg "
            "overshoot")
        assert last.params["direction"] == -1, (
            "overshot clockwise and is still being told to turn clockwise")

    def test_a_dropped_sample_does_not_lose_the_turn(self):
        """A tick with no fresh yaw costs an accumulator that movement forever.
        An error to a latched heading simply re-reads where the nose is."""
        node, sent = _nav()
        _place(node, *_HOME_BEHIND)
        node._enter_rtl(turn_first=True)

        # Jump straight to the far side, as a node that missed every
        # intermediate sample would see it.
        _set_yaw(node, math.radians(180.0))
        node._do_rtl()

        assert node._about_face_since is None, (
            "the nose is 180 deg round and the turn is not finished")
        assert _returns(sent)
