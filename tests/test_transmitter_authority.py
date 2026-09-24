"""Regression tests for the 2026-08-22 flight.

The aircraft auto-took-off, the altitude hardlock put it into BRAKE, and it
held there - ignoring the transmitter - until the pack went flat and it fell
out of the sky. Three defects combined:

  1. ``_send("brake")`` never recorded ``_commanded_mode``, so
     ``_check_pilot_override`` bailed out at "we have not commanded anything
     yet" and could never latch, no matter what the pilot did.
  2. The hardlock re-sent BRAKE on every tick (~10 Hz), pulling the aircraft
     back out of whatever mode the pilot selected within 100 ms - and, through
     ``_set_mode``, resetting the override grace timer each time so it could
     never expire.
  3. The hardlock returned before the battery checks, so
     ``battery_critical -> LAND`` was unreachable for as long as the hardlock
     was active. That is what turned a stuck hover into a fall.

The transmitter is the manual override and must always win.
"""
from __future__ import annotations

import time

from drone_stack.bus import MessageBus
from drone_stack.bus.topics import Topics
from drone_stack.msg import (
    Battery,
    FlightMode,
    FusedState,
    GpsFix,
    MissionPhase,
    RcChannels,
)
from drone_stack.nodes.navigation_node import NavigationNode
from drone_stack.srv import ServiceRegistry
from drone_stack.utils.config import Config

HOME_LAT, HOME_LON = 12.9004084, 77.6576687
NEUTRAL = [1500, 1500, 1000, 1500, 1000, 1500, 1500, 1500]


def _nav():
    """A navigator with a good fix at home, as it was on the day."""
    bus = MessageBus()
    services = ServiceRegistry()
    nav = NavigationNode(bus, Config.load(), services)
    nav._on_gps(GpsFix(fix_type=3, satellites=32, lat=HOME_LAT, lon=HOME_LON))
    return bus, nav


def _above_ceiling(nav):
    """Put the aircraft where the hardlock will trip, as it did on the day.

    Armed, because on the day it was flying. The hardlock is gated on _armed:
    relative_alt only re-zeros when the FC arms, so on the ground it reports
    whatever the barometer has drifted to and the ceiling is meaningless.
    """
    nav._phase = MissionPhase.NAVIGATE
    nav._armed = True
    nav._fused = FusedState(x=0.0, y=0.0, alt_rel_m=9.0, yaw=0.0, valid=True)


# -- the incident ------------------------------------------------------------
def test_the_transmitter_wins_while_the_hardlock_is_braking():
    """The pilot must be able to take the aircraft out of a hardlock BRAKE.

    This is the flight that was lost. The hardlock is re-braking every tick;
    the pilot flips the transmitter's mode switch and the navigator has to
    notice and stand down. Previously the grace timer was reset by our own
    re-brake on every tick, so it never expired.
    """
    bus, nav = _nav()
    _above_ceiling(nav)
    nav.step()
    assert nav._phase == MissionPhase.HOLD
    assert nav._commanded_mode == "BRAKE", "the node did not record its own BRAKE"
    nav._on_mode(FlightMode(mode_name="BRAKE"))          # the FC obeys

    nav._on_mode(FlightMode(mode_name="LOITER"))         # pilot flips the switch
    nav.step()
    assert nav._mode_mismatch_since is not None, (
        "the override grace timer never started"
    )
    started = nav._mode_mismatch_since
    for _ in range(30):
        nav.step()
        assert nav._mode_mismatch_since == started, (
            "our own re-brake reset the grace timer - the pilot can never "
            "outlast it, which is exactly how the aircraft was lost"
        )

    nav._mode_mismatch_since -= 5.0
    nav.step()
    assert nav._pilot_override is True
    assert nav._phase == MissionPhase.MANUAL

    sent: list = []
    bus.subscribe(Topics.MAVLINK_CMD, sent.append)
    sent.clear()
    for _ in range(40):
        nav.step()
    assert sent == [], f"kept commanding the aircraft while the pilot flew: {sent}"


def test_the_navigator_does_not_mistake_its_own_brake_for_the_pilot():
    """The mirror image: our BRAKE is ours, not a takeover."""
    bus, nav = _nav()
    _above_ceiling(nav)
    nav.step()
    nav._on_mode(FlightMode(mode_name="BRAKE"))
    for _ in range(30):
        nav.step()
    assert nav._pilot_override is False


def test_battery_critical_outranks_the_altitude_hardlock():
    """A dying pack must still land the aircraft, ceiling breach or not.

    The hardlock used to return before the battery checks ran, so the one
    failsafe that could have saved the airframe was unreachable for the whole
    time the aircraft was held above the ceiling.
    """
    bus, nav = _nav()
    _above_ceiling(nav)
    nav._armed = True
    nav._battery = Battery(voltage_v=13.0, remaining_pct=8.0)
    assert nav._check_failsafe() == "battery_critical"


def test_a_disarmed_bench_aircraft_trips_nothing():
    """A bench aircraft trips NEITHER failsafe.

    0 V (no pack connected) is not a critical battery, and a barometer that has
    drifted since the datum was set is not a climb. Both checks are gated on
    _armed for the same reason: neither input means anything until the FC has
    armed. Sitting on the ground the Pi read 5.08 m against a 2.0 m ceiling and
    logged the hardlock 1637 times in three minutes, burying every other error
    in the journal.
    """
    bus, nav = _nav()
    _above_ceiling(nav)
    nav._armed = False
    nav._battery = Battery(voltage_v=0.0, remaining_pct=0.0)
    assert nav._check_failsafe() is None


# -- transmitter authority ---------------------------------------------------
def test_rc_movement_hands_over_immediately():
    """Moving the mode switch is a takeover on the very next tick, no grace."""
    bus, nav = _nav()
    _above_ceiling(nav)
    nav._on_rc(RcChannels(channels=list(NEUTRAL), rssi=200, count=8))
    nav.step()
    assert nav._pilot_override is False, "the first frame is only a baseline"

    moved = list(NEUTRAL)
    moved[4] = 1900                                   # mode switch
    nav._on_rc(RcChannels(channels=moved, rssi=200, count=8))
    nav.step()
    assert nav._pilot_override is True
    assert nav._phase == MissionPhase.MANUAL


def test_a_transmitter_switched_on_but_untouched_is_not_a_takeover():
    bus, nav = _nav()
    _above_ceiling(nav)
    for _ in range(30):
        nav._on_rc(RcChannels(channels=list(NEUTRAL), rssi=200, count=8))
        nav.step()
    assert nav._pilot_override is False


def test_a_stale_rc_frame_is_not_a_takeover():
    """Frames keep arriving briefly after the transmitter is switched off."""
    bus, nav = _nav()
    _above_ceiling(nav)
    nav._on_rc(RcChannels(channels=list(NEUTRAL), rssi=200, count=8))
    nav.step()
    moved = list(NEUTRAL)
    moved[4] = 1900
    nav._on_rc(
        RcChannels(channels=moved, rssi=200, count=8, stamp=time.time() - 30.0)
    )
    nav.step()
    assert nav._pilot_override is False


def test_an_absent_transmitter_is_not_a_takeover():
    """No RC link at all must not strand the aircraft in MANUAL."""
    bus, nav = _nav()
    _above_ceiling(nav)
    nav._on_rc(RcChannels(channels=[0] * 8, rssi=0, count=0))
    for _ in range(20):
        nav.step()
    assert nav._pilot_override is False


def test_the_operator_can_resume_after_a_takeover():
    """Clearing the override re-baselines the sticks, or resume is impossible."""
    bus, nav = _nav()
    _above_ceiling(nav)
    nav._on_rc(RcChannels(channels=list(NEUTRAL), rssi=200, count=8))
    nav.step()
    moved = list(NEUTRAL)
    moved[4] = 1900
    nav._on_rc(RcChannels(channels=moved, rssi=200, count=8))
    nav.step()
    assert nav._pilot_override is True

    nav._clear_pilot_override()
    nav._on_rc(RcChannels(channels=list(moved), rssi=200, count=8))
    for _ in range(10):
        nav.step()
    assert nav._pilot_override is False, (
        "re-latched on stale stick deltas - the operator could never resume"
    )


# -- command hygiene ---------------------------------------------------------
def test_the_navigator_stops_respamming_a_mode_the_fc_already_reports():
    """SET_MODE at 10 Hz is what locked the pilot out and ratcheted the climb."""
    bus, nav = _nav()
    sent: list = []
    bus.subscribe(Topics.MAVLINK_CMD, sent.append)
    _above_ceiling(nav)
    nav.step()
    nav._on_mode(FlightMode(mode_name="BRAKE"))
    sent.clear()
    for _ in range(50):
        nav.step()
    assert [c.command for c in sent] == [], (
        f"re-sent a mode the FC was already in: {[c.command for c in sent]}"
    )


# -- refusal is not a takeover ------------------------------------------------
def test_an_fc_that_refuses_guided_is_not_a_pilot_takeover():
    """The 15:39 abort on 2026-08-23.

    ArduPilot rejects GUIDED outright on a thin fix or a complaining EKF, so
    the aircraft simply stays in STABILIZE. STABILIZE is a pilot-selectable
    mode, so the override check read "we asked for GUIDED, the FC says
    STABILIZE" as a human on the sticks and aborted a delivery nobody
    touched. The FC never *left* STABILIZE - it never entered GUIDED at all.
    """
    bus, nav = _nav()
    nav._on_mode(FlightMode(mode_name="STABILIZE"))
    nav._send("takeoff", altitude=3.0)          # implies GUIDED
    assert nav._commanded_mode == "GUIDED"
    for _ in range(40):
        nav.step()
    assert nav._pilot_override is False, (
        "a refused mode change was mistaken for a pilot takeover"
    )
    # The refusal is still the operator's real problem, so it has to be
    # recorded - not silently swallowed. _status_message is owned by whichever
    # phase is running (IDLE here) and legitimately overwrites it, so assert
    # on the durable record rather than the transient banner.
    assert nav._mode_refusal_logged == ("GUIDED", "STABILIZE")


def test_leaving_a_mode_we_actually_reached_is_a_takeover():
    """The discriminator: a departure from GUIDED is real, and must latch."""
    bus, nav = _nav()
    nav._on_mode(FlightMode(mode_name="STABILIZE"))
    nav._send("takeoff", altitude=3.0)          # implies GUIDED
    nav._on_mode(FlightMode(mode_name="GUIDED"))
    nav.step()
    assert nav._commanded_mode_reached is True

    nav._on_mode(FlightMode(mode_name="STABILIZE"))   # pilot flips the switch
    nav.step()
    assert nav._mode_mismatch_since is not None
    nav._mode_mismatch_since -= 5.0
    nav.step()
    assert nav._pilot_override is True


def test_a_refusal_does_not_block_a_later_real_takeover():
    """After a refusal, the transmitter must still work."""
    bus, nav = _nav()
    nav._on_mode(FlightMode(mode_name="STABILIZE"))
    nav._send("takeoff", altitude=3.0)
    for _ in range(20):
        nav.step()
    assert nav._pilot_override is False

    nav._on_rc(RcChannels(channels=list(NEUTRAL), rssi=200, count=8))
    nav.step()
    moved = list(NEUTRAL)
    moved[4] = 1900
    nav._on_rc(RcChannels(channels=moved, rssi=200, count=8))
    nav.step()
    assert nav._pilot_override is True


def test_the_gcs_resume_button_also_clears_the_override():
    """The GCS resume goes through the NL parser, not the ``resume`` service.

    Both entry points have to clear the latch. While only the service path did,
    resume answered "resuming mission", left ``_pilot_override`` set, and every
    delivery accepted afterwards aborted a tick later - the operator pressed
    resume over and over while the aircraft sat in MANUAL.
    """
    bus = MessageBus()
    services = ServiceRegistry()
    nav = NavigationNode(bus, Config.load(), services)
    nav._on_gps(GpsFix(fix_type=3, satellites=32, lat=HOME_LAT, lon=HOME_LON))
    nav._latch_override("transmitter moved (ch 6)", "GUIDED")
    assert nav._pilot_override is True

    result = services.call("nl_command", text="resume")

    assert result.success, result.message
    assert nav._pilot_override is False, (
        "NL resume left the override latched - step() keeps standing down, "
        "_start_requested is never consumed, and every accepted delivery "
        "aborts a tick later"
    )
    assert nav._phase != MissionPhase.MANUAL


def test_both_resume_entry_points_agree():
    """Whichever way the operator resumes, the latch must come off."""
    for describe, resume in (
        ("resume service", lambda s: s.call("resume")),
        ("GCS/NL resume", lambda s: s.call("nl_command", text="resume")),
    ):
        bus = MessageBus()
        services = ServiceRegistry()
        nav = NavigationNode(bus, Config.load(), services)
        nav._on_gps(GpsFix(fix_type=3, satellites=32, lat=HOME_LAT, lon=HOME_LON))
        nav._latch_override("transmitter moved (ch 6)", "GUIDED")

        assert resume(services).success, describe
        assert nav._pilot_override is False, describe


def test_starting_a_mission_is_refused_while_the_pilot_has_control():
    """A latched override must refuse the start, not silently swallow it.

    ``step`` stands down while the latch is set, so ``_start_requested`` is
    never consumed. Reporting success meant the delivery node went ACCEPTED and
    then ABORTED a tick later, over and over, with nothing anywhere saying that
    a deliberate hand-back was the missing step.
    """
    for describe, start in (
        ("start_mission service", lambda s: s.call("start_mission")),
        ("NL start mission", lambda s: s.call("nl_command", text="start mission")),
    ):
        bus = MessageBus()
        services = ServiceRegistry()
        nav = NavigationNode(bus, Config.load(), services)
        nav._on_gps(GpsFix(fix_type=3, satellites=32, lat=HOME_LAT, lon=HOME_LON))
        nav._mission = nav._demo_mission()
        nav._latch_override("transmitter moved (ch 1, 2)", "GUIDED")

        result = start(services)

        assert not result.success, f"{describe}: started while the pilot had control"
        assert "resume" in result.message.lower(), (
            f"{describe}: refusal does not tell the operator to press RESUME - "
            f"got {result.message!r}"
        )
        assert nav._start_requested is False, describe


def test_starting_a_mission_still_works_once_control_is_handed_back():
    """The guard must not outlive the takeover it is guarding against."""
    bus = MessageBus()
    services = ServiceRegistry()
    nav = NavigationNode(bus, Config.load(), services)
    nav._on_gps(GpsFix(fix_type=3, satellites=32, lat=HOME_LAT, lon=HOME_LON))
    nav._mission = nav._demo_mission()
    nav._latch_override("transmitter moved (ch 1, 2)", "GUIDED")
    assert not services.call("start_mission").success

    assert services.call("resume").success
    assert nav._pilot_override is False

    result = services.call("start_mission")
    assert result.success, result.message
    assert nav._start_requested is True


def test_the_hardlock_still_fires_the_moment_it_is_armed():
    """The gate must not become an off switch: arm it and the ceiling is live
    again, whatever the failsafes_enabled bench switch says."""
    bus, nav = _nav()
    _above_ceiling(nav)
    assert nav._check_failsafe() == "max_altitude"


def test_the_hardlock_logs_once_per_breach_not_once_per_tick(caplog):
    """The brake keeps being applied at loop rate; only the log is limited. An
    ERROR repeated ten times a second is how a real fault goes unnoticed."""
    import logging
    bus, nav = _nav()
    _above_ceiling(nav)
    with caplog.at_level(logging.ERROR, logger="drone.navigation"):
        for _ in range(30):
            nav.step()
    hardlock = [r for r in caplog.records if "ALTITUDE HARDLOCK" in r.getMessage()]
    assert len(hardlock) == 1, f"{len(hardlock)} hardlock ERRORs in 30 ticks"
    assert nav._phase == MissionPhase.HOLD, "but it must still be braking"
