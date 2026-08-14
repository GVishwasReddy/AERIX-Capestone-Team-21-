"""novelty/mission_fsm.py - the MissionFSM engine walking the TRANSITIONS
table declared in the same module. The table's own structural integrity
(every endpoint a valid state, every non-terminal state timeout-bound) is
checked by ``validate_table()``, which runs at import time - these tests
cover the ENGINE that walks it: event matching, guard selection, timeouts,
terminal-state rejection, and full-table sanity (every declared transition
is reachable via ``advance``)."""
from __future__ import annotations

import pytest

from drone_stack.novelty.config import NoveltyConfig
from drone_stack.novelty.mission_fsm import (
    TERMINAL_STATES,
    TRANSITIONS,
    AltitudeReachedEvent,
    AmbiguousEvent,
    AscendCompleteEvent,
    AuthFailedEvent,
    AuthSucceededEvent,
    GiveUpEvent,
    MaxRadiusExceededEvent,
    MissionFSM,
    MissionState,
    MotionAbortFsmEvent,
    PersonConfirmedEvent,
    PersonDetectedEvent,
    RadiusExpandedEvent,
    ReleaseCompleteEvent,
    ReplanEvent,
    RetryEvent,
    RetryExhaustedEvent,
    TimeoutEvent,
    ZoneConfirmedEvent,
    ZoneFoundEvent,
    validate_table,
)


@pytest.fixture
def cfg(valid_config_dir):
    return NoveltyConfig.load(valid_config_dir).mission_fsm


def _fsm(cfg, ctx: dict | None = None) -> MissionFSM:
    return MissionFSM(config=cfg, logger=None, ctx=ctx)


# --------------------------------------------------------------------------- #
# Table sanity (re-asserted explicitly, not just relied on at import time)
# --------------------------------------------------------------------------- #
def test_validate_table_passes():
    validate_table()  # raises AssertionError on failure


def test_every_declared_transition_is_reachable_via_advance(cfg):
    """Every row in TRANSITIONS can actually fire advance() from its
    from_state, given a ctx that satisfies its guard (if any)."""
    for t in TRANSITIONS:
        ctx = {"recipient_track_id": 1, "search_radius_m": 0.0, "max_search_radius_m": 30.0}
        if t.guard is not None and not t.guard(ctx):
            ctx = {"recipient_track_id": None, "search_radius_m": 0.0, "max_search_radius_m": 30.0}
            assert t.guard(ctx), f"transition {t} has a guard satisfied by neither ctx variant tried"
        fsm = _fsm(cfg, ctx=ctx)
        fsm.state = t.from_state
        fsm._state_entered_at = 0.0
        event = t.event_type(stamp=1.0)
        assert fsm.advance(event) is True, f"transition {t} did not fire"
        assert fsm.state == t.to_state


# --------------------------------------------------------------------------- #
# Basic advance()
# --------------------------------------------------------------------------- #
def test_initial_state_is_searching_person(cfg):
    assert _fsm(cfg).state == MissionState.SEARCHING_PERSON


def test_advance_on_matching_event_transitions(cfg):
    fsm = _fsm(cfg)
    ok = fsm.advance(PersonDetectedEvent(detections=()))
    assert ok is True
    assert fsm.state == MissionState.PERSON_FOUND


def test_advance_on_non_matching_event_is_a_noop(cfg):
    fsm = _fsm(cfg)
    ok = fsm.advance(ZoneFoundEvent(candidates=()))  # not a SEARCHING_PERSON event
    assert ok is False
    assert fsm.state == MissionState.SEARCHING_PERSON


def test_advance_updates_state_entered_at_to_the_events_stamp(cfg):
    fsm = _fsm(cfg)
    fsm.advance(PersonDetectedEvent(stamp=500.0, detections=()))
    assert fsm.elapsed_in_state(now=500.0) == pytest.approx(0.0)
    assert fsm.elapsed_in_state(now=510.0) == pytest.approx(10.0)


@pytest.mark.parametrize("terminal", sorted(TERMINAL_STATES, key=lambda s: s.value))
def test_terminal_states_reject_every_event(cfg, terminal):
    fsm = _fsm(cfg)
    fsm.state = terminal
    for event_cls in (PersonDetectedEvent, TimeoutEvent, ZoneFoundEvent, AuthSucceededEvent):
        assert fsm.advance(event_cls(stamp=1.0)) is False
        assert fsm.state == terminal


# --------------------------------------------------------------------------- #
# Guard selection
# --------------------------------------------------------------------------- #
def test_expanding_search_radius_routes_to_zone_search_when_recipient_confirmed(cfg):
    fsm = _fsm(cfg, ctx={"recipient_track_id": 7})
    fsm.state = MissionState.EXPANDING_SEARCH_RADIUS
    fsm.advance(RadiusExpandedEvent(new_radius_m=10.0))
    assert fsm.state == MissionState.SEARCHING_ZONE


def test_expanding_search_radius_routes_to_person_search_when_no_recipient_yet(cfg):
    fsm = _fsm(cfg, ctx={"recipient_track_id": None})
    fsm.state = MissionState.EXPANDING_SEARCH_RADIUS
    fsm.advance(RadiusExpandedEvent(new_radius_m=10.0))
    assert fsm.state == MissionState.SEARCHING_PERSON


# --------------------------------------------------------------------------- #
# check_timeout()
# --------------------------------------------------------------------------- #
def test_check_timeout_fires_after_the_configured_window(cfg):
    fsm = _fsm(cfg)
    fsm._state_entered_at = 0.0
    timeout = cfg.state_timeout_s[MissionState.SEARCHING_PERSON.value]
    fired = fsm.check_timeout(now=timeout + 1.0)
    assert fired is True
    assert fsm.state == MissionState.EXPANDING_SEARCH_RADIUS


def test_check_timeout_does_not_fire_before_the_window_elapses(cfg):
    fsm = _fsm(cfg)
    fsm._state_entered_at = 0.0
    timeout = cfg.state_timeout_s[MissionState.SEARCHING_PERSON.value]
    fired = fsm.check_timeout(now=timeout - 1.0)
    assert fired is False
    assert fsm.state == MissionState.SEARCHING_PERSON


def test_check_timeout_is_a_noop_on_terminal_states(cfg):
    fsm = _fsm(cfg)
    fsm.state = MissionState.RTL
    fsm._state_entered_at = 0.0
    assert fsm.check_timeout(now=1e9) is False
    assert fsm.state == MissionState.RTL


def test_timeout_s_is_none_for_terminal_states(cfg):
    fsm = _fsm(cfg)
    assert fsm.timeout_s(MissionState.RTL) is None
    assert fsm.timeout_s(MissionState.ABORT_RTL) is None


# --------------------------------------------------------------------------- #
# reset()
# --------------------------------------------------------------------------- #
def test_reset_returns_to_searching_person_and_restarts_the_clock(cfg):
    fsm = _fsm(cfg)
    fsm.advance(PersonDetectedEvent(stamp=1.0, detections=()))
    assert fsm.state == MissionState.PERSON_FOUND
    fsm.reset()
    assert fsm.state == MissionState.SEARCHING_PERSON
    assert fsm.elapsed_in_state() == pytest.approx(0.0, abs=0.1)


# --------------------------------------------------------------------------- #
# Evidence logging
# --------------------------------------------------------------------------- #
def test_advance_logs_a_transition_record(cfg):
    calls = []

    class _StubLogger:
        def log_event(self, **kwargs):
            calls.append(kwargs)

    fsm = MissionFSM(config=cfg, logger=_StubLogger())
    fsm.advance(PersonDetectedEvent(stamp=1.0, detections=()))
    assert len(calls) == 1
    record = calls[0]
    assert record["mission_state"] == "SEARCHING_PERSON"
    assert record["decision"] == "SEARCHING_PERSON -> PERSON_FOUND"
    assert record["computed_values"]["to_state"] == "PERSON_FOUND"
    assert record["threshold"] == cfg.state_timeout_s["SEARCHING_PERSON"]


def test_advance_does_not_log_when_no_transition_fires(cfg):
    calls = []

    class _StubLogger:
        def log_event(self, **kwargs):
            calls.append(kwargs)

    fsm = MissionFSM(config=cfg, logger=_StubLogger())
    fsm.advance(ZoneFoundEvent(candidates=()))  # not valid from SEARCHING_PERSON
    assert calls == []


# --------------------------------------------------------------------------- #
# Full-table walkthroughs
# --------------------------------------------------------------------------- #
def test_happy_path_walkthrough_reaches_rtl(cfg):
    fsm = _fsm(cfg, ctx={"recipient_track_id": None})
    fsm.advance(PersonDetectedEvent(detections=()))
    fsm.ctx["recipient_track_id"] = 1
    fsm.advance(PersonConfirmedEvent(track_id=1))
    fsm.advance(ZoneFoundEvent(candidates=()))
    fsm.advance(ZoneConfirmedEvent(candidate=None))
    fsm.advance(AltitudeReachedEvent(altitude_m=2.0))
    fsm.advance(AuthSucceededEvent(decision=None))
    fsm.advance(ReleaseCompleteEvent())
    fsm.advance(AscendCompleteEvent())
    assert fsm.state == MissionState.RTL
    assert fsm.state in TERMINAL_STATES


def test_no_zone_found_eventually_aborts_via_radius_expansion(cfg):
    fsm = _fsm(cfg, ctx={"recipient_track_id": 1})
    fsm.state = MissionState.EXPANDING_SEARCH_RADIUS
    fsm.advance(MaxRadiusExceededEvent())
    assert fsm.state == MissionState.ABORT_RTL


def test_ble_or_vision_channel_failure_aborts(cfg):
    fsm = _fsm(cfg)
    fsm.state = MissionState.AUTHENTICATING
    fsm.advance(AuthFailedEvent(decision=None))
    assert fsm.state == MissionState.ABORT_RTL


def test_ambiguous_recipients_hover_then_abort_when_retries_exhausted(cfg):
    fsm = _fsm(cfg)
    fsm.state = MissionState.AUTHENTICATING
    fsm.advance(AmbiguousEvent())
    assert fsm.state == MissionState.HOVER_AND_RETRY
    fsm.advance(RetryExhaustedEvent())
    assert fsm.state == MissionState.ABORT_RTL


def test_recipient_motion_or_intrusion_triggers_abort_descent(cfg):
    fsm = _fsm(cfg)
    fsm.state = MissionState.DESCENDING
    fsm.advance(MotionAbortFsmEvent(detail=None))
    assert fsm.state == MissionState.ABORT_DESCENT


def test_abort_descent_can_replan_back_to_zone_search_or_give_up(cfg):
    fsm = _fsm(cfg)
    fsm.state = MissionState.ABORT_DESCENT
    fsm.advance(ReplanEvent())
    assert fsm.state == MissionState.SEARCHING_ZONE

    fsm2 = _fsm(cfg)
    fsm2.state = MissionState.ABORT_DESCENT
    fsm2.advance(GiveUpEvent())
    assert fsm2.state == MissionState.ABORT_RTL
