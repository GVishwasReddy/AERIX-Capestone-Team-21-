"""§2.3 - the delivery-attempt mission state machine.

The state/event/transition *data* (states, typed events, the transition
table, and the guard predicates it references) is declared first, ahead of
the ``MissionFSM`` engine that walks it, deliberately: the table is
reviewed/tested as pure data (every transition's endpoints are valid
states, every non-terminal state has a configured timeout) independent of
the engine. ``MissionFSM`` itself (bottom of this file) is a thin,
generic walker - ``advance(event)`` looks up the current state's row for
``type(event)``, checks its guard against ``ctx``, and moves ``self.state``
if it matches; ``check_timeout()`` is the same lookup keyed on elapsed time
instead of an event. All of the actual perception/decision logic (running
landing_zone/recipient_auth/motion_monitor, deciding which ``Event`` to
raise) lives in ``delivery_node.py``, not here - see its module docstring.

State chain (happy path):

    SEARCHING_PERSON -> PERSON_FOUND -> SEARCHING_ZONE -> ZONE_FOUND
        -> DESCENDING -> AUTHENTICATING -> RELEASING -> ASCENDING -> RTL

Abort / replan branches:

    SEARCHING_PERSON  --(timeout)--------------> EXPANDING_SEARCH_RADIUS
    SEARCHING_ZONE    --(timeout)--------------> EXPANDING_SEARCH_RADIUS
    EXPANDING_SEARCH_RADIUS --(radius exceeded)-> ABORT_RTL
    DESCENDING        --(motion abort)---------> ABORT_DESCENT
    ABORT_DESCENT     --(replan)----------------> SEARCHING_ZONE
    ABORT_DESCENT     --(give up)---------------> ABORT_RTL
    AUTHENTICATING    --(ambiguous)-------------> HOVER_AND_RETRY
    AUTHENTICATING    --(auth failed)-----------> ABORT_RTL
    HOVER_AND_RETRY   --(retry exhausted)-------> ABORT_RTL
    * any non-terminal state --(timeout)--------> its configured target
      (either a named recovery state above, or ABORT_RTL - see TRANSITIONS)

``RTL`` and ``ABORT_RTL`` are terminal for *this* FSM: reaching either one
hands off to :class:`~drone_stack.nodes.navigation_node.NavigationNode`'s
own ``rtl`` service, which has its own (already-tested) RTL -> COMPLETE
lifecycle - the novelty layer does not duplicate it.
"""
from __future__ import annotations

import time
from dataclasses import dataclass, field
from enum import Enum
from typing import TYPE_CHECKING, Callable

from drone_stack.novelty.evidence_logger import EvidenceLogger
from drone_stack.novelty.types import (
    AuthDecision,
    MotionAbortEvent,
    PersonDetection,
    ZoneCandidate,
    to_jsonable,
)

if TYPE_CHECKING:
    # Deferred: config.py imports MissionState/TIMED_STATES from this module
    # for its own validator, so an unconditional top-level import here would
    # be circular - see config.py's own "avoid a config.py <-> mission_fsm.py
    # import cycle" comment.
    from drone_stack.novelty.config import MissionFsmConfig


class MissionState(str, Enum):
    SEARCHING_PERSON = "SEARCHING_PERSON"
    PERSON_FOUND = "PERSON_FOUND"
    SEARCHING_ZONE = "SEARCHING_ZONE"
    ZONE_FOUND = "ZONE_FOUND"
    DESCENDING = "DESCENDING"
    ABORT_DESCENT = "ABORT_DESCENT"
    AUTHENTICATING = "AUTHENTICATING"
    RELEASING = "RELEASING"
    ASCENDING = "ASCENDING"
    EXPANDING_SEARCH_RADIUS = "EXPANDING_SEARCH_RADIUS"
    HOVER_AND_RETRY = "HOVER_AND_RETRY"
    RTL = "RTL"
    ABORT_RTL = "ABORT_RTL"


#: States this FSM hands off from - nothing transitions FROM them, so they
#: need no configured timeout (the underlying NavigationNode RTL phase has
#: its own failsafe lifecycle; see docs/novelty/mission_fsm.md).
TERMINAL_STATES: frozenset[MissionState] = frozenset(
    {MissionState.RTL, MissionState.ABORT_RTL}
)

#: Every state that IS bounded by a config timeout - i.e. every non-terminal
#: state. Referenced by config.py's validator so a missing timeout entry
#: fails config load, not a hung flight.
TIMED_STATES: frozenset[MissionState] = frozenset(
    set(MissionState) - TERMINAL_STATES
)


# --------------------------------------------------------------------------- #
# Typed events - every transition is triggered by exactly one of these.
# --------------------------------------------------------------------------- #
@dataclass(frozen=True)
class Event:
    stamp: float = field(default_factory=time.time)


@dataclass(frozen=True)
class PersonDetectedEvent(Event):
    detections: tuple[PersonDetection, ...] = ()


@dataclass(frozen=True)
class PersonConfirmedEvent(Event):
    """Fired once disambiguation (if needed) has settled on one recipient."""
    track_id: int | None = None


@dataclass(frozen=True)
class ZoneFoundEvent(Event):
    candidates: tuple[ZoneCandidate, ...] = ()


@dataclass(frozen=True)
class ZoneConfirmedEvent(Event):
    candidate: ZoneCandidate | None = None


@dataclass(frozen=True)
class AltitudeReachedEvent(Event):
    altitude_m: float = 0.0


@dataclass(frozen=True)
class MotionAbortFsmEvent(Event):
    detail: MotionAbortEvent | None = None


@dataclass(frozen=True)
class ReplanEvent(Event):
    pass


@dataclass(frozen=True)
class GiveUpEvent(Event):
    pass


@dataclass(frozen=True)
class AuthSucceededEvent(Event):
    decision: AuthDecision | None = None


@dataclass(frozen=True)
class AuthFailedEvent(Event):
    decision: AuthDecision | None = None


@dataclass(frozen=True)
class AmbiguousEvent(Event):
    pass


@dataclass(frozen=True)
class RetryEvent(Event):
    pass


@dataclass(frozen=True)
class RetryExhaustedEvent(Event):
    pass


@dataclass(frozen=True)
class ReleaseCompleteEvent(Event):
    pass


@dataclass(frozen=True)
class AscendCompleteEvent(Event):
    pass


@dataclass(frozen=True)
class RadiusExpandedEvent(Event):
    new_radius_m: float = 0.0


@dataclass(frozen=True)
class MaxRadiusExceededEvent(Event):
    pass


@dataclass(frozen=True)
class TimeoutEvent(Event):
    """Fired by the engine itself when a state's configured timeout elapses -
    never published by another module."""
    state: MissionState | None = None


# --------------------------------------------------------------------------- #
# Guards - small, named predicates the table can reference so two rows with
# the same (from_state, event_type) but different targets stay data-driven
# instead of becoming an if/else in the engine. Each takes the FSM's mutable
# context dict (set by delivery_node.py) and returns bool.
# --------------------------------------------------------------------------- #
def has_confirmed_person(ctx: dict) -> bool:
    return ctx.get("recipient_track_id") is not None


def has_not_confirmed_person(ctx: dict) -> bool:
    return not has_confirmed_person(ctx)


def radius_below_max(ctx: dict) -> bool:
    return float(ctx.get("search_radius_m", 0.0)) < float(ctx.get("max_search_radius_m", 0.0))


def radius_at_max(ctx: dict) -> bool:
    return not radius_below_max(ctx)


# --------------------------------------------------------------------------- #
# The transition table - pure data. See module docstring for the diagram
# this table encodes; this IS the figure referenced in the patent spec.
# --------------------------------------------------------------------------- #
@dataclass(frozen=True)
class Transition:
    from_state: MissionState
    event_type: type[Event]
    to_state: MissionState
    guard: Callable[[dict], bool] | None = None
    description: str = ""


TRANSITIONS: tuple[Transition, ...] = (
    # -- SEARCHING_PERSON -----------------------------------------------
    Transition(MissionState.SEARCHING_PERSON, PersonDetectedEvent, MissionState.PERSON_FOUND,
               description="Vision reports >=1 person in frame."),
    Transition(MissionState.SEARCHING_PERSON, TimeoutEvent, MissionState.EXPANDING_SEARCH_RADIUS,
               description="No person found within state_timeout_s[SEARCHING_PERSON]."),

    # -- PERSON_FOUND ------------------------------------------------------
    Transition(MissionState.PERSON_FOUND, PersonConfirmedEvent, MissionState.SEARCHING_ZONE,
               description="Disambiguation (if needed) settled on one recipient."),
    Transition(MissionState.PERSON_FOUND, TimeoutEvent, MissionState.SEARCHING_PERSON,
               description="Person lost again before recipient was confirmed."),

    # -- SEARCHING_ZONE ------------------------------------------------------
    Transition(MissionState.SEARCHING_ZONE, ZoneFoundEvent, MissionState.ZONE_FOUND,
               description="landing_zone.score_candidates() returned >=1 candidate above min_score."),
    Transition(MissionState.SEARCHING_ZONE, TimeoutEvent, MissionState.EXPANDING_SEARCH_RADIUS,
               description="No candidate scored above min_score within the timeout."),

    # -- ZONE_FOUND ------------------------------------------------------
    Transition(MissionState.ZONE_FOUND, ZoneConfirmedEvent, MissionState.DESCENDING,
               description="Top-scoring candidate accepted; begin descent toward it."),
    Transition(MissionState.ZONE_FOUND, TimeoutEvent, MissionState.SEARCHING_ZONE,
               description="Chosen zone went stale (e.g. person/zone drifted) - re-search."),

    # -- EXPANDING_SEARCH_RADIUS ------------------------------------------------------
    Transition(MissionState.EXPANDING_SEARCH_RADIUS, RadiusExpandedEvent, MissionState.SEARCHING_ZONE,
               guard=has_confirmed_person,
               description="Recipient already confirmed - retry zone search at the wider radius."),
    Transition(MissionState.EXPANDING_SEARCH_RADIUS, RadiusExpandedEvent, MissionState.SEARCHING_PERSON,
               guard=has_not_confirmed_person,
               description="No recipient yet - retry person search at the wider radius."),
    Transition(MissionState.EXPANDING_SEARCH_RADIUS, MaxRadiusExceededEvent, MissionState.ABORT_RTL,
               description="search_radius_m reached max_search_radius_m with nothing found."),
    Transition(MissionState.EXPANDING_SEARCH_RADIUS, TimeoutEvent, MissionState.ABORT_RTL,
               description="Radius-expansion state itself timed out (safety net)."),

    # -- DESCENDING ------------------------------------------------------
    Transition(MissionState.DESCENDING, AltitudeReachedEvent, MissionState.AUTHENTICATING,
               description="Reached the descent target altitude above the chosen zone."),
    Transition(MissionState.DESCENDING, MotionAbortFsmEvent, MissionState.ABORT_DESCENT,
               description="motion_monitor flagged recipient motion or a zone intrusion."),
    Transition(MissionState.DESCENDING, TimeoutEvent, MissionState.ABORT_RTL,
               description="Descent took too long - safety abort, no retry."),

    # -- ABORT_DESCENT (transient - evaluated immediately on entry) ------
    Transition(MissionState.ABORT_DESCENT, ReplanEvent, MissionState.SEARCHING_ZONE,
               description="Re-evaluate: look for a (possibly new) safe zone."),
    Transition(MissionState.ABORT_DESCENT, GiveUpEvent, MissionState.ABORT_RTL,
               description="No safe replan available - abort."),
    Transition(MissionState.ABORT_DESCENT, TimeoutEvent, MissionState.ABORT_RTL,
               description="Re-evaluation itself timed out (safety net)."),

    # -- AUTHENTICATING ------------------------------------------------------
    Transition(MissionState.AUTHENTICATING, AuthSucceededEvent, MissionState.RELEASING,
               description="Both BLE and vision channels agreed within thresholds."),
    Transition(MissionState.AUTHENTICATING, AmbiguousEvent, MissionState.HOVER_AND_RETRY,
               description="Multiple people, disambiguation margin not met."),
    Transition(MissionState.AUTHENTICATING, AuthFailedEvent, MissionState.ABORT_RTL,
               description="One channel failed after its own retry window, or channels disagreed."),
    Transition(MissionState.AUTHENTICATING, TimeoutEvent, MissionState.ABORT_RTL,
               description="Authentication state itself timed out (safety net)."),

    # -- HOVER_AND_RETRY ------------------------------------------------------
    Transition(MissionState.HOVER_AND_RETRY, RetryEvent, MissionState.AUTHENTICATING,
               description="One retry attempt, per the disambiguation retry policy."),
    Transition(MissionState.HOVER_AND_RETRY, RetryExhaustedEvent, MissionState.ABORT_RTL,
               description="Retry also ambiguous/failed - abort."),
    Transition(MissionState.HOVER_AND_RETRY, TimeoutEvent, MissionState.ABORT_RTL,
               description="Hover-and-retry state itself timed out (safety net)."),

    # -- RELEASING ------------------------------------------------------
    Transition(MissionState.RELEASING, ReleaseCompleteEvent, MissionState.ASCENDING,
               description="Servo confirmed at the release PWM (SERVO_OUTPUT_RAW)."),
    Transition(MissionState.RELEASING, TimeoutEvent, MissionState.ABORT_RTL,
               description="Release not confirmed in time - safety abort."),

    # -- ASCENDING ------------------------------------------------------
    Transition(MissionState.ASCENDING, AscendCompleteEvent, MissionState.RTL,
               description="Reached safe ascent altitude - hand off to RTL."),
    Transition(MissionState.ASCENDING, TimeoutEvent, MissionState.RTL,
               description="Ascend timed out - already released, proceed to RTL regardless."),
)


def transitions_from(state: MissionState) -> tuple[Transition, ...]:
    return tuple(t for t in TRANSITIONS if t.from_state == state)


def validate_table() -> None:
    """Structural checks on TRANSITIONS - called by tests and at import time
    of the (future) engine. Raises AssertionError with a precise message."""
    all_states = set(MissionState)
    for t in TRANSITIONS:
        assert t.from_state in all_states, f"unknown from_state: {t.from_state}"
        assert t.to_state in all_states, f"unknown to_state: {t.to_state}"
        assert t.from_state not in TERMINAL_STATES, (
            f"terminal state {t.from_state} has an outgoing transition (table row: {t})"
        )
    for state in TIMED_STATES:
        has_timeout_row = any(
            t.from_state == state and t.event_type is TimeoutEvent for t in TRANSITIONS
        )
        assert has_timeout_row, f"non-terminal state {state} has no TimeoutEvent transition"


validate_table()


class MissionFSM:
    """Event-driven engine over TRANSITIONS - the walker for the table
    declared above.

    ``ctx`` is a plain mutable dict the caller (``delivery_node.py``) both
    reads (guards - see ``has_confirmed_person`` etc. above) and writes
    (e.g. ``recipient_track_id``, ``search_radius_m``) as the mission
    progresses - kept as a dict rather than a typed dataclass so the guard
    callables stay simple and uniform instead of needing a per-field
    accessor each.
    """

    def __init__(
        self,
        config: "MissionFsmConfig",
        logger: EvidenceLogger | None,
        ctx: dict | None = None,
    ) -> None:
        self.config = config
        self.logger = logger
        self.state = MissionState.SEARCHING_PERSON
        self.ctx: dict = ctx if ctx is not None else {}
        self._state_entered_at = time.time()

    def reset(self) -> None:
        """Return to the initial state with a fresh entry timestamp - call
        when starting a new delivery attempt. Does NOT clear ``ctx``; the
        caller owns what (if anything) should carry over."""
        self.state = MissionState.SEARCHING_PERSON
        self._state_entered_at = time.time()

    def elapsed_in_state(self, now: float | None = None) -> float:
        now = now if now is not None else time.time()
        return now - self._state_entered_at

    def timeout_s(self, state: MissionState | None = None) -> float | None:
        """The configured timeout for *state* (default: the current state),
        or ``None`` for a terminal state (never times out)."""
        state = state if state is not None else self.state
        if state in TERMINAL_STATES:
            return None
        return self.config.state_timeout_s[state.value]

    def check_timeout(self, now: float | None = None) -> bool:
        """If the current state's configured timeout has elapsed, fire a
        ``TimeoutEvent`` (advancing the FSM per TRANSITIONS) and return
        ``True``. A no-op returning ``False`` for a terminal state or if the
        timeout has not yet elapsed. Call once per step whenever no domain
        event fired this tick - see ``delivery_node.py``."""
        timeout = self.timeout_s()
        if timeout is None:
            return False
        now = now if now is not None else time.time()
        if self.elapsed_in_state(now) < timeout:
            return False
        return self.advance(TimeoutEvent(stamp=now, state=self.state))

    def advance(self, event: Event) -> bool:
        """Apply *event* to the current state via TRANSITIONS. Returns
        ``True`` and updates ``self.state`` if a matching, guard-passing row
        was found for ``type(event)``; otherwise a no-op returning
        ``False`` - an event with no matching row for the current state is
        simply not a valid transition right now, not an error (e.g. a stray
        ``PersonDetectedEvent`` while already ``DESCENDING``)."""
        if self.state in TERMINAL_STATES:
            return False
        from_state = self.state
        for t in transitions_from(from_state):
            if t.event_type is not type(event):
                continue
            if t.guard is not None and not t.guard(self.ctx):
                continue
            self.state = t.to_state
            self._state_entered_at = event.stamp
            self._log_transition(from_state, t, event)
            return True
        return False

    def _log_transition(self, from_state: MissionState, t: Transition, event: Event) -> None:
        if self.logger is None:
            return
        self.logger.log_event(
            event_name="fsm_transition",
            mission_state=from_state.value,
            inputs=to_jsonable(event),
            computed_values={"to_state": t.to_state.value, "description": t.description},
            threshold=self.timeout_s(from_state),
            decision=f"{from_state.value} -> {t.to_state.value}",
        )
