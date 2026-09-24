"""Decides when an armed aircraft has stopped flying.

Why this exists
---------------
The flight recorder starts on the ARM edge and used to end on the DISARM edge
and nothing else. That is not the same question. **Arming is a permission, not
a flight**, and there are several ordinary ways for an aircraft to stop flying
while staying armed:

* **Emergency stop.** An RC aux switch set to Motor Emergency Stop cuts the
  outputs and leaves the aircraft armed indefinitely.
* **Flight termination / a failsafe that latches.** The FC reports it through
  ``HEARTBEAT.system_status``, not through the arm bit.
* **It landed and nobody disarmed it.** Common on the bench and after an
  auto-LAND that does not reach the disarm delay.

In every one of those the recording kept running to ``max_seconds`` (1800 s),
so the render never started and the operator opened the REPLAY panel to find
the *previous* flight still sitting there. From the ground that reads exactly
as "replay is broken".

So the recorder now asks this class the real question - *is it still flying?* -
and stops the clip on the first credible "no".

Design notes
------------
**Three fast paths and one slow one.** Disarm, an emergency ``system_status``
and an emergency STATUSTEXT are unambiguous: the FC is telling us directly, so
they fire immediately. "It looks landed" is an *inference* from altitude and
speed, so it has to survive ``land_dwell_s`` of continuous agreement before it
counts. A GPS altitude blip or a momentary hover at low level must never be
able to end a clip mid-flight - that failure mode loses the rest of the
flight, which is far worse than a clip that runs a few seconds long.

**Latching, and deliberately not symmetric.** Once a flight has ended, this
detector stays ended until the next ARM edge. It does NOT restart the clip if
the aircraft lifts off again while still armed. One arm cycle produces one
clip, which is the operator's mental model, and - because only one clip is
ever kept - restarting would let a twenty-second second hop overwrite the full
flight that preceded it. Missing the second hop is a much cheaper mistake.

**Never guesses from missing data.** Until the FC has actually reported an arm
state, and while telemetry is stale, the answer is "still flying" - i.e. keep
recording. Refusing to decide is always safe here: the clip stays open and the
next real signal closes it.
"""
from __future__ import annotations

from typing import Optional

# MAV_STATE values that mean the aircraft is not going to fly out of this by
# itself. CRITICAL is deliberately NOT here: a critical failsafe is usually an
# RTL, which is still very much a flight and is the part you most want on film.
MAV_STATE_EMERGENCY = 6
MAV_STATE_POWEROFF = 7
MAV_STATE_FLIGHT_TERMINATION = 8
_DEAD_STATES = frozenset({MAV_STATE_EMERGENCY, MAV_STATE_POWEROFF,
                          MAV_STATE_FLIGHT_TERMINATION})

# ArduPilot STATUSTEXT fragments that announce the motors being cut. Matched
# lowercase against the whole line. Kept narrow on purpose: "failsafe" alone is
# far too broad - a battery failsafe triggers an RTL that is still a flight.
_DEAD_TEXT = (
    "emergency stop",
    "motor emergency stop",
    "flight termination",
    "crash: disarming",
    "disarming motors",
    # What this ArduPilot build actually prints when the RC aux switch is
    # thrown ("RC8: MotorEStop HIGH"). None of the phrases above appear in it,
    # so an RC e-stop never ended the clip. Matched with the state word: the
    # same switch coming back prints "... MotorEStop LOW", which is not a stop.
    "motorestop high",
)


class FlightEndDetector:
    """Watches telemetry and reports the moment a flight is over.

    Fed once per FC heartbeat by :class:`GcsHub`. Returns a short reason
    string - which becomes the recording's ``reason`` and the operator-facing
    console line - or ``None`` while the aircraft is still flying.
    """

    def __init__(self, settings: Optional[dict] = None) -> None:
        settings = settings or {}
        self.enabled = bool(settings.get("stop_on_flight_end", True))
        # Below this height above home the aircraft cannot meaningfully be
        # flying. The mission ceiling is 2.0 m, so this has to be well under
        # it or a legitimate low hover would end the clip.
        self.land_altitude_m = float(settings.get("land_altitude_m", 0.8))
        self.land_speed_ms = float(settings.get("land_speed_ms", 0.4))
        self.land_vspeed_ms = float(settings.get("land_vspeed_ms", 0.3))
        # How long everything above must hold before "landed" is believed.
        # Measured, not guessed: flight_20260922_150703 hopped to 1.3 m, sat
        # armed on the ground for ~10.5 s, then flew the real flight to 6 m.
        # 6 s ended that clip at the hop and lost the flight. Replaying every
        # 21-23 Sep log, 12/15/20 s never fire mid-flight (real landings are
        # disarmed first). A long dwell is nearly free: the hub trims the clip
        # back to touchdown_t, so it delays the render, not the replay's end.
        self.land_dwell_s = float(settings.get("land_dwell_s", 15.0))

        self._armed = False
        self._arm_seen = False
        # "Landed" means CAME BACK DOWN, so it needs the aircraft to have been
        # up first. Without this latch an aircraft armed on the pad - low,
        # still, exactly what landed looks like - ended its own clip
        # land_dwell_s after arming, before it ever took off.
        self.airborne_alt_m = float(settings.get("airborne_alt_m", 1.0))
        self._airborne = False
        # Latched once a flight has ended; cleared only by the next ARM edge.
        self._ended = False
        # When the landed-looking condition first became true, or None.
        self._quiet_since: Optional[float] = None
        # The _quiet_since that a "landed" verdict was based on. Kept apart
        # because mark_ended() clears _quiet_since, and the hub needs it
        # afterwards to trim the clip back to touchdown.
        self._touchdown: Optional[float] = None

    # -- edges ---------------------------------------------------------------
    def arm_edge(self, armed: bool) -> None:
        """Called by the hub on a real ARM/DISARM transition."""
        self._armed = bool(armed)
        self._arm_seen = True
        if armed:
            # A new flight. Everything the previous one latched is history.
            self._ended = False
            self._airborne = False
            self._quiet_since = None
            self._touchdown = None

    def mark_ended(self) -> None:
        """Latch the current flight as over (the hub calls this once it has
        actually stopped the clip, so the reason is never reported twice)."""
        self._ended = True
        self._quiet_since = None

    @property
    def ended(self) -> bool:
        return self._ended

    @property
    def airborne(self) -> bool:
        return self._airborne

    @property
    def touchdown_t(self) -> Optional[float]:
        """When the aircraft came to rest on the ground (same clock as the
        ``now`` passed to :meth:`update`), or None if it is not resting.

        Valid after a "landed" verdict, and also at a disarm that arrives while
        the aircraft is already sitting quietly - either way the replay can end
        here instead of at the moment the decision was made."""
        return self._touchdown if self._touchdown is not None else self._quiet_since

    # -- the question --------------------------------------------------------
    def update(self, *, armed: bool, mode: str, system_status: int,
               altitude_m: float, ground_speed_ms: float, vert_speed_ms: float,
               fc_text: str, now: float) -> Optional[str]:
        """Return a stop reason, or ``None`` to keep recording.

        Called once per FC heartbeat with the freshest telemetry the hub has.
        ``fc_text`` is the most recent autopilot STATUSTEXT (may be empty);
        ``now`` is a monotonic clock.
        """
        if not self.enabled or self._ended:
            return None

        # 1. Disarm. Unambiguous, and the only signal that was ever used.
        if self._arm_seen and not armed:
            return "disarmed"

        # 2. The FC says the aircraft is finished. Straight from HEARTBEAT.
        if int(system_status) in _DEAD_STATES:
            return "flight terminated"

        # 3. The FC says so in words - this is how an RC emergency stop, which
        #    changes neither the arm bit nor system_status, becomes visible.
        low = (fc_text or "").lower()
        if low and any(frag in low for frag in _DEAD_TEXT):
            return "emergency stop"

        # 4. The slow path: it looks like it is sitting on the ground.
        if float(altitude_m or 0.0) >= self.airborne_alt_m:
            self._airborne = True
        if self._airborne and self._looks_landed(
                altitude_m, ground_speed_ms, vert_speed_ms, mode):
            if self._quiet_since is None:
                self._quiet_since = now
            elif now - self._quiet_since >= self.land_dwell_s:
                self._touchdown = self._quiet_since
                return "landed"
        else:
            # Any disagreement resets the dwell completely. Partial credit for
            # an intermittent match is exactly how a mid-flight blip would end
            # up ending the clip.
            self._quiet_since = None
        return None

    def _looks_landed(self, altitude_m: float, ground_speed_ms: float,
                      vert_speed_ms: float, mode: str) -> bool:
        """Is the aircraft, on this single sample, sitting on the ground?

        Instantaneous only - the caller applies ``land_dwell_s``, so this must
        NOT keep state of its own.
        """
        # All three must agree. Low alone is a low hover; slow alone is a
        # hover at any height; zero climb alone is level flight.
        return (float(altitude_m or 0.0) < self.land_altitude_m
                and abs(float(ground_speed_ms or 0.0)) < self.land_speed_ms
                and abs(float(vert_speed_ms or 0.0)) < self.land_vspeed_ms)
