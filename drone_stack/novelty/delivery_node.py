"""§2.3 / §9 - the only bus-facing surface of the novelty layer.

``DeliveryNode`` is a :class:`~drone_stack.utils.node.NodeBase`, like every
other node in ``drone_stack/nodes/`` - but it lives in ``drone_stack/novelty/``
(not ``drone_stack/nodes/``) because it is optional
(``novelty.enabled: false`` by default; see ``launch/bringup.py``) and the
package boundary documents that. It is the ONE place that:

* Owns the algorithmic modules (``landing_zone.score_candidates``,
  ``recipient_auth.DualFactorAuthenticator`` / ``disambiguate_recipients``,
  ``motion_monitor.MotionMonitor``) and the :class:`~drone_stack.novelty.
  mission_fsm.MissionFSM` engine that sequences them - see that module's
  own docstring for the engine/table split.
* Ground-projects raw ``PersonDetection``s (pixel box + score only, as
  produced by ``perception/adapters.py``'s ``DetectorAdapter``) against the
  current altitude, via the same :class:`~drone_stack.novelty.perception.
  projector.GroundProjector` every other module treats as opaque. See
  "Design note" below for why this step lives here.
* Reads BLE auth events and terrain segmentation frames published on
  ``NoveltyTopics`` (by the re-gated BLE peripheral and ``gcs/cameras.py`` -
  project plan, step 6) plus the drone's own ``FusedState`` off the
  existing ``Topics.FUSED_STATE``.
* Turns each ``MissionState`` transition into the SAME command surface
  every other node already uses: ``NavCommand`` on ``Topics.MISSION_CMD``
  (``hold``/``rtl``, handled by ``NavigationNode._on_mission_cmd`` ->
  ``services.call``) and ``Topics.MAVLINK_CMD`` (``goto``/``set_servo``,
  handled by ``MavlinkNode``) - mirroring ``navigation_node.py``'s own
  ``_send`` helper exactly.

## Design note - why DeliveryNode ground-projects, not the perception layer

``perception/adapters.py``'s own module docstring is explicit that it must
stay Hailo-only and geometry-free; ``landing_zone.py`` / ``recipient_auth.py``
/ ``motion_monitor.py`` all take already-projected ``GroundPoint``s as a
precondition. Something has to own the pixel->ground step for *person*
detections specifically (``rasterize_to_grid`` does the equivalent for
*terrain*, inside ``landing_zone.py``, because a full segmentation frame
needs no additional context beyond altitude) - person detections need that
SAME per-frame altitude, which only this node has synchronised against the
detection's own timestamp (the latest ``FusedState``), so it happens here,
once, before any algorithmic module ever sees a ``PersonDetection``.

## Design note - who runs model inference

Model inference (Hailo, at frame rate) happens in ``gcs/cameras.py``
(project plan, step 6), not here - ``DeliveryNode`` only ever consumes
ALREADY-RUN ``PersonDetection`` / ``SegmentationFrame`` results published
on ``NoveltyTopics``. ``ModelRegistry`` is still constructed in
``__init__`` so ``EvidenceLogger.set_model_versions`` can stamp every
decision record with the exact ``.hef`` versions that produced it -
construction is wrapped in a broad ``try/except`` because, per
``perception/adapters.py``'s own docstring, a fully-busy/absent Hailo
device can raise OUTSIDE that module's own graceful-degradation boundary,
and an uncaught exception here would otherwise crash ``Supervisor.add()``
(``launch/bringup.py``) instead of just leaving this node without model
version strings.

## Design note - coordinating with NavigationNode

``DeliveryNode`` never touches PX4 params or the MAVLink interface
directly; it only ever publishes ``NavCommand``, exactly like
``NavigationNode`` does internally. While actively flying its own
DESCENDING/RELEASING/ASCENDING choreography it commands the autopilot
directly on ``Topics.MAVLINK_CMD`` (``goto``/``set_servo``) - the same
channel manual/NL commands already use alongside ``NavigationNode``'s own
autonomous mission. Entering ``DESCENDING`` first calls the existing
``hold`` service on ``Topics.MISSION_CMD`` so ``NavigationNode`` stops
issuing its own waypoint ``goto``s and the two drivers never fight for
control. ``RTL``/``ABORT_RTL`` hand off to ``NavigationNode``'s own,
already-tested ``rtl`` service rather than duplicating that lifecycle here
- see ``mission_fsm.py``'s own docstring on why those two states are
terminal for THIS FSM specifically.
"""
from __future__ import annotations

import math
import time

from drone_stack.bus import MessageBus
from drone_stack.bus.topics import Topics
from drone_stack.msg import FusedState, NavCommand
from drone_stack.novelty.config import NoveltyConfig
from drone_stack.novelty.evidence_logger import EvidenceLogger
from drone_stack.novelty.landing_zone import score_candidates
from drone_stack.novelty.mission_fsm import (
    AltitudeReachedEvent,
    AmbiguousEvent,
    AscendCompleteEvent,
    AuthFailedEvent,
    AuthSucceededEvent,
    Event,
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
    ZoneConfirmedEvent,
    ZoneFoundEvent,
)
from drone_stack.novelty.motion_monitor import MotionMonitor
from drone_stack.novelty.perception.model_registry import ModelRegistry
from drone_stack.novelty.perception.projector import FlatEarthPinhole
from drone_stack.novelty.recipient_auth import DualFactorAuthenticator, disambiguate_recipients
from drone_stack.novelty.topics import NoveltyTopics
from drone_stack.novelty.types import (
    BleAuthEvent,
    FsmStateSnapshot,
    GroundPoint,
    PersonDetection,
    SegmentationFrame,
    ZoneCandidate,
)
from drone_stack.utils.config import Config
from drone_stack.utils.geometry import enu_to_geodetic
from drone_stack.utils.node import NodeBase

#: How long RELEASING waits before assuming the servo reached the release
#: PWM and declaring the drop complete. GUESSED placeholder - there is no
#: SERVO_OUTPUT_RAW readback wired onto the bus yet (see "Known limitations"
#: in docs/novelty/mission_fsm.md); a real confirmation replaces this once
#: that telemetry exists.
_RELEASE_SETTLE_S = 1.0


class DeliveryNode(NodeBase):
    """Drives one delivery attempt end-to-end via ``MissionFSM``."""

    def __init__(
        self,
        bus: MessageBus,
        config: Config,
        novelty_config: NoveltyConfig | None = None,
    ) -> None:
        section = config.section("novelty")
        super().__init__("delivery", bus, config, rate_hz=float(section.get("rate_hz", 5.0)))
        self.cfg = novelty_config if novelty_config is not None else NoveltyConfig.load()

        self.evidence = EvidenceLogger(
            flight_logs_dir=section.get("flight_logs_dir", "flight_logs"),
            repo_dir=section.get("repo_dir", "."),
        )

        self._registry: ModelRegistry | None = None
        try:
            self._registry = ModelRegistry.from_config(self.cfg.models)
            self.evidence.set_model_versions(self._registry.versions())
        except Exception as exc:  # noqa: BLE001 - see module docstring
            self.log.warning(
                "model registry unavailable (%s) - decisions will log no model versions", exc
            )

        cam = self.cfg.models.camera
        self._projector = FlatEarthPinhole(
            fx=cam.fx, fy=cam.fy, cx=cam.cx, cy=cam.cy,
            tilt_from_nadir_deg=cam.tilt_from_nadir_deg,
        )

        self.fsm = MissionFSM(self.cfg.mission_fsm, self.evidence, ctx=self._fresh_ctx())
        self.authenticator = DualFactorAuthenticator(self.cfg.recipient_auth)
        self.motion = MotionMonitor(self.cfg.motion_monitor)

        # Latest sensor state (set via subscriptions; NodeBase callbacks run
        # in the publisher's thread, but this node's own step() is the only
        # reader/writer of FSM state - see NodeBase's own "callback simply
        # stores the message" convention).
        self._fused: FusedState | None = None
        self._raw_detections: list[PersonDetection] = []
        self._terrain: SegmentationFrame | None = None
        self._ble_event: BleAuthEvent | None = None

        self._grounded: list[PersonDetection] = []
        self._recipient_last_ground: GroundPoint | None = None
        self._chosen_zone: ZoneCandidate | None = None
        self._hover_retries = 0
        self._last_entered_state: MissionState | None = None
        self._now: float = time.time()

        self._subscribe_all()

    @staticmethod
    def _fresh_ctx() -> dict:
        return {"recipient_track_id": None, "search_radius_m": 0.0}

    # -- wiring --------------------------------------------------------------
    def _subscribe_all(self) -> None:
        self.subscribe(Topics.FUSED_STATE, self._set("_fused"))
        self.subscribe(NoveltyTopics.PERSON_DETECTIONS, self._set("_raw_detections"))
        self.subscribe(NoveltyTopics.TERRAIN_MAP, self._set("_terrain"))
        self.subscribe(NoveltyTopics.BLE_AUTH_EVENT, self._set("_ble_event"))

    def _set(self, attr: str):
        def _setter(msg) -> None:
            setattr(self, attr, msg)
        return _setter

    # -- lifecycle -------------------------------------------------------------
    def on_stop(self) -> None:
        self.evidence.close()

    def reset_attempt(self) -> None:
        """Start a fresh delivery attempt - clears the FSM, timers and
        per-attempt caches, but keeps sensor subscriptions and the
        authenticator/motion-monitor instances (their own ``reset()``/state
        is handled per the usual entry-state hooks below)."""
        self.fsm.ctx = self._fresh_ctx()
        self.fsm.reset()
        self.authenticator.reset()
        self.motion.reset()
        self._chosen_zone = None
        self._recipient_last_ground = None
        self._hover_retries = 0
        self._last_entered_state = None

    def step(self, now: float | None = None) -> None:
        """One control tick. *now* defaults to wall-clock time; tests pass a
        synthetic clock so multi-minute timeout scenarios run without a real
        sleep - every ``Event`` this node constructs, and every
        ``check_timeout``/``elapsed_in_state`` call, is threaded through
        ``self._now`` so a synthetic clock stays self-consistent end to
        end."""
        self._now = now if now is not None else time.time()

        self._grounded = self._project_detections()
        self.motion.update(self._grounded)
        self._update_recipient_cache()

        handler = self._STEP_HANDLERS.get(self.fsm.state)
        event = handler(self) if handler is not None else None
        if event is not None:
            self.fsm.advance(event)
        else:
            self.fsm.check_timeout(now=self._now)

        while self.fsm.state != self._last_entered_state:
            entered = self.fsm.state
            self._last_entered_state = entered
            self._on_state_entered(entered)

        self._publish_snapshot()

    # -- ground projection -----------------------------------------------------
    def _project_detections(self) -> list[PersonDetection]:
        """Ground-project every raw detection against the current altitude.
        Detections with no altitude to project against, or whose ray misses
        the ground, are dropped - see the module docstring."""
        if self._fused is None:
            return []
        altitude_m = self._fused.alt_rel_m
        grounded: list[PersonDetection] = []
        for d in self._raw_detections:
            if d.ground is None:
                cx, cy = d.centroid_px
                # frame_shape is accepted for GroundProjector protocol
                # conformance but unused by FlatEarthPinhole's own formula
                # (see projector.py) - a placeholder is safe here.
                d.ground = self._projector.pixel_to_ground(cx, cy, altitude_m, frame_shape=(0, 0))
            if d.ground is not None:
                grounded.append(d)
        return grounded

    def _update_recipient_cache(self) -> None:
        track_id = self.fsm.ctx.get("recipient_track_id")
        if track_id is None:
            return
        for d in self._grounded:
            if d.track_id == track_id and d.ground is not None:
                self._recipient_last_ground = d.ground
                return

    # -- per-state step handlers ------------------------------------------------
    def _step_searching_person(self) -> Event | None:
        if not self._grounded:
            return None
        return PersonDetectedEvent(stamp=self._now, detections=tuple(self._grounded))

    def _step_person_found(self) -> Event | None:
        if not self._grounded:
            return None
        result = disambiguate_recipients(
            self._grounded, self._ble_event, self._fused, self.cfg.recipient_auth
        )
        if result.winner is None:
            return None
        self.fsm.ctx["recipient_track_id"] = result.winner.track_id
        return PersonConfirmedEvent(stamp=self._now, track_id=result.winner.track_id)

    def _step_searching_zone(self) -> Event | None:
        if self._terrain is None or self._fused is None or self._recipient_last_ground is None:
            return None
        candidates = score_candidates(
            self._terrain, self._fused.alt_rel_m, self._recipient_last_ground,
            self._projector, self.cfg.landing_zone,
        )
        if not candidates:
            return None
        self.fsm.ctx["zone_candidates"] = candidates
        return ZoneFoundEvent(stamp=self._now, candidates=tuple(candidates))

    def _step_zone_found(self) -> Event | None:
        candidates: list[ZoneCandidate] = self.fsm.ctx.get("zone_candidates") or []
        if not candidates:
            return None
        self._chosen_zone = candidates[0]
        return ZoneConfirmedEvent(stamp=self._now, candidate=self._chosen_zone)

    def _step_descending(self) -> Event | None:
        if self._fused is None or self._chosen_zone is None:
            return None
        recipient_id = self.fsm.ctx.get("recipient_track_id")
        if recipient_id is not None:
            abort = self.motion.check_velocity_abort(recipient_id, self._fused.alt_rel_m)
            if abort is not None:
                return MotionAbortFsmEvent(stamp=self._now, detail=abort)
            intrusion = self.motion.check_zone_intrusion(
                self._grounded, recipient_id, self._chosen_zone.centroid
            )
            if intrusion is not None:
                return MotionAbortFsmEvent(stamp=self._now, detail=intrusion)
        if self._fused.alt_rel_m <= self.cfg.mission_fsm.descent_hover_altitude_m:
            return AltitudeReachedEvent(stamp=self._now, altitude_m=self._fused.alt_rel_m)
        return None

    def _step_authenticating(self) -> Event | None:
        if self._fused is None:
            return None
        vision_detection = None
        if self._grounded:
            result = disambiguate_recipients(
                self._grounded, self._ble_event, self._fused, self.cfg.recipient_auth
            )
            if result.reason == "margin_not_met":
                return AmbiguousEvent(stamp=self._now)
            vision_detection = result.winner
        decision = self.authenticator.evaluate(self._ble_event, vision_detection, self._fused, now=self._now)
        if decision.released:
            return AuthSucceededEvent(stamp=self._now, decision=decision)
        if decision.reason in ("vision_timeout", "ble_timeout", "position_disagreement_timeout"):
            return AuthFailedEvent(stamp=self._now, decision=decision)
        return None

    def _step_hover_and_retry(self) -> Event | None:
        if not self._grounded or self._fused is None:
            return None
        result = disambiguate_recipients(
            self._grounded, self._ble_event, self._fused, self.cfg.recipient_auth
        )
        if result.reason == "margin_not_met":
            return None  # still ambiguous - the state's own timeout is the safety net
        if self._hover_retries < self.cfg.mission_fsm.max_hover_retries:
            self._hover_retries += 1
            return RetryEvent(stamp=self._now)
        return RetryExhaustedEvent(stamp=self._now)

    def _step_releasing(self) -> Event | None:
        if self.fsm.elapsed_in_state(now=self._now) >= _RELEASE_SETTLE_S:
            return ReleaseCompleteEvent(stamp=self._now)
        return None

    def _step_ascending(self) -> Event | None:
        if self._fused is None:
            return None
        if self._fused.alt_rel_m >= self.cfg.mission_fsm.ascend_target_altitude_m:
            return AscendCompleteEvent(stamp=self._now)
        return None

    #: EXPANDING_SEARCH_RADIUS and ABORT_DESCENT are transient - evaluated
    #: immediately on entry (see _on_state_entered), not per-step, so they
    #: have no step handler here.
    _STEP_HANDLERS = {
        MissionState.SEARCHING_PERSON: _step_searching_person,
        MissionState.PERSON_FOUND: _step_person_found,
        MissionState.SEARCHING_ZONE: _step_searching_zone,
        MissionState.ZONE_FOUND: _step_zone_found,
        MissionState.DESCENDING: _step_descending,
        MissionState.AUTHENTICATING: _step_authenticating,
        MissionState.HOVER_AND_RETRY: _step_hover_and_retry,
        MissionState.RELEASING: _step_releasing,
        MissionState.ASCENDING: _step_ascending,
    }

    # -- state-entry actions (command issuance) ---------------------------------
    def _on_state_entered(self, state: MissionState) -> None:
        if state == MissionState.SEARCHING_PERSON:
            self.authenticator.reset()
            self._hover_retries = 0
            self._chosen_zone = None
        elif state == MissionState.DESCENDING:
            self._send_mission_cmd("hold")
            self._send_goto_descent()
        elif state == MissionState.AUTHENTICATING:
            self.authenticator.reset()
        elif state == MissionState.RELEASING:
            self._send_release()
        elif state == MissionState.ASCENDING:
            self._send_ascend()
        elif state in (MissionState.RTL, MissionState.ABORT_RTL):
            self._send_mission_cmd("rtl")
        elif state == MissionState.ABORT_DESCENT:
            self._handle_abort_descent()
        elif state == MissionState.EXPANDING_SEARCH_RADIUS:
            self._handle_radius_expansion()

    def _handle_abort_descent(self) -> None:
        """Transient: re-evaluated the instant ABORT_DESCENT is entered (no
        new sensor input needed - see mission_fsm.py's own docstring)."""
        candidates: list[ZoneCandidate] = self.fsm.ctx.get("zone_candidates") or []
        remaining = [c for c in candidates if c is not self._chosen_zone]
        if remaining:
            self.fsm.ctx["zone_candidates"] = remaining
            self.fsm.advance(ReplanEvent(stamp=self._now))
        else:
            self.fsm.advance(GiveUpEvent(stamp=self._now))

    def _handle_radius_expansion(self) -> None:
        """Transient: re-evaluated the instant EXPANDING_SEARCH_RADIUS is
        entered - pure ctx arithmetic, no new sensor input needed."""
        radius = self.fsm.ctx.get("search_radius_m", 0.0) + self.cfg.mission_fsm.search_radius_expansion_m
        self.fsm.ctx["search_radius_m"] = radius
        if radius >= self.cfg.mission_fsm.max_search_radius_m:
            self.fsm.advance(MaxRadiusExceededEvent(stamp=self._now))
        else:
            self.fsm.advance(RadiusExpandedEvent(stamp=self._now, new_radius_m=radius))

    # -- command issuance --------------------------------------------------------
    def _send_mavlink(self, command: str, **params) -> None:
        self.publish(Topics.MAVLINK_CMD, NavCommand(command=command, params=params))

    def _send_mission_cmd(self, command: str, **params) -> None:
        self.publish(Topics.MISSION_CMD, NavCommand(command=command, params=params))

    def _to_geodetic(self, ground: GroundPoint) -> tuple[float, float] | None:
        """Body-relative ``GroundPoint`` (forward/left, from the drone's
        CURRENT position) -> ``(lat, lon)``, via the same body->ENU rotation
        verified against ``drone_stack/sim/world.py`` in
        ``recipient_auth.py``, anchored at the drone's own live position
        (``FusedState.lat``/``lon``) rather than "home" - ``NavigationNode``
        tracks home internally but never publishes it, and a small local
        offset from the drone's current fix needs no home reference at
        all."""
        if self._fused is None:
            return None
        yaw = self._fused.yaw
        east_m = ground.x_m * math.cos(yaw) - ground.y_m * math.sin(yaw)
        north_m = ground.x_m * math.sin(yaw) + ground.y_m * math.cos(yaw)
        return enu_to_geodetic(east_m, north_m, self._fused.lat, self._fused.lon)

    def _send_goto_descent(self) -> None:
        if self._chosen_zone is None:
            return
        target = self._to_geodetic(self._chosen_zone.centroid)
        if target is None:
            return
        lat, lon = target
        self._send_mavlink("goto", lat=lat, lon=lon, alt=self.cfg.mission_fsm.descent_hover_altitude_m)

    def _send_ascend(self) -> None:
        if self._fused is None:
            return
        self._send_mavlink(
            "goto", lat=self._fused.lat, lon=self._fused.lon,
            alt=self.cfg.mission_fsm.ascend_target_altitude_m,
        )

    def _send_release(self) -> None:
        # Reuses the existing, already-hardware-verified payload config
        # section (config/default.yaml `payload:`) rather than introducing
        # a second place the release PWM is defined - see CLAUDE.md §6 on
        # why that value must stay singular (a prior mismatch caused a
        # mechanical failure).
        payload = self.config.section("payload")
        self._send_mavlink(
            "set_servo",
            channel=int(payload.get("out_channel", 12)),
            pwm=int(payload.get("release_us", 1410)),
        )

    # -- telemetry ---------------------------------------------------------------
    def _publish_snapshot(self) -> None:
        self.publish(
            NoveltyTopics.MISSION_FSM_STATE,
            FsmStateSnapshot(
                state=self.fsm.state.value,
                elapsed_in_state_s=round(self.fsm.elapsed_in_state(), 3),
                recipient_track_id=self.fsm.ctx.get("recipient_track_id"),
                search_radius_m=self.fsm.ctx.get("search_radius_m", 0.0),
            ),
        )
