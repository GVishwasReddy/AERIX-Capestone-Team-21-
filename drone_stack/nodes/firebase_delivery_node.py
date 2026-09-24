"""Firebase delivery node - turns a customer order into a flown mission.

This is the node that closes the loop the rest of the stack was built for:

    Firestore ``orders`` document (written by the AERIX Flutter app)
        -> DeliveryOrder on the bus
        -> NavigationNode.set_delivery_target  (one GPS point expanded into a
           waypoint list the autopilot can fly)
        -> optional mission upload into the Pixhawk's own mission slot
        -> start_mission: arm, take off to the 3 m ceiling, fly the legs
        -> 60 s position hold over the drop point
        -> SMART_RTL home
        -> phase + telemetry written back onto the order document

Everything it does is published as a :class:`DeliveryState` at the node's loop
rate, which the GCS hub folds into its 15 Hz WebSocket payload - so the panel
on the dashboard is a view of this node's actual state, not a second copy of
the logic that could drift out of step with it.

Safety posture, in order of how much it matters:

* **Orders that predate the node are never auto-accepted.** A ``DISPATCHED``
  row left in the database from yesterday must not arm an aircraft because
  someone rebooted the Pi. Pre-existing orders are shown as PENDING and wait
  for a human.
* **auto_accept is false on real hardware.** The operator presses "ACCEPT &
  FLY" on the GCS. Simulation sets it true so the whole path can be tested.
* **Targets outside ``max_delivery_radius_m`` are rejected**, before anything
  is armed, with the reason shown on the GCS and written back to Firestore.
* This node commands nothing directly. It calls NavigationNode services, so the
  altitude ceiling, geofence, failsafes and pilot-override handling apply to a
  Firebase-sourced flight exactly as they do to a hand-flown one.
"""
from __future__ import annotations

from pathlib import Path
import threading
import time
import typing
from typing import Any

from drone_stack.bus.message_bus import MessageBus
from drone_stack.bus.topics import Topics
from drone_stack.interfaces.firebase_interface import (
    LINK_DISABLED,
    FirebaseInterface,
    build_order_source,
)
from drone_stack.msg import (
    DeliveryBleResult,
    DeliveryOrder,
    DeliveryOutcome,
    DeliveryPhase,
    DeliveryState,
    GpsFix,
    MissionPhase,
    MissionUploadResult,
    NavCommand,
    ReturnOutcome,
    describe_outcome,
)
from drone_stack.srv.services import ServiceRegistry, ServiceRequest, ServiceResponse
from drone_stack.utils.config import Config
from drone_stack.utils.geometry import haversine_m
from drone_stack.utils.node import NodeBase

#: Mission phases that mean "this delivery is over, stop tracking it".
_TERMINAL = (MissionPhase.DISARMED, MissionPhase.COMPLETE)


class FirebaseDeliveryNode(NodeBase):
    """Poll Firebase for orders and drive one delivery at a time."""

    def __init__(
        self,
        bus: MessageBus,
        config: Config,
        services: ServiceRegistry,
        source: FirebaseInterface | None = None,
    ) -> None:
        section = config.section("delivery")
        # "firebase_delivery", not "delivery": the novelty layer already ships a
        # DeliveryNode (parcel vision), and two nodes answering to one name would
        # make the supervisor's node list ambiguous.
        super().__init__(
            "firebase_delivery", bus, config,
            rate_hz=float(section.get("rate_hz", 2.0)),
        )
        self.services = services
        self._cfg = section
        self._source = source or build_order_source(section, config.mode)

        self._poll_interval = float(section.get("poll_interval_s", 3.0))
        self._auto_accept = bool(section.get("auto_accept", False))
        self._max_radius = float(section.get("max_delivery_radius_m", 150.0))
        self._hover_s = float(section.get("hover_seconds", 15.0))
        self._hover_alt = float(section.get("hover_altitude_m", 2.0))
        self._upload_to_fc = bool(section.get("upload_to_fc", True))
        self._write_back = bool(section.get("write_back", True))
        #: How close to home counts as "returned". Measured from the navigator's
        #: home, not the order's target, and only read once the aircraft is
        #: down - it answers "did someone have to go and fetch it?".
        self._home_radius = float(section.get("home_radius_m", 15.0))

        self._lock = threading.RLock()
        self._state = DeliveryState(link=LINK_DISABLED)
        self._pending: DeliveryOrder | None = None
        self._active: DeliveryOrder | None = None
        self._last_poll = 0.0
        self._started_at = time.time()
        #: order ids seen on the very first poll - these predate us.
        self._preexisting: set[str] = set()
        self._first_poll_done = False
        #: order ids we have finished or refused, so they are not re-offered.
        self._handled: set[str] = set()
        self._home: tuple[float, float] | None = None
        # Mirrored from MISSION_STATE; see _on_mission_state.
        self._nav_pilot_override = False
        self._pos: tuple[float, float] | None = None
        self._inbox: list[DeliveryOrder] = []     # dispatchable, oldest first
        self._selected = ""                       # operator's explicit choice
        self._ever_armed = False                  # did this delivery get airborne?
        # ── outcome evidence, reset per delivery ────────────────────────────
        #: Order ids whose BLE drop gate passed. This is the ONLY evidence of
        #: an actual handover: the payload servo is not instrumented (see
        #: parcel_delivery/pi/payload_controller.py), so "the aircraft came
        #: home" says nothing about whether the parcel left it.
        self._ble_ok: set[str] = set()
        #: Mission phases actually traversed while airborne. RTL distinguishes
        #: "flew itself home" from "put itself down where it stood", which the
        #: final position alone cannot - an emergency landing that happens to
        #: be near home is still not a return.
        self._saw_rtl = False
        self._saw_land = False
        self._saw_emergency = False
        #: Set when something deliberately ended the delivery early (operator
        #: ABORT, pilot takeover, a failsafe), as opposed to it simply failing
        #: to get a handshake at the drop point.
        self._abort_reason = ""
        self._recent: list[DeliveryOrder] = []    # any status, newest first
        # Local record of every order's final outcome, keyed by order_id.
        # list_recent() only knows what Firestore knows, so a manually
        # injected order (Test order / Repeat) - which never has a Firestore
        # doc - would vanish the instant it left self._inbox/self._active
        # with no trace anywhere. This is merged into the published "recent"
        # list in _publish_state() so a cancelled/rejected/aborted order
        # stays visible in the panel instead of disappearing.
        self._history: dict[str, tuple[DeliveryOrder, str]] = {}
        self._HISTORY_MAX = 30
        self._last_listing = 0.0
        self._last_written_phase = ""
        #: pending order held back for a transient reason (no GPS fix yet),
        #: so the poll loop knows to re-validate it rather than skip it.
        self._deferred = False

        self.subscribe(Topics.GPS, self._on_gps)
        self.subscribe(Topics.MISSION_UPLOAD, self._on_upload_result)
        self.subscribe(Topics.MISSION_STATE, self._on_mission_state)
        self.subscribe(
            Topics.DELIVERY_BLE_RESULT, self._on_ble_result, deliver_latched=False
        )

        self.services.register("delivery_accept", self._svc_accept)
        self.services.register("delivery_reject", self._svc_reject)
        self.services.register("delivery_abort", self._svc_abort)
        self.services.register("delivery_set_auto", self._svc_set_auto)
        self.services.register("delivery_status", self._svc_status)
        self.services.register("delivery_inject", self._svc_inject)
        self.services.register("delivery_select", self._svc_select)
        self.services.register("delivery_refresh", self._svc_refresh)
        self.services.register("delivery_reset", self._svc_reset)

    # -- lifecycle -----------------------------------------------------------
    def on_start(self) -> None:
        self._source.connect()
        self._set_message(
            "waiting for orders"
            if self._source.link == "online"
            else f"order source {self._source.link}"
        )
        self.log.info(
            "delivery node up: source=%s auto_accept=%s hover=%.0fs alt=%.1fm "
            "radius=%.0fm",
            self._source.link, self._auto_accept, self._hover_s,
            self._hover_alt, self._max_radius,
        )

    def on_stop(self) -> None:
        try:
            self._source.close()
        except Exception:  # noqa: BLE001
            self.log.debug("order source close failed", exc_info=True)

    def _on_gps(self, msg) -> None:
        if isinstance(msg, GpsFix) and msg.fix_type >= 3:
            with self._lock:
                # Kept live (unlike home) so "remaining" on the dashboard counts
                # down as the aircraft flies instead of showing the fixed
                # home-to-target length.
                self._pos = (msg.lat, msg.lon)

    def _on_mission_state(self, msg) -> None:
        """Take home and the override flag from the navigator.

        This node measures the delivery radius from home. Latching it here
        independently meant two nodes could disagree about where home is - and
        this one had no fix-quality gate at all, so it would happily anchor on
        a marginal indoor fix that the navigator had already rejected.

        ``pilot_override`` is cached here rather than fetched on demand: this
        runs on the navigator's thread while it holds its own lock, so a
        service call back into it from under our lock deadlocks both.
        """
        # Bare assignment - a bool store is atomic and needs no lock, which is
        # what keeps _validate free of any cross-node lock ordering.
        self._nav_pilot_override = bool(getattr(msg, "pilot_override", False))
        if getattr(msg, "home_set", False):
            with self._lock:
                self._home = (msg.home_lat, msg.home_lon)

    def _on_ble_result(self, msg) -> None:
        """Record the recipient handshake - the proof a parcel changed hands.

        NavigationNode already consumes this to cut the hover short. This node
        needs it for a different reason: without it the delivery phase can only
        report what the AIRCRAFT did, so a flight that never got a valid
        handshake and came home with the parcel still aboard was published as
        "delivery complete".

        Recorded by order id rather than as a flag so a result that arrives
        slightly out of step with the active order cannot credit the wrong one.
        """
        if not isinstance(msg, DeliveryBleResult) or not msg.success:
            return
        order_id = str(msg.order_id or "")
        if not order_id:
            return
        with self._lock:
            self._ble_ok.add(order_id)
            if self._state.order_id == order_id:
                self._state.ble_verified = True
        self.log.info("BLE drop gate passed for order %s", order_id)

    def _on_upload_result(self, msg) -> None:
        if isinstance(msg, MissionUploadResult):
            with self._lock:
                self._state.fc_mission_uploaded = msg.ok
                if not msg.ok:
                    self._state.last_error = msg.message
            self.log.info("FC mission upload: %s", msg.message)

    # -- main loop -----------------------------------------------------------
    def step(self) -> None:
        now = time.monotonic()
        if (now - self._last_poll) >= self._poll_interval:
            self._last_poll = now
            self._poll_orders()
        # The history listing is a read of the whole collection, so it runs on
        # a slower cadence than the dispatch query it sits beside.
        if (now - self._last_listing) >= max(10.0, self._poll_interval * 4):
            self._last_listing = now
            self._refresh_listing()
        self._track_active()
        self._publish_state()

    def _refresh_listing(self) -> None:
        """Pull every recent order so the panel shows the app's whole history."""
        try:
            recent = self._source.list_recent(10)
        except Exception:  # noqa: BLE001 - a listing failure is cosmetic
            self.log.debug("order listing failed", exc_info=True)
            return
        with self._lock:
            self._recent = recent

    def _record_history(self, order: DeliveryOrder, status: str) -> None:
        """Remember an order's final outcome so it never just disappears.

        Must be called with ``self._lock`` held (every caller already holds
        it - _reject, _track_active's termination branch, _svc_abort).
        """
        self._history[order.order_id] = (order, status)
        if len(self._history) > self._HISTORY_MAX:
            del self._history[next(iter(self._history))]

    def _order_row(
        self, order: DeliveryOrder, dispatchable: bool, local_status: str = ""
    ) -> dict[str, Any]:
        """One order as the dashboard shows it."""
        ok, reason, _ = self._validate(order)
        return {
            "order_id": order.order_id,
            "recipient_id": order.recipient_id,
            "target_lat": order.target_lat,
            "target_lon": order.target_lon,
            "created_at": order.created_at,
            "status": order.status or local_status or ("DISPATCHED" if dispatchable else ""),
            "distance_m": round(self._distance_to(order), 1),
            "source": order.source,
            "dispatchable": bool(dispatchable and ok),
            "blocked_reason": "" if ok else reason,
            "flown": order.order_id in self._handled,
        }

    def _poll_orders(self) -> None:
        try:
            orders = self._source.poll()
        except Exception:  # noqa: BLE001 - a bad poll must not kill the node
            self.log.exception("order poll failed")
            return

        with self._lock:
            if not self._first_poll_done:
                # Snapshot what was already in the database. These are shown
                # but never auto-accepted: see the module docstring.
                self._preexisting = {o.order_id for o in orders}
                self._first_poll_done = True
                if self._preexisting:
                    self.log.warning(
                        "%d order(s) already DISPATCHED at startup - they will "
                        "wait for manual acceptance: %s",
                        len(self._preexisting), ", ".join(sorted(self._preexisting)),
                    )
            self._inbox = list(orders)
            if self._active is not None:
                return                      # one delivery at a time
            # An operator who picked a specific order from the inbox keeps that
            # choice; otherwise take the oldest one we have not flown.
            candidate = next(
                (o for o in orders if o.order_id == self._selected), None
            ) or next(
                (o for o in orders if o.order_id not in self._handled), None
            )
            if candidate is None:
                # A manually injected order never appears in the query, so its
                # absence means nothing - only an order that came from the
                # source can be withdrawn by the source.
                if self._pending is not None and self._pending.source != "manual":
                    # The order disappeared from the query (cancelled upstream).
                    self.log.info("pending order %s withdrawn", self._pending.order_id)
                    self._pending = None
                    self._deferred = False
                    self._set_phase(DeliveryPhase.IDLE, "waiting for orders")
                return
            already_offered = (
                self._pending is not None
                and self._pending.order_id == candidate.order_id
            )
            if already_offered and not self._deferred:
                return                      # already offered, still waiting

            self._pending = candidate
            if not already_offered:
                self.publish(Topics.DELIVERY_ORDER, candidate)
            ok, reason, retryable = self._validate(candidate)
            if not ok:
                if retryable:
                    # Not the order's fault, and not permanent: a Pi that has
                    # just booted has no fix yet. _reject() remembers an id
                    # forever, so rejecting here would kill a good delivery
                    # for the rest of the process - the customer's order would
                    # simply never fly, with no way back short of a restart.
                    # Hold it on offer and re-check every poll instead.
                    self._deferred = True
                    self._set_phase(
                        DeliveryPhase.PENDING,
                        f"order {candidate.order_id} held: {reason}",
                        order=candidate,
                    )
                    if not already_offered:
                        self.log.warning(
                            "order %s held, not rejected: %s",
                            candidate.order_id, reason,
                        )
                    return
                self._deferred = False
                self._reject(candidate, reason)
                return
            if self._deferred:
                self._deferred = False
                self.log.info(
                    "order %s is flyable now - the block has cleared",
                    candidate.order_id,
                )
            stale = candidate.order_id in self._preexisting
            self._set_phase(
                DeliveryPhase.PENDING,
                f"order {candidate.order_id} awaiting acceptance"
                + (" (pre-existing order - confirm before flying)" if stale else ""),
                order=candidate,
            )
            self.log.info(
                "order %s for %s at %.7f, %.7f (%.0f m away)%s",
                candidate.order_id, candidate.recipient_id or "?",
                candidate.target_lat, candidate.target_lon,
                self._distance_to(candidate),
                " [pre-existing]" if stale else "",
            )
            if self._auto_accept and not stale:
                self._accept(candidate)

    def _track_active(self) -> None:
        """Mirror the navigator's mission state onto the delivery phase."""
        with self._lock:
            active = self._active
        if active is None:
            return
        status = self.services.call("mission_status")
        if not status.success:
            return
        d = status.data
        phase = str(d.get("phase", ""))
        hover_remaining = float(d.get("hover_remaining_s", 0.0) or 0.0)
        refusal = str(d.get("arm_refusal", "") or "")
        if d.get("armed"):
            self._ever_armed = True

        # The mission being over is a different question from the delivery
        # phase reading "finished". An aborted delivery whose aircraft is still
        # flying home is not over, and that gap is exactly where the
        # "aborted + returned" outcome lives.
        mission_terminal = phase in {p.value for p in _TERMINAL}

        # Which legs actually happened. Position alone cannot tell "flew itself
        # home" from "put itself down where it stood" - an emergency landing
        # that happens to be near home is still not a return.
        if phase == MissionPhase.RTL.value:
            self._saw_rtl = True
        elif phase == MissionPhase.LAND.value:
            self._saw_land = True
        elif phase == MissionPhase.EMERGENCY.value:
            self._saw_emergency = True

        if d.get("pilot_override"):
            new_phase = DeliveryPhase.ABORTED
            message = f"pilot took control ({d.get('mode', '?')}) - delivery halted"
            if not self._abort_reason:
                self._abort_reason = f"pilot took control ({d.get('mode', '?')})"
        elif phase == MissionPhase.ARMING.value:
            # The autopilot may legitimately refuse for a while. Show its own
            # words rather than a silent "accepted, nothing happening".
            new_phase = DeliveryPhase.ACCEPTED
            message = (
                f"waiting for the autopilot to arm - {refusal}"
                if refusal
                else str(status.message or "arming")
            )
        elif phase == MissionPhase.IDLE.value and not self._ever_armed:
            # Never got off the ground: arming was refused or given up on.
            # Falling through to the generic branch would have shown EN ROUTE
            # for an aircraft sitting on the pad.
            new_phase = DeliveryPhase.ABORTED
            message = (
                f"could not launch - {refusal}"
                if refusal
                else str(status.message or "the autopilot would not arm")
            )
        elif phase == MissionPhase.HOVER.value:
            new_phase = DeliveryPhase.HOVERING
            message = f"holding over drop point, {hover_remaining:.0f}s left"
        elif phase == MissionPhase.RTL.value:
            new_phase = DeliveryPhase.RETURNING
            message = str(status.message or "returning to base")
        elif phase in (MissionPhase.LAND.value, MissionPhase.EMERGENCY.value):
            new_phase = DeliveryPhase.RETURNING
            message = str(status.message or "landing")
        elif mission_terminal:
            if self._ever_armed:
                new_phase = DeliveryPhase.LANDED
                message = "delivery complete - drone home and disarmed"
            else:
                # Terminal without ever arming: a refused arm or a failsafe
                # ended the mission on the pad. This branch used to report
                # "delivery complete", which told the customer their order had
                # arrived while the aircraft sat on the bench.
                new_phase = DeliveryPhase.ABORTED
                message = (
                    f"never left the ground - {refusal}"
                    if refusal
                    else str(status.message or "mission ended before takeoff")
                )
        else:
            new_phase = DeliveryPhase.ENROUTE
            message = str(status.message or "en route to drop point")

        if new_phase == DeliveryPhase.ABORTED and refusal:
            with self._lock:
                self._state.last_error = refusal

        # An abort is sticky. The aircraft usually keeps flying - home, or
        # wherever the pilot takes it - and the RTL leg used to overwrite the
        # abort with RETURNING, so the panel lost the fact that a human had
        # stopped the delivery at all.
        if self._abort_reason:
            new_phase = DeliveryPhase.ABORTED
            if not mission_terminal:
                message = f"{self._abort_reason} · {message}"

        returning = new_phase == DeliveryPhase.RETURNING
        with self._lock:
            self._state.hover_remaining_s = round(hover_remaining, 1)
            self._state.remaining_m = round(self._remaining_m(active, returning), 1)
            self._set_phase(new_phase, message, order=active)
            # Finish on the MISSION ending, not on the delivery phase looking
            # final - otherwise an abort closes the order the instant it is
            # pressed and nothing ever observes whether the aircraft got home.
            never_launched = not self._ever_armed and new_phase in (
                DeliveryPhase.ABORTED, DeliveryPhase.LANDED
            )
            if mission_terminal or never_launched:
                self._finish_delivery(active, new_phase)

    # -- accept / reject -----------------------------------------------------
    def _validate(self, order: DeliveryOrder) -> tuple[bool, str, bool]:
        """Can we fly this order? Returns (ok, reason, retryable).

        "retryable" separates *the order is wrong* from *we are not ready
        yet*. Nothing about the order changes when the GPS finally locks, so
        a cold-booted Pi must not burn a perfectly good delivery just because
        the constellation had not been acquired when it arrived.
        """
        if not (-90.0 <= order.target_lat <= 90.0) or not (
            -180.0 <= order.target_lon <= 180.0
        ):
            return False, "target coordinates out of range", False
        if order.target_lat == 0.0 and order.target_lon == 0.0:
            return (
                False,
                "target is null island (0, 0) - order has no coordinates",
                False,
            )
        if self._nav_pilot_override:
            # Retryable: the order is fine, the aircraft is simply in the
            # pilot's hands. Accepting here would plan a route, report
            # success, and abort a tick later once the navigator stood down -
            # which just burns the order and tells the operator nothing.
            #
            # Read from the MISSION_STATE the navigator pushes to us. Asking it
            # directly with services.call() deadlocks: _validate runs under our
            # lock, and the navigator publishes MISSION_STATE under its own.
            return (
                False,
                "pilot override active - press RESUME on the GCS to take control back",
                True,
            )
        if self._home is None:
            if bool(self._cfg.get("require_gps_fix", True)):
                return (
                    False,
                    "no GPS fix yet - cannot check the target against home",
                    True,
                )
            return True, "", False
        distance = self._distance_to(order)
        if distance > self._max_radius:
            return (
                False,
                f"target is {distance:.0f} m away, limit is {self._max_radius:.0f} m",
                False,
            )
        return True, "", False

    def _accept(self, order: DeliveryOrder) -> ServiceResponse:
        """Commit the aircraft: expand, upload, launch."""
        ok, reason, retryable = self._validate(order)
        if not ok:
            # A retryable block (no fix yet) leaves the order on offer so the
            # operator can press ACCEPT & FLY again once the sats come in.
            if not retryable:
                self._reject(order, reason)
            return ServiceResponse(False, reason)

        plan = self.services.call(
            "set_delivery_target",
            lat=order.target_lat,
            lon=order.target_lon,
            alt_m=order.hover_alt_m or self._hover_alt,
            hover_s=order.hover_seconds or self._hover_s,
            mission_name=f"delivery-{order.order_id}",
        )
        if not plan.success:
            self._set_phase(
                DeliveryPhase.PENDING,
                f"cannot plan route: {plan.message}",
                order=order,
            )
            self._state.last_error = plan.message
            return ServiceResponse(False, plan.message)

        waypoints = int(plan.data.get("waypoints", 0))
        distance = float(plan.data.get("distance_m", 0.0))
        # The FC upload is asynchronous; _on_upload_result flips this once the
        # autopilot answers.
        uploaded = False
        if self._upload_to_fc:
            self._upload_plan(plan.data.get("plan") or [])

        started = self.services.call("start_mission")
        if not started.success:
            self._set_phase(
                DeliveryPhase.ACCEPTED,
                f"route ready but launch refused: {started.message}",
                order=order,
            )
            self._state.last_error = started.message
            return ServiceResponse(False, started.message)

        with self._lock:
            self._active = order
            self._pending = None
            self._reset_outcome_evidence()
            self._state.waypoints = waypoints
            self._state.distance_m = round(distance, 1)
            self._state.fc_mission_uploaded = uploaded
            self._state.last_error = ""
            self._set_phase(
                DeliveryPhase.ACCEPTED,
                f"accepted: {waypoints} waypoints over {distance:.0f} m",
                order=order,
            )
        self.log.info(
            "order %s accepted: %d waypoints, %.0f m, hover %.0fs at %.1f m, "
            "FC mission %s",
            order.order_id, waypoints, distance,
            order.hover_seconds or self._hover_s,
            order.hover_alt_m or self._hover_alt,
            "uploaded" if uploaded else "not uploaded",
        )
        return ServiceResponse(
            True,
            f"{waypoints} waypoints, {distance:.0f} m",
            data={"waypoints": waypoints, "distance_m": distance},
        )

    def _upload_plan(self, plan: list[dict[str, Any]]) -> bool:
        """Push the expanded plan into the autopilot's mission slot.

        Queued on the bus so MavlinkNode performs the blocking mission
        handshake on its own thread rather than us racing its receive loop.
        The real outcome arrives asynchronously on Topics.MISSION_UPLOAD (see
        _on_upload_result); this only reports that the request went out.

        Best-effort by design: the flight is flown in GUIDED either way, so a
        refused upload is surfaced on the GCS but does not stop the delivery.
        """
        if not plan or self._home is None:
            return False
        self.publish(
            Topics.MAVLINK_CMD,
            NavCommand(
                command="upload_mission",
                params={
                    "home": list(self._home),
                    "items": plan,
                    "takeoff_alt": self._hover_alt,
                },
            ),
        )
        self.log.info("requested FC mission upload (%d waypoints)", len(plan))
        return False

    def _reject(self, order: DeliveryOrder, reason: str) -> None:
        with self._lock:
            self._handled.add(order.order_id)
            self._pending = None
            self._deferred = False
            self._state.last_error = reason
            self._record_history(order, DeliveryPhase.REJECTED.value)
            self._set_phase(
                DeliveryPhase.REJECTED, f"rejected: {reason}", order=order
            )
        try:
            for p in [Path.home() / "drone_stack" / "active_order.json", Path.cwd() / "active_order.json"]:
                if p.exists():
                    p.unlink()
        except Exception:
            pass
        self.log.warning("order %s rejected: %s", order.order_id, reason)

    # -- services ------------------------------------------------------------
    def _svc_accept(self, req: ServiceRequest) -> ServiceResponse:
        """Operator pressed ACCEPT & FLY."""
        with self._lock:
            order = self._pending
            if order is None:
                return ServiceResponse(False, "no pending order")
            if self._active is not None:
                return ServiceResponse(False, "a delivery is already in progress")
        order_id = str(req.data.get("order_id", "") or "")
        if order_id and order_id != order.order_id:
            return ServiceResponse(False, f"pending order is {order.order_id}")
        return self._accept(order)

    def _svc_select(self, req: ServiceRequest) -> ServiceResponse:
        """Choose which of the queued orders ACCEPT & FLY will send.

        Selecting does not launch anything - it only moves the offer, so the
        operator can look through a queue of orders before committing.
        """
        order_id = str(req.data.get("order_id", "") or "")
        with self._lock:
            if self._active is not None:
                return ServiceResponse(False, "a delivery is already in progress")
            if not order_id:                      # clear the choice
                self._selected = ""
                return ServiceResponse(True, "selection cleared")
            match = next(
                (o for o in self._inbox if o.order_id == order_id), None
            )
            if match is None:
                return ServiceResponse(False, f"order {order_id} is not in the queue")
            self._selected = order_id
            self._pending = match
        ok, reason, retryable = self._validate(match)
        if not ok:
            if not retryable:
                self._reject(match, reason)
            return ServiceResponse(False, reason)
        self._set_phase(
            DeliveryPhase.PENDING,
            f"order {match.order_id} selected - awaiting acceptance",
            order=match,
        )
        return ServiceResponse(
            True,
            f"order {order_id} selected",
            data={"distance_m": round(self._distance_to(match), 1)},
        )

    def _svc_refresh(self, req: ServiceRequest) -> ServiceResponse:
        """Poll the order source right now instead of waiting for the timer."""
        self._poll_orders()
        self._refresh_listing()
        self._publish_state()
        with self._lock:
            return ServiceResponse(
                True,
                f"{len(self._inbox)} order(s) awaiting dispatch, "
                f"{len(self._recent)} in history",
                data={"orders": len(self._inbox), "recent": len(self._recent)},
            )

    def _svc_reject(self, req: ServiceRequest) -> ServiceResponse:
        with self._lock:
            order = self._pending
        if order is None:
            return ServiceResponse(False, "no pending order")
        self._reject(order, str(req.data.get("reason", "declined by operator")))
        return ServiceResponse(True, "order declined")

    def _svc_abort(self, req: ServiceRequest) -> ServiceResponse:
        """Stop the delivery and bring the aircraft home."""
        with self._lock:
            order = self._active
        if order is None:
            return ServiceResponse(False, "no delivery in progress")
        response = self.services.call("abort_delivery")
        with self._lock:
            self._handled.add(order.order_id)
            # _active is deliberately KEPT. The aircraft is still in the air and
            # usually flying home, and only _track_active watching it to the
            # ground can tell "aborted · returned home" from "aborted · did not
            # return". Clearing it here closed the order the instant the button
            # was pressed, so the return half of the outcome was never observed.
            # _finish_delivery closes it once the mission actually ends.
            self._abort_reason = "aborted by operator"
            self._set_phase(
                DeliveryPhase.ABORTED,
                response.message or "delivery aborted - returning home",
                order=order,
            )
        self.log.warning("order %s aborted by operator", order.order_id)
        return ServiceResponse(True, response.message or "delivery aborted")

    def _svc_reset(self, req: ServiceRequest) -> ServiceResponse:
        """Manual escape hatch for the "back to AUTO" button.

        _track_active() only clears _active once it sees a terminal
        MissionPhase (DISARMED/COMPLETE) come back from mission_status. If
        that telemetry is ever stale or missed right after a landing (a
        MAVLink hiccup, a USB re-enumeration - this Pi has seen both), the
        node is left believing a delivery is still "in progress" forever,
        and every future ACCEPT & FLY is refused - previously recoverable
        only by restarting the whole process. Gated on the aircraft being
        disarmed so this can never step on an aircraft that is actually
        still flying; use ABORT for that.
        """
        status = self.services.call("mission_status")
        if status.success and status.data.get("armed"):
            return ServiceResponse(
                False, "aircraft is armed - use ABORT to bring it home first"
            )
        with self._lock:
            if self._active is None and self._pending is None:
                return ServiceResponse(True, "already clear - ready for a new order")
            self._active = None
            self._pending = None
            self._set_phase(
                DeliveryPhase.IDLE, "reset by operator - ready for a new order"
            )
        self.log.info("delivery state reset by operator - ready for a new order")
        return ServiceResponse(True, "ready for a new order")

    def _svc_set_auto(self, req: ServiceRequest) -> ServiceResponse:
        enabled = bool(req.data.get("enabled", not self._auto_accept))
        with self._lock:
            self._auto_accept = enabled
            self._state.auto_accept = enabled
        self.log.info("auto-accept %s", "enabled" if enabled else "disabled")
        return ServiceResponse(
            True, f"auto-accept {'enabled' if enabled else 'disabled'}"
        )

    def _svc_status(self, req: ServiceRequest) -> ServiceResponse:
        with self._lock:
            return ServiceResponse(True, self._state.message, data=self._state_dict())

    def _svc_inject(self, req: ServiceRequest) -> ServiceResponse:
        """Feed an order in directly, bypassing Firebase.

        The test path for the whole delivery chain: ``scripts/inject_order.py``
        and the GCS both use it, so a demo does not need the app, the network
        or a service-account key.
        """
        try:
            lat = float(req.data["lat"])
            lon = float(req.data["lon"])
        except (KeyError, TypeError, ValueError):
            return ServiceResponse(False, "lat and lon are required")
        order = DeliveryOrder(
            order_id=str(req.data.get("order_id", f"manual-{int(time.time())}")),
            recipient_id=str(req.data.get("recipient_id", "manual")),
            target_lat=lat,
            target_lon=lon,
            hover_alt_m=float(req.data.get("hover_alt_m", self._hover_alt)),
            hover_seconds=float(req.data.get("hover_seconds", self._hover_s)),
            created_at=time.strftime("%Y-%m-%dT%H:%M:%S"),
            source="manual",
        )
        with self._lock:
            if self._active is not None:
                return ServiceResponse(False, "a delivery is already in progress")
            self._pending = order
            self._handled.discard(order.order_id)
        self.publish(Topics.DELIVERY_ORDER, order)
        ok, reason, retryable = self._validate(order)
        if not ok:
            if not retryable:
                self._reject(order, reason)
            return ServiceResponse(False, reason)
        self._set_phase(
            DeliveryPhase.PENDING,
            f"injected order {order.order_id} awaiting acceptance",
            order=order,
        )
        if bool(req.data.get("auto_accept", self._auto_accept)):
            return self._accept(order)
        return ServiceResponse(True, f"order {order.order_id} pending", data={
            "order_id": order.order_id, "distance_m": round(self._distance_to(order), 1),
        })

    # -- state ---------------------------------------------------------------
    # -- outcome -------------------------------------------------------------
    def _reset_outcome_evidence(self) -> None:
        """Start a delivery with no opinion about how it went. Lock held.

        ``_ble_ok`` is deliberately NOT cleared: it is keyed by order id, so it
        cannot credit the wrong delivery, and keeping it means a handshake that
        lands slightly before the accept is not thrown away.
        """
        self._ever_armed = False
        self._saw_rtl = False
        self._saw_land = False
        self._saw_emergency = False
        self._abort_reason = ""
        s = self._state
        s.outcome = DeliveryOutcome.PENDING
        s.return_outcome = ReturnOutcome.PENDING
        s.outcome_label = ""
        s.outcome_reason = ""
        s.outcome_at = 0.0
        s.ble_verified = False
        s.home_distance_m = 0.0
        s.ever_armed = False

    def _home_distance_m(self) -> float | None:
        """How far the aircraft is from home, or None if that is not known."""
        if self._pos is None or self._home is None:
            return None
        return haversine_m(
            self._pos[0], self._pos[1], self._home[0], self._home[1]
        )

    def _classify_outcome(self) -> tuple[DeliveryOutcome, ReturnOutcome, str]:
        """Split "how did it go?" into the parcel's answer and the aircraft's.

        These are genuinely independent, and the panel was wrong precisely
        because it published one word for both. A delivery can succeed while
        the aircraft strands itself in a field, and the aircraft can come home
        perfectly with the parcel never handed over.

        Call with ``self._lock`` held.
        """
        order_id = self._state.order_id
        delivered = bool(order_id) and order_id in self._ble_ok

        # -- what happened to the parcel --------------------------------------
        if delivered:
            parcel = DeliveryOutcome.DELIVERED
            why = "recipient handshake verified"
        elif not self._ever_armed:
            parcel = DeliveryOutcome.NEVER_FLEW
            why = self._abort_reason or "never left the ground"
        elif self._abort_reason:
            parcel = DeliveryOutcome.ABORTED
            why = self._abort_reason
        else:
            # Flew the whole leg and came back without a handshake. NOT the
            # same as an abort, and emphatically not a completed delivery.
            parcel = DeliveryOutcome.FAILED
            why = "no valid recipient handshake at the drop point"

        # -- what happened to the aircraft ------------------------------------
        distance = self._home_distance_m()
        if not self._ever_armed:
            ret = ReturnOutcome.ON_PAD
        elif self._saw_emergency or (self._saw_land and not self._saw_rtl):
            # Put itself down rather than flying home. Where it stopped does
            # not change that - someone still has to go and collect it.
            ret = ReturnOutcome.NOT_RETURNED
            why += " · put down without flying home"
        elif distance is not None:
            ret = (
                ReturnOutcome.RETURNED
                if distance <= self._home_radius
                else ReturnOutcome.NOT_RETURNED
            )
            if ret is ReturnOutcome.NOT_RETURNED:
                why += f" · down {distance:.0f} m from home"
        elif self._saw_rtl:
            # No position to judge by, but it did fly the return leg. Best
            # available answer, and better than claiming ignorance.
            ret = ReturnOutcome.RETURNED
        else:
            ret = ReturnOutcome.UNKNOWN
        return parcel, ret, why

    def _finish_delivery(
        self, order: DeliveryOrder, phase: DeliveryPhase
    ) -> None:
        """Reach a verdict on a finished delivery and let the order go.

        Call with ``self._lock`` held.
        """
        parcel, ret, why = self._classify_outcome()
        distance = self._home_distance_m()
        s = self._state
        s.outcome = parcel
        s.return_outcome = ret
        s.outcome_label = describe_outcome(parcel, ret)
        s.outcome_reason = why
        s.outcome_at = time.time()
        s.ble_verified = bool(s.order_id) and s.order_id in self._ble_ok
        s.home_distance_m = round(distance, 1) if distance is not None else 0.0
        s.home_radius_m = self._home_radius
        s.ever_armed = self._ever_armed
        if phase is DeliveryPhase.LANDED or self._ever_armed:
            # A flight that happened is finished either way. One that never
            # armed failed on a precondition - no position estimate, a refused
            # arm, a failsafe on the pad - none of which are properties of the
            # order, so burning its id stranded a good delivery until the
            # service restarted.
            self._handled.add(order.order_id)
        # The order book now carries the verdict rather than the phase, so a
        # finished row reads "DELIVERED · DID NOT RETURN" instead of "LANDED".
        self._record_history(order, s.outcome_label)
        self._write_outcome_back(order)
        self._active = None
        self._pending = None
        self.log.info(
            "order %s finished: %s (%s)", order.order_id, s.outcome_label, why
        )

    def _write_outcome_back(self, order: DeliveryOrder) -> None:
        """Mirror the verdict onto the Firestore document, if enabled.

        Written as three fields rather than one string so the app can branch on
        the parcel and the aircraft separately - "we could not deliver" and
        "your drone is in a field" are different notifications.
        """
        if not self._write_back:
            return
        doc_id = order.doc_id or order.order_id
        if not doc_id:
            return
        s = self._state
        fields = {
            "droneOutcome": s.outcome.value,
            "droneReturn": s.return_outcome.value,
            "droneOutcomeLabel": s.outcome_label,
            "droneOutcomeReason": s.outcome_reason,
            "droneDelivered": s.outcome is DeliveryOutcome.DELIVERED,
            "droneReturned": s.return_outcome is ReturnOutcome.RETURNED,
            "droneUpdatedAt": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()),
        }
        if s.outcome is not DeliveryOutcome.DELIVERED:
            fields["status"] = "CANCELED"
        else:
            fields["status"] = "DELIVERED"
        try:
            self._source.update_order(doc_id, fields)
        except Exception:  # noqa: BLE001 - write-back is best effort
            self.log.debug("outcome write-back failed", exc_info=True)

    def _distance_to(self, order: DeliveryOrder) -> float:
        """Route length: home to the drop point. Constant for a given order."""
        if self._home is None:
            return 0.0
        return haversine_m(
            self._home[0], self._home[1], order.target_lat, order.target_lon
        )

    def _remaining_m(self, order: DeliveryOrder, returning: bool) -> float:
        """How far the aircraft still has to fly, from where it actually is.

        Outbound that is the distance to the drop point; on the way back it is
        the distance to home, so the number keeps meaning the same thing for
        the whole delivery.
        """
        pos = self._pos or self._home
        if pos is None:
            return 0.0
        dest = self._home if returning else (order.target_lat, order.target_lon)
        if dest is None:
            return 0.0
        return haversine_m(pos[0], pos[1], dest[0], dest[1])

    def _set_phase(
        self,
        phase: DeliveryPhase,
        message: str,
        order: DeliveryOrder | None = None,
    ) -> None:
        s = self._state
        changed = s.phase != phase
        s.phase = phase
        s.message = message
        if order is not None:
            s.order_id = order.order_id
            s.recipient_id = order.recipient_id
            s.target_lat = order.target_lat
            s.target_lon = order.target_lon
            s.hover_alt_m = order.hover_alt_m or self._hover_alt
            s.hover_seconds = order.hover_seconds or self._hover_s
            # Known as soon as an order is offered, not just once accepted -
            # the operator needs the distance to decide whether to accept.
            s.distance_m = round(self._distance_to(order), 1)
            s.remaining_m = round(self._remaining_m(order, False), 1)
        if changed:
            self.log.info("delivery phase -> %s (%s)", phase.value, message)
            self._write_phase_back(order)

    def _set_message(self, message: str) -> None:
        with self._lock:
            self._state.message = message

    def _write_phase_back(self, order: DeliveryOrder | None) -> None:
        """Mirror the phase onto the Firestore document, if enabled."""
        if not self._write_back or order is None:
            return
        doc_id = order.doc_id or order.order_id
        if not doc_id:
            return
        fields = {
            "dronePhase": self._state.phase.value,
            "droneMessage": self._state.message,
            "droneUpdatedAt": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()),
        }
        if self._state.phase in (DeliveryPhase.REJECTED, DeliveryPhase.ABORTED):
            fields["status"] = "CANCELED"
            fields["droneOutcome"] = self._state.phase.value
        if self._state.phase == DeliveryPhase.LANDED:
            fields["droneDelivered"] = True
        try:
            self._source.update_order(doc_id, fields)
        except Exception:  # noqa: BLE001
            self.log.debug("write-back failed", exc_info=True)

    def _state_dict(self) -> dict[str, Any]:
        s = self._state
        return {
            "phase": s.phase.value,
            "order_id": s.order_id,
            "recipient_id": s.recipient_id,
            "target_lat": s.target_lat,
            "target_lon": s.target_lon,
            "hover_alt_m": s.hover_alt_m,
            "hover_seconds": s.hover_seconds,
            "hover_remaining_s": s.hover_remaining_s,
            "distance_m": s.distance_m,
            "remaining_m": s.remaining_m,
            "waypoints": s.waypoints,
            "auto_accept": s.auto_accept,
            "fc_mission_uploaded": s.fc_mission_uploaded,
            "link": s.link,
            "message": s.message,
            "last_error": s.last_error,
            "pending": self._pending is not None,
            "active": self._active is not None,
        }

    _LINK_MESSAGE = {
        "online": "waiting for orders",
        "no-credentials": "no Firebase key - orders from the app cannot be seen "
                          "(run scripts/firebase_setup.py)",
        "error": "order source unreachable - retrying",
        "connecting": "connecting to the order source...",
        "disabled": "order source disabled",
    }

    def _publish_state(self) -> None:
        with self._lock:
            self._state.link = self._source.link
            if self._source.link == "online" and not self._source.last_error:
                # The source recovered (a key was installed, the network came
                # back). A stale error left on screen would have the operator
                # chasing a problem that no longer exists.
                self._state.last_error = ""
            elif not self._state.last_error and self._source.last_error:
                self._state.last_error = self._source.last_error
            # While nothing is in flight the headline tracks the link, so the
            # panel recovers by itself the moment a key is installed instead of
            # showing whatever the link happened to be at start-up.
            if (
                self._state.phase == DeliveryPhase.IDLE
                and self._active is None
                and self._pending is None
            ):
                self._state.message = self._LINK_MESSAGE.get(
                    self._source.link, f"order source {self._source.link}"
                )
            self._state.auto_accept = self._auto_accept
            # The radius the return verdict is judged against, and the
            # handshake flag, are both useful DURING a flight - the operator
            # can see the drop was authorised before the aircraft is home.
            self._state.home_radius_m = self._home_radius
            self._state.ever_armed = self._ever_armed
            if self._state.order_id:
                self._state.ble_verified = self._state.order_id in self._ble_ok
            queued = {o.order_id for o in self._inbox}
            self._state.orders = [self._order_row(o, True) for o in self._inbox]
            # History minus anything already shown in the queue, so an order
            # never appears twice in the panel.
            recent_rows = [
                self._order_row(o, False)
                for o in self._recent
                if o.order_id not in queued
            ]
            # Merge in locally-tracked outcomes (Test order / Repeat, and
            # anything else that never had a Firestore doc for list_recent()
            # to find) so a rejected/aborted/cancelled order stays visible
            # here instead of vanishing the moment it stops being active -
            # newest local outcome first, capped with the Firestore ones.
            seen = queued | {row["order_id"] for row in recent_rows}
            for order_id, (order, local_status) in reversed(self._history.items()):
                if order_id in seen:
                    continue
                seen.add(order_id)
                recent_rows.append(
                    self._order_row(order, False, local_status=local_status)
                )
            self._state.recent = recent_rows[:20]
            self._state.selected_order_id = (
                self._pending.order_id if self._pending else ""
            )
            snapshot = DeliveryState(**{
                f: getattr(self._state, f) for f in (
                    "phase", "order_id", "recipient_id", "target_lat",
                    "target_lon", "hover_alt_m", "hover_seconds",
                    "hover_remaining_s", "distance_m", "remaining_m",
                    "waypoints", "auto_accept", "fc_mission_uploaded",
                    "link", "message", "last_error", "orders", "recent",
                    "selected_order_id",
                )
            })
        self.publish(Topics.DELIVERY_STATE, snapshot)
