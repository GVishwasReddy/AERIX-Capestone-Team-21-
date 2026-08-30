"""Central orchestrator: ties validator, flight controller and payload
controller together, enforces valid state transitions, and pushes every
transition to Firebase.
"""
from __future__ import annotations

import asyncio
import logging
from enum import Enum
from typing import TYPE_CHECKING, Any

from exceptions import FlightAbort
from mission_validator import validate_delivery_request

if TYPE_CHECKING:  # heavy optional deps — only needed for type checking
    from firebase_client import FirebaseClient
    from flight_controller import FlightController
    from payload_controller import PayloadController

logger = logging.getLogger("state_machine")


class State(Enum):
    IDLE = "idle"
    MISSION_RECEIVED = "mission_received"
    TAKEOFF = "takeoff"
    ENROUTE = "enroute"
    ARRIVED_HOVER = "arrived_hover"
    DELIVERING = "delivering"
    DELIVERY_CONFIRMED = "delivery_confirmed"
    RETURNING = "rtl"
    LANDED = "landed"
    ERROR = "error"
    ABORTED = "aborted"


# States a mission may legally move to from each state.
_TRANSITIONS: dict[State, set[State]] = {
    State.IDLE: {State.MISSION_RECEIVED},
    State.MISSION_RECEIVED: {State.TAKEOFF, State.ERROR, State.ABORTED},
    State.TAKEOFF: {State.ENROUTE, State.ERROR, State.ABORTED, State.RETURNING},
    State.ENROUTE: {State.ARRIVED_HOVER, State.ERROR, State.ABORTED, State.RETURNING},
    State.ARRIVED_HOVER: {State.DELIVERING, State.RETURNING, State.ERROR, State.ABORTED},
    State.DELIVERING: {State.DELIVERY_CONFIRMED, State.RETURNING, State.ERROR},
    State.DELIVERY_CONFIRMED: {State.RETURNING},
    State.RETURNING: {State.LANDED, State.ERROR},
    State.LANDED: set(),
    State.ERROR: set(),
    State.ABORTED: set(),
}


class InvalidTransition(Exception):
    pass


class MissionStateMachine:
    def __init__(
        self,
        delivery_id: str,
        delivery_doc: dict[str, Any],
        firebase_client: "FirebaseClient",
        flight_controller: "FlightController",
        payload_controller: "PayloadController",
        home_lat: float,
        home_lon: float,
        geofence_radius_m: float,
        min_altitude_m: float,
        max_altitude_m: float,
        hover_timeout_s: float,
    ) -> None:
        self.delivery_id = delivery_id
        self.delivery_doc = delivery_doc
        self.firebase = firebase_client
        self.flight = flight_controller
        self.payload = payload_controller
        self.home_lat = home_lat
        self.home_lon = home_lon
        self.geofence_radius_m = geofence_radius_m
        self.min_altitude_m = min_altitude_m
        self.max_altitude_m = max_altitude_m
        self.hover_timeout_s = hover_timeout_s
        self.state = State.IDLE

    async def _transition(self, new_state: State, error_message: str | None = None) -> None:
        allowed = _TRANSITIONS.get(self.state, set())
        if new_state not in allowed and new_state is not self.state:
            raise InvalidTransition(f"cannot go from {self.state} to {new_state}")
        logger.info("delivery %s: %s -> %s", self.delivery_id, self.state, new_state)
        self.state = new_state
        await self.firebase.push_status(
            self.delivery_id,
            new_state.value,
            error_message=error_message,
        )

    async def run(self) -> None:
        try:
            await self._transition(State.MISSION_RECEIVED)

            destination = self.delivery_doc["destination"]
            lat, lon = destination["lat"], destination["lon"]
            alt_agl_m = destination.get("alt_agl_m", self.flight.default_hover_altitude_m)

            result = validate_delivery_request(
                lat, lon, alt_agl_m,
                self.home_lat, self.home_lon,
                self.geofence_radius_m, self.min_altitude_m, self.max_altitude_m,
            )
            if not result.ok:
                await self._transition(State.ERROR, error_message=result.reason)
                return

            await self._transition(State.TAKEOFF)
            await self.flight.arm_and_takeoff(alt_agl_m)

            await self._transition(State.ENROUTE)
            telemetry_task = asyncio.create_task(
                self.flight.stream_telemetry(self.delivery_id, self.firebase)
            )
            try:
                await self.flight.fly_to(lat, lon, alt_agl_m)
            finally:
                telemetry_task.cancel()

            await self._transition(State.ARRIVED_HOVER)

            await self._transition(State.DELIVERING)
            location = await self.payload.precision_locate()
            released = await self.payload.release_payload(location)
            if not released:
                # The parcel is still aboard, but the drone must come home
                # regardless. Record the reason on the RTL transition rather
                # than entering the terminal ERROR state, which cannot fly.
                logger.error("delivery %s: payload release failed", self.delivery_id)
                await self._transition(
                    State.RETURNING, error_message="payload release failed"
                )
                await self.flight.return_to_launch()
                await self._transition(State.LANDED)
                return

            try:
                confirmed = await self.flight.await_delivery_confirmation(
                    timeout_s=self.hover_timeout_s
                )
            except asyncio.TimeoutError:
                confirmed = False

            if confirmed:
                await self._transition(State.DELIVERY_CONFIRMED)
            # Whether confirmed or timed out, we return home either way.
            await self._transition(State.RETURNING)
            await self.flight.return_to_launch()
            await self._transition(State.LANDED)

        except FlightAbort as exc:
            logger.error("delivery %s aborted mid-flight: %s", self.delivery_id, exc)
            await self._transition(State.ABORTED, error_message=str(exc))
        except InvalidTransition:
            raise
        except Exception as exc:  # noqa: BLE001 - report and stop, never crash the listener
            logger.exception("delivery %s failed", self.delivery_id)
            await self._transition(State.ERROR, error_message=str(exc))
