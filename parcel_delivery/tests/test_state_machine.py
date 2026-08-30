"""State machine tests with fake collaborators — no hardware, no network."""
import asyncio

import pytest

from exceptions import FlightAbort
from state_machine import InvalidTransition, MissionStateMachine, State

HOME_LAT, HOME_LON = -35.363262, 149.165237
GOOD_DESTINATION = {"lat": HOME_LAT + 0.001, "lon": HOME_LON, "alt_agl_m": 15.0}


class FakeFirebase:
    def __init__(self):
        self.statuses = []
        self.errors = []

    async def push_status(self, delivery_id, status, telemetry=None, error_message=None):
        self.statuses.append(status)
        if error_message:
            self.errors.append(error_message)

    async def push_telemetry(self, delivery_id, telemetry):
        pass


class FakeFlight:
    default_hover_altitude_m = 5.0

    def __init__(self, fail_at=None, abort_with=None):
        self.calls = []
        self.fail_at = fail_at
        self.abort_with = abort_with
        self.confirmation_result = True

    def _maybe_fail(self, name):
        if self.fail_at == name:
            if self.abort_with:
                raise FlightAbort(self.abort_with)
            raise RuntimeError(f"{name} failed")

    async def arm_and_takeoff(self, altitude_m):
        self.calls.append("arm_and_takeoff")
        self._maybe_fail("arm_and_takeoff")

    async def fly_to(self, lat, lon, altitude_m):
        self.calls.append("fly_to")
        self._maybe_fail("fly_to")

    async def stream_telemetry(self, delivery_id, firebase_client):
        while True:
            await asyncio.sleep(0.01)

    async def await_delivery_confirmation(self, timeout_s):
        self.calls.append("await_delivery_confirmation")
        self._maybe_fail("await_delivery_confirmation")
        return self.confirmation_result

    async def return_to_launch(self):
        self.calls.append("return_to_launch")


class FakePayload:
    def __init__(self, release_ok=True):
        self.release_ok = release_ok
        self.calls = []

    async def precision_locate(self):
        self.calls.append("precision_locate")
        return None

    async def release_payload(self, location=None):
        self.calls.append("release_payload")
        return self.release_ok


def build_machine(destination=None, flight=None, payload=None, firebase=None):
    return MissionStateMachine(
        delivery_id="test-delivery",
        delivery_doc={"destination": destination or dict(GOOD_DESTINATION)},
        firebase_client=firebase or FakeFirebase(),
        flight_controller=flight or FakeFlight(),
        payload_controller=payload or FakePayload(),
        home_lat=HOME_LAT,
        home_lon=HOME_LON,
        geofence_radius_m=300.0,
        min_altitude_m=2.0,
        max_altitude_m=50.0,
        hover_timeout_s=5.0,
    )


class TestHappyPath:
    def test_reaches_landed(self):
        firebase = FakeFirebase()
        machine = build_machine(firebase=firebase)
        asyncio.run(machine.run())
        assert machine.state is State.LANDED

    def test_pushes_every_status_in_order(self):
        firebase = FakeFirebase()
        machine = build_machine(firebase=firebase)
        asyncio.run(machine.run())
        assert firebase.statuses == [
            "mission_received", "takeoff", "enroute", "arrived_hover",
            "delivering", "delivery_confirmed", "rtl", "landed",
        ]

    def test_calls_flight_and_payload_in_order(self):
        flight, payload = FakeFlight(), FakePayload()
        machine = build_machine(flight=flight, payload=payload)
        asyncio.run(machine.run())
        assert flight.calls == [
            "arm_and_takeoff", "fly_to", "await_delivery_confirmation", "return_to_launch",
        ]
        assert payload.calls == ["precision_locate", "release_payload"]


class TestValidationRejection:
    def test_out_of_geofence_never_takes_off(self):
        flight = FakeFirebase()  # unused
        flight_ctl = FakeFlight()
        firebase = FakeFirebase()
        machine = build_machine(
            destination={"lat": 12.9716, "lon": 77.5946, "alt_agl_m": 15.0},
            flight=flight_ctl,
            firebase=firebase,
        )
        asyncio.run(machine.run())

        assert machine.state is State.ERROR
        assert flight_ctl.calls == []
        assert firebase.statuses == ["mission_received", "error"]
        assert "geofence" in firebase.errors[0]

    def test_bad_altitude_rejected(self):
        firebase = FakeFirebase()
        machine = build_machine(
            destination={"lat": HOME_LAT + 0.001, "lon": HOME_LON, "alt_agl_m": 500.0},
            firebase=firebase,
        )
        asyncio.run(machine.run())
        assert machine.state is State.ERROR
        assert "altitude" in firebase.errors[0]

    def test_invalid_coordinates_rejected(self):
        machine = build_machine(
            destination={"lat": 999.0, "lon": HOME_LON, "alt_agl_m": 15.0}
        )
        asyncio.run(machine.run())
        assert machine.state is State.ERROR


class TestFailsafeAbort:
    def test_abort_enroute_marks_aborted(self):
        firebase = FakeFirebase()
        flight = FakeFlight(fail_at="fly_to", abort_with="battery 15% below failsafe 20%")
        machine = build_machine(flight=flight, firebase=firebase)
        asyncio.run(machine.run())

        assert machine.state is State.ABORTED
        assert "aborted" in firebase.statuses
        assert "battery" in firebase.errors[0]

    def test_abort_during_takeoff(self):
        flight = FakeFlight(fail_at="arm_and_takeoff", abort_with="geofence breach")
        machine = build_machine(flight=flight)
        asyncio.run(machine.run())
        assert machine.state is State.ABORTED

    def test_unexpected_error_marks_error(self):
        flight = FakeFlight(fail_at="fly_to")  # plain RuntimeError
        machine = build_machine(flight=flight)
        asyncio.run(machine.run())
        assert machine.state is State.ERROR


class TestPayloadFailure:
    def test_failed_release_still_returns_home(self):
        flight, payload = FakeFlight(), FakePayload(release_ok=False)
        firebase = FakeFirebase()
        machine = build_machine(flight=flight, payload=payload, firebase=firebase)
        asyncio.run(machine.run())

        # A failed release must still bring the drone home, not strand it in a
        # terminal ERROR state while airborne.
        assert machine.state is State.LANDED
        assert "return_to_launch" in flight.calls
        assert firebase.statuses[-2:] == ["rtl", "landed"]
        assert "payload release failed" in firebase.errors

    def test_hover_timeout_still_returns_home(self):
        flight = FakeFlight()
        flight.confirmation_result = False  # timed out waiting for confirmation
        firebase = FakeFirebase()
        machine = build_machine(flight=flight, firebase=firebase)
        asyncio.run(machine.run())

        assert machine.state is State.LANDED
        assert "delivery_confirmed" not in firebase.statuses
        assert "rtl" in firebase.statuses
        assert "return_to_launch" in flight.calls


class TestTransitionRules:
    def test_illegal_transition_raises(self):
        machine = build_machine()

        async def jump():
            await machine._transition(State.LANDED)

        with pytest.raises(InvalidTransition):
            asyncio.run(jump())

    def test_terminal_states_have_no_exits(self):
        from state_machine import _TRANSITIONS
        assert _TRANSITIONS[State.LANDED] == set()
        assert _TRANSITIONS[State.ERROR] == set()
        assert _TRANSITIONS[State.ABORTED] == set()

    def test_every_state_has_a_transition_entry(self):
        from state_machine import _TRANSITIONS
        assert set(_TRANSITIONS) == set(State)
