"""The delivery verdict: what happened to the parcel, and to the aircraft.

These two are independent, and the panel used to publish one word for both.
``DeliveryPhase.LANDED`` was rendered as "DELIVERED" whenever the aircraft had
armed and reached a terminal phase - so a flight that went out, never got a
valid recipient handshake, and came home with the parcel still aboard told the
operator the delivery was complete.

The BLE drop gate is the only evidence of an actual handover on this airframe:
the payload servo is not instrumented (see
``parcel_delivery/pi/payload_controller.py``). So "delivered" means that gate
passed for *this* order id, and nothing else does.
"""
from __future__ import annotations

from drone_stack.bus import MessageBus
from drone_stack.msg import (
    DeliveryBleResult,
    DeliveryOrder,
    DeliveryOutcome,
    DeliveryPhase,
    ReturnOutcome,
    describe_outcome,
)
from drone_stack.nodes.firebase_delivery_node import FirebaseDeliveryNode
from drone_stack.nodes.navigation_node import NavigationNode
from drone_stack.srv import ServiceRegistry
from drone_stack.utils.config import Config

HOME_LAT, HOME_LON = 12.9017, 77.6540


def _node(**overrides) -> FirebaseDeliveryNode:
    config = Config.load()
    raw = config.raw
    raw.setdefault("delivery", {}).update({"source": "none", **overrides})
    config = Config(raw)
    bus = MessageBus()
    services = ServiceRegistry()
    NavigationNode(bus, config, services)
    node = FirebaseDeliveryNode(bus, config, services)
    node._home = (HOME_LAT, HOME_LON)
    node._pos = (HOME_LAT, HOME_LON)          # sitting on the pad
    node._state.order_id = "ORD1"
    return node


def _north(metres: float) -> tuple[float, float]:
    return HOME_LAT + metres / 111320.0, HOME_LON


def _flew(node: FirebaseDeliveryNode) -> None:
    """The ordinary shape of a completed flight: armed, went out, flew RTL."""
    node._ever_armed = True
    node._saw_rtl = True


# -- the parcel --------------------------------------------------------------
def test_a_verified_handshake_is_a_delivery():
    node = _node()
    _flew(node)
    node._ble_ok.add("ORD1")
    parcel, ret, _ = node._classify_outcome()
    assert parcel is DeliveryOutcome.DELIVERED
    assert ret is ReturnOutcome.RETURNED


def test_a_flight_home_without_a_handshake_is_not_a_delivery():
    """The headline regression.

    Nothing about this flight differs from a successful one as far as the
    airframe is concerned - it armed, flew the legs, hovered, came home and
    disarmed. The only thing that did not happen is the handover.
    """
    node = _node()
    _flew(node)
    parcel, ret, why = node._classify_outcome()
    assert parcel is DeliveryOutcome.FAILED
    assert ret is ReturnOutcome.RETURNED
    assert describe_outcome(parcel, ret) == "NOT DELIVERED · RETURNED WITH PARCEL"
    assert "handshake" in why


def test_a_handshake_for_another_order_does_not_credit_this_one():
    node = _node()
    _flew(node)
    node._ble_ok.add("SOMEONE-ELSE")
    parcel, _, _ = node._classify_outcome()
    assert parcel is DeliveryOutcome.FAILED


def test_an_abort_is_not_a_failed_delivery():
    """"We stopped it" and "it did not work" are different stories."""
    node = _node()
    _flew(node)
    node._abort_reason = "aborted by operator"
    parcel, ret, why = node._classify_outcome()
    assert parcel is DeliveryOutcome.ABORTED
    assert describe_outcome(parcel, ret) == "ABORTED · RETURNED HOME"
    assert "operator" in why


def test_a_delivery_that_completed_before_an_abort_still_counts():
    """The parcel is gone. Aborting the return leg does not bring it back."""
    node = _node()
    _flew(node)
    node._ble_ok.add("ORD1")
    node._abort_reason = "aborted by operator"
    parcel, _, _ = node._classify_outcome()
    assert parcel is DeliveryOutcome.DELIVERED


def test_never_arming_is_neither_delivered_nor_failed():
    node = _node()
    node._abort_reason = "the autopilot would not arm"
    parcel, ret, _ = node._classify_outcome()
    assert parcel is DeliveryOutcome.NEVER_FLEW
    assert ret is ReturnOutcome.ON_PAD
    assert describe_outcome(parcel, ret) == "NEVER LAUNCHED"


# -- the aircraft ------------------------------------------------------------
def test_down_beyond_the_home_radius_is_not_a_return():
    node = _node(home_radius_m=15.0)
    _flew(node)
    node._ble_ok.add("ORD1")
    node._pos = _north(120.0)
    parcel, ret, why = node._classify_outcome()
    assert parcel is DeliveryOutcome.DELIVERED
    assert ret is ReturnOutcome.NOT_RETURNED
    assert describe_outcome(parcel, ret) == "DELIVERED · DID NOT RETURN"
    assert "from home" in why


def test_landing_just_inside_the_radius_is_a_return():
    node = _node(home_radius_m=15.0)
    _flew(node)
    node._pos = _north(10.0)
    _, ret, _ = node._classify_outcome()
    assert ret is ReturnOutcome.RETURNED


def test_an_emergency_landing_at_home_is_still_not_a_return():
    """It put itself down rather than flying home.

    Someone has to go and inspect an aircraft that failsafed, wherever it
    stopped - so proximity to the pad does not make this a clean return.
    """
    node = _node()
    node._ever_armed = True
    node._saw_emergency = True
    node._pos = (HOME_LAT, HOME_LON)          # right on the pad
    _, ret, why = node._classify_outcome()
    assert ret is ReturnOutcome.NOT_RETURNED
    assert "without flying home" in why


def test_landing_where_it_stood_is_not_a_return():
    node = _node()
    node._ever_armed = True
    node._saw_land = True                     # LAND, and no RTL leg ever
    _, ret, _ = node._classify_outcome()
    assert ret is ReturnOutcome.NOT_RETURNED


def test_land_after_an_rtl_leg_is_the_normal_way_home():
    node = _node()
    node._ever_armed = True
    node._saw_rtl = True
    node._saw_land = True                     # RTL then touchdown - ordinary
    _, ret, _ = node._classify_outcome()
    assert ret is ReturnOutcome.RETURNED


def test_no_position_falls_back_to_whether_it_flew_the_return_leg():
    node = _node()
    _flew(node)
    node._pos = None
    _, ret, _ = node._classify_outcome()
    assert ret is ReturnOutcome.RETURNED


def test_no_position_and_no_return_leg_is_unknown_not_a_guess():
    node = _node()
    node._ever_armed = True
    node._pos = None
    _, ret, _ = node._classify_outcome()
    assert ret is ReturnOutcome.UNKNOWN


# -- every combination the operator can be shown -----------------------------
def test_every_outcome_pair_has_its_own_headline():
    pairs = [
        (DeliveryOutcome.DELIVERED, ReturnOutcome.RETURNED),
        (DeliveryOutcome.DELIVERED, ReturnOutcome.NOT_RETURNED),
        (DeliveryOutcome.FAILED, ReturnOutcome.RETURNED),
        (DeliveryOutcome.FAILED, ReturnOutcome.NOT_RETURNED),
        (DeliveryOutcome.ABORTED, ReturnOutcome.RETURNED),
        (DeliveryOutcome.ABORTED, ReturnOutcome.NOT_RETURNED),
        (DeliveryOutcome.ABORTED, ReturnOutcome.ON_PAD),
        (DeliveryOutcome.NEVER_FLEW, ReturnOutcome.ON_PAD),
    ]
    labels = [describe_outcome(a, b) for a, b in pairs]
    assert len(set(labels)) == len(labels), "two outcomes read the same"
    assert all(labels), "an outcome rendered blank"
    # None of them may claim a delivery that did not happen.
    for (parcel, _), label in zip(pairs, labels):
        if parcel is not DeliveryOutcome.DELIVERED:
            assert not label.startswith("DELIVERED"), label


# -- the BLE subscription ----------------------------------------------------
def test_the_drop_gate_marks_the_order_it_names():
    node = _node()
    node._on_ble_result(DeliveryBleResult(order_id="ORD1", success=True))
    assert "ORD1" in node._ble_ok
    assert node._state.ble_verified is True


def test_a_failed_drop_gate_marks_nothing():
    node = _node()
    node._on_ble_result(DeliveryBleResult(order_id="ORD1", success=False))
    assert not node._ble_ok
    assert node._state.ble_verified is False


# -- an abort has to be followed home ----------------------------------------
def test_aborting_keeps_tracking_the_aircraft_so_the_return_is_observed():
    """ABORT used to close the order the instant it was pressed.

    The aircraft is still airborne at that moment, so nothing ever observed
    whether it made it home - and "aborted · returned home" and "aborted · did
    not return" are very different things to hand an operator.
    """
    node = _node()
    order = DeliveryOrder(order_id="ORD1", target_lat=HOME_LAT, target_lon=HOME_LON)
    node._active = order
    node._ever_armed = True

    node.services.call("delivery_abort")

    assert node._active is order, "stopped following the aircraft mid-flight"
    assert node._abort_reason
    assert node._state.phase is DeliveryPhase.ABORTED


def test_finishing_publishes_the_verdict_and_releases_the_order():
    node = _node()
    order = DeliveryOrder(order_id="ORD1", target_lat=HOME_LAT, target_lon=HOME_LON)
    node._active = order
    _flew(node)
    node._ble_ok.add("ORD1")

    node._finish_delivery(order, DeliveryPhase.LANDED)

    s = node._state
    assert s.outcome is DeliveryOutcome.DELIVERED
    assert s.return_outcome is ReturnOutcome.RETURNED
    assert s.outcome_label == "DELIVERED · RETURNED HOME"
    assert s.ble_verified is True
    assert s.outcome_at > 0
    assert node._active is None
    # The order book shows the verdict, not the phase.
    assert node._history["ORD1"][1] == "DELIVERED · RETURNED HOME"


def test_accepting_a_new_order_clears_the_previous_verdict():
    node = _node()
    _flew(node)
    node._abort_reason = "aborted by operator"
    node._state.outcome = DeliveryOutcome.ABORTED
    node._state.outcome_label = "ABORTED · RETURNED HOME"

    node._reset_outcome_evidence()

    assert node._state.outcome is DeliveryOutcome.PENDING
    assert node._state.outcome_label == ""
    assert node._abort_reason == ""
    assert node._saw_rtl is False
    assert node._ever_armed is False
