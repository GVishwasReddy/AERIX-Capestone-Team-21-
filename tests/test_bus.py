"""Unit tests for the MessageBus."""
from __future__ import annotations

from drone_stack.bus import MessageBus


def test_publish_subscribe_delivers():
    bus = MessageBus()
    received = []
    bus.subscribe("/t", received.append)
    bus.publish("/t", 42)
    assert received == [42]


def test_latched_delivery_to_late_subscriber():
    bus = MessageBus()
    bus.publish("/t", "hello")
    received = []
    bus.subscribe("/t", received.append)
    assert received == ["hello"]


def test_unsubscribe_stops_delivery():
    bus = MessageBus()
    received = []
    sub = bus.subscribe("/t", received.append, deliver_latched=False)
    bus.publish("/t", 1)
    sub.unsubscribe()
    bus.publish("/t", 2)
    assert received == [1]


def test_faulty_subscriber_is_isolated():
    bus = MessageBus()
    good = []

    def boom(_msg):
        raise RuntimeError("subscriber failure")

    bus.subscribe("/t", boom, deliver_latched=False)
    bus.subscribe("/t", good.append, deliver_latched=False)
    bus.publish("/t", "x")  # must not raise
    assert good == ["x"]


def test_snapshot_and_counts():
    bus = MessageBus()
    bus.publish("/a", 1)
    bus.publish("/b", 2)
    bus.publish("/a", 3)
    snap = bus.snapshot()
    assert snap == {"/a": 3, "/b": 2}
    assert bus.publish_count("/a") == 2
    assert bus.latest("/b") == 2
