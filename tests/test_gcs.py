"""Tests for the GCS hub: payload shape, commands, mission plan round-trip.

These exercise the engine through the hub (no FastAPI/uvicorn needed).
"""
from __future__ import annotations

import time

import pytest

from drone_stack.gcs.hub import GcsHub
from drone_stack.utils.config import Config


@pytest.mark.timeout(40)
def test_hub_payload_and_commands(monkeypatch):
    monkeypatch.setenv("DRONE_NAVIGATION_AUTO_ARM", "false")
    hub = GcsHub(Config.load())
    hub.start()
    try:
        # let telemetry + lidar + obstacles + avoidance flow
        deadline = time.time() + 8
        payload = {}
        while time.time() < deadline:
            payload = hub.build_payload()
            if payload["scan"] and payload["avoidance"] is not None:
                break
            time.sleep(0.2)

        assert payload["telemetry"]["gps_fix"] == "3D"
        assert payload["scan"] and len(payload["scan"]["ranges"]) == 360
        assert payload["avoidance"] is not None
        assert isinstance(payload["obstacles"], list)
        assert len(payload["cameras"]) == 2
        assert "cpu" in payload["health"]

        # NL command via hub
        r = hub.command("nl", {"text": "take off to 5m"})
        assert r["ok"], r

        # scan control + avoidance toggle services reachable through the hub
        assert hub.command("scan_pause")["ok"]
        assert hub.command("scan_resume")["ok"]
        assert hub.command("avoid_disable")["ok"]
        assert hub.command("avoid_enable")["ok"]

        # give it time to arm/climb, then verify it is moving/armed
        time.sleep(3)
        payload = hub.build_payload()
        assert payload["telemetry"]["armed"] is True
    finally:
        hub.stop()


@pytest.mark.timeout(30)
def test_mission_plan_roundtrip(monkeypatch):
    monkeypatch.setenv("DRONE_NAVIGATION_AUTO_ARM", "false")
    hub = GcsHub(Config.load())
    hub.start()
    try:
        time.sleep(2)  # allow home/GPS to be set
        hub.command("upload_mission", {"waypoints": [
            {"lat": 47.39780, "lon": 8.54560, "alt": 5},
            {"lat": 47.39790, "lon": 8.54570, "alt": 6},
        ]})
        plan = hub.export_plan()
        assert plan["fileType"] == "Plan"
        assert len(plan["mission"]["items"]) == 2
        result = hub.import_plan(plan)
        assert result["ok"], result
    finally:
        hub.stop()


@pytest.mark.timeout(20)
def test_unknown_command():
    hub = GcsHub(Config.load())
    # no start needed for a pure command-routing check
    r = hub.command("does_not_exist")
    assert not r["ok"]
    assert "unknown command" in r["message"]


@pytest.mark.timeout(20)
def test_ble_auth_event_publishes_onto_the_novelty_bus():
    """The BLE peripheral (drone_ble_peripheral.py) is a separate process
    that bridges here over POST /api/command - see hub.py's own
    _on_ble_auth_event docstring for why BLE carries no release power."""
    from drone_stack.novelty.topics import NoveltyTopics
    from drone_stack.novelty.types import BleAuthEvent

    hub = GcsHub(Config.load())
    r = hub.command("ble_auth_event", {"authenticated": True, "phone_gps": [12.34, 56.78]})
    assert r["ok"], r

    event = hub.bus.latest(NoveltyTopics.BLE_AUTH_EVENT)
    assert isinstance(event, BleAuthEvent)
    assert event.authenticated is True
    assert event.phone_gps == (12.34, 56.78)
    assert event.rssi_dbm is None


@pytest.mark.timeout(20)
def test_ble_auth_event_without_phone_gps_carries_rssi_only():
    from drone_stack.novelty.topics import NoveltyTopics

    hub = GcsHub(Config.load())
    hub.command("ble_auth_event", {"authenticated": False, "rssi_dbm": -70.0})

    event = hub.bus.latest(NoveltyTopics.BLE_AUTH_EVENT)
    assert event.authenticated is False
    assert event.phone_gps is None
    assert event.rssi_dbm == -70.0


@pytest.mark.timeout(20)
def test_ble_auth_event_defaults_to_unauthenticated_when_missing():
    from drone_stack.novelty.topics import NoveltyTopics

    hub = GcsHub(Config.load())
    hub.command("ble_auth_event", {})

    event = hub.bus.latest(NoveltyTopics.BLE_AUTH_EVENT)
    assert event.authenticated is False
