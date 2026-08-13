"""Integration test: bring up the whole stack in simulation and verify it flies.

This exercises the real MessageBus, all nodes, the mock Pixhawk + LiDAR, fusion,
obstacle detection and the mission state machine together - no hardware, no web
server (disabled so the test never binds a port).
"""
from __future__ import annotations

import time

import pytest

from drone_stack.bus.topics import Topics
from drone_stack.launch.bringup import build_supervisor
from drone_stack.msg import MissionPhase
from drone_stack.utils.config import Config


@pytest.mark.timeout(40)
def test_sim_stack_boots_and_arms(monkeypatch):
    monkeypatch.setenv("DRONE_WEB_ENABLED", "false")
    config = Config.load()
    assert config.mode == "sim"

    supervisor = build_supervisor(config)
    supervisor.start()
    bus = supervisor.bus
    try:
        armed = False
        flying = False
        deadline = time.time() + 25
        while time.time() < deadline:
            arm = bus.latest(Topics.ARMED)
            if arm is not None and arm.armed:
                armed = True
            mission = bus.latest(Topics.MISSION_STATE)
            if mission is not None and mission.phase in (
                MissionPhase.TAKEOFF,
                MissionPhase.NAVIGATE,
                MissionPhase.AVOID,
                MissionPhase.RTL,
            ):
                flying = True
            if armed and flying:
                break
            time.sleep(0.2)

        # Telemetry is flowing
        assert bus.latest(Topics.HEARTBEAT) is not None, "no heartbeat"
        assert bus.latest(Topics.GPS) is not None, "no GPS"
        assert bus.latest(Topics.BATTERY) is not None, "no battery"

        # LiDAR + perception are flowing
        scan = bus.latest(Topics.SCAN)
        assert scan is not None and scan.count == 360, "no lidar scan"
        assert bus.latest(Topics.OBSTACLES) is not None, "no obstacle output"
        assert bus.latest(Topics.FUSED_STATE) is not None, "no fused state"

        # Diagnostics are flowing
        assert bus.latest(Topics.DIAGNOSTICS) is not None, "no diagnostics"

        # The mission actually started and armed autonomously
        assert armed, "vehicle did not auto-arm in simulation"
        assert flying, "mission did not progress past arming"
    finally:
        supervisor.stop()


@pytest.mark.timeout(20)
def test_nodes_recover_from_crash(monkeypatch):
    """A crash inside a node's step must not kill the node thread."""
    monkeypatch.setenv("DRONE_WEB_ENABLED", "false")
    config = Config.load()
    supervisor = build_supervisor(config)

    # Find the obstacle node and make its first few steps explode.
    nodes = {n.node_name: n for n in supervisor.nodes}
    obstacle = nodes["obstacles"]
    original = obstacle.step
    calls = {"n": 0}

    def exploding_step():
        calls["n"] += 1
        if calls["n"] <= 3:
            raise RuntimeError("boom")
        return original()

    obstacle.step = exploding_step

    supervisor.start()
    try:
        time.sleep(3.0)
        # Despite the induced crashes, the node recovered and is still alive.
        assert obstacle.is_alive()
        assert obstacle._restarts >= 1
    finally:
        supervisor.stop()
