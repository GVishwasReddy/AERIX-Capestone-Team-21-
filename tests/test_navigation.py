"""Unit tests for navigation: collision avoidance, services, state (Phase 6)."""
from __future__ import annotations

from drone_stack.bus import MessageBus
from drone_stack.msg import MissionPhase, Obstacle, ObstacleArray
from drone_stack.nodes.navigation_node import CLEAR, SLOW, STOP, CollisionAvoider, NavigationNode
from drone_stack.srv import ServiceRegistry
from drone_stack.utils.config import Config


def _avoider() -> CollisionAvoider:
    return CollisionAvoider(Config.load())


def _front(distance: float) -> ObstacleArray:
    return ObstacleArray(obstacles=[Obstacle(distance_m=distance, bearing_deg=0.0)])


def test_avoider_stop_slow_clear():
    av = _avoider()
    assert av.evaluate(_front(1.0))[0] == STOP
    assert av.evaluate(_front(2.0))[0] == SLOW
    assert av.evaluate(_front(6.0))[0] == CLEAR


def test_avoider_ignores_obstacles_behind():
    av = _avoider()
    behind = ObstacleArray(obstacles=[Obstacle(distance_m=1.0, bearing_deg=175.0)])
    assert av.evaluate(behind)[0] == CLEAR


def test_avoider_empty_is_clear():
    av = _avoider()
    assert av.evaluate(None)[0] == CLEAR
    assert av.evaluate(ObstacleArray())[0] == CLEAR


def _nav() -> tuple[NavigationNode, ServiceRegistry]:
    services = ServiceRegistry()
    node = NavigationNode(MessageBus(), Config.load(), services)
    return node, services


def test_services_registered():
    _, services = _nav()
    for name in ("load_mission", "start_mission", "rtl", "emergency_stop"):
        assert services.has(name)


def test_load_and_start_mission():
    _, services = _nav()
    resp = services.call(
        "load_mission",
        waypoints=[{"x_m": 5.0, "y_m": 0.0}, {"x_m": 10.0, "y_m": 5.0}],
    )
    assert resp.success
    started = services.call("start_mission")
    assert started.success
    status = services.call("mission_status")
    assert status.data["total_wp"] == 2


def test_start_without_mission_fails():
    # A freshly constructed node has no mission (the demo is loaded in on_start).
    services = ServiceRegistry()
    NavigationNode(MessageBus(), Config.load(), services)
    assert not services.call("start_mission").success


def test_emergency_stop_sets_phase():
    node, services = _nav()
    services.call("emergency_stop")
    assert node._phase == MissionPhase.EMERGENCY
    assert services.call("mission_status").data["phase"] == "EMERGENCY"
