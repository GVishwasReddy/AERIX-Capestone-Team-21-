"""Unit tests for the simulated world / mock hardware (Phase 7)."""
from __future__ import annotations

import math

from drone_stack.msg import NavCommand
from drone_stack.sim import MockLidar, MockMavlink, SimWorld
from drone_stack.utils.config import Config
from drone_stack.utils.geometry import enu_to_geodetic


def _world() -> SimWorld:
    return SimWorld(Config.load())


def test_disarmed_vehicle_does_not_move():
    world = _world()
    for _ in range(20):
        world.step(dt=0.1)
    assert world.state.x == 0.0 and world.state.z == 0.0


def test_takeoff_climbs_to_altitude():
    world = _world()
    world.command(NavCommand("arm"))
    world.command(NavCommand("takeoff", {"altitude": 5.0}))
    for _ in range(80):
        world.step(dt=0.1)
    assert world.state.z > 4.5


def test_goto_moves_horizontally():
    world = _world()
    world.command(NavCommand("arm"))
    world.command(NavCommand("takeoff", {"altitude": 5.0}))
    for _ in range(60):
        world.step(dt=0.1)
    lat, lon, _ = world.home
    tgt_lat, tgt_lon = enu_to_geodetic(6.0, 0.0, lat, lon)
    world.command(NavCommand("goto", {"lat": tgt_lat, "lon": tgt_lon, "alt": 5.0}))
    for _ in range(120):
        world.step(dt=0.1)
    assert world.state.x > 5.0


def test_position_hold_parks_the_aircraft():
    """POSHOLD is what the delivery hover sits in - it must actually stop."""
    world = _world()
    world.command(NavCommand("arm"))
    world.command(NavCommand("takeoff", {"altitude": 3.0}))
    for _ in range(60):
        world.step(dt=0.1)
    lat, lon, _ = world.home
    tgt_lat, tgt_lon = enu_to_geodetic(20.0, 0.0, lat, lon)
    world.command(NavCommand("goto", {"lat": tgt_lat, "lon": tgt_lon, "alt": 3.0}))
    for _ in range(60):
        world.step(dt=0.1)
    world.command(NavCommand("set_mode", {"mode": "POSHOLD"}))
    held = world.state.x
    for _ in range(100):
        world.step(dt=0.1)
    assert abs(world.state.x - held) < 0.6
    assert world.state.armed


def test_smart_rtl_comes_home_descends_and_disarms():
    """The whole delivery ends here: without the descent-and-disarm stage the
    mission would sit in RTL forever and never report COMPLETE."""
    world = _world()
    world.command(NavCommand("arm"))
    world.command(NavCommand("takeoff", {"altitude": 3.0}))
    for _ in range(60):
        world.step(dt=0.1)
    lat, lon, _ = world.home
    tgt_lat, tgt_lon = enu_to_geodetic(25.0, 0.0, lat, lon)
    world.command(NavCommand("goto", {"lat": tgt_lat, "lon": tgt_lon, "alt": 3.0}))
    for _ in range(80):
        world.step(dt=0.1)
    world.command(NavCommand("set_mode", {"mode": "SMART_RTL"}))
    for _ in range(1200):
        world.step(dt=0.1)
        if not world.state.armed:
            break
    assert not world.state.armed
    assert math.hypot(world.state.x, world.state.y) < 1.5
    assert world.state.z <= 0.05


def test_raycast_hits_wall():
    world = _world()  # default config has a wall centred at x=8, width 6 -> face at x=5
    distance = world.raycast(world_angle=0.0, max_range=12.0)
    assert math.isclose(distance, 5.0, abs_tol=0.2)


def test_battery_drains_when_armed():
    world = _world()
    start = world.battery_pct()
    world.command(NavCommand("arm"))
    for _ in range(200):
        world.step(dt=0.1)
    assert world.battery_pct() < start


def test_mock_lidar_produces_scan():
    world = _world()
    lidar = MockLidar(world, Config.load().section("lidar"))
    assert lidar.connect()
    scan = lidar.read_scan()
    assert scan is not None
    assert scan.count == 360
    assert any(math.isfinite(r) for r in scan.ranges)


def test_mock_mavlink_roundtrip():
    world = _world()
    mav = MockMavlink(world, Config.load().section("mavlink"))
    assert mav.connect()
    mav.send_command(NavCommand("arm"))
    messages = mav.receive()
    types = {type(m).__name__ for m in messages}
    assert "Heartbeat" in types and "GpsFix" in types and "Battery" in types
