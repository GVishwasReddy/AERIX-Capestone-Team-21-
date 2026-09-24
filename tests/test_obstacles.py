"""Unit tests for obstacle detection & classification (Phase 5)."""
from __future__ import annotations

import math

from drone_stack.bus import MessageBus
from drone_stack.msg import LaserScan, ObstacleClass
from drone_stack.nodes.obstacle_node import ObstacleNode
from drone_stack.utils.config import Config


def make_scan(returns: dict[int, float], bins: int = 360) -> LaserScan:
    """Build a LaserScan (angle_min=0); ``returns`` maps beam index -> range (m)."""
    inc = 2.0 * math.pi / bins
    ranges = [math.inf] * bins
    for idx, rng in returns.items():
        ranges[idx % bins] = rng
    return LaserScan(
        angle_min=0.0,
        angle_max=2.0 * math.pi,
        angle_increment=inc,
        range_min=0.15,
        range_max=12.0,
        ranges=ranges,
        intensities=[0.0] * bins,
    )


def _node() -> ObstacleNode:
    return ObstacleNode(MessageBus(), Config.load())


def test_pole_detected_in_front():
    scan = make_scan({358: 3.0, 359: 3.0, 0: 3.0, 1: 3.0, 2: 3.0})
    obstacles = _node()._detect(scan)
    assert len(obstacles) == 1
    o = obstacles[0]
    assert o.classification == ObstacleClass.POLE
    assert abs(o.bearing_deg) < 5.0          # dead ahead
    assert abs(o.distance_m - 3.0) < 0.1


def test_wall_detected():
    scan = make_scan({i: 5.0 for i in range(0, 31)})
    obstacles = _node()._detect(scan)
    assert len(obstacles) == 1
    assert obstacles[0].classification == ObstacleClass.WALL
    assert obstacles[0].width_m >= 2.5


def test_person_detected():
    scan = make_scan({i: 3.0 for i in range(8, 18)})
    obstacles = _node()._detect(scan)
    assert len(obstacles) == 1
    assert obstacles[0].classification == ObstacleClass.PERSON


def test_vehicle_detected():
    scan = make_scan({i: 5.0 for i in range(0, 24)})
    obstacles = _node()._detect(scan)
    assert len(obstacles) == 1
    assert obstacles[0].classification == ObstacleClass.VEHICLE


def test_bearing_sign_is_right_positive():
    # A cluster centred near beam 350 (~ -10 deg sensor) is to the right.
    scan = make_scan({348: 4.0, 349: 4.0, 350: 4.0, 351: 4.0, 352: 4.0})
    obstacles = _node()._detect(scan)
    assert len(obstacles) == 1
    assert obstacles[0].bearing_deg > 0.0


def test_danger_flag_when_close():
    scan = make_scan({0: 1.0, 1: 1.0, 2: 1.0})
    obstacles = _node()._detect(scan)
    assert obstacles[0].danger is True


def test_an_obstacle_straddling_the_nose_has_its_true_angular_width():
    """Beam 0 is the nose, so anything straight ahead is merged across the
    0/360 seam. Last-minus-first read that as ~358 deg wide, which blocked
    the entire VFH+ histogram: no gap anywhere, straight into the brake."""
    scan = make_scan({358: 3.0, 359: 3.0, 0: 3.0, 1: 3.0, 2: 3.0})
    o = _node()._detect(scan)[0]
    assert abs(o.angular_width_deg - 4.0) < 0.5


def test_small_clusters_ignored():
    # Close in, a single beam is far too small for anything min_object_width_m
    # wide (which fills ~5.7 bins at 1 m). Far out a single beam CAN be a real
    # pole - see test_long_range_avoidance for how those are gated instead.
    scan = make_scan({0: 1.0})
    assert _node()._detect(scan) == []
