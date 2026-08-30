"""LiDAR beam handedness - the scan must not be mirrored left/right.

The RPLIDAR numbers its beams CLOCKWISE looking down on the unit. LaserScan,
PointCloud, ObstacleNode bearings and the GCS radar are all COUNTER-CLOCKWISE
(x forward, y left). ``RealLidar`` converts between the two; these tests pin
the direction of that conversion, because getting it backwards mirrors the
radar AND makes collision avoidance dodge toward the obstacle.

Reference points used throughout: a raw (device, clockwise) angle of 90 deg is
physically to the RIGHT of the nose, 270 deg is to the LEFT.
"""
from __future__ import annotations

import math

from drone_stack.bus import MessageBus
from drone_stack.interfaces.lidar_interface import DEFAULT_BINS, RealLidar
from drone_stack.nodes.obstacle_node import ObstacleNode
from drone_stack.utils.config import Config
from drone_stack.utils.geometry import polar_to_cartesian

INC = 2.0 * math.pi / DEFAULT_BINS


def _beams(raw_degs, distance_m: float = 3.0):
    return [(47.0, float(a), distance_m * 1000.0) for a in raw_degs]


def _hits(scan) -> list[int]:
    return [i for i, r in enumerate(scan.ranges) if math.isfinite(r)]


def _xy(scan, idx) -> tuple[float, float]:
    return polar_to_cartesian(scan.ranges[idx], idx * INC)


# -- driver output -----------------------------------------------------------
def test_return_on_the_right_stays_on_the_right():
    scan = RealLidar({})._to_laserscan(_beams([90]))   # device CW 90 = right
    assert _hits(scan) == [270]
    x, y = _xy(scan, 270)
    assert y < -2.9 and abs(x) < 0.1                   # y = +left, so right is negative


def test_return_on_the_left_stays_on_the_left():
    scan = RealLidar({})._to_laserscan(_beams([270]))  # device CW 270 = left
    assert _hits(scan) == [90]
    x, y = _xy(scan, 90)
    assert y > 2.9 and abs(x) < 0.1


def test_dead_ahead_is_unaffected_by_the_flip():
    scan = RealLidar({})._to_laserscan(_beams([0]))
    assert _hits(scan) == [0]
    x, y = _xy(scan, 0)
    assert x > 2.9 and abs(y) < 0.1


def test_front_right_quadrant_is_forward_and_right():
    scan = RealLidar({})._to_laserscan(_beams([45]))   # CW 45 = ahead and right
    assert _hits(scan) == [315]
    x, y = _xy(scan, 315)
    assert x > 0 and y < 0


def test_clockwise_false_passes_the_raw_angle_through():
    """Escape hatch for a sensor that already reports counter-clockwise."""
    scan = RealLidar({"clockwise": False})._to_laserscan(_beams([90]))
    assert _hits(scan) == [90]


def test_offset_is_the_raw_angle_pointing_at_the_nose():
    """With the nose at raw 90, a raw-90 return must come out dead ahead."""
    lidar = RealLidar({"angle_offset_deg": 90.0})
    assert _hits(lidar._to_laserscan(_beams([90]))) == [0]
    # ...and something 90 deg clockwise of the nose is still on the right.
    scan = lidar._to_laserscan(_beams([180]))
    assert _hits(scan) == [270]
    assert _xy(scan, 270)[1] < 0


def test_fov_window_is_unchanged_by_the_flip():
    """The 250 deg window is symmetric about the front, so mirroring cannot
    move the blind wedge off the tail - but the beams inside it do swap sides."""
    lidar = RealLidar({})
    scan = lidar._to_laserscan(_beams(range(360)))
    assert not any(126 <= i <= 234 for i in _hits(scan))
    assert len(_hits(scan)) == 251


# -- end to end: driver -> obstacle bearings ---------------------------------
def _bearing_of(raw_degs) -> float:
    scan = RealLidar({})._to_laserscan(_beams(raw_degs))
    obstacles = ObstacleNode(MessageBus(), Config.load())._detect(scan)
    assert len(obstacles) == 1, f"expected one cluster, got {len(obstacles)}"
    return obstacles[0].bearing_deg


def test_obstacle_to_the_right_is_reported_to_the_right():
    """ObstacleNode bearings are positive to the right; +90 means starboard."""
    assert abs(_bearing_of(range(88, 93)) - 90.0) < 3.0


def test_obstacle_to_the_left_is_reported_to_the_left():
    assert abs(_bearing_of(range(268, 273)) + 90.0) < 3.0


def test_obstacle_dead_ahead_has_zero_bearing():
    assert abs(_bearing_of([358, 359, 0, 1, 2])) < 3.0
