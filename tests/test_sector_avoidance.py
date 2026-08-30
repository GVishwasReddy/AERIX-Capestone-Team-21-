"""Reactive sector avoidance, ported from Drone-Autonomy-ROS2.

Pins the three things the port had to change to be safe on this airframe:

* the side cones cover the whole 270 deg the LiDAR actually scans, not the
  source node's 90 deg, and never more than lidar.fov_deg allows;
* the masked rear 90 deg is never read as clearance, so a trapped aircraft
  holds instead of reversing into the clutter the mask exists to ignore;
* dodge velocity signs are MAV_FRAME_BODY_NED (+y = right), not the source's
  body-ENU Twist (+y = left).
"""
from __future__ import annotations

import copy
import math

from drone_stack.bus import MessageBus
from drone_stack.bus.topics import Topics
from drone_stack.interfaces.lidar_interface import RealLidar
from drone_stack.msg import Obstacle, ObstacleArray
from drone_stack.nodes.navigation_node import (
    DODGE_LEFT,
    DODGE_RIGHT,
    DODGE_TRAPPED,
    CollisionAvoider,
    NavigationNode,
)
from drone_stack.nodes.obstacle_node import ObstacleNode
from drone_stack.srv import ServiceRegistry
from drone_stack.utils.config import Config


# -- helpers -----------------------------------------------------------------
def _cfg(**overrides) -> Config:
    raw = copy.deepcopy(Config.load().raw)
    for name, values in overrides.items():
        raw.setdefault(name, {}).update(values)
    return Config(raw)


def _avoider(**overrides) -> CollisionAvoider:
    return CollisionAvoider(_cfg(**overrides))


def _obs(*pairs: tuple[float, float]) -> ObstacleArray:
    """ObstacleArray from (distance_m, bearing_deg) pairs. + bearing = right."""
    return ObstacleArray(
        obstacles=[Obstacle(distance_m=d, bearing_deg=b) for d, b in pairs]
    )


def _nav(**overrides) -> tuple[NavigationNode, list]:
    """A navigation node plus the list of commands it sends to the FC."""
    bus = MessageBus()
    sent: list = []
    bus.subscribe(Topics.MAVLINK_CMD, sent.append)
    node = NavigationNode(bus, _cfg(**overrides), ServiceRegistry())
    node._avoid_enabled = True
    sent.clear()  # drop anything emitted during construction
    return node, sent


# -- sector geometry ---------------------------------------------------------
def test_sectors_split_front_left_right_by_bearing_sign():
    av = _avoider()
    view = av.sectors(_obs((3.0, 0.0), (4.0, -90.0), (5.0, 90.0)))
    assert view.front == 3.0
    assert view.left == 4.0     # negative bearing is left of the nose
    assert view.right == 5.0


def test_sectors_keep_the_nearest_return_per_cone():
    av = _avoider()
    view = av.sectors(_obs((9.0, 100.0), (2.0, 120.0), (7.0, 80.0)))
    assert view.right == 2.0
    assert not math.isfinite(view.left)
    assert not math.isfinite(view.front)


def test_empty_field_is_all_clear():
    view = _avoider().sectors(None)
    assert not math.isfinite(view.front)
    assert not math.isfinite(view.left)
    assert not math.isfinite(view.right)


def test_side_cones_reach_125_not_the_source_nodes_90():
    """The port widens the sides to the edge of the 250 deg window."""
    av = _avoider()
    assert av.side_half_deg == 125.0
    view = av.sectors(_obs((3.0, 120.0)))
    assert view.right == 3.0    # a 90 deg cone would have discarded this
    # ...but nothing past the window edge is admitted, however it is configured.
    assert av.sectors(_obs((3.0, 130.0))).right == math.inf


def test_side_cones_clamped_to_the_scanned_window():
    """Configuring wider cones than the LiDAR scans must not invent clearance."""
    av = _avoider(navigation={"avoidance_side_half_deg": 175.0})
    assert av.side_clamped
    assert av.side_half_deg == 125.0
    assert av.sectors(_obs((1.0, 170.0))).right == math.inf


def test_a_full_circle_lidar_leaves_the_cones_alone():
    av = _avoider(
        lidar={"fov_enabled": False},
        navigation={"avoidance_side_half_deg": 175.0},
    )
    assert av.fov_half_deg == 180.0
    assert av.rear_blind is False
    assert av.side_half_deg == 175.0


def test_rear_is_marked_blind_under_the_270_window():
    assert _avoider().rear_blind is True


def test_front_cone_never_exceeds_the_side_cone():
    av = _avoider(navigation={"avoidance_front_half_deg": 200.0})
    assert av.sector_deg == av.side_half_deg == 125.0


# -- dodge decision ----------------------------------------------------------
def test_dodge_picks_the_side_with_more_room():
    av = _avoider()
    assert av.dodge(av.sectors(_obs((1.0, 0.0), (8.0, -90.0), (4.0, 90.0)))) == DODGE_LEFT
    assert av.dodge(av.sectors(_obs((1.0, 0.0), (4.0, -90.0), (8.0, 90.0)))) == DODGE_RIGHT


def test_dodge_refuses_a_side_that_is_not_actually_open():
    """The source picks the larger of two blocked sides; this one does not."""
    av = _avoider()
    # Left (1.0 m) beats right (0.6 m) but is well inside dodge_clear_m.
    view = av.sectors(_obs((1.0, 0.0), (1.0, -90.0), (0.6, 90.0)))
    assert av.dodge(view) == DODGE_TRAPPED


def test_dodge_takes_the_only_open_side():
    av = _avoider()
    assert av.dodge(av.sectors(_obs((1.0, 0.0), (0.5, -90.0), (9.0, 90.0)))) == DODGE_RIGHT
    assert av.dodge(av.sectors(_obs((1.0, 0.0), (9.0, -90.0), (0.5, 90.0)))) == DODGE_LEFT


def test_dodge_is_clear_when_nothing_is_beside_us():
    av = _avoider()
    assert av.dodge(av.sectors(_obs((1.0, 0.0)))) in (DODGE_LEFT, DODGE_RIGHT)


# -- the masked rear ---------------------------------------------------------
def test_masked_rear_returns_never_reach_the_sectors():
    """End to end: a wall behind the aircraft must not produce a rear sector."""
    bus, config = MessageBus(), _cfg()
    node = ObstacleNode(bus, config)
    lidar = RealLidar(config.section("lidar"))
    # A solid return every degree at 1 m, all the way round.
    scan = lidar._to_laserscan([(47.0, float(a), 1000.0) for a in range(360)])
    obstacles = node._detect(scan)
    assert obstacles, "the front 250 deg should still detect the wall"
    assert all(abs(o.bearing_deg) <= 125.0 + 1.0 for o in obstacles)

    view = CollisionAvoider(config).sectors(
        ObstacleArray(obstacles=obstacles)
    )
    # Front and both sides see the wall; there is no fourth reading to consult.
    assert math.isfinite(view.front)
    assert view.rear_blind is True


# -- commanded behaviour -----------------------------------------------------
def _commands(sent: list) -> list[str]:
    return [c.command for c in sent]


def test_dodge_right_commands_positive_vy_in_body_ned():
    node, sent = _nav()
    node._obstacles = _obs((1.0, 0.0), (1.0, -90.0), (9.0, 90.0))
    node._do_avoid()
    vel = [c for c in sent if c.command == "velocity"]
    assert len(vel) == 1
    assert vel[0].params["vy"] > 0     # +y is right in MAV_FRAME_BODY_NED
    assert vel[0].params["vx"] == 0.0
    assert node._dodge_dir == DODGE_RIGHT


def test_dodge_left_commands_negative_vy():
    node, sent = _nav()
    node._obstacles = _obs((1.0, 0.0), (9.0, -90.0), (1.0, 90.0))
    node._do_avoid()
    vel = [c for c in sent if c.command == "velocity"]
    assert vel[0].params["vy"] < 0
    assert node._dodge_dir == DODGE_LEFT


def test_trapped_holds_and_never_reverses():
    """The rear is unmeasured, so backing out is not an escape route."""
    node, sent = _nav()
    node._obstacles = _obs((1.0, 0.0), (0.5, -90.0), (0.5, 90.0))
    for _ in range(5):
        node._do_avoid()
    assert _commands(sent) == ["brake"]          # deduped mode-only command
    assert not [c for c in sent if c.command == "velocity"]
    assert node._dodge_dir is None
    assert "trapped" in node._status_message


def test_dodge_direction_is_latched_across_ticks():
    """Scan jitter must not flip between GUIDED velocity and BRAKE at 10 Hz."""
    node, sent = _nav()
    node._obstacles = _obs((1.0, 0.0), (9.0, -90.0), (8.9, 90.0))
    node._do_avoid()
    assert node._dodge_dir == DODGE_LEFT
    # The right side is now marginally roomier - the latch must ignore that.
    node._obstacles = _obs((1.0, 0.0), (8.9, -90.0), (9.0, 90.0))
    node._do_avoid()
    assert node._dodge_dir == DODGE_LEFT
    assert all(c.params["vy"] < 0 for c in sent if c.command == "velocity")


def test_dodge_stops_when_the_chosen_side_closes_in():
    node, sent = _nav()
    node._obstacles = _obs((1.0, 0.0), (9.0, -90.0), (1.0, 90.0))
    node._do_avoid()
    assert node._dodge_dir == DODGE_LEFT
    # Something appears in the side we committed to, inside the stop distance.
    node._obstacles = _obs((1.0, 0.0), (0.8, -90.0), (1.0, 90.0))
    node._do_avoid()
    assert node._dodge_dir is None
    assert sent[-1].command == "brake"
    assert "blocked" in node._status_message


def test_dodge_times_out_into_a_hold():
    node, sent = _nav(navigation={"avoidance_dodge_timeout_s": 0.0})
    node._obstacles = _obs((1.0, 0.0), (9.0, -90.0), (1.0, 90.0))
    node._do_avoid()                      # latches and sidesteps
    assert node._dodge_dir == DODGE_LEFT
    node._do_avoid()                      # timeout is measured from the latch
    assert sent[-1].command == "brake"
    assert node._dodge_dir is None
    assert "timed out" in node._status_message


def test_dodging_off_falls_back_to_braking():
    node, sent = _nav(navigation={"avoidance_dodge_enabled": False})
    node._obstacles = _obs((1.0, 0.0), (9.0, -90.0), (1.0, 90.0))
    node._do_avoid()
    assert _commands(sent) == ["brake"]
    assert node._dodge_dir is None


def test_clear_front_releases_the_dodge_and_resumes():
    from drone_stack.msg import MissionPhase

    node, _ = _nav()
    node._obstacles = _obs((1.0, 0.0), (9.0, -90.0), (1.0, 90.0))
    node._do_avoid()
    assert node._dodge_dir == DODGE_LEFT
    node._obstacles = _obs((9.0, 0.0))
    node._do_avoid()
    assert node._dodge_dir is None
    assert node._phase == MissionPhase.NAVIGATE


def test_disabled_avoidance_never_dodges():
    node, sent = _nav()
    node._avoid_enabled = False
    node._obstacles = _obs((0.3, 0.0), (9.0, -90.0), (1.0, 90.0))
    node._do_avoid()
    assert not [c for c in sent if c.command == "velocity"]


# -- GCS wire ----------------------------------------------------------------
def test_avoidance_status_carries_the_sector_picture():
    node, _ = _nav()
    node._obstacles = _obs((1.0, 0.0), (4.0, -90.0), (0.5, 90.0))
    node._publish_avoidance()
    status = node.bus.latest(Topics.AVOIDANCE)
    assert status.front_m == 1.0
    assert status.left_m == 4.0
    assert status.right_m == 0.5
    assert status.dodge == DODGE_LEFT
    assert status.rear_blind is True
    assert status.side_half_deg == 125.0
    assert status.front_half_deg == 50.0


def test_absent_sector_reports_zero_not_infinity():
    """inf is not JSON-representable; 0.0 is the existing "nothing" convention."""
    node, _ = _nav()
    node._obstacles = _obs((1.0, 0.0))
    node._publish_avoidance()
    status = node.bus.latest(Topics.AVOIDANCE)
    assert status.left_m == 0.0 and status.right_m == 0.0
    assert status.front_m == 1.0
