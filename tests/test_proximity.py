"""OBSTACLE_DISTANCE feed to the flight controller.

This is the avoidance path that keeps working when the pilot is flying, so the
two things it must never get wrong are pinned here: the handedness of the
sector array (a mirrored picture makes the FC avoid the wrong way) and the
250 deg window (the rear 110 deg must reach the FC as "unmeasured", never as a
distance it can act on) - plus the two noise gates added after the first manual
flight, which are the difference between a steady hover and an oscillation.
"""
from __future__ import annotations

import copy
import math

from drone_stack.bus import MessageBus
from drone_stack.bus.topics import Topics
from drone_stack.interfaces.lidar_interface import FovMask, RealLidar
from drone_stack.msg import LaserScan, NavCommand
from drone_stack.nodes.proximity_node import (
    DISTANCE_UNKNOWN,
    SECTOR_COUNT,
    ProximityNode,
    SectorFilter,
    scan_to_sectors,
    sector_keep_mask,
)
from drone_stack.utils.config import Config


def _cfg(**overrides) -> Config:
    raw = copy.deepcopy(Config.load().raw)
    for name, values in overrides.items():
        raw.setdefault(name, {}).update(values)
    return Config(raw)


def _scan(beams: dict[int, float], bins: int = 360) -> LaserScan:
    """LaserScan with `beams` mapping counter-clockwise degrees -> metres."""
    ranges = [math.inf] * bins
    for deg, metres in beams.items():
        ranges[deg % bins] = metres
    return LaserScan(
        ranges=ranges,
        angle_min=0.0,
        angle_increment=math.radians(360.0 / bins),
    )


# -- the 270 degree window ---------------------------------------------------
def test_mask_keeps_54_sectors_and_blanks_the_rear_18():
    keep = sector_keep_mask(FovMask({"fov_deg": 270.0}))
    assert len(keep) == SECTOR_COUNT
    assert sum(keep) == 54                      # 54 * 5 deg = 270 deg
    assert sum(1 for k in keep if not k) == 18  # 18 * 5 deg = 90 deg
    masked = [i for i, k in enumerate(keep) if not k]
    assert masked == list(range(27, 45))        # the wedge behind the nose


def test_mask_is_symmetric_about_the_nose():
    keep = sector_keep_mask(FovMask({"fov_deg": 270.0}))
    # Sector i spans [5i, 5i+5), so its reflection about the nose is sector
    # 71 - i. They must always agree, or the window has drifted off the tail.
    for i in range(SECTOR_COUNT):
        assert keep[i] == keep[SECTOR_COUNT - 1 - i]


def test_full_circle_config_masks_nothing():
    assert sum(sector_keep_mask(FovMask({"fov_enabled": False}))) == SECTOR_COUNT


def test_rear_beams_cannot_reach_the_flight_controller():
    """Even a scan that somehow carries rear returns must not publish them."""
    keep = sector_keep_mask(FovMask({"fov_deg": 270.0}))
    # 180 deg CCW is dead astern; +/-150 deg is inside the masked wedge.
    rear = _scan({180: 0.5, 150: 0.5, 210: 0.5})
    sectors = scan_to_sectors(rear, 15, 1200, keep)
    assert sectors == [DISTANCE_UNKNOWN] * SECTOR_COUNT


# -- handedness --------------------------------------------------------------
def test_nose_beam_lands_in_sector_zero():
    assert scan_to_sectors(_scan({0: 2.0}), 15, 1200)[0] == 200


def test_left_beam_goes_clockwise_to_sector_54():
    """Scan is counter-clockwise; OBSTACLE_DISTANCE runs clockwise from the nose."""
    sectors = scan_to_sectors(_scan({90: 3.0}), 15, 1200)
    assert sectors[54] == 300                  # 270 deg clockwise == 90 deg left
    assert sectors[18] == DISTANCE_UNKNOWN


def test_right_beam_goes_clockwise_to_sector_18():
    sectors = scan_to_sectors(_scan({270: 3.0}), 15, 1200)
    assert sectors[18] == 300                  # 90 deg clockwise == to the right
    assert sectors[54] == DISTANCE_UNKNOWN


def test_forward_right_beam_stays_forward_and_right():
    sectors = scan_to_sectors(_scan({315: 4.0}), 15, 1200)
    assert sectors[9] == 400                   # 45 deg clockwise of the nose


# -- bucketing ---------------------------------------------------------------
def test_closest_beam_wins_within_a_sector():
    # Clockwise sector 0 spans 0-5 deg CW, which is 0 and 355-360 deg CCW.
    sectors = scan_to_sectors(_scan({0: 5.0, 359: 2.0, 358: 8.0}), 15, 1200)
    assert sectors[0] == 200


def test_out_of_range_beams_are_dropped():
    sectors = scan_to_sectors(_scan({0: 0.05, 90: 40.0}), 15, 1200)
    assert sectors[0] == DISTANCE_UNKNOWN
    assert sectors[54] == DISTANCE_UNKNOWN


def test_empty_scan_is_all_unknown():
    assert scan_to_sectors(_scan({}), 15, 1200) == [DISTANCE_UNKNOWN] * SECTOR_COUNT


# -- end to end --------------------------------------------------------------
def test_driver_mask_carries_through_to_the_sector_array():
    """A wall all the way round reaches the FC as 250 deg of wall, no more."""
    config = _cfg()
    lidar = RealLidar(config.section("lidar"))
    scan = lidar._to_laserscan([(47.0, float(a), 2000.0) for a in range(360)])
    sectors = scan_to_sectors(
        scan, 15, 1200, sector_keep_mask(FovMask(config.section("lidar")))
    )
    live = [i for i, d in enumerate(sectors) if d != DISTANCE_UNKNOWN]
    assert len(live) == 50
    assert all(not (25 <= i < 47) for i in live)
    assert all(sectors[i] == 200 for i in live)


def test_node_publishes_a_72_sector_command():
    bus = MessageBus()
    sent: list = []
    bus.subscribe(Topics.MAVLINK_CMD, sent.append)
    node = ProximityNode(bus, _cfg())
    # Two beams in clockwise sector 0 (CCW 0 and CCW 359), so the default
    # min_points=2 gate is satisfied.
    node._on_scan(_scan({0: 2.0, 359: 2.5}))
    node.step()

    assert len(sent) == 1
    cmd = sent[0]
    assert isinstance(cmd, NavCommand) and cmd.command == "obstacle_distance"
    assert len(cmd.params["distances"]) == SECTOR_COUNT
    assert cmd.params["frame"] == 12          # MAV_FRAME_BODY_FRD
    assert cmd.params["increment_deg"] == 5.0
    # The 2nd closest sets the sector, not the closest.
    assert cmd.params["distances"][0] == 250


def test_node_sends_nothing_without_a_fresh_scan():
    bus = MessageBus()
    sent: list = []
    bus.subscribe(Topics.MAVLINK_CMD, sent.append)
    node = ProximityNode(bus, _cfg())
    node.step()
    node._on_scan(_scan({0: 2.0}))
    node.step()
    node.step()                                # same revolution, already sent
    assert len(sent) == 1


# -- noise gates -------------------------------------------------------------
# The first manual flight oscillated: a single stray return set a whole sector
# and the aircraft lurched at it. These pin the two filters that fixed it.
def test_a_lone_beam_cannot_set_a_sector():
    sectors = scan_to_sectors(_scan({0: 1.0}), 15, 1200, min_points=2)
    assert sectors[0] == DISTANCE_UNKNOWN


def test_two_beams_do_set_a_sector():
    sectors = scan_to_sectors(_scan({0: 1.0, 359: 1.2}), 15, 1200, min_points=2)
    assert sectors[0] == 120                   # the 2nd closest, not the 1st


def test_min_points_takes_the_nth_closest_not_the_minimum():
    """A stray close return among real ones must not drag the sector in."""
    beams = {0: 0.5, 359: 3.0, 358: 3.1, 357: 3.2}   # 0.5 m is the outlier
    assert scan_to_sectors(_scan(beams), 15, 1200, min_points=2)[0] == 300


def test_min_points_one_is_the_old_unfiltered_behaviour():
    assert scan_to_sectors(_scan({0: 1.0}), 15, 1200, min_points=1)[0] == 100


def test_near_field_returns_are_gated_out():
    """Below min_distance is the airframe and prop wash, not obstacles."""
    sectors = scan_to_sectors(_scan({0: 0.2, 359: 0.2}), 30, 1200, min_points=2)
    assert sectors[0] == DISTANCE_UNKNOWN


def test_median_filter_discards_a_one_frame_spike():
    f = SectorFilter(depth=3)
    clear = [DISTANCE_UNKNOWN] * SECTOR_COUNT
    spike = list(clear)
    spike[0] = 100                             # one frame of "wall at 1 m"
    f.update(clear)
    f.update(clear)
    assert f.update(spike)[0] == DISTANCE_UNKNOWN


def test_median_filter_reports_an_obstacle_seen_twice():
    f = SectorFilter(depth=3)
    clear = [DISTANCE_UNKNOWN] * SECTOR_COUNT
    wall = list(clear)
    wall[0] = 100
    f.update(clear)
    f.update(wall)
    assert f.update(wall)[0] == 100            # 2 of 3 frames -> believed


def test_median_filter_releases_when_the_obstacle_goes_away():
    f = SectorFilter(depth=3)
    wall = [DISTANCE_UNKNOWN] * SECTOR_COUNT
    wall[0] = 100
    clear = [DISTANCE_UNKNOWN] * SECTOR_COUNT
    for _ in range(3):
        f.update(wall)
    f.update(clear)
    assert f.update(clear)[0] == DISTANCE_UNKNOWN


def test_median_filter_passes_through_while_warming_up():
    """The FC must be fed from the first revolution, not after depth frames."""
    f = SectorFilter(depth=3)
    first = [DISTANCE_UNKNOWN] * SECTOR_COUNT
    first[0] = 250
    assert f.update(first)[0] == 250


def test_filter_depth_one_is_a_passthrough():
    f = SectorFilter(depth=1)
    frame = [DISTANCE_UNKNOWN] * SECTOR_COUNT
    frame[0] = 100
    assert f.update(frame)[0] == 100
    assert f.update(frame)[0] == 100


def test_node_rides_out_a_one_frame_spike_end_to_end():
    bus = MessageBus()
    sent: list = []
    bus.subscribe(Topics.MAVLINK_CMD, sent.append)
    node = ProximityNode(bus, _cfg())

    steady = {0: 5.0, 359: 5.0, 358: 5.0}
    for _ in range(3):
        node._on_scan(_scan(steady))
        node.step()
    # One revolution where a pair of beams reads a phantom wall at 0.8 m.
    spike = {**steady, 0: 0.8, 359: 0.8}
    node._on_scan(_scan(spike))
    node.step()

    assert sent[-1].params["distances"][0] == 500   # unmoved by the spike


def test_node_issues_no_flight_commands():
    """It is a sensor feed. It must never move the aircraft or change mode."""
    bus = MessageBus()
    sent: list = []
    bus.subscribe(Topics.MAVLINK_CMD, sent.append)
    node = ProximityNode(bus, _cfg())
    for _ in range(5):
        node._on_scan(_scan({0: 0.2}))         # something very close
        node.step()
    assert {c.command for c in sent} == {"obstacle_distance"}
