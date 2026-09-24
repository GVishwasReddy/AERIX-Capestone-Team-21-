"""VFH+ steering, the avoidance approach taken from Drone-Autonomy-ROS2.

The three-cone dodge answers "left or right?". These pin the thing that
replaces it: a polar histogram over the scanned window, obstacles widened by
the room the airframe needs, and a course set to the best surviving GAP.

The safety property that matters most is the last group: the histogram spans
only ``lidar.fov_deg``, so the masked rear has no bins at all and no gap
search can steer into the 110 deg the LiDAR cannot see.
"""
from __future__ import annotations

import copy
import math

from drone_stack.msg import Obstacle, ObstacleArray
from drone_stack.nodes.navigation_node import CollisionAvoider
from drone_stack.utils.config import Config


#: These files assert angles computed from a STATED safety radius - the
#: docstrings say things like "a 1 m safety radius blocks asin(1/2) = 30 deg" -
#: so the radius has to be pinned here rather than inherited from whichever
#: profile Config.load() resolves. It was inherited until 2026-09-19, and
#: raising the shipped radius (1.0 -> 1.6 in default.yaml, 2.5 in real.yaml, so
#: that VFH+ stops steering to miss by less than the brake distance) silently
#: closed the very corridors these tests exist to prove are flyable.
_VFH_GEOMETRY = {"avoidance_vfh_safety_radius_m": 1.0}


def _cfg(**overrides) -> Config:
    raw = copy.deepcopy(Config.load().raw)
    raw.setdefault("navigation", {}).update(_VFH_GEOMETRY)
    for name, values in overrides.items():
        raw.setdefault(name, {}).update(values)
    return Config(raw)


def _avoider(**overrides) -> CollisionAvoider:
    return CollisionAvoider(_cfg(**overrides))


def _obs(*triples) -> ObstacleArray:
    """(distance_m, bearing_deg[, angular_width_deg]). + bearing = right."""
    out = []
    for t in triples:
        d, b = t[0], t[1]
        w = t[2] if len(t) > 2 else 0.0
        out.append(Obstacle(distance_m=d, bearing_deg=b, angular_width_deg=w))
    return ObstacleArray(obstacles=out)


# -- histogram ---------------------------------------------------------------
def test_histogram_covers_only_the_scanned_window():
    av = _avoider()
    centres = av._bin_centres()
    assert min(centres) > -av.fov_half_deg
    assert max(centres) < av.fov_half_deg
    # 250 deg of window at 5 deg per bin.
    assert len(centres) == 50


def test_a_closer_obstacle_blocks_a_wider_arc():
    """The whole point of angular enlargement."""
    av = _avoider()
    far = sum(av.build_histogram(_obs((8.0, 0.0))))
    near = sum(av.build_histogram(_obs((2.0, 0.0))))
    assert near > far


def test_an_obstacle_inside_the_safety_radius_saturates():
    av = _avoider()
    # asin(1.0) = 90 deg, so it blocks +/-90 deg around itself.
    blocked = av.build_histogram(_obs((0.5, 0.0)))
    centres = av._bin_centres()
    for centre, is_blocked in zip(centres, blocked):
        if abs(centre) <= 85.0:
            assert is_blocked, f"{centre:.0f} deg should be blocked"


def test_distant_obstacles_do_not_steer():
    av = _avoider()
    assert not any(av.build_histogram(_obs((av.vfh_range_m + 5.0, 0.0))))


def test_empty_field_blocks_nothing():
    assert not any(_avoider().build_histogram(_obs()))


# -- gap selection -----------------------------------------------------------
def test_steers_through_a_gap_a_three_cone_dodge_would_miss():
    """Two obstacles either side of a corridor dead ahead.

    front/left/right sees "front blocked" and commits to a whole side. VFH+
    finds the gap between them and flies it.
    """
    av = _avoider()
    heading = av.choose_heading(_obs((3.0, -35.0), (3.0, 35.0)), goal_bearing_deg=0.0)
    assert heading is not None
    assert abs(heading) < 20.0, f"expected the middle corridor, got {heading}"


def test_goal_attraction_picks_the_gap_nearest_the_waypoint():
    av = _avoider()
    field = _obs((3.0, 0.0))          # blocked straight ahead
    left = av.choose_heading(field, goal_bearing_deg=-90.0)
    right = av.choose_heading(field, goal_bearing_deg=90.0)
    assert left is not None and right is not None
    assert left < 0.0, "goal to the left should steer left"
    assert right > 0.0, "goal to the right should steer right"


def test_a_gap_narrower_than_min_valley_is_not_flyable():
    """Same obstacle field, two different airframe widths.

    Obstacles at 2 m with a 1 m safety radius each block asin(1/2) = 30 deg
    either side, so the corridor between bearings -50 and +50 is exactly
    40 deg wide. A drone that needs 18 deg flies it; one that needs 60 deg
    must refuse it and report trapped rather than thread a gap it does not
    fit through.
    """
    field = _obs((2.0, -50.0), (2.0, 50.0))

    fits = _avoider(navigation={"avoidance_vfh_min_valley_deg": 18.0})
    heading = fits.choose_heading(field, goal_bearing_deg=0.0)
    assert heading is not None and abs(heading) < 5.0

    # 60 deg needed: the 40 deg corridor and both 45 deg outer margins all
    # fail, so there is no flyable gap anywhere in the window.
    too_wide = _avoider(navigation={"avoidance_vfh_min_valley_deg": 60.0})
    assert too_wide.choose_heading(field, goal_bearing_deg=0.0) is None


def test_hysteresis_holds_the_course_between_equal_gaps():
    av = _avoider()
    field = _obs((3.0, 0.0))       # symmetric: left and right equally good
    assert av.choose_heading(field, 0.0, previous_deg=-60.0) < 0.0
    assert av.choose_heading(field, 0.0, previous_deg=60.0) > 0.0


def test_trapped_returns_none_rather_than_a_guess():
    av = _avoider()
    ring = _obs(*[(1.5, b) for b in range(-120, 121, 10)])
    assert av.choose_heading(ring, 0.0) is None


# -- the rear is unmeasured, not clear ---------------------------------------
def test_no_chosen_heading_ever_leaves_the_scanned_window():
    av = _avoider()
    assert av.rear_blind
    for goal in (-180.0, -150.0, 0.0, 150.0, 180.0):
        heading = av.choose_heading(_obs((2.0, 0.0)), goal_bearing_deg=goal)
        if heading is not None:
            assert abs(heading) <= av.fov_half_deg, (
                f"steered to {heading} deg, outside the {av.fov_half_deg} deg window")


def test_a_goal_behind_us_never_becomes_a_reverse():
    """The source node reverses out of a dead end. This airframe must not."""
    av = _avoider()
    heading = av.choose_heading(_obs((2.0, 0.0)), goal_bearing_deg=180.0)
    assert heading is None or abs(heading) <= av.fov_half_deg


def test_valleys_do_not_wrap_across_the_masked_rear():
    """+125 and -125 are the mask edges; joining them would invent a gap."""
    av = _avoider()
    blocked = [False] * len(av._bin_centres())
    spans = av.valleys(blocked)
    assert len(spans) == 1
    lo, hi = spans[0]
    assert lo >= -av.fov_half_deg - 1e-6 and hi <= av.fov_half_deg + 1e-6


# -- slew limiting -----------------------------------------------------------
def test_course_slew_is_rate_limited():
    av = _avoider()
    # 45 deg/s over a 0.1 s tick = 4.5 deg of movement, not 90.
    assert av.slew_heading(90.0, previous_deg=0.0, dt_s=0.1) == 4.5


def test_first_command_is_not_slewed():
    av = _avoider()
    assert av.slew_heading(90.0, previous_deg=None, dt_s=0.1) == 90.0
