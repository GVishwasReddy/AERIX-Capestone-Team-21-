"""Dynamic-obstacle tracking and the reaction built on top of it (2026-08-30).

Covers the four things that were missing when avoidance was extended from
manual flight into autonomous GUIDED delivery:

* obstacles keep an identity across revolutions, so a velocity can be measured
  at all - previously an obstacle's id was its index in a distance-sorted list
  rebuilt every frame;
* our own motion is removed from that velocity, translation *and yaw rate*, so
  a wall we are flying past is not reported as a wall flying at us;
* the brake distance grows with closing speed, so something walking into the
  path keeps the same real margin a stationary wall gets;
* a dodge makes forward progress and is capped in how far off the planned
  track it may push the aircraft.
"""
from __future__ import annotations

import copy
import math

from drone_stack.msg import FusedState, Obstacle, ObstacleArray
from drone_stack.nodes.navigation_node import CollisionAvoider
from drone_stack.nodes.obstacle_tracker import (
    ObstacleTracker,
    body_to_enu,
    enu_to_body,
)
from drone_stack.nodes.proximity_node import DISTANCE_UNKNOWN, SECTOR_COUNT, SectorFilter
from drone_stack.utils.config import Config


# -- helpers -----------------------------------------------------------------
def _cfg(**overrides) -> Config:
    raw = copy.deepcopy(Config.load().raw)
    for name, values in overrides.items():
        raw.setdefault(name, {}).update(values)
    return Config(raw)


def _at(x: float, y: float) -> Obstacle:
    """One obstacle at body-frame (x forward, y left)."""
    return Obstacle(
        distance_m=math.hypot(x, y),
        bearing_deg=math.degrees(math.atan2(-y, x)),   # +bearing = right
        x_m=x,
        y_m=y,
    )


def _ego(vx: float = 0.0, vy: float = 0.0, yaw: float = 0.0) -> FusedState:
    """ENU velocity (vx = East, vy = North) and NED heading in radians."""
    return FusedState(vx=vx, vy=vy, yaw=yaw, valid=True)


# -- frame conversions -------------------------------------------------------
def test_enu_to_body_round_trips():
    for yaw in (0.0, 0.5, math.pi / 2, 2.0, -1.3):
        for east, north in ((1.0, 0.0), (0.0, 1.0), (-2.0, 3.5)):
            x_b, y_b = enu_to_body(east, north, yaw)
            back_e, back_n = body_to_enu(x_b, y_b, yaw)
            assert math.isclose(back_e, east, abs_tol=1e-9)
            assert math.isclose(back_n, north, abs_tol=1e-9)


def test_heading_north_puts_north_velocity_on_the_nose():
    # Facing North (yaw 0): flying North is straight ahead, flying East is to
    # the right - and +y is LEFT, so East must come out negative.
    forward, left = enu_to_body(0.0, 1.0, 0.0)
    assert math.isclose(forward, 1.0, abs_tol=1e-9)
    assert math.isclose(left, 0.0, abs_tol=1e-9)
    forward, left = enu_to_body(1.0, 0.0, 0.0)
    assert math.isclose(forward, 0.0, abs_tol=1e-9)
    assert math.isclose(left, -1.0, abs_tol=1e-9)


# -- tracking ----------------------------------------------------------------
def test_first_frame_publishes_no_velocity():
    """One revolution is a position. Inventing a velocity from it would be a
    guess, and the guess would arrive at the avoider as fact."""
    tracker = ObstacleTracker()
    obstacles = [_at(5.0, 0.0)]
    tracker.update(obstacles, 0.0, _ego())
    assert obstacles[0].track_id > 0
    assert obstacles[0].closing_ms == 0.0
    assert obstacles[0].is_dynamic is False


def test_identity_survives_between_revolutions():
    tracker = ObstacleTracker()
    first = [_at(5.0, 0.0)]
    tracker.update(first, 0.0, _ego())
    second = [_at(4.9, 0.0)]
    tracker.update(second, 0.1, _ego())
    assert second[0].track_id == first[0].track_id
    assert second[0].hits == 2


def test_a_jump_beyond_the_gate_is_a_new_object():
    tracker = ObstacleTracker(gate_m=1.0)
    first = [_at(5.0, 0.0)]
    tracker.update(first, 0.0, _ego())
    far = [_at(5.0, 4.0)]          # 4 m away, well outside the gate
    tracker.update(far, 0.1, _ego())
    assert far[0].track_id != first[0].track_id


def test_wall_ahead_while_we_fly_at_it_closes_but_is_not_dynamic():
    """The case that must not regress: flying at a wall reads as closing (so
    the brake distance grows) without the wall being called a moving object."""
    tracker = ObstacleTracker()
    ego = _ego(vx=0.0, vy=1.0, yaw=0.0)      # 1 m/s due North, facing North
    tracker.update([_at(5.0, 0.0)], 0.0, ego)
    second = [_at(4.9, 0.0)]
    tracker.update(second, 0.1, ego)
    assert math.isclose(second[0].closing_ms, 1.0, abs_tol=0.05)
    assert math.isclose(second[0].speed_m_s, 0.0, abs_tol=0.05)
    assert second[0].is_dynamic is False


def test_object_walking_into_a_hover_is_dynamic():
    """A pedestrian closing on a hovering aircraft. Takes dynamic_min_hits
    consecutive revolutions to be believed - 0.3 s at 10 Hz."""
    tracker = ObstacleTracker(dynamic_min_hits=3)
    ego = _ego()                              # hovering
    latest = None
    for step in range(5):
        latest = [_at(5.0 - 0.1 * step, 0.0)]
        tracker.update(latest, 0.1 * step, ego)
    assert math.isclose(latest[0].closing_ms, 1.0, abs_tol=0.05)
    assert math.isclose(latest[0].speed_m_s, 1.0, abs_tol=0.05)
    assert latest[0].is_dynamic is True


def test_one_frame_of_motion_is_not_enough_to_be_dynamic():
    """Clusters split and merge where returns are sparse, and a split makes the
    centroid jump - which differentiates into a large one-frame velocity. On a
    bench run this reported stationary room clutter as moving at 4.7 m/s."""
    tracker = ObstacleTracker(dynamic_min_hits=3)
    ego = _ego()
    tracker.update([_at(5.0, 0.0)], 0.0, ego)
    jumped = [_at(4.7, 0.0)]                  # one-frame jump, then still
    tracker.update(jumped, 0.1, ego)
    assert jumped[0].is_dynamic is False
    for step in range(2, 5):
        settled = [_at(4.7, 0.0)]
        tracker.update(settled, 0.1 * step, ego)
        assert settled[0].is_dynamic is False


def test_an_implausible_speed_is_discarded_not_braked_for():
    """Beyond max_speed_ms the estimate is two objects matched to one track.
    Acting on it would inflate the brake distance and stop for nothing."""
    tracker = ObstacleTracker(gate_m=5.0, max_speed_ms=6.0, dynamic_min_hits=1)
    ego = _ego()
    tracker.update([_at(8.0, 0.0)], 0.0, ego)
    leapt = [_at(4.0, 0.0)]                   # 4 m in 0.1 s = 40 m/s
    tracker.update(leapt, 0.1, ego)
    assert leapt[0].is_dynamic is False
    assert leapt[0].speed_m_s == 0.0
    assert leapt[0].closing_ms == 0.0         # must not pad the brake distance


def test_yaw_alone_does_not_manufacture_a_moving_obstacle():
    """A stationary post seen through a turn.

    At 1 rad/s an obstacle 5 m out sweeps through the body frame at 5 m/s -
    far faster than the aircraft's own 1 m/s cruise. Without yaw-rate
    compensation every fence post reads as a fast crossing target the moment
    the aircraft turns.
    """
    tracker = ObstacleTracker()
    tracker.update([_at(5.0, 0.0)], 0.0, _ego(yaw=0.0))
    # Yawed +0.1 rad (clockwise/right); a fixed point ahead swings to the LEFT.
    yawed = 0.1
    moved = [_at(5.0 * math.cos(yawed), 5.0 * math.sin(yawed))]
    tracker.update(moved, 0.1, _ego(yaw=yawed))
    assert moved[0].speed_m_s < 0.5
    assert moved[0].is_dynamic is False


def test_without_a_fix_nothing_is_called_dynamic():
    """No FusedState means ego-motion cannot be removed. Reporting the raw
    body-frame velocity as the object's own would label every wall dynamic the
    instant the aircraft moved."""
    tracker = ObstacleTracker()
    tracker.update([_at(5.0, 0.0)], 0.0, None)
    second = [_at(4.9, 0.0)]
    tracker.update(second, 0.1, None)
    assert second[0].is_dynamic is False
    assert second[0].speed_m_s == 0.0
    # Closing speed needs no ego knowledge, so it is still available.
    assert math.isclose(second[0].closing_ms, 1.0, abs_tol=0.05)


def test_a_long_gap_restarts_rather_than_differentiating_across_it():
    tracker = ObstacleTracker(max_dt_s=1.0)
    tracker.update([_at(5.0, 0.0)], 0.0, _ego())
    late = [_at(2.0, 0.0)]
    tracker.update(late, 9.0, _ego())        # node stalled for 9 s
    assert late[0].closing_ms == 0.0         # not 0.33 m/s of fiction


# -- closing-speed-aware braking ---------------------------------------------
def test_stop_distance_grows_with_closing_speed():
    avoider = CollisionAvoider(
        _cfg(navigation={"avoidance_stop_m": 1.7, "avoidance_reaction_s": 1.0,
                         "avoidance_reaction_max_m": 2.0})
    )
    assert math.isclose(avoider.stop_distance_for(0.0), 1.7)
    assert math.isclose(avoider.stop_distance_for(1.5), 3.2)


def test_stop_distance_is_capped():
    avoider = CollisionAvoider(
        _cfg(navigation={"avoidance_stop_m": 1.7, "avoidance_reaction_s": 1.0,
                         "avoidance_reaction_max_m": 2.0})
    )
    assert math.isclose(avoider.stop_distance_for(50.0), 3.7)


def test_receding_never_shrinks_the_configured_standoff():
    """The stand-off is the number the operator set. An object moving away is
    not a reason to let the aircraft closer to it than that."""
    avoider = CollisionAvoider(_cfg(navigation={"avoidance_stop_m": 1.7}))
    assert math.isclose(avoider.stop_distance_for(-5.0), 1.7)


def test_a_fast_approach_brakes_before_a_nearer_static_object():
    """Judging only the nearest obstacle misses the one that matters: a wall
    standing at 3 m while someone walks in at 4 m and 2 m/s."""
    avoider = CollisionAvoider(
        _cfg(navigation={"avoidance_stop_m": 1.7, "avoidance_distance_m": 3.0,
                         "avoidance_reaction_s": 1.0, "avoidance_reaction_max_m": 2.0,
                         "avoidance_front_half_deg": 50.0})
    )
    wall = _at(3.0, 0.0)                     # nearer, but static -> only SLOW
    walker = _at(3.5, 0.0)                   # further, but closing at 2 m/s
    walker.closing_ms = 2.0                  # its stop distance becomes 3.7 m
    decision, nearest = avoider.evaluate(ObstacleArray(obstacles=[wall, walker]))
    assert decision == "stop"
    # The verdict came from the further object; `nearest` still reports the
    # closest range, which is the wall.
    assert math.isclose(nearest, 3.0)
    assert math.isclose(avoider.last_stop_m, 3.7)


def test_a_static_field_still_uses_the_configured_distance():
    avoider = CollisionAvoider(
        _cfg(navigation={"avoidance_stop_m": 1.7, "avoidance_distance_m": 3.0,
                         "avoidance_front_half_deg": 50.0})
    )
    decision, nearest = avoider.evaluate(ObstacleArray(obstacles=[_at(2.5, 0.0)]))
    assert decision == "slow"
    assert math.isclose(nearest, 2.5)
    assert math.isclose(avoider.last_stop_m, 1.7)


# -- asymmetric proximity filter ---------------------------------------------
def _frame(**sectors) -> list[int]:
    out = [DISTANCE_UNKNOWN] * SECTOR_COUNT
    for index, value in sectors.items():
        out[int(index)] = value
    return out


def test_a_single_frame_spike_is_still_rejected():
    """The gate that fixed the unstable first manual flight. One stray close
    reading must not reach the FC, asymmetric filtering or not."""
    filt = SectorFilter(depth=3, fast_approach=True)
    steady = _frame(**{"0": 500})
    filt.update(steady)
    filt.update(steady)
    spike = _frame(**{"0": 80})
    out = filt.update(spike)
    assert out[0] == 500


def test_two_consecutive_close_frames_are_published_at_once():
    """A real approach is corroborated by the second revolution and does not
    wait to win the median."""
    filt = SectorFilter(depth=3, fast_approach=True)
    steady = _frame(**{"0": 500})
    filt.update(steady)
    filt.update(steady)
    filt.update(_frame(**{"0": 300}))        # first close frame - not yet
    out = filt.update(_frame(**{"0": 290}))  # second agrees
    assert out[0] == 300                     # the farther of the pair


def test_the_farther_of_the_corroborating_pair_is_used():
    filt = SectorFilter(depth=3, fast_approach=True)
    steady = _frame(**{"0": 900})
    filt.update(steady)
    filt.update(steady)
    filt.update(_frame(**{"0": 400}))
    out = filt.update(_frame(**{"0": 100}))
    assert out[0] == 400                     # not the 100 cm outlier


def test_receding_is_still_smoothed():
    filt = SectorFilter(depth=3, fast_approach=True)
    close = _frame(**{"0": 200})
    filt.update(close)
    filt.update(close)
    out = filt.update(_frame(**{"0": 800}))  # suddenly far
    assert out[0] == 200                     # median holds it


def test_fast_approach_can_be_switched_off():
    filt = SectorFilter(depth=3, fast_approach=False)
    steady = _frame(**{"0": 500})
    filt.update(steady)
    filt.update(steady)
    filt.update(_frame(**{"0": 300}))
    out = filt.update(_frame(**{"0": 290}))
    assert out[0] == 300                     # pure median of (500, 300, 290)
