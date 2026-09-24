"""Long-range, smooth avoidance (2026-09-23).

Pins the four changes made after "it starts avoiding far too late and almost
hits the obstacle":

* ObstacleNode's size gate is range-aware, so a thin pole 4-10 m out is no
  longer thrown away for being 1-2 bins wide - but a sparse cluster must
  persist for ``sparse_min_hits`` revolutions before it is published;
* each LiDAR revolution is processed once, however fast the node polls;
* a confirmed obstacle that drops out for a revolution keeps being published
  (coasted), so the avoider does not flip SLOW/CLEAR on sensor dropout;
* with no flyable gap the cruise band decelerates to the brake distance
  instead of arriving at cruise speed and slamming into BRAKE;
* avoidance is held in STANDBY until the aircraft is airborne, then latched.
"""
from __future__ import annotations

import copy
import math
import time
from types import SimpleNamespace

from drone_stack.bus import MessageBus
from drone_stack.bus.topics import Topics
from drone_stack.msg import FusedState, LaserScan, Obstacle, ObstacleArray
from drone_stack.msg.messages import MissionPhase
from drone_stack.nodes.navigation_node import CLEAR, STOP, NavigationNode
from drone_stack.nodes.obstacle_node import ObstacleNode
from drone_stack.srv import ServiceRegistry
from drone_stack.utils.config import Config


def _cfg(**overrides) -> Config:
    raw = copy.deepcopy(Config.load().raw)
    for name, values in overrides.items():
        raw.setdefault(name, {}).update(values)
    return Config(raw)


def _scan(returns: dict[int, float], bins: int = 360) -> LaserScan:
    ranges = [math.inf] * bins
    for idx, rng in returns.items():
        ranges[idx % bins] = rng
    return LaserScan(
        angle_min=0.0, angle_max=2.0 * math.pi,
        angle_increment=2.0 * math.pi / bins, range_min=0.15, range_max=12.0,
        ranges=ranges, intensities=[0.0] * bins,
    )


def _obstacle_node(**obstacles) -> tuple[ObstacleNode, list]:
    bus = MessageBus()
    out: list = []
    bus.subscribe(Topics.OBSTACLES, out.append)
    node = ObstacleNode(bus, _cfg(obstacles=obstacles) if obstacles else _cfg())
    return node, out


def _feed(node: ObstacleNode, scan: LaserScan) -> None:
    node._on_scan(scan)
    node.step()
    time.sleep(0.002)          # the tracker differentiates over real dt


# -- range-aware size gate ---------------------------------------------------
def test_points_required_shrink_with_range_but_never_below_one():
    node, _ = _obstacle_node()
    inc = math.radians(1.0)
    assert node._required_points(1.0, inc) == 3     # close in: old rule
    assert node._required_points(2.0, inc) == 2     # 0.1 m fills 2.9 deg
    assert node._required_points(4.0, inc) == 1
    assert node._required_points(11.0, inc) == 1


def test_a_thin_pole_far_out_is_now_a_candidate():
    node, _ = _obstacle_node()
    found = node._detect(_scan({0: 6.0, 1: 6.0}))
    assert len(found) == 1 and abs(found[0].distance_m - 6.0) < 0.01


def test_a_single_return_close_in_is_still_rejected():
    node, _ = _obstacle_node()
    assert node._detect(_scan({0: 1.0})) == []


# -- sparse persistence ------------------------------------------------------
def test_a_sparse_cluster_publishes_only_after_it_persists():
    node, out = _obstacle_node()
    hits = node._sparse_min_hits
    for _ in range(hits - 1):
        _feed(node, _scan({0: 6.0, 1: 6.0}))
        assert out[-1].count == 0, "published before it had persisted"
    _feed(node, _scan({0: 6.0, 1: 6.0}))
    assert out[-1].count == 1


def test_a_dense_cluster_still_publishes_on_first_sight():
    node, out = _obstacle_node()
    _feed(node, _scan({358: 3.0, 359: 3.0, 0: 3.0, 1: 3.0, 2: 3.0}))
    assert out[-1].count == 1


def test_one_revolution_polled_many_times_is_one_sighting():
    """The node polls faster than the LiDAR spins. Re-processing the same
    revolution must not count as fresh hits, or one speckle would satisfy
    sparse_min_hits on its own."""
    node, out = _obstacle_node()
    scan = _scan({0: 6.0})
    for _ in range(10):
        _feed(node, scan)
    assert len(out) == 1, "the same revolution was processed more than once"
    assert out[-1].count == 0


# -- coasting ----------------------------------------------------------------
def _confirm_pole(node: ObstacleNode) -> None:
    for _ in range(node._sparse_min_hits):
        _feed(node, _scan({0: 6.0, 1: 6.0}))


def test_a_confirmed_obstacle_survives_a_dropped_revolution():
    node, out = _obstacle_node()
    _confirm_pole(node)
    assert out[-1].count == 1
    _feed(node, _scan({}))                 # the C1 missed it this revolution
    assert out[-1].count == 1, "dropout made the obstacle vanish"
    assert abs(out[-1].obstacles[0].distance_m - 6.0) < 0.2


def test_coasting_expires():
    node, out = _obstacle_node(coast_s=0.0)
    _confirm_pole(node)
    _feed(node, _scan({}))
    assert out[-1].count == 0


def test_nothing_is_coasted_that_was_never_confirmed():
    node, out = _obstacle_node()
    _feed(node, _scan({0: 6.0}))          # one sighting, never published
    _feed(node, _scan({}))
    assert out[-1].count == 0


# -- dead ahead is steered round, not braked for ----------------------------
def test_an_obstacle_dead_ahead_leaves_a_gap_to_steer_through():
    """End to end, detector into VFH+: a trunk on the nose at 10 m must leave
    a flyable heading. Before the seam fix choose_heading returned None here
    and every obstacle straight ahead ended in the brake."""
    node, _ = _obstacle_node()
    found = node._detect(_scan({359: 10.0, 0: 10.0, 1: 10.0}))
    nav, _ = _nav()
    heading = nav._avoider.choose_heading(ObstacleArray(obstacles=found), 0.0, None)
    assert heading is not None, "no gap found round a single trunk"
    assert abs(heading) >= 10.0


# -- trapped approach decelerates -------------------------------------------
def _nav(**navigation) -> tuple[NavigationNode, list]:
    bus = MessageBus()
    sent: list = []
    bus.subscribe(Topics.MAVLINK_CMD, sent.append)
    node = NavigationNode(bus, _cfg(navigation=navigation), ServiceRegistry())
    node._avoid_enabled = True
    sent.clear()
    return node, sent


def _trapped_speed(node, sent, distance) -> float:
    node._avoider.choose_heading = lambda *a, **k: None
    node._goal_bearing_deg = lambda: 0.0
    node._steer_step(distance, SimpleNamespace(seq=1), 20.0)
    cmd = sent[-1]
    assert cmd.command == "velocity"
    return math.hypot(cmd.params["vx"], cmd.params["vy"])


def test_no_gap_decelerates_smoothly_into_the_brake_distance():
    node, sent = _nav(avoidance_trapped_decel_ms2=0.4)
    stop = node._avoider.last_stop_m
    cruise = node._avoider.steer_speed_ms or node._cruise_speed
    # Room needed to shed cruise speed at 0.4 m/s^2 is v^2 / 2a; any more than
    # that and the aircraft must still be at full cruise.
    far = _trapped_speed(node, sent, stop + cruise ** 2 / (2 * 0.4) + 0.5)
    mid = _trapped_speed(node, sent, stop + 0.5)
    at = _trapped_speed(node, sent, stop)
    assert math.isclose(far, cruise), "slowed long before it needed to"
    assert at < 0.05, "arrived at the brake distance still moving"
    assert at < mid < far
    assert math.isclose(mid, min(cruise, math.sqrt(2 * 0.4 * 0.5)), rel_tol=1e-6)


def test_no_gap_does_not_leave_a_stale_goto_marked_as_issued():
    node, sent = _nav()
    node._last_goto_wp = node._current_wp
    _trapped_speed(node, sent, 5.0)
    assert node._last_goto_wp == -1


# -- takeoff gate ------------------------------------------------------------
def _gated(min_alt=1.5):
    node, _ = _nav(avoidance_min_alt_m=min_alt)
    node._obstacles = ObstacleArray(
        obstacles=[Obstacle(distance_m=1.0, bearing_deg=0.0)])
    return node


def _fly(node, *, armed, alt, phase):
    node._armed = armed
    node._phase = phase
    node._fused = FusedState(alt_rel_m=alt, valid=True)
    node._update_avoid_airborne()
    return node._avoid_decision()[0]


def test_avoidance_is_standby_on_the_ground_and_during_the_climb():
    node = _gated()
    assert _fly(node, armed=False, alt=0.0, phase=MissionPhase.IDLE) == CLEAR
    assert _fly(node, armed=True, alt=0.0, phase=MissionPhase.ARMING) == CLEAR
    assert _fly(node, armed=True, alt=0.6, phase=MissionPhase.TAKEOFF) == CLEAR
    node._publish_avoidance()
    assert node.bus.latest(Topics.AVOIDANCE).status == "STANDBY"


def test_a_mission_takeoff_hands_over_only_after_the_climb():
    """Past the gate altitude but still in TAKEOFF (climbing / settling): a
    brake or swerve here would fight the climb, so avoidance waits for
    NAVIGATE."""
    node = _gated()
    assert _fly(node, armed=True, alt=1.8, phase=MissionPhase.TAKEOFF) == CLEAR
    assert node._avoid_standby()
    assert _fly(node, armed=True, alt=1.9, phase=MissionPhase.NAVIGATE) == STOP


def test_an_invalid_altitude_estimate_never_latches():
    node = _gated()
    node._armed = True
    node._phase = MissionPhase.NAVIGATE
    node._fused = FusedState(alt_rel_m=5.0, valid=False)
    node._update_avoid_airborne()
    assert node._avoid_decision()[0] == CLEAR


def test_avoidance_activates_once_airborne_and_stays_latched_through_a_dip():
    node = _gated()
    assert _fly(node, armed=True, alt=2.0, phase=MissionPhase.NAVIGATE) == STOP
    # In-flight dips to ~1.5 m are in the logs; avoidance must not blink off.
    assert _fly(node, armed=True, alt=0.9, phase=MissionPhase.NAVIGATE) == STOP


def test_a_manual_takeoff_also_hands_over_once_airborne():
    node = _gated()
    assert _fly(node, armed=True, alt=0.4, phase=MissionPhase.MANUAL) == CLEAR
    assert _fly(node, armed=True, alt=2.0, phase=MissionPhase.MANUAL) == STOP


def test_disarm_resets_the_gate_for_the_next_flight():
    node = _gated()
    _fly(node, armed=True, alt=2.0, phase=MissionPhase.NAVIGATE)
    assert _fly(node, armed=False, alt=0.0, phase=MissionPhase.IDLE) == CLEAR
    assert _fly(node, armed=True, alt=0.2, phase=MissionPhase.ARMING) == CLEAR


def test_no_altitude_estimate_never_latches():
    node = _gated()
    node._armed = True
    node._phase = MissionPhase.NAVIGATE
    node._fused = None
    node._update_avoid_airborne()
    assert node._avoid_decision()[0] == CLEAR


def test_gate_off_keeps_the_old_behaviour():
    node = _gated(min_alt=0.0)
    assert _fly(node, armed=False, alt=0.0, phase=MissionPhase.IDLE) == STOP


def test_the_real_profile_gates_below_its_takeoff_altitude():
    """A gate at or above the takeoff altitude would never open on a normal
    mission; one at 0 would leave the climb unprotected from itself."""
    nav = Config.load("config/real.yaml").section("navigation")
    gate = float(nav.get("avoidance_min_alt_m", 0.0))
    assert 0.5 <= gate < 0.95 * float(nav["takeoff_altitude_m"])
