"""Drop-point person scan, the handshake gate, hover avoidance, and the RSSI
phone locator that picks WHICH person to lock when several are in view.

Requirement (2026-09-24): over the drop point come down to 3 m with the camera
down and search (in a small orbit) for a person. Only once a person is locked
may the BLE handshake release the parcel; no person within the search time, or
no handshake within the window after the lock, returns home with avoidance.
Obstacle avoidance must work while hovering. With 2+ people, use the phone's
RSSI to find the right one.

Geometry and timings are pinned here rather than read from the shipped
profile, so re-tuning real.yaml cannot silently change what these assert.
"""
from __future__ import annotations

import copy
import math
import random
import time
from types import SimpleNamespace

import numpy as np

from drone_stack.bus import MessageBus
from drone_stack.bus.topics import Topics
from drone_stack.msg import (
    DeliveryBleResult,
    FusedState,
    MissionPhase,
    Obstacle,
    ObstacleArray,
    PersonLockState,
    PhoneHint,
    Waypoint,
)
from drone_stack.nodes.navigation_node import NavigationNode
from drone_stack.nodes.phone_locator import (
    PhoneLocator,
    body_to_enu,
    enu_to_body_fr,
    ground_to_image,
    image_to_ground,
)
from drone_stack.srv import ServiceRegistry
from drone_stack.utils.config import Config

_SCAN = {
    "person_scan_enabled": True, "scan_altitude_m": 3.0,
    "person_search_s": 25.0, "handshake_window_s": 25.0,
    "scan_orbit_radius_m": 2.0, "scan_orbit_points": 8, "scan_orbit_step_s": 3.0,
    "scan_descend_timeout_s": 8.0, "scan_max_total_s": 90.0,
    "locator_enabled": True, "locator_pick_prob": 0.8, "locator_log_dir": "",
    "scan_pause_on_person": True, "scan_pause_conf": 0.30, "scan_pause_max_s": 6.0,
    "scan_resume_s": 1.5, "scan_found_grace_s": 3.0,
}
DROP = Waypoint(seq=1, x_m=0.0, y_m=20.0, alt_m=4.0, hold_s=20.0,
                radius_m=1.5, kind="hover")
LEG = Waypoint(seq=0, x_m=0.0, y_m=10.0, alt_m=4.0, kind="nav")


def _nav(**scan):
    raw = copy.deepcopy(Config.load().raw)
    raw.setdefault("delivery", {}).update({**_SCAN, **scan})
    raw.setdefault("navigation", {})["avoidance_rtl_router"] = "pi"
    # The operating ceiling on the aircraft (real.yaml, 2026-09-24). The
    # default profile's 2 m would clamp the 3 m scan altitude.
    raw.setdefault("safety", {})["max_altitude_m"] = 4.0
    bus = MessageBus()
    sent: list = []
    hints: list = []
    bus.subscribe(Topics.MAVLINK_CMD, sent.append)
    bus.subscribe(Topics.PHONE_HINT, hints.append)
    node = NavigationNode(bus, Config(raw), ServiceRegistry())
    node._armed = True
    node._has_flown = True
    node._home = (12.9, 77.6)
    node._mode = "GUIDED"
    node._phase = MissionPhase.NAVIGATE
    node._mission.waypoints = [copy.copy(LEG), copy.copy(DROP)]
    node._current_wp = 1
    node._avoid_min_alt = 0.0                     # takeoff gate is not under test
    _at(node, 20.0, alt=4.0)
    sent.clear()
    return node, sent, hints


def _at(node, y, alt=3.0, x=0.0, yaw=0.0):
    node._fused = FusedState(x=x, y=y, alt_rel_m=alt, yaw=yaw, valid=True)


def _lock(node, people_xy, locked=0, state="lock"):
    """The camera reporting ``people_xy`` with ``locked`` as its target."""
    p = people_xy[locked]
    node._on_person_lock(PersonLockState(
        state=state, score=float(p[2]), lock_id=1, nx=float(p[0]), ny=float(p[1]),
        people=len(people_xy), people_xy=[list(q) for q in people_xy]))


def _searching(**scan):
    node, sent, hints = _nav(**scan)
    node._enter_hover(node._mission.waypoints[1])
    _at(node, 20.0, alt=3.1)
    node._do_scan()
    assert node._scan_stage == "search"
    sent.clear()
    return node, sent, hints


def _commands(sent, name):
    return [c for c in sent if c.command == name]


# --------------------------------------------------------------------------- #
# stages
# --------------------------------------------------------------------------- #
class TestScanStages:
    def test_arrival_descends_to_scan_altitude_with_the_camera_down(self):
        node, sent, _ = _nav()
        node._enter_hover(node._mission.waypoints[1])
        assert node._phase == MissionPhase.HOVER
        assert node._scan_stage == "descend"
        goto = _commands(sent, "goto")[-1]
        assert goto.params["alt"] == 3.0
        assert node._cam_tilt == "down"

    def test_search_starts_once_at_scan_altitude(self):
        node, _, _ = _nav()
        node._enter_hover(node._mission.waypoints[1])
        _at(node, 20.0, alt=3.8)
        node._do_scan()
        assert node._scan_stage == "descend"
        _at(node, 20.0, alt=3.2)
        node._do_scan()
        assert node._scan_stage == "search"

    def test_no_person_in_the_search_time_returns_home(self):
        node, _, _ = _searching()
        node._hover_until = time.monotonic() - 0.1
        node._do_scan()
        assert node._phase == MissionPhase.RTL
        assert node._scan_stage == ""

    def test_a_lock_opens_the_window_and_holds_position(self):
        node, sent, _ = _searching()
        _lock(node, [[0.05, -0.1, 0.7]])
        node._do_scan()
        assert node._scan_stage == "locked"
        left = node._hover_until - time.monotonic()
        assert 24.0 < left <= 25.0
        assert _commands(sent, "goto")         # re-anchored where it is

    def test_no_handshake_in_the_window_returns_home(self):
        node, _, _ = _searching()
        _lock(node, [[0.0, 0.0, 0.7]])
        node._do_scan()
        node._hover_until = time.monotonic() - 0.1
        node._do_scan()
        assert node._phase == MissionPhase.RTL

    def test_handshake_after_the_lock_returns_home_after_the_short_wait(self):
        node, _, _ = _searching()
        _lock(node, [[0.0, 0.0, 0.7]])
        node._do_scan()
        node._on_ble_delivery_result(DeliveryBleResult(order_id="o", success=True))
        node._ble_delivered_at -= 60.0
        node._do_scan()
        assert node._phase == MissionPhase.RTL

    def test_a_lock_seen_before_the_search_began_does_not_count(self):
        node, _, _ = _nav()
        node._enter_hover(node._mission.waypoints[1])
        _lock(node, [[0.0, 0.0, 0.7]])           # during the descent
        _at(node, 20.0, alt=3.1)
        node._do_scan()                           # -> search
        node._do_scan()
        assert node._scan_stage == "search"

    def test_a_hold_lock_is_not_a_lock(self):
        node, _, _ = _searching()
        _lock(node, [[0.0, 0.0, 0.7]], state="hold")
        node._do_scan()
        assert node._scan_stage == "search"

    def test_the_orbit_moves_the_aircraft_while_searching(self):
        node, sent, _ = _searching()
        node._scan_next_step = 0.0
        for _ in range(3):
            node._do_scan()
            _at(node, 20.0, alt=3.0, yaw=node._scan_orbit_a0)
        targets = [(round(c.params["lat"], 7), round(c.params["lon"], 7))
                   for c in _commands(sent, "goto")]
        assert targets, "the search never moved"


# --------------------------------------------------------------------------- #
# stop the circle where the person is (operator, 2026-09-24)
# --------------------------------------------------------------------------- #
def _seen(node, score=0.35, state="search"):
    """The camera seeing one person it has not locked yet."""
    node._on_person_lock(PersonLockState(
        state=state, score=0.0, lock_id=0, people=1, people_xy=[[0.1, -0.1, score]]))


def _nobody(node):
    node._on_person_lock(PersonLockState(state="search", people=0, people_xy=[]))


def _mid_orbit(node, sent):
    """Advance the orbit until a leg has actually been flown."""
    node._scan_next_step = 0.0
    for _ in range(3):
        node._do_scan()
        _at(node, 20.0, alt=3.0, yaw=node._scan_orbit_a0)
    assert _commands(sent, "goto"), "the orbit never started"
    sent.clear()


class TestStopForPerson:
    def test_a_person_in_view_stops_the_orbit_where_it_is(self):
        node, sent, _ = _searching()
        _mid_orbit(node, sent)
        _at(node, 21.3, alt=3.0, x=0.7, yaw=node._scan_orbit_a0)
        _seen(node)
        node._do_scan()
        assert node._scan_stage == "search"
        assert node._scan_paused_at is not None
        assert node._scan_pending is None
        assert node._scan_target == (0.7, 21.3)        # held on the spot it saw them
        sent.clear()
        node._scan_next_step = 0.0                      # an orbit step would be due
        for _ in range(5):
            _seen(node)
            node._do_scan()
        assert not _commands(sent, "goto"), "the orbit kept going with a person in view"

    def test_the_lock_then_holds_that_same_spot(self):
        node, sent, _ = _searching()
        _mid_orbit(node, sent)
        _at(node, 21.3, alt=3.0, x=0.7)
        _seen(node)
        node._do_scan()
        held = node._scan_target
        _at(node, 21.5, alt=3.0, x=0.9)                  # overshoot while stopping
        _lock(node, [[0.1, -0.1, 0.7]])
        node._do_scan()
        assert node._scan_stage == "locked"
        assert node._scan_target == held, "the lock re-anchored on the overshoot"
        assert node._handshake_open()
        sent.clear()
        node._scan_next_step = 0.0
        for _ in range(10):
            node._do_scan()
        assert not _commands(sent, "goto"), "moved during the handshake window"
        assert node._scan_stage == "locked"

    def test_a_lock_straight_from_the_orbit_also_holds(self):
        node, sent, _ = _searching()
        _mid_orbit(node, sent)
        _at(node, 21.3, alt=3.0, x=0.7)
        _lock(node, [[0.1, -0.1, 0.7]])
        node._do_scan()
        assert node._scan_stage == "locked"
        assert node._scan_target == (0.7, 21.3)

    def test_a_weak_detection_does_not_stop_the_search(self):
        node, sent, _ = _searching()
        _seen(node, score=0.22)                          # HEF floor, below pause_conf
        node._do_scan()
        assert node._scan_paused_at is None

    def test_acquiring_stops_it_whatever_the_score(self):
        node, _, _ = _searching()
        _seen(node, score=0.25, state="acquire")
        node._do_scan()
        assert node._scan_paused_at is not None

    def test_a_one_frame_dropout_keeps_holding(self):
        node, sent, _ = _searching()
        _seen(node)
        node._do_scan()
        _nobody(node)
        node._scan_next_step = 0.0
        sent.clear()
        node._do_scan()
        assert node._scan_paused_at is not None
        assert not _commands(sent, "goto")

    def test_the_orbit_resumes_once_they_have_left(self):
        node, sent, _ = _searching()
        _seen(node)
        node._do_scan()
        node._scan_seen_at -= 2.0                        # > scan_resume_s ago
        _nobody(node)
        node._do_scan()
        assert node._scan_paused_at is None

    def test_no_lock_after_the_pause_limit_moves_on_and_does_not_restop(self):
        node, _, _ = _searching()
        _seen(node)
        node._do_scan()
        node._scan_paused_at -= 7.0                      # > scan_pause_max_s
        _seen(node)
        node._do_scan()
        assert node._scan_paused_at is None
        _seen(node)
        node._do_scan()
        assert node._scan_paused_at is None, "re-stopped on the same sighting"
        node._scan_seen_at -= 2.0                        # they leave ...
        _nobody(node)
        node._do_scan()
        _seen(node)                                      # ... and come back
        node._do_scan()
        assert node._scan_paused_at is not None

    def test_stopped_on_someone_at_the_deadline_gets_the_grace(self):
        node, _, _ = _searching()
        _seen(node)
        node._do_scan()
        node._hover_until = time.monotonic() - 1.0       # 25 s up, 1 s ago
        _seen(node)
        node._do_scan()
        assert node._scan_stage == "search", "left mid-acquire"
        node._hover_until = time.monotonic() - 3.5       # past the 3 s grace
        _seen(node)
        node._do_scan()
        assert node._phase == MissionPhase.RTL

    def test_nobody_in_view_at_the_deadline_gets_no_grace(self):
        node, _, _ = _searching()
        node._hover_until = time.monotonic() - 0.1
        node._do_scan()
        assert node._phase == MissionPhase.RTL

    def test_stopping_mid_turn_pins_the_heading(self):
        node, sent, _ = _searching()
        node._scan_next_step = 0.0
        _at(node, 20.0, alt=3.0, yaw=node._scan_orbit_a0 + math.pi)  # leg is behind
        node._nose_yaw_sent_at = 0.0
        node._do_scan()                                  # picks a leg, needs a turn
        node._do_scan()
        assert node._scan_pending is not None
        sent.clear()
        _seen(node)
        node._do_scan()
        yaws = _commands(sent, "yaw")
        assert yaws and yaws[-1].params["angle"] == 0.0

    def test_can_be_switched_off(self):
        node, _, _ = _searching(scan_pause_on_person=False)
        _seen(node)
        node._do_scan()
        assert node._scan_paused_at is None


# --------------------------------------------------------------------------- #
# the handshake gate the BLE peripheral reads
# --------------------------------------------------------------------------- #
class TestHandshakeGate:
    def test_closed_while_descending_and_searching(self):
        node, _, _ = _nav()
        node._enter_hover(node._mission.waypoints[1])
        assert node._handshake_open() is False
        _at(node, 20.0, alt=3.1)
        node._do_scan()
        assert node._handshake_open() is False

    def test_open_only_while_locked(self):
        node, _, _ = _searching()
        _lock(node, [[0.0, 0.0, 0.7]])
        node._do_scan()
        assert node._handshake_open() is True
        node._hover_until = time.monotonic() - 0.1
        node._do_scan()
        assert node._handshake_open() is False

    def test_always_open_on_the_ground_for_bench_tests(self):
        node, _, _ = _nav()
        node._armed = False
        assert node._handshake_open() is True

    def test_always_open_with_the_scan_disabled(self):
        node, _, _ = _nav(person_scan_enabled=False)
        node._enter_hover(node._mission.waypoints[1])
        assert node._handshake_open() is True

    def test_mission_state_carries_it(self):
        node, _, _ = _searching()
        fields = node._scan_state_fields()
        assert fields["handshake_open"] is False
        assert fields["scan_stage"] == "search"

    def test_a_drop_reported_before_any_lock_still_returns_home(self):
        node, _, _ = _searching()
        node._on_ble_delivery_result(DeliveryBleResult(order_id="o", success=True))
        node._ble_delivered_at -= 60.0
        node._do_scan()
        assert node._phase == MissionPhase.RTL


# --------------------------------------------------------------------------- #
# obstacle avoidance while hovering
# --------------------------------------------------------------------------- #
def _obstacle(dist, bearing):
    r = math.radians(bearing)
    return Obstacle(distance_m=dist, bearing_deg=bearing,
                    x_m=dist * math.cos(r), y_m=-dist * math.sin(r), num_points=10)


class TestHoverAvoidance:
    def test_an_obstacle_to_the_side_pushes_the_aircraft_away(self):
        node, sent, _ = _searching()
        node._obstacles = ObstacleArray(obstacles=[_obstacle(0.6, 90.0)])
        node._do_scan()
        vel = _commands(sent, "velocity")
        assert vel, "no response to an obstacle while hovering"
        assert vel[-1].params["vy"] < 0.0          # obstacle right -> move left
        assert vel[-1].params["vx"] >= 0.0

    def test_dead_ahead_slides_sideways_and_never_reverses_blind(self):
        node, sent, _ = _searching()
        node._obstacles = ObstacleArray(obstacles=[_obstacle(0.6, 0.0)])
        node._do_scan()
        vel = _commands(sent, "velocity")
        assert vel
        assert vel[-1].params["vx"] >= 0.0
        assert abs(vel[-1].params["vy"]) > 0.0

    def test_it_also_works_in_the_plain_hover(self):
        node, sent, _ = _nav(person_scan_enabled=False)
        node._enter_hover(node._mission.waypoints[1])
        sent.clear()
        node._obstacles = ObstacleArray(obstacles=[_obstacle(0.6, -90.0)])
        node._do_hover()
        vel = _commands(sent, "velocity")
        assert vel and vel[-1].params["vy"] > 0.0

    def test_no_orbit_step_while_dodging(self):
        node, sent, _ = _searching()
        node._scan_next_step = 0.0
        node._obstacles = ObstacleArray(obstacles=[_obstacle(0.6, 90.0)])
        node._do_scan()
        assert not _commands(sent, "goto")


# --------------------------------------------------------------------------- #
# which person: the navigator side
# --------------------------------------------------------------------------- #
def _feed_orbit(locator, phone_en, *, centre=(0.0, 20.0), r=2.0, alt=3.0,
                sigma=2.0, ref=-58.0, n_path=2.0, seed=1, laps=2):
    rng = random.Random(seed)
    now = time.time()
    k = 0
    for lap in range(laps):
        for i in range(16):
            a = 2 * math.pi * i / 16
            e, n = centre[0] + r * math.sin(a), centre[1] + r * math.cos(a)
            d = math.sqrt((e - phone_en[0]) ** 2 + (n - phone_en[1]) ** 2 + (alt - 1.2) ** 2)
            for _ in range(4):
                rssi = ref - 10 * n_path * math.log10(d) + rng.gauss(0, sigma)
                locator.add_rssi(rssi, e, n, alt, now - 60 + k * 0.25)
                k += 1
    return now


def _people_in_image(node, people_en):
    """people_xy as the camera would report them from the current pose."""
    f = node._fused
    pitch = node._cam_pitch_from_nadir()
    h = f.alt_rel_m - node._scan_person_h
    out = []
    for e, n in people_en:
        fwd, right = enu_to_body_fr(e - f.x, n - f.y, f.yaw)
        nx, ny, _ = ground_to_image(fwd, right, h, pitch)
        out.append([nx, ny, 0.7])
    return out


class TestRssiPick:
    def test_confident_rssi_on_the_locked_person_accepts(self):
        node, _, _ = _searching()
        people = [(0.8, 20.5), (-1.8, 19.6)]
        _feed_orbit(node._locator, people[0])
        _lock(node, _people_in_image(node, people), locked=0)
        node._do_scan()
        assert node._scan_stage == "locked"
        assert "RSSI picked" in node._scan_note

    def test_confident_rssi_on_someone_else_asks_for_a_relock(self):
        node, _, hints = _searching()
        people = [(0.8, 20.5), (-1.8, 19.6)]
        _feed_orbit(node._locator, people[1])
        img = _people_in_image(node, people)
        _lock(node, img, locked=0)
        node._do_scan()
        assert node._scan_stage == "search"
        relock = [h for h in hints if isinstance(h, PhoneHint) and h.relock]
        assert relock
        assert abs(relock[-1].nx - img[1][0]) < 1e-6

    def test_no_rssi_falls_back_to_the_cameras_pick(self):
        node, _, _ = _searching()
        people = [(0.8, 20.5), (-1.8, 19.6)]
        _lock(node, _people_in_image(node, people), locked=1)
        node._do_scan()
        assert node._scan_stage == "locked"

    def test_one_person_needs_no_rssi(self):
        node, _, _ = _searching()
        _lock(node, [[0.1, 0.1, 0.5]])
        node._do_scan()
        assert node._scan_stage == "locked"

    def test_camera_projection_round_trip(self):
        for fwd, right in [(0.0, 0.0), (1.5, -0.7), (-0.5, 2.0)]:
            nx, ny, _ = ground_to_image(fwd, right, 2.0, 15.0)
            g = image_to_ground(nx, ny, 2.0, 15.0)
            assert math.isclose(g[0], fwd, abs_tol=1e-6)
            assert math.isclose(g[1], right, abs_tol=1e-6)

    def test_body_enu_round_trip(self):
        for yaw in (0.0, 0.7, -2.4):
            e, n = body_to_enu(1.3, -0.4, yaw)
            f, r = enu_to_body_fr(e, n, yaw)
            assert math.isclose(f, 1.3, abs_tol=1e-9) and math.isclose(r, -0.4, abs_tol=1e-9)


class TestPhoneLocator:
    def test_scores_the_phone_holder_highest(self):
        wrong = 0
        for seed in range(20):
            loc = PhoneLocator()
            now = _feed_orbit(loc, (0.8, 20.5), seed=seed, sigma=3.0)
            p = loc.score_candidates([(0.8, 20.5), (-1.8, 19.6)], 0.0, 20.0, now)
            wrong += p[1] > p[0]
        assert wrong <= 2

    def test_no_evidence_is_no_answer(self):
        loc = PhoneLocator()
        assert loc.score_candidates([(0.0, 0.0), (1.0, 1.0)], 0.0, 0.0, time.time()) is None

    def test_rejects_impossible_rssi(self):
        loc = PhoneLocator()
        loc.add_rssi(0.0, 0, 0, 3, time.time())
        loc.add_rssi(-200.0, 0, 0, 3, time.time())
        assert loc.rssi_count == 0

    def test_bearing_is_good_even_when_range_is_not(self):
        loc = PhoneLocator()
        now = _feed_orbit(loc, (4.0, 24.0), sigma=2.0, laps=3)
        fix = loc.estimate(0.0, 20.0, now)
        assert fix is not None
        bearing = math.degrees(math.atan2(fix.east - 0.0, fix.north - 20.0))
        assert abs(bearing - 45.0) < 30.0


# --------------------------------------------------------------------------- #
# which person: the camera side
# --------------------------------------------------------------------------- #
class TestLockHint:
    def _lock(self):
        from drone_stack.gcs.person_lock import TargetLock
        return TargetLock(acquire_conf=0.4, keep_conf=0.2, confirm_hits=1)

    def test_most_confident_without_a_hint(self):
        lk = self._lock()
        a, b = (100, 100, 150, 200, 0.8), (600, 100, 650, 200, 0.5)
        assert lk._pick_candidate([a, b]) == a

    def test_the_hint_picks_the_phone_holder(self):
        lk = self._lock()
        a, b = (100, 100, 150, 200, 0.8), (600, 100, 650, 200, 0.5)
        lk.hint = (625.0, 150.0, 60.0)
        assert lk._pick_candidate([a, b]) == b

    def test_a_single_person_ignores_the_hint(self):
        lk = self._lock()
        a = (100, 100, 150, 200, 0.8)
        lk.hint = (625.0, 150.0, 60.0)
        assert lk._pick_candidate([a]) == a

    def test_forget_clears_the_reacquire_memory(self):
        lk = self._lock()
        lk._start_lock((100, 100, 150, 200, 0.8), 0.0)
        lk._remember(1.0)
        lk.reset()
        assert lk._mem is not None
        lk.forget()
        assert lk._mem is None and lk.state == lk.SEARCH

    # -- re-acquisition: the SAME person comes back after a dropout --------
    # Locked on a 50x100 px box centred (125, 150), lost at t=1.0. Base search
    # radius = reacquire_dist 3.0 x 100 px = 300 px, growing to 600 px by t=2.
    def _lost(self):
        from drone_stack.gcs.person_lock import TargetLock
        lk = TargetLock(acquire_conf=0.4, keep_conf=0.2, confirm_hits=2,
                        reacquire_s=8.0, reacquire_dist=3.0)
        lk._start_lock((100, 100, 150, 200, 0.8), 0.0)
        lk._remember(1.0)
        lk.reset()
        return lk

    def test_a_faint_return_of_the_lost_person_relocks_at_once(self):
        lk = self._lost()
        lk.observe([(110, 105, 160, 205, 0.25)], 1.2)   # below acquire_conf
        assert lk.state == lk.LOCKED and lk.reacquires == 1

    def test_nearest_beats_a_more_confident_bystander(self):
        lk = self._lost()
        near, far = (130, 110, 180, 210, 0.3), (450, 100, 500, 200, 0.9)
        assert lk._reacquire_pick([far, near], 1.2) == near

    def test_beyond_the_radius_is_left_to_normal_acquire(self):
        lk = self._lost()
        assert lk._reacquire_pick([(700, 100, 750, 200, 0.9)], 1.2) is None
        # ...and the radius stops growing, so waiting does not let it in.
        assert lk._reacquire_pick([(820, 100, 870, 200, 0.9)], 8.0) is None

    def test_the_radius_grows_for_a_walking_person(self):
        lk = self._lost()
        walked = (575, 100, 625, 200, 0.5)                # 475 px away
        assert lk._reacquire_pick([walked], 1.1) is None
        assert lk._reacquire_pick([walked], 2.0) == walked

    def test_a_wrong_sized_box_is_not_the_same_person(self):
        lk = self._lost()
        assert lk._reacquire_pick([(100, 100, 250, 400, 0.9)], 1.2) is None

    def test_below_the_keep_floor_is_ignored(self):
        lk = self._lost()
        assert lk._reacquire_pick([(110, 105, 160, 205, 0.15)], 1.2) is None

    def test_an_expired_memory_needs_the_full_acquire(self):
        lk = self._lost()
        lk.observe([(110, 105, 160, 205, 0.25)], 9.5)    # > reacquire_s
        assert lk.state == lk.SEARCH and lk.reacquires == 0 and lk._mem is None

    def _pipe(self):
        from drone_stack.gcs.person_lock import PersonLockPipeline
        pipe = PersonLockPipeline(None, self._lock(), hfov_deg=66.0)
        pipe.enabled = True
        return pipe

    def test_people_are_reported_on_the_normalised_plane(self):
        pipe = self._pipe()
        frame = np.zeros((720, 1280, 3), np.uint8)
        pipe.process(frame, None, None, now=1.0)
        fx = 640.0 / math.tan(math.radians(33.0))
        pipe._dets = [(640 - 20, 360 - 40, 640 + 20, 360 + 40, 0.7),
                      (640 + fx * 0.5 - 20, 360 - 40, 640 + fx * 0.5 + 20, 360 + 40, 0.6)]
        pipe._dets_t = 1.0
        snap = pipe.process(frame, None, None, now=1.05)
        xy = snap["people_xy"]
        assert len(xy) == 2
        assert abs(xy[0][0]) < 1e-3 and abs(xy[0][1]) < 1e-3
        assert abs(xy[1][0] - 0.5) < 1e-3

    def test_a_relock_hint_drops_the_current_lock_once(self):
        pipe = self._pipe()
        frame = np.zeros((720, 1280, 3), np.uint8)
        pipe.process(frame, None, None, now=1.0)
        pipe.lock._start_lock((100, 100, 150, 200, 0.8), 1.0)
        pipe.set_hint(SimpleNamespace(valid=True, nx=0.4, ny=0.0, sigma=0.15,
                                      relock=True, seq=1))
        pipe.process(frame, None, None, now=1.05)
        assert pipe.lock.state == pipe.lock.SEARCH
        assert pipe.lock.hint is not None
        pipe.lock._start_lock((100, 100, 150, 200, 0.8), 1.1)
        pipe.process(frame, None, None, now=1.15)
        assert pipe.lock.state == pipe.lock.LOCKED      # same seq: not again

    def test_an_invalid_hint_clears_it(self):
        pipe = self._pipe()
        pipe.set_hint(SimpleNamespace(valid=True, nx=0.4, ny=0.0, sigma=0.15,
                                      relock=False, seq=0))
        pipe.set_hint(SimpleNamespace(valid=False))
        pipe.process(np.zeros((720, 1280, 3), np.uint8), None, None, now=1.0)
        assert pipe.lock.hint is None


# --------------------------------------------------------------------------- #
# the BLE side
# --------------------------------------------------------------------------- #
class TestRssiProbeParsing:
    def _probe(self):
        import importlib.util
        from pathlib import Path
        path = Path(__file__).resolve().parents[1] / "drone_stack" / "ble_handshake" / "rssi_probe.py"
        spec = importlib.util.spec_from_file_location("rssi_probe", path)
        mod = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(mod)
        return mod

    def test_command_bytes(self):
        m = self._probe()
        assert m.build_read_rssi(0x0040) == bytes([0x01, 0x05, 0x14, 0x02, 0x40, 0x00])

    def test_command_complete(self):
        m = self._probe()
        pkt = bytes([0x04, 0x0E, 0x07, 0x01, 0x05, 0x14, 0x00, 0x40, 0x00, 0xC4])
        assert m.is_reply(pkt)
        assert m.parse_read_rssi(pkt, 0x40) == -60

    def test_wrong_handle_or_failed_status(self):
        m = self._probe()
        other = bytes([0x04, 0x0E, 0x07, 0x01, 0x05, 0x14, 0x00, 0x41, 0x00, 0xC4])
        failed = bytes([0x04, 0x0E, 0x07, 0x01, 0x05, 0x14, 0x02, 0x40, 0x00, 0x00])
        assert m.parse_read_rssi(other, 0x40) is None
        assert m.parse_read_rssi(failed, 0x40) is None

    def test_command_status_is_a_reply_but_no_reading(self):
        m = self._probe()
        pkt = bytes([0x04, 0x0F, 0x04, 0x0C, 0x01, 0x05, 0x14])
        assert m.is_reply(pkt)
        assert m.parse_read_rssi(pkt, 0x40) is None

    def test_conn_list_layout(self):
        import struct
        m = self._probe()
        buf = struct.pack("<HH", 0, 2)
        buf += struct.pack("<H6sBBHI", 0x40, m.addr_to_bytes("AA:BB:CC:DD:EE:FF"), 0x80, 0, 1, 0)
        buf += struct.pack("<H6sBBHI", 0x41, m.addr_to_bytes("11:22:33:44:55:66"), 0x01, 1, 1, 0)
        links = m.parse_conn_list(buf)
        assert links[0] == {"handle": 0x40, "addr": "AA:BB:CC:DD:EE:FF", "type": 0x80,
                            "out": 0, "state": 1, "link_mode": 0}
        assert links[1]["type"] == 0x01

    def test_device_path(self):
        m = self._probe()
        assert m.addr_from_device_path("/org/bluez/hci0/dev_AA_BB_CC_DD_EE_FF") == \
            (0, "AA:BB:CC:DD:EE:FF")
        assert m.addr_from_device_path("") is None


class TestHubBridge:
    def test_rssi_batch_goes_on_the_bus_per_sample(self):
        from drone_stack.gcs.hub import GcsHub
        bus = MessageBus()
        got: list = []
        bus.subscribe(Topics.BLE_PHONE, got.append)
        out = GcsHub._on_ble_phone_signal(SimpleNamespace(bus=bus), {
            "order_id": "o", "samples": [[100.0, -61], [100.25, -63], [1, 5], ["x"]]})
        assert out["ok"]
        assert [(m.t, m.rssi_dbm) for m in got] == [(100.0, -61.0), (100.25, -63.0)]

    def test_phone_fix(self):
        from drone_stack.gcs.hub import GcsHub
        bus = MessageBus()
        got: list = []
        bus.subscribe(Topics.BLE_PHONE, got.append)
        assert GcsHub._on_ble_phone_fix(SimpleNamespace(bus=bus),
                                        {"lat": 12.9, "lon": 77.6})["ok"]
        assert not GcsHub._on_ble_phone_fix(SimpleNamespace(bus=bus),
                                            {"lat": 0.0, "lon": 0.0})["ok"]
        assert len(got) == 1 and got[0].kind == "gps"
