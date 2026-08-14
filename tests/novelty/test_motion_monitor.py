"""novelty/motion_monitor.py - §2.4 track association, velocity estimation,
and the two independent descent-abort triggers."""
from __future__ import annotations

import pytest

from drone_stack.novelty.config import NoveltyConfig
from drone_stack.novelty.motion_monitor import MotionMonitor
from drone_stack.novelty.types import GroundPoint, PersonDetection, PixelBox


@pytest.fixture
def cfg(valid_config_dir):
    return NoveltyConfig.load(valid_config_dir).motion_monitor


def _person(x_m: float, y_m: float, stamp: float, track_id: int | None = None) -> PersonDetection:
    return PersonDetection(
        bbox=PixelBox(0, 0, 10, 10), score=0.9,
        ground=GroundPoint(x_m=x_m, y_m=y_m), track_id=track_id, stamp=stamp,
    )


# --------------------------------------------------------------------------- #
# Track association
# --------------------------------------------------------------------------- #
def test_update_assigns_same_track_id_to_nearby_successive_detections(cfg):
    mon = MotionMonitor(cfg)
    [d1] = mon.update([_person(0.0, 0.0, stamp=0.0)])
    [d2] = mon.update([_person(0.2, 0.0, stamp=1.0)])  # small move, within the gate
    assert d1.track_id == d2.track_id


def test_update_starts_a_new_track_beyond_association_gate(cfg):
    mon = MotionMonitor(cfg)
    [d1] = mon.update([_person(0.0, 0.0, stamp=0.0)])
    far = cfg.max_association_distance_m + 5.0
    [d2] = mon.update([_person(far, 0.0, stamp=1.0)])
    assert d1.track_id != d2.track_id


def test_update_tracks_two_simultaneous_people_independently(cfg):
    mon = MotionMonitor(cfg)
    a0 = _person(0.0, 0.0, stamp=0.0)
    b0 = _person(20.0, 0.0, stamp=0.0)
    mon.update([a0, b0])
    assert a0.track_id != b0.track_id

    a1 = _person(0.3, 0.0, stamp=1.0)
    b1 = _person(20.3, 0.0, stamp=1.0)
    mon.update([a1, b1])
    assert a1.track_id == a0.track_id
    assert b1.track_id == b0.track_id


def test_update_ignores_detections_without_ground(cfg):
    mon = MotionMonitor(cfg)
    ungrounded = PersonDetection(bbox=PixelBox(0, 0, 5, 5), score=0.5, ground=None, stamp=0.0)
    [out] = mon.update([ungrounded])
    assert out.track_id is None


def test_missed_frame_does_not_lose_the_track(cfg):
    """A track with no matching detection this frame is kept, not dropped -
    the recipient reappearing later still gets their original track_id."""
    mon = MotionMonitor(cfg)
    [d1] = mon.update([_person(0.0, 0.0, stamp=0.0)])
    mon.update([])  # frame with no detections at all
    [d3] = mon.update([_person(0.1, 0.0, stamp=2.0)])
    assert d3.track_id == d1.track_id


# --------------------------------------------------------------------------- #
# Velocity estimation
# --------------------------------------------------------------------------- #
def test_velocity_mps_is_none_before_min_track_frames(cfg):
    mon = MotionMonitor(cfg)
    for i in range(cfg.min_track_frames - 1):
        [d] = mon.update([_person(float(i) * 0.1, 0.0, stamp=float(i))])
    assert mon.velocity_mps(d.track_id) is None


def test_velocity_mps_uses_net_displacement_over_window_not_last_delta(cfg):
    """Regression for the shipped config comment: 'simple average
    displacement over the window, not a single frame-to-frame delta.' A
    track that moves steadily 1 m/s must report ~1 m/s even though we only
    check after several frames (net displacement / elapsed time)."""
    mon = MotionMonitor(cfg)
    track_id = None
    for i in range(cfg.min_track_frames):
        [d] = mon.update([_person(float(i) * 1.0, 0.0, stamp=float(i))])
        track_id = d.track_id
    # min_track_frames-1 seconds elapsed, min_track_frames-1 metres moved -> 1.0 m/s
    assert mon.velocity_mps(track_id) == pytest.approx(1.0, abs=1e-6)


def test_velocity_mps_window_is_bounded_by_track_history_len(cfg):
    """Old positions outside track_history_len must not affect the estimate:
    move briskly for track_history_len frames, then stand still for
    track_history_len more - once the brisk phase has fully scrolled out of
    the bounded deque, velocity should read ~0."""
    mon = MotionMonitor(cfg)
    track_id = None
    step = 0
    for i in range(cfg.track_history_len):
        [d] = mon.update([_person(float(i) * 1.0, 0.0, stamp=float(step))])
        track_id = d.track_id
        step += 1
    last_x = float(cfg.track_history_len - 1) * 1.0
    for _ in range(cfg.track_history_len):
        mon.update([_person(last_x, 0.0, stamp=float(step))])
        step += 1
    assert mon.velocity_mps(track_id) == pytest.approx(0.0, abs=1e-6)


def test_velocity_mps_unknown_track_is_none(cfg):
    mon = MotionMonitor(cfg)
    assert mon.velocity_mps(999) is None


# --------------------------------------------------------------------------- #
# check_velocity_abort - both conditions required (AND, not OR)
# --------------------------------------------------------------------------- #
def _fast_speed(cfg) -> float:
    """A per-frame (1s steps) speed that is both above
    max_recipient_velocity_mps AND within max_association_distance_m, so a
    fast-moving track is still correctly associated frame-to-frame."""
    return (cfg.max_recipient_velocity_mps + cfg.max_association_distance_m) / 2.0


def _fast_track(mon: MotionMonitor, cfg) -> int:
    """Build a track moving above max_recipient_velocity_mps."""
    fast_speed = _fast_speed(cfg)
    track_id = None
    for i in range(cfg.min_track_frames):
        [d] = mon.update([_person(float(i) * fast_speed, 0.0, stamp=float(i))])
        track_id = d.track_id
    return track_id


def test_velocity_abort_fires_when_fast_and_low(cfg):
    mon = MotionMonitor(cfg)
    track_id = _fast_track(mon, cfg)
    event = mon.check_velocity_abort(track_id, altitude_m=cfg.abort_altitude_ceiling_m - 1.0)
    assert event is not None
    assert event.reason == "recipient_velocity"
    assert event.track_id == track_id
    assert event.velocity_mps > cfg.max_recipient_velocity_mps


def test_velocity_abort_does_not_fire_above_altitude_ceiling(cfg):
    mon = MotionMonitor(cfg)
    track_id = _fast_track(mon, cfg)
    event = mon.check_velocity_abort(track_id, altitude_m=cfg.abort_altitude_ceiling_m + 1.0)
    assert event is None


def test_velocity_abort_does_not_fire_when_slow_even_if_low(cfg):
    mon = MotionMonitor(cfg)
    track_id = None
    slow_speed = cfg.max_recipient_velocity_mps * 0.1
    for i in range(cfg.min_track_frames):
        [d] = mon.update([_person(float(i) * slow_speed, 0.0, stamp=float(i))])
        track_id = d.track_id
    event = mon.check_velocity_abort(track_id, altitude_m=cfg.abort_altitude_ceiling_m - 1.0)
    assert event is None


def test_velocity_abort_none_before_min_track_frames(cfg):
    mon = MotionMonitor(cfg)
    [d] = mon.update([_person(0.0, 0.0, stamp=0.0)])
    event = mon.check_velocity_abort(d.track_id, altitude_m=0.0)
    assert event is None


# --------------------------------------------------------------------------- #
# check_zone_intrusion
# --------------------------------------------------------------------------- #
def test_zone_intrusion_fires_for_a_different_person_inside_radius(cfg):
    mon = MotionMonitor(cfg)
    recipient = _person(0.0, 0.0, stamp=0.0, track_id=1)
    intruder = _person(0.0, cfg.zone_intrusion_radius_m - 0.5, stamp=0.0, track_id=2)
    centroid = GroundPoint(x_m=0.0, y_m=0.0)

    event = mon.check_zone_intrusion([recipient, intruder], recipient_track_id=1, landing_zone_centroid=centroid)
    assert event is not None
    assert event.reason == "zone_intrusion"
    assert event.track_id == 2
    assert event.intruder_distance_m < cfg.zone_intrusion_radius_m


def test_zone_intrusion_ignores_the_recipients_own_track(cfg):
    mon = MotionMonitor(cfg)
    recipient = _person(0.0, 0.0, stamp=0.0, track_id=1)
    centroid = GroundPoint(x_m=0.0, y_m=0.0)
    event = mon.check_zone_intrusion([recipient], recipient_track_id=1, landing_zone_centroid=centroid)
    assert event is None


def test_zone_intrusion_ignores_a_person_outside_the_radius(cfg):
    mon = MotionMonitor(cfg)
    recipient = _person(0.0, 0.0, stamp=0.0, track_id=1)
    far_person = _person(0.0, cfg.zone_intrusion_radius_m + 10.0, stamp=0.0, track_id=2)
    centroid = GroundPoint(x_m=0.0, y_m=0.0)
    event = mon.check_zone_intrusion([recipient, far_person], recipient_track_id=1, landing_zone_centroid=centroid)
    assert event is None


def test_zone_intrusion_ignores_detections_without_ground(cfg):
    mon = MotionMonitor(cfg)
    ungrounded = PersonDetection(bbox=PixelBox(0, 0, 5, 5), score=0.5, ground=None, track_id=2, stamp=0.0)
    centroid = GroundPoint(x_m=0.0, y_m=0.0)
    event = mon.check_zone_intrusion([ungrounded], recipient_track_id=1, landing_zone_centroid=centroid)
    assert event is None


# --------------------------------------------------------------------------- #
# evaluate() - combined per-frame entry point
# --------------------------------------------------------------------------- #
def test_evaluate_returns_empty_list_when_all_clear(cfg):
    mon = MotionMonitor(cfg)
    track_id = None
    for i in range(cfg.min_track_frames):
        [d] = mon.update([_person(0.0, 0.0, stamp=float(i))])
        track_id = d.track_id
    events = mon.evaluate(
        [_person(0.0, 0.0, stamp=float(cfg.min_track_frames), track_id=track_id)],
        recipient_track_id=track_id, altitude_m=cfg.abort_altitude_ceiling_m - 1.0,
        landing_zone_centroid=GroundPoint(x_m=0.0, y_m=0.0),
    )
    assert events == []


def test_evaluate_can_return_both_events_in_one_frame(cfg):
    mon = MotionMonitor(cfg)
    fast_speed = _fast_speed(cfg)
    track_id = None
    for i in range(cfg.min_track_frames):
        [d] = mon.update([_person(float(i) * fast_speed, 0.0, stamp=float(i))])
        track_id = d.track_id

    intruder = _person(0.0, cfg.zone_intrusion_radius_m - 0.5, stamp=float(cfg.min_track_frames))
    recipient_now = _person(
        float(cfg.min_track_frames) * fast_speed, 0.0,
        stamp=float(cfg.min_track_frames), track_id=track_id,
    )

    events = mon.evaluate(
        [recipient_now, intruder],
        recipient_track_id=track_id, altitude_m=cfg.abort_altitude_ceiling_m - 1.0,
        landing_zone_centroid=GroundPoint(x_m=0.0, y_m=0.0),
    )
    reasons = {e.reason for e in events}
    assert reasons == {"recipient_velocity", "zone_intrusion"}


def test_reset_clears_all_track_history(cfg):
    mon = MotionMonitor(cfg)
    track_id = None
    for i in range(cfg.min_track_frames):
        [d] = mon.update([_person(float(i), 0.0, stamp=float(i))])
        track_id = d.track_id
    assert mon.velocity_mps(track_id) is not None  # sanity: real history exists

    mon.reset()
    assert mon.velocity_mps(track_id) is None  # history wiped, not just the counter

    [d2] = mon.update([_person(0.0, 0.0, stamp=0.0)])
    assert d2.track_id == 1  # id counter also restarted, not continuing from before
