"""novelty/recipient_auth.py - §2.2 dual-factor fusion + §2.5 disambiguation.

A mock harness drives BLE + vision inputs through every named branch (see
the module's own docstring): both-ok, A-only, B-only, disagree, timeout,
sync-window miss, plus the two defensive edge cases (no channels active,
BLE authenticated with no position signal at all).
"""
from __future__ import annotations

import math

import pytest

from drone_stack.msg.messages import FusedState
from drone_stack.novelty.config import NoveltyConfig
from drone_stack.novelty.recipient_auth import (
    DualFactorAuthenticator,
    _phone_gps_ground_point,
    _rssi_range_m,
    disambiguate_recipients,
)
from drone_stack.novelty.types import BleAuthEvent, GroundPoint, PersonDetection, PixelBox
from drone_stack.utils.geometry import enu_to_geodetic


@pytest.fixture
def cfg(valid_config_dir):
    return NoveltyConfig.load(valid_config_dir).recipient_auth


@pytest.fixture
def own_state():
    return FusedState(lat=0.0, lon=0.0, yaw=0.0, valid=True)


def _person(ground: GroundPoint | None, track_id: int | None = None, stamp: float = 0.0) -> PersonDetection:
    return PersonDetection(bbox=PixelBox(0, 0, 10, 10), score=0.9, ground=ground, track_id=track_id, stamp=stamp)


def _ble(authenticated: bool, rssi_dbm: float | None = None,
         phone_gps: tuple[float, float] | None = None, stamp: float = 0.0) -> BleAuthEvent:
    return BleAuthEvent(authenticated=authenticated, rssi_dbm=rssi_dbm, phone_gps=phone_gps, stamp=stamp)


def _rssi_for_range(cfg, range_m: float) -> float:
    """Invert the path-loss model to find the rssi_dbm that implies range_m,
    so tests can construct agree/disagree scenarios from a target distance."""
    return cfg.rssi.tx_power_dbm - 10.0 * cfg.rssi.path_loss_exponent * math.log10(range_m)


# --------------------------------------------------------------------------- #
# _phone_gps_ground_point - the ENU -> body-frame rotation, verified against
# drone_stack/sim/world.py's own body->ENU rotation (see module docstring).
# --------------------------------------------------------------------------- #
def test_phone_gps_ground_point_yaw_zero_forward_is_east():
    own = FusedState(lat=0.0, lon=0.0, yaw=0.0, valid=True)
    phone_gps = enu_to_geodetic(east=3.0, north=0.5, ref_lat=0.0, ref_lon=0.0)
    ground = _phone_gps_ground_point(phone_gps, own)
    assert ground.x_m == pytest.approx(3.0, abs=1e-6)
    assert ground.y_m == pytest.approx(0.5, abs=1e-6)


def test_phone_gps_ground_point_yaw_90deg_forward_is_north():
    own = FusedState(lat=0.0, lon=0.0, yaw=math.pi / 2, valid=True)
    phone_gps = enu_to_geodetic(east=0.0, north=4.0, ref_lat=0.0, ref_lon=0.0)
    ground = _phone_gps_ground_point(phone_gps, own)
    # Facing North (yaw=90deg CCW from East): forward=north, left=-east.
    assert ground.x_m == pytest.approx(4.0, abs=1e-6)
    assert ground.y_m == pytest.approx(0.0, abs=1e-6)


# --------------------------------------------------------------------------- #
# Branch 1: both-ok, agree -> released
# --------------------------------------------------------------------------- #
def test_both_ok_and_positions_agree_releases(cfg, own_state):
    range_m = 5.0
    rssi = _rssi_for_range(cfg, range_m)
    ble = _ble(authenticated=True, rssi_dbm=rssi, stamp=100.0)
    vision = _person(GroundPoint(x_m=range_m, y_m=0.0), track_id=7, stamp=100.0)

    auth = DualFactorAuthenticator(cfg)
    decision = auth.evaluate(ble, vision, own_state, now=100.0)

    assert decision.released is True
    assert decision.reason == "both_channels_confirmed"
    assert decision.ble_ok is True and decision.vision_ok is True
    assert decision.recipient_track_id == 7
    assert decision.position_disagreement_m == pytest.approx(0.0, abs=1e-6)


# --------------------------------------------------------------------------- #
# Branch 2: A-only (BLE ok, vision not) -> awaiting, then timeout
# --------------------------------------------------------------------------- #
def test_a_only_awaits_vision_then_times_out(cfg, own_state):
    ble = _ble(authenticated=True, rssi_dbm=-60.0, stamp=0.0)
    auth = DualFactorAuthenticator(cfg)

    d1 = auth.evaluate(ble, None, own_state, now=0.0)
    assert d1.released is False
    assert d1.reason == "awaiting_vision"
    assert d1.ble_ok is True and d1.vision_ok is False

    d2 = auth.evaluate(ble, None, own_state, now=cfg.vision_retry_timeout_s + 1.0)
    assert d2.released is False
    assert d2.reason == "vision_timeout"


# --------------------------------------------------------------------------- #
# Branch 3: B-only (vision ok, BLE not) -> awaiting, then timeout
# --------------------------------------------------------------------------- #
def test_b_only_awaits_ble_then_times_out(cfg, own_state):
    vision = _person(GroundPoint(x_m=3.0, y_m=0.0), track_id=2, stamp=0.0)
    auth = DualFactorAuthenticator(cfg)

    d1 = auth.evaluate(None, vision, own_state, now=0.0)
    assert d1.released is False
    assert d1.reason == "awaiting_ble"
    assert d1.ble_ok is False and d1.vision_ok is True
    assert d1.recipient_track_id == 2

    d2 = auth.evaluate(None, vision, own_state, now=cfg.ble_retry_timeout_s + 1.0)
    assert d2.reason == "ble_timeout"


# --------------------------------------------------------------------------- #
# Branch 4: disagree -> retried, then timeout
# --------------------------------------------------------------------------- #
def test_disagree_then_times_out(cfg, own_state):
    range_m = 5.0
    rssi = _rssi_for_range(cfg, range_m)
    # Vision reports a person far enough away to exceed max_position_disagreement_m_rssi_only.
    bad_range = range_m + cfg.max_position_disagreement_m_rssi_only + 5.0
    ble = _ble(authenticated=True, rssi_dbm=rssi, stamp=0.0)
    vision = _person(GroundPoint(x_m=bad_range, y_m=0.0), stamp=0.0)

    auth = DualFactorAuthenticator(cfg)
    d1 = auth.evaluate(ble, vision, own_state, now=0.0)
    assert d1.released is False
    assert d1.reason == "position_disagreement"
    assert d1.position_disagreement_m > cfg.max_position_disagreement_m_rssi_only

    d2 = auth.evaluate(ble, vision, own_state, now=cfg.disagreement_retry_timeout_s + 1.0)
    assert d2.reason == "position_disagreement_timeout"


# --------------------------------------------------------------------------- #
# Branch 5 (implicit): timeout is exercised above (vision_timeout / ble_timeout
# / position_disagreement_timeout) - each retry-window's terminal outcome.
# --------------------------------------------------------------------------- #

# --------------------------------------------------------------------------- #
# Branch 6: sync-window miss
# --------------------------------------------------------------------------- #
def test_sync_window_miss(cfg, own_state):
    range_m = 5.0
    rssi = _rssi_for_range(cfg, range_m)
    ble = _ble(authenticated=True, rssi_dbm=rssi, stamp=0.0)
    gap_s = (cfg.sync_window_ms / 1000.0) + 5.0
    vision = _person(GroundPoint(x_m=range_m, y_m=0.0), stamp=gap_s)

    auth = DualFactorAuthenticator(cfg)
    decision = auth.evaluate(ble, vision, own_state, now=gap_s)
    assert decision.released is False
    assert decision.reason == "sync_window_exceeded"


# --------------------------------------------------------------------------- #
# Defensive cases (beyond the six brief-numbered branches)
# --------------------------------------------------------------------------- #
def test_no_channels_active(cfg, own_state):
    auth = DualFactorAuthenticator(cfg)
    decision = auth.evaluate(None, None, own_state, now=0.0)
    assert decision.released is False
    assert decision.reason == "no_channels_active"
    assert decision.ble_ok is False and decision.vision_ok is False


def test_ble_authenticated_with_no_position_signal(cfg, own_state):
    ble = _ble(authenticated=True, rssi_dbm=None, phone_gps=None, stamp=0.0)
    vision = _person(GroundPoint(x_m=3.0, y_m=0.0), stamp=0.0)

    auth = DualFactorAuthenticator(cfg)
    decision = auth.evaluate(ble, vision, own_state, now=0.0)
    assert decision.released is False
    assert decision.reason == "ble_position_unavailable"


# --------------------------------------------------------------------------- #
# Channel-drop resets the retry clock (regression: must not fire "timeout"
# using a stale first-seen time from a much earlier, unrelated ok period).
# --------------------------------------------------------------------------- #
def test_channel_recovery_resets_retry_clock(cfg, own_state):
    ble = _ble(authenticated=True, rssi_dbm=-60.0, stamp=0.0)
    auth = DualFactorAuthenticator(cfg)

    auth.evaluate(ble, None, own_state, now=0.0)  # BLE first seen ok at t=0
    # BLE drops out entirely.
    auth.evaluate(None, None, own_state, now=5.0)
    # BLE reappears at t=6 - its retry clock must restart from 6, not 0.
    ble_again = _ble(authenticated=True, rssi_dbm=-60.0, stamp=6.0)
    decision = auth.evaluate(ble_again, None, own_state, now=6.0 + cfg.vision_retry_timeout_s - 1.0)
    assert decision.reason == "awaiting_vision"  # not yet timed out from the NEW start time


# --------------------------------------------------------------------------- #
# §2.5 disambiguation
# --------------------------------------------------------------------------- #
def test_disambiguation_no_candidates(cfg, own_state):
    result = disambiguate_recipients([], None, own_state, cfg)
    assert result.winner is None
    assert result.reason == "no_candidates"
    assert result.scores == []


def test_disambiguation_single_candidate_is_unambiguous(cfg, own_state):
    person = _person(GroundPoint(x_m=3.0, y_m=0.0), track_id=1)
    result = disambiguate_recipients([person], None, own_state, cfg)
    assert result.winner is person
    assert result.reason == "unambiguous_single_candidate"


def test_disambiguation_clear_margin_picks_closer_candidate(cfg, own_state):
    range_m = 5.0
    rssi = _rssi_for_range(cfg, range_m)
    ble = _ble(authenticated=True, rssi_dbm=rssi)

    close = _person(GroundPoint(x_m=range_m, y_m=0.0), track_id=1)      # matches BLE range exactly
    far = _person(GroundPoint(x_m=range_m + 50.0, y_m=0.0), track_id=2)  # way off

    result = disambiguate_recipients([far, close], ble, own_state, cfg)

    assert result.reason == "margin_met"
    assert result.winner is close
    assert result.scores[0].track_id == 1
    assert result.scores[0].likelihood > result.scores[1].likelihood


def test_disambiguation_within_margin_is_ambiguous(cfg, own_state):
    range_m = 5.0
    rssi = _rssi_for_range(cfg, range_m)
    ble = _ble(authenticated=True, rssi_dbm=rssi)

    # Both candidates equidistant from the BLE-implied range -> identical scores.
    a = _person(GroundPoint(x_m=range_m, y_m=0.0), track_id=1)
    b = _person(GroundPoint(x_m=0.0, y_m=range_m), track_id=2)

    result = disambiguate_recipients([a, b], ble, own_state, cfg)

    assert result.winner is None
    assert result.reason == "margin_not_met"


def test_disambiguation_scores_sorted_descending_by_likelihood(cfg, own_state):
    range_m = 5.0
    rssi = _rssi_for_range(cfg, range_m)
    ble = _ble(authenticated=True, rssi_dbm=rssi)

    candidates = [
        _person(GroundPoint(x_m=range_m + 40.0, y_m=0.0), track_id=1),
        _person(GroundPoint(x_m=range_m, y_m=0.0), track_id=2),
        _person(GroundPoint(x_m=range_m + 20.0, y_m=0.0), track_id=3),
    ]
    result = disambiguate_recipients(candidates, ble, own_state, cfg)

    likelihoods = [s.likelihood for s in result.scores]
    assert likelihoods == sorted(likelihoods, reverse=True)
    assert result.scores[0].track_id == 2


def test_disambiguation_motion_cue_is_a_documented_noop_at_shipped_gamma(cfg, own_state):
    """gamma_motion_cue ships as 0.0 (conftest.RECIPIENT_AUTH) - a huge
    motion_cue_scores value must NOT change the ranking."""
    assert cfg.disambiguation_weights.gamma_motion_cue == 0.0

    range_m = 5.0
    rssi = _rssi_for_range(cfg, range_m)
    ble = _ble(authenticated=True, rssi_dbm=rssi)
    a = _person(GroundPoint(x_m=range_m, y_m=0.0), track_id=1)
    b = _person(GroundPoint(x_m=range_m + 30.0, y_m=0.0), track_id=2)

    baseline = disambiguate_recipients([a, b], ble, own_state, cfg)
    boosted = disambiguate_recipients([a, b], ble, own_state, cfg, motion_cue_scores={2: 1.0})

    assert baseline.winner is a
    assert boosted.winner is a
    assert baseline.scores[0].likelihood == pytest.approx(boosted.scores[0].likelihood)


def test_disambiguation_candidate_without_ground_raises(cfg, own_state):
    ungrounded = _person(None, track_id=1)
    with pytest.raises(ValueError, match="ground-projected"):
        disambiguate_recipients([ungrounded], None, own_state, cfg)


def test_rssi_range_m_matches_inverse_helper(cfg):
    """Sanity check: the test harness's own inverse-of-path-loss helper
    round-trips through the module's forward formula."""
    d = _rssi_range_m(_rssi_for_range(cfg, 8.0), cfg.rssi)
    assert d == pytest.approx(8.0, rel=1e-6)
