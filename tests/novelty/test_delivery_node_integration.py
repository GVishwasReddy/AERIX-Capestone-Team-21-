"""Integration tests: DeliveryNode driving MissionFSM end to end against a
real MessageBus, with scripted sensor "tapes" (published messages) standing
in for perception/BLE/telemetry - no Hailo, no MAVLink, no real sim
physics. These are the project plan's seven scripted scenarios (see the
plan's own Verification section): exact terminal states, not just "it ran".

Every scenario drives ``node.step(now=...)`` with an explicit, monotonically
advancing synthetic clock rather than real ``time.sleep`` - see
``DeliveryNode.step``'s own docstring on why every event/timeout check is
threaded through that one clock.

Two scenarios (recipient walks off / intruder) assert against the
EvidenceLogger's own JSONL flight log rather than the FSM's live state,
because ``ABORT_DESCENT`` is a transient state (per mission_fsm.py's own
TRANSITIONS table comment: "evaluated immediately on entry") - by the time
``step()`` returns, the FSM has already resolved onward to a replan or
``ABORT_RTL``. The flight log is the correct, already-available place to
prove ``DESCENDING -> ABORT_DESCENT`` genuinely fired.
"""
from __future__ import annotations

import numpy as np
import pytest

from drone_stack.bus import MessageBus
from drone_stack.bus.topics import Topics
from drone_stack.msg import FusedState
from drone_stack.novelty import delivery_node as dn_mod
from drone_stack.novelty.config import NoveltyConfig
from drone_stack.novelty.delivery_node import DeliveryNode
from drone_stack.novelty.mission_fsm import MissionState
from drone_stack.novelty.topics import NoveltyTopics
from drone_stack.novelty.types import (
    BleAuthEvent,
    GroundPoint,
    PersonDetection,
    PixelBox,
    SegmentationFrame,
    TerrainClass,
)
from drone_stack.utils.config import Config

#: Large enough that, at a plausible descent altitude (~10 m) through the
#: shipped test camera intrinsics (see conftest.py's MODELS fixture), the
#: rasterized all-grass frame yields a connected safe component comfortably
#: above min_safe_radius_m and within the r_min_m..r_max_m band - verified
#: empirically against the real FlatEarthPinhole formula while writing this
#: test (not asserted by geometry derivation, deliberately, per
#: test_landing_zone.py's own note that coupling formula tests to these
#: GUESSED intrinsics is undesirable - this module cares about FSM
#: sequencing, not projector correctness).
_TERRAIN_H, _TERRAIN_W = 720, 1280


@pytest.fixture
def node(valid_config_dir, tmp_path):
    novelty_cfg = NoveltyConfig.load(valid_config_dir)
    core_cfg = Config.load().with_overrides(
        novelty={"flight_logs_dir": str(tmp_path / "flight_logs"), "rate_hz": 1000.0}
    )
    n = DeliveryNode(MessageBus(), core_cfg, novelty_config=novelty_cfg)
    yield n
    n.evidence.close()


def _all_grass_terrain() -> SegmentationFrame:
    return SegmentationFrame(
        class_indices=np.zeros((_TERRAIN_H, _TERRAIN_W), dtype=np.int8),
        index_to_class={0: TerrainClass.GRASS},
    )


def _person(ground: GroundPoint, stamp: float) -> PersonDetection:
    return PersonDetection(bbox=PixelBox(600, 340, 680, 380), score=0.9, ground=ground, stamp=stamp)


def _confirm_single_recipient(node: DeliveryNode, now: float) -> tuple[PersonDetection, float]:
    """Drive SEARCHING_PERSON -> PERSON_FOUND -> SEARCHING_ZONE with one
    unambiguous person at the origin. Returns (detection, now)."""
    person = _person(GroundPoint(x_m=0.0, y_m=0.0), stamp=now)
    node.bus.publish(Topics.FUSED_STATE, FusedState(alt_rel_m=10.0, lat=0.0, lon=0.0, yaw=0.0))
    node.bus.publish(NoveltyTopics.PERSON_DETECTIONS, [person])

    node.step(now=now)
    now += 0.5
    assert node.fsm.state == MissionState.PERSON_FOUND

    node.step(now=now)
    now += 0.5
    assert node.fsm.state == MissionState.SEARCHING_ZONE
    assert node.fsm.ctx["recipient_track_id"] is not None
    return person, now


def _reach_zone_found(node: DeliveryNode, now: float) -> float:
    node.bus.publish(NoveltyTopics.TERRAIN_MAP, _all_grass_terrain())
    node.step(now=now)
    now += 0.5
    assert node.fsm.state == MissionState.ZONE_FOUND
    return now


def _reach_descending(node: DeliveryNode, now: float) -> float:
    node.step(now=now)
    now += 0.5
    assert node.fsm.state == MissionState.DESCENDING
    assert node._chosen_zone is not None
    return now


def _reach_authenticating(node: DeliveryNode, now: float, person: PersonDetection) -> float:
    for alt in (8.0, 6.0, 4.0, 2.0, 1.5):
        node.bus.publish(Topics.FUSED_STATE, FusedState(alt_rel_m=alt, lat=0.0, lon=0.0, yaw=0.0))
        person.stamp = now
        node.bus.publish(NoveltyTopics.PERSON_DETECTIONS, [person])
        node.step(now=now)
        now += 0.5
    assert node.fsm.state == MissionState.AUTHENTICATING
    return now


# --------------------------------------------------------------------------- #
# Scenario 1 - happy path: RELEASING -> ASCENDING -> RTL
# --------------------------------------------------------------------------- #
def test_happy_path_reaches_rtl(node, monkeypatch):
    monkeypatch.setattr(dn_mod, "_RELEASE_SETTLE_S", 0.0)

    now = 1_000.0
    person, now = _confirm_single_recipient(node, now)
    now = _reach_zone_found(node, now)
    now = _reach_descending(node, now)
    now = _reach_authenticating(node, now, person)

    node.bus.publish(
        NoveltyTopics.BLE_AUTH_EVENT,
        BleAuthEvent(authenticated=True, phone_gps=(0.0, 0.0), stamp=now),
    )
    person.stamp = now
    node.bus.publish(NoveltyTopics.PERSON_DETECTIONS, [person])
    node.step(now=now)
    now += 0.5
    assert node.fsm.state == MissionState.RELEASING

    node.step(now=now)  # _RELEASE_SETTLE_S patched to 0.0 - completes immediately
    now += 0.5
    assert node.fsm.state == MissionState.ASCENDING

    for alt in (3.0, 5.0, 7.0, 8.5):
        node.bus.publish(Topics.FUSED_STATE, FusedState(alt_rel_m=alt, lat=0.0, lon=0.0, yaw=0.0))
        node.step(now=now)
        now += 0.5

    assert node.fsm.state == MissionState.RTL


# --------------------------------------------------------------------------- #
# Scenario 2 - no landing zone ever found -> ABORT_RTL via radius expansion
# --------------------------------------------------------------------------- #
def test_no_zone_found_aborts_to_rtl(node):
    now = 1_000.0
    _person_obj, now = _confirm_single_recipient(node, now)
    # Deliberately never publish a terrain frame - SEARCHING_ZONE can never
    # find a candidate, so it must repeatedly time out into
    # EXPANDING_SEARCH_RADIUS until max_search_radius_m is exceeded.
    assert node.fsm.state == MissionState.SEARCHING_ZONE

    timeout = node.cfg.mission_fsm.state_timeout_s["SEARCHING_ZONE"]
    for _ in range(int(node.cfg.mission_fsm.max_search_radius_m / node.cfg.mission_fsm.search_radius_expansion_m) + 2):
        now += timeout + 1.0
        node.step(now=now)
        if node.fsm.state == MissionState.ABORT_RTL:
            break

    assert node.fsm.state == MissionState.ABORT_RTL


# --------------------------------------------------------------------------- #
# Scenario 3 - BLE channel never confirms -> ABORT_RTL
# --------------------------------------------------------------------------- #
def test_ble_channel_failure_aborts_to_rtl(node):
    now = 1_000.0
    person, now = _confirm_single_recipient(node, now)
    now = _reach_zone_found(node, now)
    now = _reach_descending(node, now)
    now = _reach_authenticating(node, now, person)
    # No BLE event is ever published - vision is ok, BLE never is.

    ble_timeout = node.cfg.recipient_auth.ble_retry_timeout_s
    for _ in range(8):
        now += ble_timeout / 3.0
        person.stamp = now
        node.bus.publish(NoveltyTopics.PERSON_DETECTIONS, [person])
        node.step(now=now)
        if node.fsm.state == MissionState.ABORT_RTL:
            break

    assert node.fsm.state == MissionState.ABORT_RTL


# --------------------------------------------------------------------------- #
# Scenario 4 - BLE and vision positions never agree -> ABORT_RTL
# --------------------------------------------------------------------------- #
def test_position_disagreement_aborts_to_rtl(node):
    now = 1_000.0
    person, now = _confirm_single_recipient(node, now)
    now = _reach_zone_found(node, now)
    now = _reach_descending(node, now)
    now = _reach_authenticating(node, now, person)

    # ~111 m away at the equator - far beyond max_position_disagreement_m.
    disagree_timeout = node.cfg.recipient_auth.disagreement_retry_timeout_s
    for _ in range(8):
        now += disagree_timeout / 3.0
        person.stamp = now
        node.bus.publish(NoveltyTopics.PERSON_DETECTIONS, [person])
        node.bus.publish(
            NoveltyTopics.BLE_AUTH_EVENT,
            BleAuthEvent(authenticated=True, phone_gps=(0.001, 0.001), stamp=now),
        )
        node.step(now=now)
        if node.fsm.state == MissionState.ABORT_RTL:
            break

    assert node.fsm.state == MissionState.ABORT_RTL


# --------------------------------------------------------------------------- #
# Scenario 5 - ambiguous pair -> HOVER_AND_RETRY -> ABORT_RTL
# --------------------------------------------------------------------------- #
def test_ambiguous_pair_hovers_then_aborts(node):
    now = 1_000.0
    person, now = _confirm_single_recipient(node, now)
    now = _reach_zone_found(node, now)
    now = _reach_descending(node, now)
    now = _reach_authenticating(node, now, person)

    # A second, equally-unsupported candidate appears (no BLE event at all,
    # so both candidates score exactly 0.0 - a guaranteed tie).
    bystander = _person(GroundPoint(x_m=20.0, y_m=20.0), stamp=now)
    node.bus.publish(NoveltyTopics.PERSON_DETECTIONS, [person, bystander])
    node.step(now=now)
    now += 0.5
    assert node.fsm.state == MissionState.HOVER_AND_RETRY

    hover_timeout = node.cfg.mission_fsm.state_timeout_s["HOVER_AND_RETRY"]
    for _ in range(6):
        now += hover_timeout / 2.0
        person.stamp = now
        bystander.stamp = now
        node.bus.publish(NoveltyTopics.PERSON_DETECTIONS, [person, bystander])
        node.step(now=now)
        if node.fsm.state == MissionState.ABORT_RTL:
            break

    assert node.fsm.state == MissionState.ABORT_RTL


# --------------------------------------------------------------------------- #
# Scenario 6 - recipient walks off during descent -> ABORT_DESCENT
# --------------------------------------------------------------------------- #
def test_recipient_motion_triggers_abort_descent(node):
    now = 1_000.0
    _person_obj, now = _confirm_single_recipient(node, now)
    now = _reach_zone_found(node, now)
    now = _reach_descending(node, now)

    node.motion.reset()  # clean velocity window - matches test_motion_monitor.py's own pattern
    speed = (
        node.cfg.motion_monitor.max_recipient_velocity_mps
        + node.cfg.motion_monitor.max_association_distance_m
    ) / 2.0

    state_after = node.fsm.state
    for i in range(6):
        moving = _person(GroundPoint(x_m=speed * i, y_m=0.0), stamp=now)
        node.bus.publish(Topics.FUSED_STATE, FusedState(alt_rel_m=7.0, lat=0.0, lon=0.0, yaw=0.0))
        node.bus.publish(NoveltyTopics.PERSON_DETECTIONS, [moving])
        node.step(now=now)
        now += 1.0
        state_after = node.fsm.state
        if state_after != MissionState.DESCENDING:
            break

    assert state_after == MissionState.ABORT_RTL
    log_text = node.evidence.path.read_text(encoding="utf-8")
    assert "DESCENDING -> ABORT_DESCENT" in log_text


# --------------------------------------------------------------------------- #
# Scenario 7 - a bystander enters the landing zone during descent -> ABORT_DESCENT
# --------------------------------------------------------------------------- #
def test_intruder_in_landing_zone_triggers_abort_descent(node):
    now = 1_000.0
    person, now = _confirm_single_recipient(node, now)
    now = _reach_zone_found(node, now)
    now = _reach_descending(node, now)
    centroid = node._chosen_zone.centroid  # altitude stays 10 m > abort_altitude_ceiling_m,
    # so the recipient's own (stationary) velocity can never trigger the
    # OTHER abort condition - this isolates the zone-intrusion trigger.

    intruder = _person(GroundPoint(x_m=centroid.x_m, y_m=centroid.y_m), stamp=now)
    person.stamp = now
    node.bus.publish(NoveltyTopics.PERSON_DETECTIONS, [person, intruder])
    node.step(now=now)

    assert node.fsm.state == MissionState.ABORT_RTL
    log_text = node.evidence.path.read_text(encoding="utf-8")
    assert "DESCENDING -> ABORT_DESCENT" in log_text
