"""§2.2 / §2.5 - recipient authentication: dual-factor release gate, then
multi-person disambiguation.

Two independent, physically unrelated channels must agree before a parcel
may release - a single spoofed/failed channel can never authorize release on
its own (see the project brief, §2.2 and §7 "why two channels"):

* **BLE channel** - ``BleAuthEvent`` from the (re-gated) BLE peripheral.
  Carries NO release power itself; it only reports whether the handshake
  succeeded (``authenticated``), the received signal strength
  (``rssi_dbm``), and - if the phone app sent one - a GPS fix
  (``phone_gps``).
* **Vision channel** - a ground-projected ``PersonDetection`` from the
  person-detector model (today: ``yolov8n.hef``; see
  ``perception/adapters.py``).

## §2.2 - dual-factor fusion (``DualFactorAuthenticator``)

Six branches, driven by ``config/novelty/recipient_auth.yaml``:

1. **both-ok** - BLE authenticated AND a person is vision-detected AND their
   estimated ground positions agree within tolerance -> ``released=True``.
2. **A-only** (BLE ok, vision not yet) - wait up to ``vision_retry_timeout_s``.
3. **B-only** (vision ok, BLE not yet) - wait up to ``ble_retry_timeout_s``.
4. **disagree** - both individually ok but positions disagree - wait up to
   ``disagreement_retry_timeout_s`` for the disagreement to resolve (e.g.
   transient GPS jitter) before treating it as a hard failure.
5. **timeout** - the retry window in branch 2, 3, or 4 is exceeded -> a
   terminal, non-released decision (the FSM reads this as "abort", not
   "keep waiting").
6. **sync-window miss** - both channels are individually ok and positions
   agree, but their timestamps are more than ``sync_window_ms`` apart -> not
   accepted as the same real-world event.

Two further **defensive** (not brief-numbered) cases exist because the
config schema allows them even though they should be rare in practice:
neither channel active (``no_channels_active``), and BLE "authenticated"
with no position signal at all - no ``phone_gps`` and no ``rssi_dbm``
(``ble_position_unavailable``) - both are `released=False`, never a silent
release.

## §2.5 - multi-person disambiguation (``disambiguate_recipients``)

When vision detects more than one person, each candidate is scored:

    likelihood(person) = alpha_ble_position    * position_score(person)
                        + beta_rssi_consistency * rssi_consistency_score(person)
                        + gamma_motion_cue      * motion_cue_score(person)

- ``position_score`` = ``clamp(1 - dist_m / max_position_disagreement_m, 0, 1)``
  if the BLE event carries ``phone_gps``, else ``0.0`` (no full 2D fix ->
  no evidence either way, not a free pass).
- ``rssi_consistency_score`` = ``clamp(1 - |range_diff_m| / max_position_disagreement_m_rssi_only, 0, 1)``
  if the BLE event carries ``rssi_dbm``, else ``0.0``.
- ``motion_cue_score`` is a caller-supplied, per-``track_id`` hook (§2.4's
  ``motion_monitor.py`` is the natural future source of "walking toward the
  drone" style cues) - it defaults to ``0.0`` for every candidate today.
  ``gamma_motion_cue`` ships as ``0.0`` in ``recipient_auth.yaml`` (see that
  file's own comment), so this term does not yet affect ranking - a
  documented no-op, not a missing feature silently doing nothing.

**Margin rule.** The top-scoring candidate is only accepted
(``DisambiguationResult.winner``) if it beats the runner-up by at least
``disambiguation_margin``; otherwise the pass returns ``winner=None`` and the
mission FSM's ``HOVER_AND_RETRY`` state re-observes rather than committing to
a guess between two similarly-likely people.

## BLE position fusion (shared by both features above)

Locked design decision: BLE's own ground-position estimate is **fused**,
phone GPS when available, RSSI-only range as a fallback:

- With ``phone_gps`` (lat, lon): converted to a body-relative
  ``GroundPoint`` via ``drone_stack.utils.geometry.geodetic_to_enu`` (ENU:
  x=East, y=North) followed by the ENU -> body-frame (x=forward, y=left)
  rotation ``FusedState.yaw`` already uses elsewhere in this stack (verified
  against ``drone_stack/sim/world.py``'s body->ENU rotation
  ``vx=bvx*cos(yaw)-bvy*sin(yaw), vy=bvx*sin(yaw)+bvy*cos(yaw)`` - yaw is
  CCW-positive from the ENU +x/East axis, per that file's own "our ENU yaw
  is CCW-positive" comment - so this module applies that rotation's
  inverse). This gives a full 2D position, comparable directly to vision's
  ``GroundPoint`` with ``max_position_disagreement_m``.
- With only ``rssi_dbm``: the log-distance path-loss model in
  ``config.RssiPathLoss`` gives a **scalar range only** (no bearing), so it
  can only be compared against vision's own range-from-drone, using the
  wider ``max_position_disagreement_m_rssi_only`` (RSSI ranging is coarse
  and worse with body blocking - see the project brief's risk list).
"""
from __future__ import annotations

import math

from drone_stack.msg.messages import FusedState
from drone_stack.novelty.config import RecipientAuthConfig, RssiPathLoss
from drone_stack.novelty.types import (
    AuthDecision,
    BleAuthEvent,
    DisambiguationResult,
    DisambiguationScore,
    GroundPoint,
    PersonDetection,
)
from drone_stack.utils.geometry import geodetic_to_enu


def _clamp01(value: float) -> float:
    return max(0.0, min(1.0, value))


def _phone_gps_ground_point(phone_gps: tuple[float, float], own_state: FusedState) -> GroundPoint:
    """Convert a phone-reported (lat, lon) fix to a body-relative
    ``GroundPoint`` using the drone's own fused global position + yaw."""
    lat, lon = phone_gps
    east_m, north_m = geodetic_to_enu(lat, lon, own_state.lat, own_state.lon)
    cos_y, sin_y = math.cos(own_state.yaw), math.sin(own_state.yaw)
    forward_m = east_m * cos_y + north_m * sin_y
    left_m = -east_m * sin_y + north_m * cos_y
    return GroundPoint(x_m=forward_m, y_m=left_m)


def _rssi_range_m(rssi_dbm: float, rssi_cfg: RssiPathLoss) -> float:
    """Log-distance path-loss range estimate - see ``config.RssiPathLoss``."""
    return 10.0 ** ((rssi_cfg.tx_power_dbm - rssi_dbm) / (10.0 * rssi_cfg.path_loss_exponent))


def _range_from_drone_m(ground: GroundPoint) -> float:
    return math.hypot(ground.x_m, ground.y_m)


def _ble_disagreement_m(
    ble_event: BleAuthEvent,
    vision_ground: GroundPoint,
    own_state: FusedState,
    cfg: RecipientAuthConfig,
) -> tuple[float | None, float | None]:
    """Returns ``(disagreement_m, threshold_m)``, or ``(None, None)`` if the
    BLE event carries no position signal at all to compare against."""
    if ble_event.phone_gps is not None:
        ble_ground = _phone_gps_ground_point(ble_event.phone_gps, own_state)
        return ble_ground.distance_to(vision_ground), cfg.max_position_disagreement_m
    if ble_event.rssi_dbm is not None:
        ble_range = _rssi_range_m(ble_event.rssi_dbm, cfg.rssi)
        return abs(ble_range - _range_from_drone_m(vision_ground)), cfg.max_position_disagreement_m_rssi_only
    return None, None


# --------------------------------------------------------------------------- #
# §2.2 - dual-factor fusion
# --------------------------------------------------------------------------- #
class DualFactorAuthenticator:
    """Stateful §2.2 release gate - call :meth:`evaluate` once per frame with
    the latest BLE + vision channel state. Tracks per-channel "first seen ok"
    times against ``RecipientAuthConfig``'s retry timeouts internally, so the
    caller never has to manage that bookkeeping itself."""

    def __init__(self, cfg: RecipientAuthConfig) -> None:
        self._cfg = cfg
        self._ble_ok_since: float | None = None
        self._vision_ok_since: float | None = None
        self._disagree_since: float | None = None

    def reset(self) -> None:
        """Clear all timing state - call when starting a new recipient
        handoff attempt (e.g. re-entering the FSM's authentication state)."""
        self._ble_ok_since = None
        self._vision_ok_since = None
        self._disagree_since = None

    def evaluate(
        self,
        ble_event: BleAuthEvent | None,
        vision_detection: PersonDetection | None,
        own_state: FusedState,
        now: float | None = None,
    ) -> AuthDecision:
        import time as _time

        now = now if now is not None else _time.time()
        cfg = self._cfg

        ble_ok = ble_event is not None and ble_event.authenticated
        vision_ok = vision_detection is not None and vision_detection.ground is not None

        self._ble_ok_since = now if (ble_ok and self._ble_ok_since is None) else (
            self._ble_ok_since if ble_ok else None
        )
        self._vision_ok_since = now if (vision_ok and self._vision_ok_since is None) else (
            self._vision_ok_since if vision_ok else None
        )

        if ble_ok and vision_ok:
            sync_gap_ms = abs(ble_event.stamp - vision_detection.stamp) * 1000.0
            if sync_gap_ms > cfg.sync_window_ms:
                self._disagree_since = None
                return AuthDecision(
                    released=False, reason="sync_window_exceeded",
                    ble_ok=True, vision_ok=True,
                    recipient_track_id=vision_detection.track_id, stamp=now,
                )

            disagreement, threshold = _ble_disagreement_m(
                ble_event, vision_detection.ground, own_state, cfg
            )
            if disagreement is None:
                self._disagree_since = None
                return AuthDecision(
                    released=False, reason="ble_position_unavailable",
                    ble_ok=True, vision_ok=True,
                    recipient_track_id=vision_detection.track_id, stamp=now,
                )

            if disagreement > threshold:
                if self._disagree_since is None:
                    self._disagree_since = now
                elapsed = now - self._disagree_since
                reason = (
                    "position_disagreement_timeout"
                    if elapsed > cfg.disagreement_retry_timeout_s
                    else "position_disagreement"
                )
                return AuthDecision(
                    released=False, reason=reason, ble_ok=True, vision_ok=True,
                    position_disagreement_m=disagreement,
                    recipient_track_id=vision_detection.track_id, stamp=now,
                )

            self._disagree_since = None
            return AuthDecision(
                released=True, reason="both_channels_confirmed",
                ble_ok=True, vision_ok=True, position_disagreement_m=disagreement,
                recipient_track_id=vision_detection.track_id, stamp=now,
            )

        self._disagree_since = None  # disagreement only meaningful while both are ok

        if ble_ok and not vision_ok:
            elapsed = now - self._ble_ok_since
            reason = "vision_timeout" if elapsed > cfg.vision_retry_timeout_s else "awaiting_vision"
            return AuthDecision(released=False, reason=reason, ble_ok=True, vision_ok=False, stamp=now)

        if vision_ok and not ble_ok:
            elapsed = now - self._vision_ok_since
            reason = "ble_timeout" if elapsed > cfg.ble_retry_timeout_s else "awaiting_ble"
            return AuthDecision(
                released=False, reason=reason, ble_ok=False, vision_ok=True,
                recipient_track_id=vision_detection.track_id, stamp=now,
            )

        return AuthDecision(released=False, reason="no_channels_active", ble_ok=False, vision_ok=False, stamp=now)


# --------------------------------------------------------------------------- #
# §2.5 - multi-person disambiguation
# --------------------------------------------------------------------------- #
def disambiguate_recipients(
    candidates: list[PersonDetection],
    ble_event: BleAuthEvent | None,
    own_state: FusedState,
    cfg: RecipientAuthConfig,
    motion_cue_scores: dict[int, float] | None = None,
) -> DisambiguationResult:
    """Score every vision candidate against the BLE channel and pick the one
    recipient, if any candidate is unambiguously ahead (see module
    docstring's "margin rule"). Every ``candidates`` entry must already be
    ground-projected (``.ground is not None``) - that is the
    ``GroundProjector``'s job, not this function's."""
    motion_cue_scores = motion_cue_scores or {}

    if not candidates:
        return DisambiguationResult(winner=None, scores=[], reason="no_candidates")

    for c in candidates:
        if c.ground is None:
            raise ValueError("disambiguate_recipients: every candidate must be ground-projected first")

    weights = cfg.disambiguation_weights
    scores: list[DisambiguationScore] = []
    for c in candidates:
        if ble_event is not None and ble_event.phone_gps is not None:
            ble_ground = _phone_gps_ground_point(ble_event.phone_gps, own_state)
            position_score = _clamp01(1.0 - c.ground.distance_to(ble_ground) / cfg.max_position_disagreement_m)
        else:
            position_score = 0.0

        if ble_event is not None and ble_event.rssi_dbm is not None:
            ble_range = _rssi_range_m(ble_event.rssi_dbm, cfg.rssi)
            range_diff = abs(ble_range - _range_from_drone_m(c.ground))
            rssi_consistency_score = _clamp01(1.0 - range_diff / cfg.max_position_disagreement_m_rssi_only)
        else:
            rssi_consistency_score = 0.0

        motion_cue_score = motion_cue_scores.get(c.track_id, 0.0) if c.track_id is not None else 0.0

        likelihood = (
            weights.alpha_ble_position * position_score
            + weights.beta_rssi_consistency * rssi_consistency_score
            + weights.gamma_motion_cue * motion_cue_score
        )
        scores.append(DisambiguationScore(
            track_id=c.track_id, ground=c.ground,
            position_score=position_score,
            rssi_consistency_score=rssi_consistency_score,
            motion_cue_score=motion_cue_score,
            likelihood=likelihood,
        ))

    order = sorted(range(len(candidates)), key=lambda i: scores[i].likelihood, reverse=True)
    scores = [scores[i] for i in order]
    ranked_candidates = [candidates[i] for i in order]

    if len(candidates) == 1:
        return DisambiguationResult(winner=ranked_candidates[0], scores=scores, reason="unambiguous_single_candidate")

    margin = scores[0].likelihood - scores[1].likelihood
    if margin >= cfg.disambiguation_margin:
        return DisambiguationResult(winner=ranked_candidates[0], scores=scores, reason="margin_met")
    return DisambiguationResult(winner=None, scores=scores, reason="margin_not_met")
