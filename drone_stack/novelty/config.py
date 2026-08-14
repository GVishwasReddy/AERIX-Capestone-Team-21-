"""Validated, fail-loud configuration for the novelty layer.

Every threshold used by the novelty layer lives in one of the five YAML
files under ``config/novelty/`` and is loaded exactly once, at boot, through
this module. Nothing below reads an environment variable or a magic number -
see ``docs/novelty/*.md`` for what each threshold means operationally.

Deliberately separate from ``config/default.yaml`` / ``drone_stack.utils.
config.Config``: that file holds *profiles* that get deep-merged
(``sim.yaml``/``real.yaml`` on top of ``default.yaml``); the five files here
are not profiles, they are the patent-evidence parameter set, and each is
validated with its own Pydantic model so a missing or misspelled key raises
immediately (``extra="forbid"``) instead of silently defaulting to 0.
"""
from __future__ import annotations

from pathlib import Path
from typing import Literal

import yaml
from pydantic import BaseModel, ConfigDict, Field, model_validator

from drone_stack.novelty.types import TerrainClass


def _repo_root() -> Path:
    # drone_stack/novelty/config.py -> novelty -> drone_stack -> <repo root>
    return Path(__file__).resolve().parents[2]


def default_novelty_config_dir() -> Path:
    return _repo_root() / "config" / "novelty"


class _StrictModel(BaseModel):
    """Base for every novelty config section: unknown keys are an error."""

    model_config = ConfigDict(extra="forbid", frozen=True)


# --------------------------------------------------------------------------- #
# §2.1 landing_zone.yaml
# --------------------------------------------------------------------------- #
class LandingZoneWeights(_StrictModel):
    w1_surface: float
    w2_slope: float
    w3_clutter: float
    w4_area: float


class LandingZoneConfig(_StrictModel):
    weights: LandingZoneWeights
    surface_suitability: dict[TerrainClass, float] = Field(
        description="0..1 suitability per terrain class; every TerrainClass "
        "value must be present (fail loud otherwise)."
    )
    r_min_m: float
    r_max_m: float
    min_score: float
    max_slope_deg: float
    min_safe_radius_m: float
    top_k: int
    cell_size_m: float
    clutter_norm: float

    @model_validator(mode="after")
    def _all_classes_scored(self) -> "LandingZoneConfig":
        missing = set(TerrainClass) - set(self.surface_suitability)
        if missing:
            raise ValueError(
                f"surface_suitability is missing classes: {sorted(m.value for m in missing)}"
            )
        if self.r_min_m > self.r_max_m:
            raise ValueError("r_min_m must be <= r_max_m")
        if self.top_k < 1:
            raise ValueError("top_k must be >= 1")
        return self


# --------------------------------------------------------------------------- #
# §2.2 / §2.5 recipient_auth.yaml
# --------------------------------------------------------------------------- #
class DisambiguationWeights(_StrictModel):
    alpha_ble_position: float
    beta_rssi_consistency: float
    gamma_motion_cue: float


class RssiPathLoss(_StrictModel):
    """Log-distance path-loss model: d = 10 ** ((tx_power_dbm - rssi_dbm) /
    (10 * path_loss_exponent)). See docs/novelty/recipient_auth.md."""

    tx_power_dbm: float
    path_loss_exponent: float


class RecipientAuthConfig(_StrictModel):
    sync_window_ms: int
    max_position_disagreement_m: float
    max_position_disagreement_m_rssi_only: float
    vision_retry_timeout_s: float
    ble_retry_timeout_s: float
    disagreement_retry_timeout_s: float
    disambiguation_margin: float
    disambiguation_weights: DisambiguationWeights
    rssi: RssiPathLoss

    @model_validator(mode="after")
    def _rssi_wider_than_gps(self) -> "RecipientAuthConfig":
        if self.max_position_disagreement_m_rssi_only < self.max_position_disagreement_m:
            raise ValueError(
                "max_position_disagreement_m_rssi_only must be >= "
                "max_position_disagreement_m (RSSI-only ranging is coarser than GPS)"
            )
        return self


# --------------------------------------------------------------------------- #
# §2.3 mission_fsm.yaml
# --------------------------------------------------------------------------- #
class MissionFsmConfig(_StrictModel):
    state_timeout_s: dict[str, float] = Field(
        description="Per-state timeout, keyed by MissionState.value. Every "
        "state that can time out must be present (fail loud otherwise)."
    )
    search_radius_expansion_m: float
    max_search_radius_m: float
    max_hover_retries: int
    descent_hover_altitude_m: float = Field(
        description="Altitude (alt_rel_m) above the chosen landing zone at "
        "which DESCENDING hands off to AUTHENTICATING (AltitudeReachedEvent) "
        "- the hover height authentication and release happen at, before the "
        "final touchdown. See docs/novelty/mission_fsm.md."
    )
    ascend_target_altitude_m: float = Field(
        description="Altitude (alt_rel_m) at which ASCENDING hands off to "
        "RTL (AscendCompleteEvent) after a successful release."
    )

    @model_validator(mode="after")
    def _all_states_bounded(self) -> "MissionFsmConfig":
        # Imported lazily to avoid a config.py <-> mission_fsm.py import cycle.
        from drone_stack.novelty.mission_fsm import MissionState, TIMED_STATES

        missing = {s.value for s in TIMED_STATES} - set(self.state_timeout_s)
        if missing:
            raise ValueError(f"state_timeout_s is missing states: {sorted(missing)}")
        unknown = set(self.state_timeout_s) - {s.value for s in MissionState}
        if unknown:
            raise ValueError(f"state_timeout_s has unknown states: {sorted(unknown)}")
        if self.descent_hover_altitude_m <= 0:
            raise ValueError("descent_hover_altitude_m must be > 0")
        if self.ascend_target_altitude_m <= self.descent_hover_altitude_m:
            raise ValueError(
                "ascend_target_altitude_m must be > descent_hover_altitude_m "
                "(ascent climbs out above the hover/release altitude)"
            )
        return self


# --------------------------------------------------------------------------- #
# §2.4 motion_monitor.yaml
# --------------------------------------------------------------------------- #
class MotionMonitorConfig(_StrictModel):
    max_recipient_velocity_mps: float
    abort_altitude_ceiling_m: float
    zone_intrusion_radius_m: float
    track_history_len: int
    min_track_frames: int
    max_association_distance_m: float = Field(
        description="Frame-to-frame tracking gate: a new detection is matched "
        "to an existing track only if it falls within this ground distance of "
        "that track's last known position; otherwise it starts a new track. "
        "See docs/novelty/motion_monitor.md 'Track association'."
    )

    @model_validator(mode="after")
    def _positive_history_window(self) -> "MotionMonitorConfig":
        if self.track_history_len < 1:
            raise ValueError("track_history_len must be >= 1")
        if self.min_track_frames < 1 or self.min_track_frames > self.track_history_len:
            raise ValueError("min_track_frames must be between 1 and track_history_len")
        return self


# --------------------------------------------------------------------------- #
# §2.6 models.yaml
# --------------------------------------------------------------------------- #
class CameraIntrinsics(_StrictModel):
    """Pinhole intrinsics + mount pose for GroundProjector.

    GUESSED until the delivery camera is calibrated - see
    docs/novelty/landing_zone.md "Known limitations".
    """

    fx: float
    fy: float
    cx: float
    cy: float
    tilt_from_nadir_deg: float = Field(
        description="0 = camera points straight down (nadir), 90 = camera "
        "points straight forward (horizontal). Roll about the optical axis "
        "is assumed 0 - see projector.py docstring."
    )


class BinaryFallback(_StrictModel):
    """How a legacy binary (safe/unsafe) segmentation model's boolean mask
    maps onto the 7-class :class:`TerrainClass` vocabulary, until a
    multi-class model is trained (see ``docs/novelty/landing_zone.md``
    "Model interface"). Lives on the segmenter's own model spec (not
    landing_zone.yaml) because it describes the ADAPTER's output contract,
    not the scoring formula that consumes it."""

    safe_class: TerrainClass
    unsafe_class: TerrainClass


class ModelSpecConfig(_StrictModel):
    kind: Literal["detector", "segmenter"]
    hef_path: str
    input_shape: tuple[int, int, int]  # (h, w, c)
    score_thr: float | None = None                  # detector only
    seg_threshold: float | None = None               # binary segmenter only (sigmoid threshold)
    binary_fallback: BinaryFallback | None = None     # binary segmenter only
    class_map: list[TerrainClass] | None = Field(
        default=None,
        description="Multi-class segmenter only. Ordered list of TerrainClass "
        "values, one per output channel in the .hef's own channel order "
        "(index i of this list <-> output channel i). The adapter argmaxes "
        "per pixel and looks up the winning channel here - see "
        "MultiClassSegmenterAdapter.",
    )

    @model_validator(mode="after")
    def _kind_specific_fields(self) -> "ModelSpecConfig":
        if self.kind == "detector":
            if self.score_thr is None:
                raise ValueError("detector models require score_thr")
            if self.seg_threshold is not None or self.binary_fallback is not None \
                    or self.class_map is not None:
                raise ValueError(
                    "detector models must not set segmenter-only fields "
                    "(seg_threshold / binary_fallback / class_map)"
                )
            return self

        # kind == "segmenter"
        if self.score_thr is not None:
            raise ValueError("segmenter models must not set score_thr")
        is_binary = self.binary_fallback is not None
        is_multiclass = self.class_map is not None
        if is_binary and is_multiclass:
            raise ValueError(
                "segmenter models must set exactly one of binary_fallback "
                "(legacy sigmoid model) or class_map (argmax model), not both"
            )
        if not is_binary and not is_multiclass:
            raise ValueError(
                "segmenter models require exactly one of binary_fallback "
                "(legacy sigmoid model) or class_map (multi-class argmax model)"
            )
        if is_binary and self.seg_threshold is None:
            raise ValueError("binary segmenter models (binary_fallback set) require seg_threshold")
        if is_multiclass:
            if self.seg_threshold is not None:
                raise ValueError(
                    "multi-class segmenter models (class_map set) must not set "
                    "seg_threshold - argmax has no threshold"
                )
            if len(self.class_map) < 2:
                raise ValueError(
                    "class_map must list at least 2 TerrainClass values (one "
                    "per output channel, in channel order)"
                )
        return self


class ModelsConfig(_StrictModel):
    camera: CameraIntrinsics
    models: dict[str, ModelSpecConfig]


# --------------------------------------------------------------------------- #
# Aggregate: loaded once at boot
# --------------------------------------------------------------------------- #
class NoveltyConfig(_StrictModel):
    landing_zone: LandingZoneConfig
    recipient_auth: RecipientAuthConfig
    mission_fsm: MissionFsmConfig
    motion_monitor: MotionMonitorConfig
    models: ModelsConfig

    @classmethod
    def load(cls, config_dir: Path | str | None = None) -> "NoveltyConfig":
        """Load and validate all five YAML files. Raises on the FIRST error
        encountered per file, but reports every file's error before raising
        so a bad config is fixed in one pass, not five."""
        base = Path(config_dir) if config_dir else default_novelty_config_dir()
        errors: list[str] = []
        loaded: dict[str, object] = {}

        specs: list[tuple[str, str, type[BaseModel]]] = [
            ("landing_zone", "landing_zone.yaml", LandingZoneConfig),
            ("recipient_auth", "recipient_auth.yaml", RecipientAuthConfig),
            ("mission_fsm", "mission_fsm.yaml", MissionFsmConfig),
            ("motion_monitor", "motion_monitor.yaml", MotionMonitorConfig),
            ("models", "models.yaml", ModelsConfig),
        ]
        for field_name, filename, model_cls in specs:
            path = base / filename
            try:
                data = _load_yaml(path)
                loaded[field_name] = model_cls(**data)
            except Exception as exc:  # noqa: BLE001 - collected, not swallowed
                errors.append(f"{filename}: {exc}")

        if errors:
            raise NoveltyConfigError(
                "novelty config failed to load:\n  - " + "\n  - ".join(errors)
            )
        return cls(**loaded)


class NoveltyConfigError(Exception):
    """Raised when any config/novelty/*.yaml file is missing or invalid."""


def _load_yaml(path: Path) -> dict:
    if not path.exists():
        raise FileNotFoundError(f"config file not found: {path}")
    with path.open("r", encoding="utf-8") as handle:
        data = yaml.safe_load(handle) or {}
    if not isinstance(data, dict):
        raise ValueError(f"{path} must contain a mapping at the top level")
    return data
