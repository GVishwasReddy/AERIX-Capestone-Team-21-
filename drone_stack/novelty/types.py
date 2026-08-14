"""Shared value types for the novelty layer.

Coordinate convention (matches ``drone_stack/utils/geometry.py`` exactly, so
novelty-layer ground points compose directly with ``Obstacle`` and
``FusedState`` without another conversion):

    Body/ground frame: x = forward (m), y = left (m). Bearing 0 deg = front,
    positive bearing = right (clockwise from above).

All dataclasses carry plain floats/lists (no numpy inside dataclasses) so
they serialise straight to JSON for ``evidence_logger`` and the bus, mirroring
the convention in ``drone_stack/msg/messages.py``. Numpy arrays are used only
inside the perception layer (segmentation masks, raw model buffers) and are
converted to these types before crossing onto the bus.
"""
from __future__ import annotations

import time
from dataclasses import dataclass, field
from enum import Enum
from typing import Any

import numpy as np


# --------------------------------------------------------------------------- #
# Geometry
# --------------------------------------------------------------------------- #
@dataclass(frozen=True)
class GroundPoint:
    """A point on the (assumed-flat) ground plane, body-relative metres."""

    x_m: float  # forward
    y_m: float  # left

    def distance_to(self, other: "GroundPoint") -> float:
        return ((self.x_m - other.x_m) ** 2 + (self.y_m - other.y_m) ** 2) ** 0.5


@dataclass(frozen=True)
class PixelBox:
    """Axis-aligned pixel box in the ORIGINAL (unletterboxed) frame."""

    x1: int
    y1: int
    x2: int
    y2: int

    @property
    def centroid_px(self) -> tuple[float, float]:
        return ((self.x1 + self.x2) / 2.0, (self.y1 + self.y2) / 2.0)

    @property
    def width(self) -> int:
        return self.x2 - self.x1

    @property
    def height(self) -> int:
        return self.y2 - self.y1


# --------------------------------------------------------------------------- #
# Terrain classes (§2.1) - the canonical 7-class vocabulary. A binary
# segmentation model (today's terrain.hef) is adapted onto a 2-class subset
# of this same vocabulary by drone_stack/novelty/perception/adapters.py -
# every other module only ever sees TerrainClass, never "binary vs
# multi-class", so a real multi-class .hef drops in with no code change.
# --------------------------------------------------------------------------- #
class TerrainClass(str, Enum):
    GRASS = "grass"
    PAVEMENT = "pavement"
    DIRT = "dirt"
    WATER = "water"
    VEGETATION = "vegetation"
    OBSTACLE = "obstacle"
    UNKNOWN = "unknown"


@dataclass
class TerrainMap:
    """Per-cell terrain classification + slope, in a ground-plane grid.

    ``classes``/``slope_deg``/``clutter_count`` are all indexed
    ``[row][col]`` over the same ``rows x cols`` grid, where each cell covers
    ``cell_size_m`` x ``cell_size_m`` of ground and cell (0, 0) is the
    top-left of the *frame* (row increases with image y / away from camera
    top edge, matching the pixel convention of the source frame before
    projection). ``origin`` is the ``GroundPoint`` of cell (0, 0)'s centre,
    so callers can go from a cell index back to a body-relative location.
    """

    rows: int
    cols: int
    cell_size_m: float
    origin: GroundPoint
    classes: list[list[TerrainClass]]
    slope_deg: list[list[float]]
    clutter_count: list[list[int]]
    slope_is_estimated: bool = True  # False only when a real depth source fed it
    stamp: float = field(default_factory=time.time)

    def cell_centre(self, row: int, col: int) -> GroundPoint:
        return GroundPoint(
            x_m=self.origin.x_m - row * self.cell_size_m,
            y_m=self.origin.y_m + col * self.cell_size_m,
        )


# --------------------------------------------------------------------------- #
# Perception outputs (§2.6 VisionModel.infer() return type)
# --------------------------------------------------------------------------- #
@dataclass
class PersonDetection:
    """One person detection, pixel box plus (once projected) ground point."""

    bbox: PixelBox
    score: float
    ground: GroundPoint | None = None   # filled in by GroundProjector
    track_id: int | None = None         # filled in by MotionMonitor's tracker
    stamp: float = field(default_factory=time.time)

    @property
    def centroid_px(self) -> tuple[float, float]:
        return self.bbox.centroid_px


@dataclass
class SegmentationFrame:
    """Raw per-pixel classification from a segmenter model - the model
    adapter's output. Deliberately NOT ground-projected or gridded here:
    that requires ``cell_size_m`` (a landing_zone.yaml scoring parameter)
    and the current altitude, so it is done by landing_zone.py's own
    ``rasterize_to_grid`` using a GroundProjector, keeping this class a pure
    model-I/O type. class_indices is (h, w) int8; index_to_class maps each
    index back to a TerrainClass so the adapter's own class ordering is
    self-describing. NOT logged verbatim to evidence_logger (too large/binary
    for a JSON-lines flight log) - only the scored candidates derived from
    it are logged."""

    class_indices: np.ndarray  # shape (h, w), dtype int8
    index_to_class: dict[int, "TerrainClass"]


@dataclass
class ModelOutput:
    """Generic wrapper returned by every :class:`VisionModel.infer`.

    Exactly one of ``detections`` / ``segmentation`` is populated, selected
    by ``kind`` - this keeps the ``VisionModel`` protocol single-method
    while still being precisely typed for each of the two model kinds we
    support.
    """

    kind: str  # "detections" | "terrain"
    detections: list[PersonDetection] = field(default_factory=list)
    segmentation: SegmentationFrame | None = None
    model_version: str = "unknown"
    stamp: float = field(default_factory=time.time)


# --------------------------------------------------------------------------- #
# Landing-zone selection (§2.1)
# --------------------------------------------------------------------------- #
@dataclass
class ZoneCandidate:
    score: float
    centroid: GroundPoint
    safe_radius_m: float
    surface_class: TerrainClass
    slope_deg: float
    clutter_density: float
    cell_row: int
    cell_col: int


# --------------------------------------------------------------------------- #
# Recipient authentication (§2.2 / §2.5)
# --------------------------------------------------------------------------- #
@dataclass
class BleAuthEvent:
    """Emitted by the (re-gated) BLE peripheral - carries NO release power."""

    authenticated: bool
    rssi_dbm: float | None = None
    phone_gps: tuple[float, float] | None = None  # (lat, lon), if the app sent one
    stamp: float = field(default_factory=time.time)


@dataclass
class AuthDecision:
    released: bool
    reason: str
    ble_ok: bool
    vision_ok: bool
    position_disagreement_m: float | None = None
    recipient_track_id: int | None = None
    stamp: float = field(default_factory=time.time)


@dataclass
class DisambiguationScore:
    """One person candidate's full score breakdown from one §2.5
    disambiguation pass - kept even for losing candidates so the caller can
    log the complete scoring table (patent evidence), not just the winner."""

    track_id: int | None
    ground: GroundPoint
    position_score: float
    rssi_consistency_score: float
    motion_cue_score: float
    likelihood: float


@dataclass
class DisambiguationResult:
    """Outcome of one §2.5 multi-person disambiguation pass.

    ``winner`` is ``None`` whenever the top candidate does not beat the
    runner-up by at least ``disambiguation_margin`` (or there were no
    candidates at all) - the mission FSM reads that as "still ambiguous",
    not as an error, and re-observes rather than committing to a guess.
    ``scores`` is every candidate, sorted descending by ``likelihood``.
    """

    winner: PersonDetection | None
    scores: list[DisambiguationScore]
    reason: str  # "no_candidates" | "unambiguous_single_candidate" | "margin_met" | "margin_not_met"


# --------------------------------------------------------------------------- #
# §2.3 mission FSM
# --------------------------------------------------------------------------- #
@dataclass
class FsmStateSnapshot:
    """Published on ``NoveltyTopics.MISSION_FSM_STATE`` every
    ``DeliveryNode`` step - lets the GCS (and a live observer) see the
    mission's current phase without replaying the JSON-Lines flight log."""

    state: str
    elapsed_in_state_s: float
    recipient_track_id: int | None = None
    search_radius_m: float = 0.0
    stamp: float = field(default_factory=time.time)


# --------------------------------------------------------------------------- #
# Descent safety (§2.4)
# --------------------------------------------------------------------------- #
@dataclass
class MotionAbortEvent:
    reason: str  # "recipient_velocity" | "zone_intrusion"
    track_id: int | None
    velocity_mps: float | None = None
    altitude_m: float | None = None
    intruder_distance_m: float | None = None
    stamp: float = field(default_factory=time.time)


def to_jsonable(value: Any) -> Any:
    """Recursively convert dataclasses/enums/tuples to JSON-safe primitives.

    Used by :mod:`evidence_logger` so any of the types above (and anything
    nesting them) can be logged without a bespoke encoder per type.
    """
    from dataclasses import is_dataclass, asdict

    if is_dataclass(value) and not isinstance(value, type):
        return to_jsonable(asdict(value))
    if isinstance(value, Enum):
        return value.value
    if isinstance(value, dict):
        return {k: to_jsonable(v) for k, v in value.items()}
    if isinstance(value, (list, tuple)):
        return [to_jsonable(v) for v in value]
    return value
