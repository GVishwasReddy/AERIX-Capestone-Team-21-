"""§2.1 landing_zone.py - synthetic segmentation masks with known
ground-truth safe/unsafe patterns (brief §2.1 deliverable: "unit tests with
synthetic segmentation masks").

Uses a deterministic ``_TopDownStub`` projector instead of the real
``FlatEarthPinhole`` for most tests: the real projector's camera intrinsics
(fx/fy/cx/cy in config/novelty/models.yaml) are GUESSED placeholders pending
calibration, and coupling these formula-correctness tests to that placeholder
calibration would make them fail/change for reasons that have nothing to do
with landing_zone.py's own logic. With ``meters_per_pixel == cell_size_m``,
the stub's projection makes ``rasterize_to_grid``'s own binning map pixel
(row, col) to grid cell (row, col) 1:1 (derived in the class docstring below),
so a synthetic mask's ground-truth pattern is exactly the resulting
TerrainMap - no coordinate-geometry translation needed to write assertions.
"""
from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np
import pytest

from drone_stack.novelty.landing_zone import (
    _band_violation,
    _score_cell,
    rasterize_to_grid,
    score_candidates,
)
from drone_stack.novelty.types import (
    GroundPoint,
    SegmentationFrame,
    TerrainClass,
    TerrainMap,
)


@dataclass(frozen=True)
class _TopDownStub:
    """Deterministic test-only GroundProjector - see module docstring for
    why this exists instead of FlatEarthPinhole. Every ray "hits the ground"
    (valid=True) whenever altitude_m > 0, mirroring FlatEarthPinhole's own
    altitude_m <= 0 -> invalid contract without needing a lens model.

    Derivation of the 1:1 pixel->cell mapping (mpp = meters_per_pixel):
        x_m(py) = (h - 1 - py) * mpp   ranges [0, (h-1)*mpp], max at py=0
        y_m(px) = (px - (w-1)/2) * mpp centred on 0, min at px=0
    rasterize_to_grid bins with origin = (x_max, y_min) = the py=0/px=0
    corner, and row_idx = floor((origin.x - x_m)/cell_size_m). Substituting:
    origin.x - x_m(py) = py*mpp, so row_idx = floor(py*mpp/cell_size_m) = py
    exactly when mpp == cell_size_m (and symmetrically col_idx = px).
    """

    meters_per_pixel: float

    def pixel_to_ground(self, px, py, altitude_m, frame_shape):
        x, y, valid = self.pixels_to_ground(
            np.array([px], dtype=float), np.array([py], dtype=float), altitude_m, frame_shape
        )
        return GroundPoint(x_m=float(x[0]), y_m=float(y[0])) if valid[0] else None

    def pixels_to_ground(self, px, py, altitude_m, frame_shape):
        h, w = frame_shape
        px = np.asarray(px, dtype=float)
        py = np.asarray(py, dtype=float)
        x_m = (h - 1 - py) * self.meters_per_pixel
        y_m = (px - (w - 1) / 2.0) * self.meters_per_pixel
        valid = np.full(px.shape, altitude_m > 0, dtype=bool)
        return x_m, y_m, valid


def _grid_centre(h: int, w: int, mpp: float) -> GroundPoint:
    """The geometric centre of a _TopDownStub-projected h x w frame."""
    return GroundPoint(x_m=(h - 1) * mpp / 2.0, y_m=0.0)


def _make_segmentation(grid: list[list[TerrainClass]]) -> SegmentationFrame:
    classes_present = sorted({c for row in grid for c in row}, key=lambda c: c.value)
    class_to_index = {c: i for i, c in enumerate(classes_present)}
    index_to_class = {i: c for c, i in class_to_index.items()}
    h, w = len(grid), len(grid[0])
    arr = np.zeros((h, w), dtype=np.int8)
    for r in range(h):
        for c in range(w):
            arr[r, c] = class_to_index[grid[r][c]]
    return SegmentationFrame(class_indices=arr, index_to_class=index_to_class)


@pytest.fixture
def cfg(valid_config_dir):
    from drone_stack.novelty.config import NoveltyConfig

    return NoveltyConfig.load(valid_config_dir).landing_zone


# --------------------------------------------------------------------------- #
# _band_violation - pure function, no grid needed
# --------------------------------------------------------------------------- #
@pytest.mark.parametrize(
    "distance_m, expected",
    [
        (0.0, 2.0),   # inside r_min (2.0) -> violation = r_min - distance
        (2.0, 0.0),   # exactly at r_min -> zero
        (7.0, 0.0),   # mid-band -> zero
        (12.0, 0.0),  # exactly at r_max -> zero
        (15.0, 3.0),  # outside r_max (12.0) -> violation = distance - r_max
    ],
)
def test_band_violation(cfg, distance_m, expected):
    assert _band_violation(distance_m, cfg) == pytest.approx(expected)


# --------------------------------------------------------------------------- #
# Scenario: all-grass
# --------------------------------------------------------------------------- #
def test_all_grass_yields_one_full_candidate(cfg):
    h = w = 14  # 7m square: comfortably clears min_safe_radius_m, fits r_max
    seg = _make_segmentation([[TerrainClass.GRASS] * w for _ in range(h)])
    projector = _TopDownStub(meters_per_pixel=cfg.cell_size_m)
    person = _grid_centre(h, w, cfg.cell_size_m)

    candidates = score_candidates(seg, altitude_m=10.0, person_ground=person, projector=projector, cfg=cfg)

    assert len(candidates) == 1
    c = candidates[0]
    assert c.surface_class == TerrainClass.GRASS
    expected_radius = math.sqrt((h * w * cfg.cell_size_m**2) / math.pi)
    assert c.safe_radius_m == pytest.approx(expected_radius)
    # Independently recomputed "known ground truth" score - a band-satisfying
    # cell exists near the grid centre (see test below), so penalty is 0.
    expected_area_term = expected_radius / (expected_radius + cfg.min_safe_radius_m)
    expected_score = (
        cfg.weights.w1_surface * cfg.surface_suitability[TerrainClass.GRASS]
        + cfg.weights.w2_slope * 1.0
        + cfg.weights.w3_clutter * 1.0
        + cfg.weights.w4_area * expected_area_term
    )
    assert c.score == pytest.approx(expected_score)
    assert c.score > cfg.min_score


# --------------------------------------------------------------------------- #
# Scenario: water patch splits grass into two disjoint candidates
# --------------------------------------------------------------------------- #
def test_water_patch_splits_grass_into_two_disjoint_candidates(cfg):
    rows, cols = 10, 10
    grid = [[TerrainClass.GRASS] * cols for _ in range(rows)]
    for r in range(rows):
        grid[r][5] = TerrainClass.WATER  # full-height column: disconnects left/right
    seg = _make_segmentation(grid)
    projector = _TopDownStub(meters_per_pixel=cfg.cell_size_m)
    person = _grid_centre(rows, cols, cfg.cell_size_m)

    candidates = score_candidates(seg, altitude_m=10.0, person_ground=person, projector=projector, cfg=cfg)

    assert len(candidates) == 2
    assert all(c.surface_class == TerrainClass.GRASS for c in candidates)
    left_cells, right_cells = rows * 5, rows * 4  # cols 0-4 vs cols 6-9
    expected_radii = sorted(
        math.sqrt(n * cfg.cell_size_m**2 / math.pi) for n in (left_cells, right_cells)
    )
    assert sorted(c.safe_radius_m for c in candidates) == pytest.approx(expected_radii)


# --------------------------------------------------------------------------- #
# Scenario: obstacle ring isolates its interior as a separate component
# --------------------------------------------------------------------------- #
def test_obstacle_ring_isolates_interior_as_separate_candidate(cfg):
    n = 15
    arr = np.zeros((n, n), dtype=np.int8)  # 0 = grass everywhere
    arr[3:12, 3:12] = 1  # fill a 9x9 box with obstacle...
    arr[4:11, 4:11] = 0  # ...then carve a 7x7 interior back to grass
    seg = SegmentationFrame(
        class_indices=arr, index_to_class={0: TerrainClass.GRASS, 1: TerrainClass.OBSTACLE}
    )
    projector = _TopDownStub(meters_per_pixel=cfg.cell_size_m)
    person = _grid_centre(n, n, cfg.cell_size_m)

    candidates = score_candidates(seg, altitude_m=10.0, person_ground=person, projector=projector, cfg=cfg)

    assert len(candidates) == 2
    assert all(c.surface_class == TerrainClass.GRASS for c in candidates)
    interior_radius = math.sqrt((7 * 7 * cfg.cell_size_m**2) / math.pi)
    exterior_radius = math.sqrt(((n * n - 9 * 9) * cfg.cell_size_m**2) / math.pi)
    assert sorted(c.safe_radius_m for c in candidates) == pytest.approx(
        sorted([interior_radius, exterior_radius])
    )


# --------------------------------------------------------------------------- #
# Scenario: safe area too small is filtered before scoring
# --------------------------------------------------------------------------- #
def test_safe_area_too_small_is_filtered_before_scoring(cfg):
    n = 8
    arr = np.ones((n, n), dtype=np.int8)  # 1 = water everywhere
    arr[3:5, 3:5] = 0  # a tiny 2x2 grass island
    seg = SegmentationFrame(
        class_indices=arr, index_to_class={0: TerrainClass.GRASS, 1: TerrainClass.WATER}
    )
    projector = _TopDownStub(meters_per_pixel=cfg.cell_size_m)
    person = _grid_centre(n, n, cfg.cell_size_m)

    island_radius = math.sqrt((2 * 2 * cfg.cell_size_m**2) / math.pi)
    assert island_radius < cfg.min_safe_radius_m  # sanity: the island really is too small

    candidates = score_candidates(seg, altitude_m=10.0, person_ground=person, projector=projector, cfg=cfg)
    assert candidates == []


# --------------------------------------------------------------------------- #
# Scenario: everything below threshold (person out of range) -> []
# --------------------------------------------------------------------------- #
def test_everything_below_threshold_returns_empty_list(cfg):
    n = 14
    seg = _make_segmentation([[TerrainClass.GRASS] * n for _ in range(n)])
    projector = _TopDownStub(meters_per_pixel=cfg.cell_size_m)
    # Far outside r_max_m from every cell in this ~7m-wide grid, so
    # penalty_if_outside_radius dominates regardless of how good the terrain
    # is. This is the case the w4 area-term saturation fix (see
    # landing_zone.py's module docstring) was written to keep gate-able: an
    # unbounded raw-metres area term could have let a big enough blob push
    # score back above min_score even here.
    person = GroundPoint(x_m=200.0, y_m=200.0)

    candidates = score_candidates(seg, altitude_m=10.0, person_ground=person, projector=projector, cfg=cfg)
    assert candidates == []


def test_score_candidates_returns_empty_when_nothing_projects_to_ground(cfg):
    seg = _make_segmentation([[TerrainClass.GRASS] * 4 for _ in range(4)])
    projector = _TopDownStub(meters_per_pixel=cfg.cell_size_m)

    candidates = score_candidates(
        seg, altitude_m=0.0, person_ground=GroundPoint(x_m=0.0, y_m=0.0), projector=projector, cfg=cfg
    )
    assert candidates == []
    assert rasterize_to_grid(seg, altitude_m=0.0, projector=projector, cell_size_m=cfg.cell_size_m) is None


# --------------------------------------------------------------------------- #
# Scenario: cluttered - obstacle-pixel density within an otherwise-safe cell
# --------------------------------------------------------------------------- #
def test_rasterize_to_grid_counts_obstacle_pixels_as_clutter(cfg):
    # 2x2 pixels per cell (mpp = cell_size_m/2), so a 4x4 pixel frame bins
    # into a 2x2 TerrainMap - lets a cell's majority class and its clutter
    # count differ, which a 1:1 pixel:cell mapping can never exercise.
    mpp = cfg.cell_size_m / 2.0
    arr = np.zeros((4, 4), dtype=np.int8)  # 0 = grass
    arr[0, 0] = 1  # one OBSTACLE pixel inside the top-left 2x2 cell block
    seg = SegmentationFrame(
        class_indices=arr, index_to_class={0: TerrainClass.GRASS, 1: TerrainClass.OBSTACLE}
    )
    projector = _TopDownStub(meters_per_pixel=mpp)

    terrain = rasterize_to_grid(seg, altitude_m=10.0, projector=projector, cell_size_m=cfg.cell_size_m)

    assert (terrain.rows, terrain.cols) == (2, 2)
    # Majority vote still GRASS (3 grass pixels beat 1 obstacle per cell)...
    assert all(terrain.classes[r][c] == TerrainClass.GRASS for r in range(2) for c in range(2))
    # ...but the single obstacle pixel still shows up as that cell's clutter.
    assert terrain.clutter_count == [[1, 0], [0, 0]]


def test_score_cell_penalises_clutter_and_clamps_at_clutter_norm(cfg):
    terrain = TerrainMap(
        rows=1,
        cols=1,
        cell_size_m=cfg.cell_size_m,
        origin=GroundPoint(x_m=0.0, y_m=0.0),
        classes=[[TerrainClass.GRASS]],
        slope_deg=[[0.0]],
        clutter_count=[[0]],
        slope_is_estimated=True,
    )
    person = terrain.cell_centre(0, 0)  # distance 0 - a fixed penalty offset
    # shared by every call below, so it cancels out of the comparisons; the
    # radius_m argument is likewise a fixed placeholder decoupled from this
    # 1-cell TerrainMap's own (irrelevant here) size - only clutter_count
    # varies between calls.

    def _score(clutter: int) -> float:
        terrain.clutter_count[0][0] = clutter
        score, *_ = _score_cell(terrain, 0, 0, radius_m=5.0, person_ground=person, cfg=cfg)
        return score

    clean = _score(0)
    half = _score(int(cfg.clutter_norm / 2))
    full = _score(int(cfg.clutter_norm))
    beyond = _score(int(cfg.clutter_norm) * 10)

    assert clean > half > full
    assert full == pytest.approx(beyond)  # clamps at 1.0 once clutter_count >= clutter_norm


# --------------------------------------------------------------------------- #
# Regression: representative-cell tie-break minimises band violation, not
# raw distance to the person (the fix applied alongside _band_violation).
# --------------------------------------------------------------------------- #
def test_representative_cell_minimises_band_violation_not_raw_distance(cfg):
    """On a component where every cell is tied on suitability/slope, the
    representative cell must minimise penalty_if_outside_radius, not raw
    distance to the person - the two disagree whenever the whole component
    sits inside r_min, where being CLOSER to the person means a LARGER
    penalty, not a smaller one. A raw-distance tie-break would pick the
    cell closest to the person here; the fix must pick the farthest."""
    n = 6
    seg = _make_segmentation([[TerrainClass.GRASS] * n for _ in range(n)])
    projector = _TopDownStub(meters_per_pixel=cfg.cell_size_m)
    person = _grid_centre(n, n, cfg.cell_size_m)

    terrain = rasterize_to_grid(seg, altitude_m=10.0, projector=projector, cell_size_m=cfg.cell_size_m)
    all_distances = [
        person.distance_to(terrain.cell_centre(r, c))
        for r in range(terrain.rows)
        for c in range(terrain.cols)
    ]
    # Sanity: this scenario depends on every cell sitting inside r_min.
    assert max(all_distances) < cfg.r_min_m

    candidates = score_candidates(seg, altitude_m=10.0, person_ground=person, projector=projector, cfg=cfg)
    assert len(candidates) == 1
    chosen = candidates[0]
    chosen_distance = person.distance_to(terrain.cell_centre(chosen.cell_row, chosen.cell_col))
    assert chosen_distance == pytest.approx(max(all_distances))


# --------------------------------------------------------------------------- #
# Ranking + top_k
# --------------------------------------------------------------------------- #
def test_score_candidates_ranks_by_score_and_respects_top_k(cfg):
    rows, block_w = 6, 5
    order = [TerrainClass.GRASS, TerrainClass.DIRT, TerrainClass.PAVEMENT, TerrainClass.VEGETATION]
    row_pattern: list[TerrainClass] = []
    for i, cls in enumerate(order):
        if i:
            row_pattern.append(TerrainClass.WATER)  # 1-cell gap disconnects blocks
        row_pattern.extend([cls] * block_w)
    grid = [list(row_pattern) for _ in range(rows)]
    seg = _make_segmentation(grid)
    projector = _TopDownStub(meters_per_pixel=cfg.cell_size_m)
    # Placed so every cell in this grid falls within [r_min_m, r_max_m] of
    # the person (verified below) - isolates surface_suitability as the only
    # score-differentiating term across the four equal-sized blocks.
    person = GroundPoint(x_m=-6.0, y_m=0.0)

    terrain = rasterize_to_grid(seg, altitude_m=10.0, projector=projector, cell_size_m=cfg.cell_size_m)
    for r in range(terrain.rows):
        for c in range(terrain.cols):
            d = person.distance_to(terrain.cell_centre(r, c))
            assert cfg.r_min_m <= d <= cfg.r_max_m

    assert cfg.top_k == 3  # this test's len()/exclusion assertions depend on it
    candidates = score_candidates(seg, altitude_m=10.0, person_ground=person, projector=projector, cfg=cfg)

    assert len(candidates) == 3
    scores = [c.score for c in candidates]
    assert scores == sorted(scores, reverse=True)
    classes = [c.surface_class for c in candidates]
    assert classes == [TerrainClass.GRASS, TerrainClass.DIRT, TerrainClass.PAVEMENT]
    assert TerrainClass.VEGETATION not in classes
