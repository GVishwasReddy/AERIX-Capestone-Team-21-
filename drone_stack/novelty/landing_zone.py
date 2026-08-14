r"""§2.1 - Markerless landing-zone selection.

Problem statement, prior-art positioning (Amazon US10,198,955 et al.), and a
worked numeric example live in ``docs/novelty/landing_zone.md``. This
docstring is the algorithm itself - kept here, not just in the doc, because
it is the text that gets lifted directly into the patent specification (see
brief §2.1 "Docstring at top of file with the full pseudocode + formula").

-------------------------------------------------------------------------
Scoring formula (as specified in the brief, verbatim)
-------------------------------------------------------------------------

    score(cell) = w1 * surface_suitability(class)
                + w2 * (1 - normalized_slope)
                + w3 * (1 - clutter_density)
                + w4 * size_of_contiguous_safe_area
                - penalty_if_outside_radius(cell, person_centroid, r_min, r_max)

``w1..w4``, ``r_min_m``, ``r_max_m`` and per-class ``surface_suitability``
all come from ``config/novelty/landing_zone.yaml`` (drone_stack.novelty.
config.LandingZoneConfig) - nothing here is a hardcoded threshold.

This implementation defines the formula's four free terms as follows -
each choice is a deliberate design decision, documented so it can be cited
or revised independently of the formula itself:

    normalized_slope
        = min(slope_deg(cell) / max_slope_deg, 1.0)

    clutter_density
        = min(clutter_count(cell) / clutter_norm, 1.0)
        clutter_count is the number of segmentation pixels landing in the
        cell that classify as TerrainClass.OBSTACLE (the brief's fallback
        signal - "else from segmentation obstacle class density" - since no
        non-person-detection channel exists yet; see docs/novelty/
        landing_zone.md "Known limitations").

    size_of_contiguous_safe_area
        = radius_m / (radius_m + min_safe_radius_m), where radius_m is the
        equivalent-circle radius (sqrt(area_m2 / pi)) of the 4-connected
        component of "safe" cells (surface_suitability(class) > 0) that
        contains `cell`, and area_m2 = component_cell_count * cell_size_m^2.
        This is a smooth 0..1 saturating function of radius_m - 0.5 at
        exactly the minimum viable footprint (min_safe_radius_m), rising
        asymptotically toward 1 for much larger areas - chosen over a raw
        metre value so this term stays comparable in scale to the other
        three (which are already 0..1), consistent with w1..w4 summing to
        1.0 in config/novelty/landing_zone.yaml and with min_score being a
        meaningful fraction of that scale (a raw-metres term would let an
        arbitrarily large blob's area alone push score past min_score
        regardless of how unsuitable its surface/clutter is, which defeats
        the point of a minimum-score gate). It reuses min_safe_radius_m -
        already meaningful in this domain as "the smallest usable footprint"
        - rather than introduce a second, unexplained area-scale constant.

    penalty_if_outside_radius(cell, person_centroid, r_min, r_max)
        = max(0, r_min - d, d - r_max), where d is the ground distance from
        `cell`'s centre to `person_centroid`. Zero when r_min <= d <= r_max;
        grows linearly (in metres) the further `cell` sits outside the band.

-------------------------------------------------------------------------
Algorithm (pseudocode)
-------------------------------------------------------------------------
    1. rasterize_to_grid: project every pixel of the terrain segmentation
       frame to the ground plane (GroundProjector + altitude) and bin into
       a cell_size_m x cell_size_m grid (TerrainMap). Each cell gets a
       majority terrain class (plurality vote of the pixels landing in it;
       empty cells default to TerrainClass.UNKNOWN) and an obstacle-pixel
       count. Slope is always 0 deg with slope_is_estimated=True - there is
       no depth source yet, so §2.1's slope term is a documented
       low-confidence flat-earth fallback (see brief §2.1 "assume flat and
       flag low-confidence").
    2. Flood-fill 4-connected components over cells where
       surface_suitability(class) > 0 ("safe" cells). Cells with
       suitability 0 (water, obstacle) are never part of a safe component
       and can never become a candidate.
    3. For every component whose equivalent-circle radius is >=
       min_safe_radius_m (smaller components cannot fit the airframe's
       footprint and are dropped before scoring): pick the member cell with
       the highest surface_suitability as that component's representative
       landing point (ties broken by lower slope, then by ground distance
       to the person), and score it with the formula above.
    4. Keep candidates with score >= min_score, sort by score descending,
       return the top ``top_k``. An empty frame, no safe cells, or every
       candidate scoring below min_score all correctly fall out as `[]` -
       the mission FSM's SEARCHING_ZONE state treats an empty list as "not
       found yet" and eventually transitions to EXPANDING_SEARCH_RADIUS.
"""
from __future__ import annotations

import math
from collections import deque

import numpy as np

from drone_stack.novelty.config import LandingZoneConfig
from drone_stack.novelty.perception.projector import GroundProjector
from drone_stack.novelty.types import (
    GroundPoint,
    SegmentationFrame,
    TerrainClass,
    TerrainMap,
    ZoneCandidate,
)


def rasterize_to_grid(
    segmentation: SegmentationFrame,
    altitude_m: float,
    projector: GroundProjector,
    cell_size_m: float,
) -> TerrainMap | None:
    """Project every pixel of *segmentation* to the ground plane and bin
    into a ``cell_size_m`` grid. Returns ``None`` if no pixel's ray
    intersects the ground (e.g. ``altitude_m`` unset/non-positive) - callers
    treat that the same as "no candidates", not as an error."""
    h, w = segmentation.class_indices.shape
    py_idx, px_idx = np.mgrid[0:h, 0:w]
    x_m, y_m, valid = projector.pixels_to_ground(
        px_idx.astype(float), py_idx.astype(float), altitude_m, (h, w)
    )
    if not np.any(valid):
        return None

    xs = x_m[valid]
    ys = y_m[valid]
    class_idx_flat = segmentation.class_indices[valid]

    x_min, x_max = float(xs.min()), float(xs.max())
    y_min, y_max = float(ys.min()), float(ys.max())
    rows = max(1, int(math.floor((x_max - x_min) / cell_size_m)) + 1)
    cols = max(1, int(math.floor((y_max - y_min) / cell_size_m)) + 1)
    origin = GroundPoint(x_m=x_max, y_m=y_min)

    row_idx = np.clip(((origin.x_m - xs) / cell_size_m).astype(int), 0, rows - 1)
    col_idx = np.clip(((ys - origin.y_m) / cell_size_m).astype(int), 0, cols - 1)
    linear_idx = row_idx * cols + col_idx
    num_cells = rows * cols

    class_ids = sorted(segmentation.index_to_class)
    counts = np.zeros((num_cells, len(class_ids)), dtype=np.int64)
    for k, class_id in enumerate(class_ids):
        mask = class_idx_flat == class_id
        counts[:, k] = np.bincount(linear_idx[mask], minlength=num_cells)

    totals = counts.sum(axis=1)
    majority_k = counts.argmax(axis=1)

    obstacle_k = next(
        (k for k, cid in enumerate(class_ids) if segmentation.index_to_class[cid] == TerrainClass.OBSTACLE),
        None,
    )
    obstacle_counts = counts[:, obstacle_k] if obstacle_k is not None else np.zeros(num_cells, dtype=np.int64)

    classes: list[list[TerrainClass]] = []
    clutter: list[list[int]] = []
    slope: list[list[float]] = []
    for r in range(rows):
        class_row: list[TerrainClass] = []
        clutter_row: list[int] = []
        slope_row: list[float] = []
        for c in range(cols):
            cell = r * cols + c
            if totals[cell] == 0:
                class_row.append(TerrainClass.UNKNOWN)
            else:
                class_row.append(segmentation.index_to_class[class_ids[majority_k[cell]]])
            clutter_row.append(int(obstacle_counts[cell]))
            slope_row.append(0.0)
        classes.append(class_row)
        clutter.append(clutter_row)
        slope.append(slope_row)

    return TerrainMap(
        rows=rows,
        cols=cols,
        cell_size_m=cell_size_m,
        origin=origin,
        classes=classes,
        slope_deg=slope,
        clutter_count=clutter,
        slope_is_estimated=True,
    )


def _safe_components(terrain: TerrainMap, cfg: LandingZoneConfig) -> list[list[tuple[int, int]]]:
    """4-connected flood fill over cells where surface_suitability > 0."""
    visited = [[False] * terrain.cols for _ in range(terrain.rows)]
    components: list[list[tuple[int, int]]] = []
    for r0 in range(terrain.rows):
        for c0 in range(terrain.cols):
            if visited[r0][c0] or cfg.surface_suitability[terrain.classes[r0][c0]] <= 0.0:
                continue
            component: list[tuple[int, int]] = []
            queue: deque[tuple[int, int]] = deque([(r0, c0)])
            visited[r0][c0] = True
            while queue:
                r, c = queue.popleft()
                component.append((r, c))
                for nr, nc in ((r - 1, c), (r + 1, c), (r, c - 1), (r, c + 1)):
                    if (
                        0 <= nr < terrain.rows
                        and 0 <= nc < terrain.cols
                        and not visited[nr][nc]
                        and cfg.surface_suitability[terrain.classes[nr][nc]] > 0.0
                    ):
                        visited[nr][nc] = True
                        queue.append((nr, nc))
            components.append(component)
    return components


def _band_violation(distance_m: float, cfg: LandingZoneConfig) -> float:
    """penalty_if_outside_radius's magnitude for a single distance value -
    shared by the scorer and by representative-cell tie-breaking below, so
    "best cell in a tie" and "least-penalised cell" never disagree."""
    return max(0.0, cfg.r_min_m - distance_m, distance_m - cfg.r_max_m)


def _score_cell(
    terrain: TerrainMap,
    row: int,
    col: int,
    radius_m: float,
    person_ground: GroundPoint,
    cfg: LandingZoneConfig,
) -> tuple[float, GroundPoint, float, float]:
    """Returns (score, cell_centre, slope_deg, normalized_clutter) for the
    single representative cell of a safe component."""
    cell_class = terrain.classes[row][col]
    slope_deg = terrain.slope_deg[row][col]
    centre = terrain.cell_centre(row, col)

    surface_term = cfg.surface_suitability[cell_class]
    normalized_slope = min(slope_deg / cfg.max_slope_deg, 1.0)
    normalized_clutter = min(terrain.clutter_count[row][col] / cfg.clutter_norm, 1.0)
    normalized_area = radius_m / (radius_m + cfg.min_safe_radius_m)

    penalty = _band_violation(person_ground.distance_to(centre), cfg)

    score = (
        cfg.weights.w1_surface * surface_term
        + cfg.weights.w2_slope * (1.0 - normalized_slope)
        + cfg.weights.w3_clutter * (1.0 - normalized_clutter)
        + cfg.weights.w4_area * normalized_area
        - penalty
    )
    return score, centre, slope_deg, normalized_clutter


def score_candidates(
    segmentation: SegmentationFrame,
    altitude_m: float,
    person_ground: GroundPoint,
    projector: GroundProjector,
    cfg: LandingZoneConfig,
) -> list[ZoneCandidate]:
    """Full §2.1 pipeline: rasterize -> find safe components -> score ->
    filter by min_score -> return the top ``cfg.top_k``, best first.

    Never raises on "no signal" conditions (empty terrain, no safe area,
    every candidate below threshold) - all correctly return `[]`, which the
    mission FSM's SEARCHING_ZONE state reads as "not found yet"."""
    terrain = rasterize_to_grid(segmentation, altitude_m, projector, cfg.cell_size_m)
    if terrain is None:
        return []

    candidates: list[ZoneCandidate] = []
    for component in _safe_components(terrain, cfg):
        area_m2 = len(component) * cfg.cell_size_m**2
        radius_m = math.sqrt(area_m2 / math.pi)
        if radius_m < cfg.min_safe_radius_m:
            continue

        def _rank_key(cell: tuple[int, int]) -> tuple[float, float, float]:
            # Prefer highest suitability, then lowest slope, then the cell
            # that least violates the r_min..r_max band - NOT raw distance,
            # which would fight the penalty term by always dragging the
            # representative cell toward the person regardless of r_min.
            r, c = cell
            suitability = cfg.surface_suitability[terrain.classes[r][c]]
            slope = terrain.slope_deg[r][c]
            violation = _band_violation(person_ground.distance_to(terrain.cell_centre(r, c)), cfg)
            return (-suitability, slope, violation)

        rep_row, rep_col = min(component, key=_rank_key)
        score, centre, slope_deg, normalized_clutter = _score_cell(
            terrain, rep_row, rep_col, radius_m, person_ground, cfg
        )
        candidates.append(
            ZoneCandidate(
                score=score,
                centroid=centre,
                safe_radius_m=radius_m,
                surface_class=terrain.classes[rep_row][rep_col],
                slope_deg=slope_deg,
                clutter_density=normalized_clutter,
                cell_row=rep_row,
                cell_col=rep_col,
            )
        )

    candidates = [c for c in candidates if c.score >= cfg.min_score]
    candidates.sort(key=lambda c: c.score, reverse=True)
    return candidates[: cfg.top_k]
