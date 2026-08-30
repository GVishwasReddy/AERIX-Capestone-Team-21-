"""LiDAR field-of-view masking.

0 deg is the LiDAR's front reference (the face aimed at the nose, placed by
``lidar.angle_offset_deg``). 125 deg either side of it is scanned - 250 deg
total - and the 110 deg wedge directly behind is blanked before any consumer
sees it, because it permanently contains the airframe and whatever the drone
is parked next to. Narrowed from 270 deg on 2026-08-29: that ran the window to
the very edge of the clutter, leaving no grace for a remount or an object
sitting on the boundary.
"""
from __future__ import annotations

import math

from drone_stack.interfaces.lidar_interface import (
    DEFAULT_BINS,
    DEFAULT_FOV_DEG,
    FovMask,
    RealLidar,
)


def _scan_every_degree(distance_mm: float = 3000.0):
    """One synthetic return per whole degree, all at the same distance."""
    return [(47.0, float(a), distance_mm) for a in range(360)]


def _traceable_scan():
    """One return per whole degree, with the raw angle encoded in the distance."""
    return [(47.0, float(a), 1000.0 + a) for a in range(360)]


def _raw_angle_at(scan, idx: int) -> int:
    """Recover which raw beam angle ended up in bin ``idx``."""
    return round((scan.ranges[idx] - 1.0) * 1000.0)


# -- the mask itself ---------------------------------------------------------
def test_defaults_are_250_scanned_110_blind():
    fov = FovMask({})
    assert fov.enabled
    assert fov.fov_deg == DEFAULT_FOV_DEG == 250.0
    assert fov.half_deg == 125.0
    assert fov.blind_deg == 110.0


def test_window_is_125_left_and_125_right_of_the_front_line():
    fov = FovMask({})
    assert fov.keeps(0.0)                      # dead ahead
    assert fov.keeps(125.0) and fov.keeps(-125.0)   # both boundaries, inclusive
    assert fov.keeps(235.0)                    # same beam as -125
    assert not fov.keeps(125.5)
    assert not fov.keeps(-125.5)
    assert not fov.keeps(135.0)                # inside the old 270 window
    assert not fov.keeps(180.0)                # dead astern
    assert not fov.keeps(200.0)


def test_blind_bins_are_exactly_the_rear_wedge():
    keep = FovMask({}).keep_bins(DEFAULT_BINS)
    blind = [i for i, ok in enumerate(keep) if not ok]
    # closed [-125, +125] window -> 251 kept bins, 109 strictly inside the wedge
    assert blind == list(range(126, 235))
    assert sum(keep) == 251
    assert all(abs(((i + 180) % 360) - 180) > 125 for i in blind)


def test_apply_blanks_ranges_and_intensities_in_place():
    ranges = [1.0] * DEFAULT_BINS
    intensities = [47.0] * DEFAULT_BINS
    FovMask({}).apply(ranges, intensities)
    assert ranges[0] == 1.0 and ranges[125] == 1.0 and ranges[235] == 1.0
    assert math.isinf(ranges[180]) and intensities[180] == 0.0
    assert math.isinf(ranges[135])          # was inside the old 270 window
    assert sum(1 for r in ranges if math.isfinite(r)) == 251


def test_mask_can_be_disabled_or_widened_to_a_full_circle():
    assert all(FovMask({"fov_enabled": False}).keep_bins(DEFAULT_BINS))
    assert all(FovMask({"fov_deg": 360.0}).keep_bins(DEFAULT_BINS))
    narrow = FovMask({"fov_deg": 180.0})
    assert narrow.half_deg == 90.0 and narrow.blind_deg == 180.0
    assert narrow.keeps(90.0) and not narrow.keeps(91.0)


# -- the real driver ---------------------------------------------------------
def test_real_driver_never_emits_a_beam_from_the_rear_wedge():
    lidar = RealLidar({"min_range_m": 0.15, "max_range_m": 12.0})
    scan = lidar._to_laserscan(_scan_every_degree())
    assert scan.count == DEFAULT_BINS               # layout unchanged
    finite = [i for i, r in enumerate(scan.ranges) if math.isfinite(r)]
    assert finite[0] == 0 and 125 in finite and 235 in finite
    assert not any(126 <= i <= 234 for i in finite)
    assert len(finite) == 251


def test_front_reference_follows_the_mounting_offset():
    """angle_offset_deg is the RAW angle aimed at the nose; the window follows.

    See tests/test_lidar_handedness.py for why the raw angle is negated before
    the offset is applied.
    """
    lidar = RealLidar({"angle_offset_deg": 90.0})
    scan = lidar._to_laserscan(_traceable_scan())
    # the nose sits at raw 90 deg, so that beam must come out dead ahead
    assert _raw_angle_at(scan, 0) == 90
    # which puts raw 270 deg dead astern -> inside the blind wedge
    assert math.isinf(scan.ranges[180])


def test_disabled_mask_restores_the_full_circle():
    lidar = RealLidar({"fov_enabled": False})
    scan = lidar._to_laserscan(_scan_every_degree())
    assert all(math.isfinite(r) for r in scan.ranges)
