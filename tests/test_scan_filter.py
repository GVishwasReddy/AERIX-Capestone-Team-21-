"""LiDAR noise rejection - the quality gate and the persistence filter.

Sized from a measurement on the real aircraft (bench, 2026-09-21, 202
revolutions at 10.1 Hz), not from taste:

* the C1's ranges are already good - 2.4 cm median spread on a bin a real
  surface occupies, 0.1% of consecutive frames moving more than a cluster gap;
* what it actually does is flicker - 12.6% of the bins that ever hold a return
  hold one in a quarter of revolutions or fewer;
* and the quality byte separates the two, median 25 on solid bins against 2 on
  flickering ones.

So the tests below pin *persistence* behaviour, not smoothing behaviour.

⚠️ The load-bearing test in this file is
``test_a_pole_drifting_under_a_turning_airframe_is_not_erased``. Every other
test here is about throwing noise away; that one is about NOT throwing away a
real obstacle, and it is the failure a static bench can never show you. Per
CLAUDE.md 8, each of these was falsified against the unfiltered code before
being trusted.
"""
from __future__ import annotations

import math

from drone_stack.interfaces.lidar_interface import (
    DEFAULT_QUALITY_MIN,
    RealLidar,
    ScanFilter,
)
from drone_stack.utils.config import Config

BINS = 36


def _ranges(hits: dict[int, float], bins: int = BINS) -> list[float]:
    """A scan with returns only at the given bins."""
    out = [math.inf] * bins
    for idx, rng in hits.items():
        out[idx] = rng
    return out


def _filter(**over) -> ScanFilter:
    cfg = {
        "filter_enabled": True,
        "filter_depth": 3,
        "filter_min_support": 2,
        "filter_angular_tol_bins": 3,
        "filter_range_tol_m": 0.30,
        "filter_fast_approach": True,
    }
    cfg.update(over)
    return ScanFilter(cfg)


def _warm(f: ScanFilter, frame: list[float]) -> None:
    """Push ``frame`` through until the filter is past its warm-up."""
    for _ in range(f.depth):
        f.apply(list(frame))


def _finite(ranges: list[float]) -> dict[int, float]:
    return {i: r for i, r in enumerate(ranges) if math.isfinite(r)}


# -- stage 1: the per-sample quality gate ------------------------------------
def test_a_weak_return_is_dropped_at_the_sample():
    """Below the gate a sample never even competes for its bin."""
    scan = RealLidar({"quality_min": 4})._to_laserscan([(2.0, 0.0, 3000.0)])
    assert _finite(scan.ranges) == {}


def test_a_confident_return_survives_the_gate():
    scan = RealLidar({"quality_min": 4})._to_laserscan([(25.0, 0.0, 3000.0)])
    assert _finite(scan.ranges) == {0: 3.0}


def test_the_threshold_is_the_measured_knee_not_a_guess():
    """4 keeps 99.8% of solid-bin signal; higher starts destroying real returns."""
    assert DEFAULT_QUALITY_MIN == 4


def test_the_gate_can_be_disabled():
    scan = RealLidar({"quality_min": 0})._to_laserscan([(1.0, 0.0, 3000.0)])
    assert _finite(scan.ranges) == {0: 3.0}


def test_the_gate_leaves_ordinary_returns_alone():
    """The quality the rest of the suite and the simulator emit must pass."""
    scan = RealLidar({})._to_laserscan([(47.0, 0.0, 3000.0)])
    assert _finite(scan.ranges) == {0: 3.0}


# -- stage 2: persistence ----------------------------------------------------
def test_warm_up_passes_the_raw_scan_through():
    """Better a noisy radar for 300 ms than an empty one."""
    f = _filter()
    frame = _ranges({10: 4.0})
    for _ in range(f.depth):
        assert _finite(f.apply(list(frame))) == {10: 4.0}


def test_a_surface_seen_every_revolution_passes():
    f = _filter()
    frame = _ranges({10: 4.0})
    _warm(f, frame)
    assert _finite(f.apply(list(frame))) == {10: 4.0}


def test_a_one_frame_speckle_never_reaches_the_scan():
    """The 12.6% of bins that flicker are what reach avoidance as phantoms."""
    f = _filter()
    _warm(f, _ranges({10: 4.0}))
    out = f.apply(_ranges({10: 4.0, 25: 2.0}))   # 25 appears for one frame
    assert _finite(out) == {10: 4.0}
    assert f.rejected == 1


def test_an_isolated_return_with_no_history_at_all_is_rejected():
    f = _filter()
    _warm(f, _ranges({}))
    assert _finite(f.apply(_ranges({7: 5.0}))) == {}


def test_the_filter_only_ever_suppresses_never_invents():
    """Safety property: it can be late, but it must never manufacture range."""
    f = _filter()
    _warm(f, _ranges({10: 4.0}))
    src = _ranges({10: 4.0, 20: 6.0, 30: 1.2})
    out = f.apply(list(src))
    for i, r in enumerate(out):
        assert not math.isfinite(r) or r == src[i]


# -- the one that matters: a real obstacle must survive ----------------------
def test_a_pole_drifting_under_a_turning_airframe_is_not_erased():
    """⚠️ The regression this filter could most easily cause.

    avoidance_steer_rate_deg_s is 10.0, so across a depth-3 window at 10 Hz the
    airframe turns 3 deg - three 1 deg bins - while a 0.3 m pole at 8 m is only
    about two bins wide. A strict per-bin test would watch the pole slide out of
    its bin and blank it, exactly while the aircraft is turning to avoid it.
    """
    f = _filter()
    for idx in (10, 11, 12):                      # the pole drifts one bin/frame
        f.apply(_ranges({idx: 8.0}))
    out = f.apply(_ranges({13: 8.0}))
    assert _finite(out) == {13: 8.0}, "angular tolerance failed - real pole erased"


def test_a_pole_is_erased_without_angular_tolerance():
    """Pins WHY the tolerance exists: at 0 bins the same pole disappears."""
    f = _filter(filter_angular_tol_bins=0)
    for idx in (10, 11, 12):
        f.apply(_ranges({idx: 8.0}))
    assert _finite(f.apply(_ranges({13: 8.0}))) == {}


def test_a_distant_wall_does_not_vouch_for_a_near_speckle():
    """Angular grace must not become range amnesty."""
    f = _filter()
    _warm(f, _ranges({10: 10.0}))                 # a wall, every revolution
    out = f.apply(_ranges({10: 10.0, 12: 2.0}))   # speckle 2 bins away, 8 m nearer
    assert _finite(out) == {10: 10.0}


# -- asymmetry: closing is worth latency, receding is not --------------------
def test_a_closing_return_is_published_a_revolution_early():
    f = _filter()
    _warm(f, _ranges({}))
    f.apply(_ranges({15: 5.00}))                  # first sighting - rejected
    out = f.apply(_ranges({15: 4.80}))            # nearer, corroborated once
    assert _finite(out) == {15: 4.80}


def test_one_frame_alone_never_takes_the_fast_path():
    """A single close reading is the stray that made the first flight unflyable."""
    f = _filter()
    _warm(f, _ranges({}))
    assert _finite(f.apply(_ranges({15: 4.0}))) == {}


def test_a_receding_return_still_has_to_win_the_gate():
    f = _filter()
    _warm(f, _ranges({}))
    f.apply(_ranges({15: 4.80}))
    assert _finite(f.apply(_ranges({15: 5.00}))) == {}


def test_a_jump_larger_than_the_range_tolerance_is_not_an_approach():
    """Closing fast is one object; teleporting 4 m in 100 ms is two."""
    f = _filter()
    _warm(f, _ranges({}))
    f.apply(_ranges({15: 6.0}))
    assert _finite(f.apply(_ranges({15: 2.0}))) == {}


def test_fast_approach_can_be_turned_off():
    f = _filter(filter_fast_approach=False)
    _warm(f, _ranges({}))
    f.apply(_ranges({15: 5.00}))
    assert _finite(f.apply(_ranges({15: 4.80}))) == {}


# -- plumbing ----------------------------------------------------------------
def test_disabled_filter_is_a_passthrough():
    f = _filter(filter_enabled=False)
    src = _ranges({3: 1.0})
    assert f.apply(list(src)) == src


def test_reset_clears_the_evidence():
    f = _filter()
    _warm(f, _ranges({10: 4.0}))
    f.reset()
    assert _finite(f.apply(_ranges({10: 4.0}))) == {10: 4.0}   # warming up again


def test_describe_reports_what_it_rejected():
    f = _filter()
    _warm(f, _ranges({10: 4.0}))
    f.apply(_ranges({10: 4.0, 25: 2.0}))
    d = f.describe()
    assert d["enabled"] and d["rejected"] == 1 and d["passed"] == 1


def test_every_shipped_profile_rejects_noise():
    """CLAUDE.md 5: config is the source of truth - and every profile must agree.

    real.yaml is what the aircraft flies; default.yaml is what a bare
    Config.load() in a test or a script gets. A profile that silently shipped
    the filter disabled would put raw scans back under NavigationNode, which
    since CLAUDE.md 12 is the thing that actually steers. Same shape as
    test_every_shipped_profile_satisfies_the_clearance_invariant in
    test_smooth_avoidance.py, and for the same reason: the invariant must not
    be breakable by editing YAML.
    """
    import pathlib

    root = pathlib.Path(__file__).resolve().parents[1] / "config"
    checked = 0
    for name in ("default.yaml", "real.yaml", "sim.yaml"):
        path = root / name
        if not path.exists():
            continue
        lidar = Config.load(str(path)).section("lidar")
        assert lidar.get("filter_enabled", True) is True, (
            "%s: persistence filter disabled - raw scans reach NavigationNode" % name)
        quality = int(lidar.get("quality_min", DEFAULT_QUALITY_MIN))
        assert quality >= 1, "%s: quality gate off" % name
        assert quality <= 6, (
            "%s: quality_min %d is past the measured knee - beyond ~6 the gate "
            "destroys returns off real surfaces and the flicker count climbs "
            "back (see DEFAULT_QUALITY_MIN)" % (name, quality))
        assert int(lidar.get("filter_min_support", 2)) >= 2, (
            "%s: one revolution must never be enough - that is the stray return "
            "that made the first manual flight unflyable" % name)
        assert int(lidar.get("filter_angular_tol_bins", 3)) >= 1, (
            "%s: zero angular tolerance erases a real pole while the airframe "
            "turns under it" % name)
        checked += 1
    assert checked >= 2
