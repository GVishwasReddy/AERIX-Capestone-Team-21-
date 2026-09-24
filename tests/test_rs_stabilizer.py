"""Rolling-shutter ("jello") correction, focus and exposure - hardware-free.

The frames are synthetic: a textured picture bent row by row with a KNOWN
displacement, which is exactly what a rolling-shutter sensor does under
vibration (each row is read at a different instant, so each row sits where the
camera was at that instant). Every test here checks against that ground truth,
so a sign error anywhere in measure -> accumulate -> unbend shows up as the
correction making the picture worse instead of better.
"""
from __future__ import annotations

import math

import numpy as np
import pytest

cv2 = pytest.importorskip("cv2")

from drone_stack.gcs.cameras import CameraManager, PiCamera
from drone_stack.gcs.replay_render import plan_corrections, plan_shear
from drone_stack.gcs.stabilizer import (IDENTITY, RsWarp, VideoStabilizer, band_nodes,
                                        correction_matrix, fit_banded, measure_motion,
                                        stabilizer_from_config)


def _textured(w, h, seed=3):
    rng = np.random.default_rng(seed)
    base = (rng.random((h // 8, w // 8, 3)) * 255).astype(np.uint8)
    return cv2.GaussianBlur(cv2.resize(base, (w, h), interpolation=cv2.INTER_CUBIC), (0, 0), 1.2)


def _bend(img, fx, fy):
    """Row y's content moved by (fx(y), fy(y)) - a rolling-shutter frame."""
    h, w = img.shape[:2]
    ys = np.arange(h, dtype=np.float32)
    mx = np.arange(w, dtype=np.float32)[None, :] - fx(ys)[:, None].astype(np.float32)
    my = (ys - fy(ys))[:, None].astype(np.float32) + np.zeros((1, w), np.float32)
    return cv2.remap(img, mx, my, cv2.INTER_LINEAR, borderMode=cv2.BORDER_REFLECT)


def _gray(img):
    return cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)


def _err(a, b, pad=40):
    """Mean abs difference away from the borders."""
    return float(np.mean(np.abs(a[pad:-pad, pad:-pad].astype(np.float32)
                                - b[pad:-pad, pad:-pad].astype(np.float32))))


# ------------------------------------------------------------ the model ---
def test_fit_banded_separates_roll_from_a_rolling_shutter_bend():
    rng = np.random.default_rng(1)
    w, h = 576.0, 324.0
    p0 = np.column_stack([rng.uniform(0, w, 600), rng.uniform(0, h, 600)])
    th = math.radians(0.8)
    c = np.array([w / 2, h / 2])
    R = np.array([[math.cos(th), -math.sin(th)], [math.sin(th), math.cos(th)]])

    def shear(y):   # a wobble down the frame, zero at the centre row
        return np.column_stack([4.0 * np.sin(2 * np.pi * (y - h / 2) / h),
                                8.0 * ((y - h / 2) / h) ** 2])
    s0 = shear(np.array([h / 2]))[0]
    p1 = (p0 - c) @ R.T + c + (3.0, -2.0) + shear(p0[:, 1])
    # 15% of the points belong to something moving on its own.
    bad = rng.random(600) < 0.15
    p1[bad] += rng.uniform(-25, 25, (int(bad.sum()), 2))

    A, off, inl = fit_banded(p0, p1, w, h, bands=12)
    assert math.degrees(math.atan2(A[1, 0], A[0, 0])) == pytest.approx(0.8, abs=0.08)
    # Centre-row translation = the rigid part + the bend at the centre row.
    centre = A @ np.array([w / 2, h / 2, 1.0]) - c
    np.testing.assert_allclose(centre, np.array([3.0, -2.0]) + s0, atol=0.3)
    want = shear(band_nodes(h, 12)) - s0
    np.testing.assert_allclose(off, want, atol=0.6)
    assert inl[bad].mean() < 0.1 and inl[~bad].mean() > 0.9


def test_a_rigid_move_measures_no_bend():
    img = _textured(576, 324)
    moved = cv2.warpAffine(img, np.float64([[1, 0, 5], [0, 1, -3]]), (576, 324),
                           borderMode=cv2.BORDER_REFLECT)
    A, off = measure_motion(_gray(img), _gray(moved))
    assert A[0, 2] == pytest.approx(5.0, abs=0.3) and A[1, 2] == pytest.approx(-3.0, abs=0.3)
    assert np.abs(off).max() < 0.5


# ------------------------------------------- measure -> unbend, sign check ---
def _wobble(y, h=324, amp=6.0):
    return amp * np.sin(2 * np.pi * 1.3 * (y - h / 2) / h)


def test_the_measured_bend_straightens_the_frame_it_came_from():
    """End to end: measure the bend between a straight and a bent frame, feed
    it to RsWarp, and the bent frame must come out closer to the straight one.
    With the sign flipped anywhere, it comes out worse."""
    img = _textured(576, 324)
    bent = _bend(img, lambda y: _wobble(y), lambda y: 0.5 * _wobble(y))
    A, off = measure_motion(_gray(img), _gray(bent))
    M = correction_matrix(0, 0, 0, 576, 324, 480, 270)
    want = cv2.warpAffine(img, M, (480, 270))
    rigid = cv2.warpAffine(bent, M, (480, 270))
    fixed = RsWarp().warp(bent, M, off, 480, 270)
    assert _err(fixed, want) < 0.35 * _err(rigid, want)
    flipped = RsWarp().warp(bent, M, -off, 480, 270)
    assert _err(flipped, want) > _err(rigid, want)


def test_a_negligible_bend_takes_the_plain_affine_path():
    img = _textured(576, 324)
    M = correction_matrix(3, -2, 0.01, 576, 324, 480, 270)
    a = RsWarp().warp(img, M, np.full((12, 2), 0.05), 480, 270)
    b = cv2.warpAffine(img, M, (480, 270), flags=cv2.INTER_LINEAR,
                       borderMode=cv2.BORDER_REPLICATE)
    np.testing.assert_array_equal(a, b)


# ------------------------------------------------------ replay planning ---
def test_plan_shear_recovers_a_vibrating_bend_and_respects_its_limit():
    nodes = band_nodes(324, 12)
    shape = np.column_stack([np.sin(nodes / 324 * np.pi * 2), np.zeros(12)])
    n = 200
    bend = [5.0 * math.sin(2 * math.pi * 8 * i / 30) * shape for i in range(n)]
    # What the measurement sees: the CHANGE in bend between frames.
    offsets = [None] + [bend[i] - bend[i - 1] for i in range(1, n)]
    D, reserve = plan_shear(offsets, sigma=8.0, limit_px=100.0)
    got = np.array(D)[30:-30]
    np.testing.assert_allclose(got, np.array(bend)[30:-30], atol=0.6)
    D2, r2 = plan_shear(offsets, sigma=8.0, limit_px=2.0)
    assert r2 == 2.0 and np.abs(D2).max() <= 2.0


def test_plan_shear_with_nothing_measured_is_a_no_op():
    assert plan_shear([None, None, None], 8.0, 40.0) == (None, 0.0)
    flat = [np.zeros((12, 2))] * 50
    assert plan_shear(flat, 8.0, 40.0) == (None, 0.0)


def test_the_shear_reserve_shrinks_the_global_margin():
    motions = [None] + [np.float64([[1, 0, 400 * (-1) ** i], [0, 1, 0]]) for i in range(59)]
    free = plan_corrections(motions, 2304, 1296, 1920, 1080, sigma=5.0)
    held = plan_corrections(motions, 2304, 1296, 1920, 1080, sigma=5.0, reserve_px=30.0)
    cx = lambda mats: np.abs(np.array([M[0, 2] + 192 for M in mats])).max()
    assert cx(free) <= 192.01
    assert cx(held) <= 162.01 and cx(held) < cx(free)


# ------------------------------------------------------ live stabiliser ---
def _vibrating_sequence(n=60):
    img = _textured(1536, 864)
    frames = []
    for i in range(n):
        a = 10.0 * math.sin(2 * math.pi * 9 * i / 30.0)
        frames.append(_bend(img, lambda y, a=a: a * np.sin(2 * np.pi * (y - 432) / 864),
                            lambda y: np.zeros_like(y)))
    return img, frames


def test_live_unbend_is_off_unless_configured():
    stab = stabilizer_from_config({"enabled": True}, (1280, 720), 30)
    assert stab.rs_bands == 0
    img = _textured(1536, 864)
    stab.step(img)
    assert stab.step(img).rs is None


def test_live_unbend_reduces_jello_on_the_published_picture():
    img, frames = _vibrating_sequence()
    errs = {}
    for bands in (0, 12):
        stab = VideoStabilizer(out_size=(1280, 720), rs_bands=bands)
        e = []
        for i, f in enumerate(frames):
            res = stab.step(f)
            out = stab.render(f, res.matrix, res.rs)
            if i >= 10:
                e.append(_err(out, cv2.warpAffine(img, res.matrix, (1280, 720))))
        errs[bands] = float(np.mean(e))
    assert errs[12] < 0.6 * errs[0]
    assert stab.stats()["rs_px"] > 0


# ------------------------------------------------------- focus / exposure ---
def test_custom_exposure_table_is_validated():
    ok = PiCamera.custom_exposure_mode({"shutter_us": [100, 3000, 10000], "gain": [1, 4, 8]})
    assert ok == {"shutter": [100, 3000, 10000], "gain": [1.0, 4.0, 8.0]}
    for bad in (None, {}, {"shutter_us": [100], "gain": [1]},
                {"shutter_us": [100, 50], "gain": [1, 2]},
                {"shutter_us": [100, 200], "gain": [1, 2, 3]},
                {"shutter_us": [0, 200], "gain": [1, 2]},
                {"shutter_us": [100, 200], "gain": [0.5, 2]},
                {"shutter_us": ["x", 200], "gain": [1, 2]}):
        assert PiCamera.custom_exposure_mode(bad) is None


def test_real_config_fixes_focus_short_shutter_and_unbends():
    from pathlib import Path
    from drone_stack.utils.config import Config
    real = Path(__file__).resolve().parents[1] / "config" / "real.yaml"
    cams = Config.load(real).get("cameras", {})
    pi = CameraManager(bus=None, enabled=True, settings=cams).get(1)
    # 1 m was the stock lens position; everything the aircraft looks at is further.
    assert 0.0 < pi._lens_position < 1.0
    assert pi._ae_mode == "custom"
    table = PiCamera.custom_exposure_mode(pi._ae_custom)
    assert table is not None and table["shutter"][2] <= 4000
    assert pi._stab.rs_bands >= 6


def test_lens_position_reaches_the_controls():
    pytest.importorskip("libcamera")
    cam = PiCamera(1, "T", lens_position=0.45)
    ctl = cam._sensor_controls()
    assert ctl["LensPosition"] == pytest.approx(0.45)
    assert "AfMode" in ctl
    assert "LensPosition" not in PiCamera(1, "T")._sensor_controls()
