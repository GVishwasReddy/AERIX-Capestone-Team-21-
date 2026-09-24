"""Stabiliser, person lock, replay planning and the recorder's record stream.

All hardware-free: no camera, no Hailo. The NPU is replaced by a fake detector
that returns scripted boxes, so the lock policy can be tested frame by frame.
"""
from __future__ import annotations

import json
import math

import numpy as np
import pytest

cv2 = pytest.importorskip("cv2")

from drone_stack.gcs import person_lock as pl
from drone_stack.gcs.cameras import CameraManager, _BaseCamera
from drone_stack.gcs.recorder import FlightRecorder
from drone_stack.gcs.replay_render import gauss_smooth, load_index, plan_corrections
from drone_stack.gcs.stabilizer import (IDENTITY, VideoStabilizer, correction_matrix,
                                        similarity_params, soft_limit)


# ------------------------------------------------------------- stabiliser ---
def _textured(w=1536, h=864, seed=3):
    rng = np.random.default_rng(seed)
    base = (rng.random((h // 8, w // 8, 3)) * 255).astype(np.uint8)
    return cv2.GaussianBlur(cv2.resize(base, (w, h), interpolation=cv2.INTER_CUBIC), (0, 0), 1.5)


def test_similarity_params_measure_translation_at_the_centre():
    A = cv2.getRotationMatrix2D((768, 432), 1.0, 1.0)
    A[:, 2] += (7.0, -4.0)
    dx, dy, da = similarity_params(A, 768, 432)
    assert dx == pytest.approx(7.0, abs=1e-6)
    assert dy == pytest.approx(-4.0, abs=1e-6)
    assert math.degrees(da) == pytest.approx(-1.0, abs=1e-6)   # cv2 angle is CCW


def test_soft_limit_never_exceeds_the_margin_and_is_linear_below_the_knee():
    assert soft_limit(10.0, 100.0) == 10.0
    assert soft_limit(1e6, 100.0) <= 100.0
    assert soft_limit(-1e6, 100.0) >= -100.0
    assert soft_limit(5.0, 0.0) == 0.0


def test_correction_matrix_with_no_correction_is_a_centred_crop():
    M = correction_matrix(0, 0, 0, 1536, 864, 1280, 720)
    np.testing.assert_allclose(M, [[1, 0, -128], [0, 1, -72]], atol=1e-9)


def test_stabiliser_recovers_a_known_shift():
    img = _textured()
    stab = VideoStabilizer(out_size=(1280, 720))
    stab.step(img)
    shifted = cv2.warpAffine(img, np.float64([[1, 0, 6], [0, 1, -3]]), (1536, 864),
                             borderMode=cv2.BORDER_REFLECT)
    res = stab.step(shifted)
    assert res.ok
    assert res.motion[0, 2] == pytest.approx(6.0, abs=1.0)
    assert res.motion[1, 2] == pytest.approx(-3.0, abs=1.0)


def test_stabiliser_removes_vibration_from_the_published_picture():
    """A 10 Hz shake of +-12 px must come out much smaller."""
    img = _textured()
    stab = VideoStabilizer(out_size=(1280, 720), smooth_hz=2.0)
    raw, out = [], []
    for i in range(90):
        dx = 12.0 * math.sin(2 * math.pi * 10 * i / 30.0 + 0.3)
        frame = cv2.warpAffine(img, np.float64([[1, 0, dx], [0, 1, 0]]), (1536, 864),
                               borderMode=cv2.BORDER_REFLECT)
        res = stab.step(frame)
        raw.append(dx)
        # Where does a fixed scene point land in the published frame?
        out.append(res.matrix[0, 2] + dx)
    raw, out = np.array(raw[30:]), np.array(out[30:])
    assert np.std(out) < 0.5 * np.std(raw)


def test_a_featureless_frame_is_treated_as_still_not_as_a_jump():
    stab = VideoStabilizer(out_size=(1280, 720))
    flat = np.full((864, 1536, 3), 90, np.uint8)
    stab.step(flat)
    res = stab.step(flat)
    assert not res.ok
    np.testing.assert_allclose(res.motion, IDENTITY)


def test_render_returns_a_published_size_frame():
    stab = VideoStabilizer(out_size=(1280, 720))
    img = _textured()
    res = stab.step(img)
    assert stab.render(img, res.matrix).shape == (720, 1280, 3)


# ---------------------------------------------------------------- lock ------
class _FakeDetector:
    """Stands in for HailoPersonDetector: returns a scripted result whenever
    a frame is submitted and polled."""

    def __init__(self):
        self.script = []
        self._pending = None

    def start(self):
        pass

    def stop(self):
        pass

    def try_submit(self, seq, frame):
        self._pending = (seq, self.script.pop(0) if self.script else [])
        return True

    def poll(self):
        out, self._pending = self._pending, None
        return out

    def stats(self):
        return {"npu": "ready", "det_hz": 30.0, "infer_ms": 1.0}


def _run(pipe, frames, dets_per_frame, t0=0.0, dt=1 / 30):
    snaps = []
    for i in range(frames):
        pipe.detector.script.append(dets_per_frame(i))
        snaps.append(pipe.process(np.zeros((720, 1280, 3), np.uint8), None, None,
                                  now=t0 + i * dt))
    return snaps


def _pipe(**kw):
    pipe = pl.PersonLockPipeline(_FakeDetector(), pl.TargetLock(**kw))
    # In production the mission FSM sets this True on HOVER/IDLE
    # (cameras.py). Without it PersonLockPipeline.process() early-returns an
    # empty snapshot, so the lock can never acquire and every "box is not
    # None" assertion below would fail while the "box is None" ones pass
    # trivially. Enabling here makes the drop/acquire tests actually exercise
    # the lock policy.
    pipe.enabled = True
    return pipe


def test_host_nms_keeps_the_best_of_overlapping_boxes():
    dets = [(0, 0, 100, 200, 0.5), (5, 5, 105, 205, 0.9), (500, 0, 600, 200, 0.4)]
    kept = pl.host_nms(dets, 0.5)
    assert [d[4] for d in kept] == [0.9, 0.4]


def test_parse_nms_buffer_undoes_the_letterbox():
    # 1280x720 in 1280x1280: pad_y = 280. A box covering the whole image.
    flat = np.zeros(11, np.float32)
    flat[0] = 1
    flat[1:6] = (280 / 1280, 0.0, 1000 / 1280, 1.0, 0.8)
    out = pl.parse_nms_buffer(flat, (1280, 1280, 0, 280, 1.0, 1280, 720), 0.2)
    assert len(out) == 1
    x1, y1, x2, y2, s = out[0]
    assert (x1, y1) == pytest.approx((0, 0), abs=1e-3)
    assert (x2, y2) == pytest.approx((1279, 719), abs=1e-3)


def test_parse_multihead_yolo_decodes_ltrb_box_at_cell_centre():
    # 2x2 score grid on a 640 input -> stride 320. One hot cell at (gy=1,gx=1)
    # whose anchor centre is ((1+0.5)*320, ...) = (480, 480), with an LTRB box
    # of 1 stride (320 px) on every side. Correct decode:
    #   x1 = 480 - 320 = 160,  x2 = 480 + 320 = 800 -> clipped to fw-1 = 639.
    score = np.full((2, 2, 1), -50.0, np.float32)   # sigmoid ~ 0 everywhere
    score[1, 1, 0] = 10.0                            # sigmoid ~ 1 in this cell
    box = np.zeros((2, 2, 4), np.float32)
    box[1, 1] = (1.0, 1.0, 1.0, 1.0)                 # l, t, r, b = 1 stride each
    outs = {"cvbox": box, "cvscore": score}
    # geom = (w_in, h_in, pad_x, pad_y, scale, fw, fh); no letterbox pad here.
    dets = pl.parse_multihead_yolo(outs, (640, 640, 0, 0, 1.0, 640, 640), 0.2)
    assert len(dets) == 1
    x1, y1, x2, y2, s = dets[0]
    assert (x1, y1, x2, y2) == pytest.approx((160.0, 160.0, 639.0, 639.0), abs=1e-3)
    assert s > 0.99


def test_a_single_frame_false_positive_never_draws_a_box():
    pipe = _pipe()
    person = (400, 200, 500, 500, 0.8)
    snaps = _run(pipe, 10, lambda i: [person] if i == 0 else [])
    assert all(s["box"] is None for s in snaps)


def test_the_most_confident_person_is_locked():
    pipe = _pipe()
    weak, strong = (100, 200, 200, 500, 0.55), (800, 200, 900, 500, 0.9)
    snaps = _run(pipe, 5, lambda i: [weak, strong])
    box = snaps[-1]["box"]
    assert box is not None and 790 < (box[0] + box[2]) / 2 < 910


def test_the_lock_survives_missed_detections_without_flicker():
    pipe = _pipe(hold_s=1.5)
    person = (400, 200, 500, 500, 0.8)
    # Detected for 5 frames, then the detector misses 20 frames (0.67 s).
    snaps = _run(pipe, 30, lambda i: [person] if i < 5 else [])
    assert all(s["box"] is not None for s in snaps[2:])
    assert snaps[-1]["state"] == "hold"


def test_the_lock_is_dropped_after_the_hold_time():
    pipe = _pipe(hold_s=0.5)
    person = (400, 200, 500, 500, 0.8)
    snaps = _run(pipe, 40, lambda i: [person] if i < 5 else [])
    assert snaps[-1]["box"] is None


def test_a_low_confidence_match_keeps_an_existing_lock():
    pipe = _pipe()
    snaps = _run(pipe, 20, lambda i: [(400, 200, 500, 500, 0.8 if i < 4 else 0.25)])
    assert snaps[-1]["state"] == "lock"


def test_another_person_cannot_steal_the_lock():
    pipe = _pipe()
    target, other = (400, 200, 500, 500, 0.6), (1000, 200, 1100, 500, 0.95)
    snaps = _run(pipe, 30, lambda i: [target] if i < 4 else [target, other])
    box = snaps[-1]["box"]
    assert 390 < (box[0] + box[2]) / 2 < 510


def test_the_box_follows_a_walking_person_smoothly():
    pipe = _pipe()
    snaps = _run(pipe, 60, lambda i: [(300 + 4 * i, 200, 400 + 4 * i, 500, 0.8)])
    cx = np.array([(s["box"][0] + s["box"][2]) / 2 for s in snaps[10:]])
    truth = np.array([350 + 4 * i for i in range(10, 60)])
    assert np.max(np.abs(cx - truth)) < 12
    assert np.all(np.diff(cx) > -2)          # never jumps backwards


def test_the_lock_is_dropped_once_the_person_leaves_the_frame():
    pipe = _pipe(exit_s=0.2, hold_s=5.0)
    # Walk right off the 1280-wide frame, fast.
    snaps = _run(pipe, 40, lambda i: [(1000 + 30 * i, 200, 1100 + 30 * i, 500, 0.8)]
                 if i < 12 else [])
    assert snaps[-1]["box"] is None


def test_camera_motion_does_not_throw_the_box_off_the_person():
    """The whole scene shifts 20 px right between frames (the camera moved).
    Applying that motion to the lock keeps the box on the person even with no
    detection to correct it."""
    lock = pl.TargetLock()
    lock._start_lock((400, 200, 500, 500, 0.8), 0.0)
    A = np.float64([[1, 0, 20], [0, 1, 0]])
    lock.begin_frame(A)
    x1, _, x2, _, _ = lock.box()
    assert (x1 + x2) / 2 == pytest.approx(470, abs=1e-6)


def test_hud_layout_stays_inside_the_frame():
    lay = pl.hud_layout((0, 0, 60, 120), "lock", 0.9, 1280, 720)
    lx1, ly1, lx2, ly2 = lay["label_rect"]
    assert ly1 >= 0 and lx2 < 1280


def test_person_lock_config_is_lazy_and_optional():
    assert pl.person_lock_from_config(None) is None
    pipe = pl.person_lock_from_config({"enabled": True, "hef": "/nonexistent.hef"})
    assert pipe is not None and pipe.detector.state == "idle"   # nothing loaded yet


# -------------------------------------------------------- replay planning ---
def test_zero_phase_smoothing_has_no_lag():
    t = np.arange(300, dtype=float)
    ramp = np.stack([t, t, t], 1)
    sm = gauss_smooth(ramp, 5.0)
    np.testing.assert_allclose(sm[20:-20], ramp[20:-20], atol=1e-6)


def test_plan_corrections_cancels_shake_within_the_margin():
    rng = np.random.default_rng(0)
    motions = []
    for _ in range(300):
        A = IDENTITY.copy()
        A[:, 2] = rng.normal(0, 6, 2)
        motions.append(A)
    mats = plan_corrections(motions, 2304, 1296, 1920, 1080, sigma=5.0)
    path = np.cumsum([m[:, 2] for m in motions], axis=0)
    # A scene point's position in the output = its path + the correction offset.
    shown = np.array([p + M[:, 2] + (192, 108) for p, M in zip(path, mats)])
    jitter_in = np.std(np.diff(path, axis=0))
    jitter_out = np.std(np.diff(shown, axis=0))
    assert jitter_out < 0.5 * jitter_in
    assert np.all(np.abs(np.array([M[:, 2] for M in mats]) + (192, 108)) <= (192.01, 108.01))


# --------------------------------------------------------- camera wiring ---
def test_min_height_floor_overrides_a_downscaling_rung():
    cam = _BaseCamera(1, "T", adaptive=True, ladder=[[1.0, 55], [0.5, 34]], min_height=720)
    cam._rung = 1
    _, out = cam._encode_frame(cv2, np.zeros((720, 1280, 3), np.uint8))
    assert out.shape[:2] == (720, 1280)


def test_real_config_enables_stabilisation_and_the_lock_without_touching_hardware():
    from pathlib import Path
    from drone_stack.utils.config import Config
    real = Path(__file__).resolve().parents[1] / "config" / "real.yaml"
    config = Config.load(real)
    mgr = CameraManager(bus=None, enabled=True, settings=config.get("cameras", {}))
    pi = mgr.get(1)
    assert pi._stab is not None and (pi._cap_w, pi._cap_h) == (1536, 864)
    assert pi._vision is not None and pi._vision.detector.state == "idle"
    assert pi._record_size == (2304, 1296)
    assert pi._min_height == 720
    assert all(s >= 1.0 for s, _ in pi._ladder)


# --------------------------------------------------------------- recorder ---
def test_recorder_writes_an_index_and_folds_dropped_motion(tmp_path, monkeypatch):
    monkeypatch.setattr("drone_stack.gcs.recorder._QUEUE_RAW_FRAMES", 1)
    rec = FlightRecorder({"dir": str(tmp_path), "min_seconds": 0})
    rec.set_geometry(main=(64, 48), lores=(32, 24), fps=30)
    rec.start("test")
    planes = (np.zeros((48, 64), np.uint8), np.full((24, 32), 128, np.uint8),
              np.full((24, 32), 128, np.uint8))
    shift = np.float64([[1, 0, 2], [0, 1, 0]])
    # Stall the writer so the queue fills, forcing drops.
    q = rec._queue
    import queue as _q
    blocker = _q.Queue(maxsize=1)
    rec._queue = blocker
    blocker.put_nowait(("x", {}))                 # full
    rec.offer_frame(planes, {"t": 0.0, "A": shift})   # dropped
    rec.offer_frame(planes, {"t": 0.1, "A": shift})   # dropped
    assert rec._dropped == 2
    np.testing.assert_allclose(rec._pending_motion[:2, 2], (4, 0))
    blocker.get_nowait()
    rec.offer_frame(planes, {"t": 0.2, "A": shift, "box": (1, 2, 3, 4), "st": "lock", "sc": 0.9})
    item = blocker.get_nowait()
    np.testing.assert_allclose(np.asarray(item[1]["A"])[:, 2], (6, 0))
    rec._queue = q
    import time as _t
    rec.offer_frame(planes, {"t": 0.3, "A": None})
    deadline = _t.monotonic() + 5
    while not q.empty() and _t.monotonic() < deadline:   # queue holds 1: let the writer take it
        _t.sleep(0.005)
    rec.offer_frame(planes, {"t": 0.4, "A": shift, "box": (1, 2, 3, 4), "st": "lock", "sc": 0.9})
    # Drain the writer directly (stop() would hand the files to the renderer).
    from drone_stack.gcs import recorder as recmod
    rec._queue = None
    q.put(recmod._SENTINEL)
    rec._writer.join(timeout=10)
    header, frames = load_index(rec._index)
    assert header["main"] == [64, 48] and header["lores"] == [32, 24]
    assert len(frames) == 2
    assert frames[0]["A"] is None
    assert frames[1]["A"][2] == pytest.approx(2.0) and frames[1]["box"] == [1, 2, 3, 4]
    data = rec._src.read_bytes()
    f = frames[1]
    assert data[f["o"]:f["o"] + 2] == b"\xff\xd8"
    assert data[f["o"] + f["n"] - 2:f["o"] + f["n"]] == b"\xff\xd9"
