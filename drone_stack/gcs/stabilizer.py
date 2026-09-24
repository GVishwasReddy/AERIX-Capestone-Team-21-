"""Digital image stabilisation for the Pi camera - live and causal.

Why this exists
---------------
The airframe shakes the camera hard. Measured on the 2026-09-14 flight clip the
picture moved 24/33 px (x/y, at 1280 wide) and rolled ~1 deg between
*consecutive* frames, with most of that energy between 3 and 15 Hz. That is the
"wobbly" video: the scene itself barely moves, the camera does.

A denoiser cannot help with that - wavelet/NLM/temporal filters remove noise
*inside* a frame, and every frame here is individually fine. The fix is
geometric: measure how the camera moved, decide how it *should* have moved
(smoothly), and shift each frame by the difference.

The chain, per frame
--------------------
1. **Measure.** Corners are tracked with pyramidal Lucas-Kanade on a 1/4-scale
   grey copy (384x216 of the 1536x864 ISP frame - ~4 ms of one core), and a
   RANSAC affine fit rejects anything that moves on its own (a person, a
   propeller blade) as an outlier. The fit is reduced to the three things a
   hand-held-style stabiliser corrects: x, y, roll - measured at the frame
   CENTRE so a rotation never masquerades as a translation.
2. **Smooth.** The accumulated camera path is followed by a critically-damped
   second-order filter (no overshoot, no ringing). Its corner frequency is the
   one knob that matters: below it motion is treated as intentional (a pan, a
   climb) and kept; above it is vibration and removed.
3. **Limit.** The correction can never exceed the crop margin - past it the
   frame edge would show. It is compressed with a soft knee rather than a hard
   wall: a hard clamp turns a big shake into a visible jerk exactly when the
   picture is already worst.
4. **Render.** ``similarity`` warps (rotation + translation, ~18 ms CPU at
   720p, or ~6 ms wall on 4 threads); ``translate`` is an integer crop and
   costs nothing but cannot remove roll.

Rolling shutter ("jello", the wavy lines)
-----------------------------------------
The imx708 reads its rows top to bottom over ~20 ms, so under vibration each
row sits where the camera was *at that row's instant* and straight lines come
out bent. A single rigid fit cannot describe such a frame: RANSAC locks onto
whichever strip of rows happens to agree, and the choice flips between frames -
which is itself jitter. With ``rs_bands > 0`` the measurement is instead a
shared roll/zoom plus a smooth per-row translation over K nodes
(``fit_banded``, Tukey IRLS with a light curvature penalty). The global path
is taken from the centre row, and the frame-to-frame *change* of the bend is
accumulated and high-passed (``rs_hp_hz``) into an unbend that ``RsWarp``
applies per output row. Replay does the same with zero-phase smoothing
(``replay_render.plan_shear``).

Honest limits
-------------
This is causal - it only knows the past - so it always lags a little. The
flight *recording* is stabilised again after landing with zero-phase smoothing
(``replay_render.py``), which measured roughly 3x steadier on the same data.
Only the bend's *change* between frames is observable, so a constant tilt of
the rows is left alone. Nothing here removes motion blur: that needs a short
shutter (``cameras.picam.ae_custom``) and a damped camera mount.
"""
from __future__ import annotations

import math
import time
from dataclasses import dataclass
from typing import Optional

import numpy as np

from drone_stack.utils.logging_setup import get_logger

_log = get_logger("gcs.stabilizer")

try:
    import cv2
except Exception:  # noqa: BLE001 - numpy-only environments (tests on a Mac)
    cv2 = None  # type: ignore


IDENTITY = np.array([[1.0, 0.0, 0.0], [0.0, 1.0, 0.0]], np.float64)


def similarity_params(A: np.ndarray, cx: float, cy: float) -> tuple[float, float, float]:
    """Reduce an inter-frame 2x3 affine to (dx, dy, dangle).

    Translation is taken as the displacement of the frame centre, not the
    affine's raw translation column: that column is the motion of pixel (0, 0),
    which a pure roll about the centre moves by hundreds of pixels.
    """
    a = math.atan2(A[1, 0] - A[0, 1], A[0, 0] + A[1, 1])
    px = A[0, 0] * cx + A[0, 1] * cy + A[0, 2]
    py = A[1, 0] * cx + A[1, 1] * cy + A[1, 2]
    return px - cx, py - cy, a


def soft_limit(value: float, limit: float, knee: float = 0.6) -> float:
    """Pass ``value`` untouched up to ``knee * limit``, then compress it
    smoothly so it approaches but never exceeds ``limit``."""
    if limit <= 0.0:
        return 0.0
    k = knee * limit
    a = abs(value)
    if a <= k:
        return value
    return math.copysign(k + (limit - k) * math.tanh((a - k) / (limit - k)), value)


def correction_matrix(dx: float, dy: float, angle: float, w: int, h: int,
                      out_w: int, out_h: int) -> np.ndarray:
    """2x3 map from a raw ``w x h`` frame to the centred ``out_w x out_h`` crop,
    rotated by ``angle`` about the raw centre and shifted by (dx, dy)."""
    c, s = math.cos(angle), math.sin(angle)
    cx, cy = w / 2.0, h / 2.0
    ox, oy = (w - out_w) / 2.0, (h - out_h) / 2.0
    return np.array([
        [c, -s, cx - c * cx + s * cy + dx - ox],
        [s, c, cy - s * cx - c * cy + dy - oy],
    ], np.float64)


def band_nodes(h: float, bands: int) -> np.ndarray:
    """Row positions of the rolling-shutter model's nodes, top to bottom."""
    return np.linspace(0.0, float(h), max(2, int(bands)))


def _hat_weights(y: np.ndarray, nodes: np.ndarray) -> np.ndarray:
    """N x K linear-interpolation weights of rows ``y`` onto ``nodes``."""
    k = len(nodes)
    pos = np.clip((y - nodes[0]) / max(nodes[-1] - nodes[0], 1e-9) * (k - 1), 0.0, k - 1.0)
    i0 = np.minimum(pos.astype(int), k - 2)
    f = pos - i0
    W = np.zeros((len(y), k))
    r = np.arange(len(y))
    W[r, i0] = 1.0 - f
    W[r, i0 + 1] = f
    return W


def fit_banded(p0: np.ndarray, p1: np.ndarray, w: float, h: float, bands: int = 12,
               reg: float = 0.1, iters: int = 4, floor_px: float = 0.25):
    """Robust inter-frame motion with a rolling-shutter term.

    Model, per tracked point: one roll + zoom about the frame centre shared by
    the whole frame, plus a translation ``t(y)`` that varies smoothly DOWN the
    frame (linear between ``bands`` nodes, curvature-penalised by ``reg``).
    A rolling-shutter sensor exposes each row at a different instant, so under
    vibration the rows genuinely move by different amounts - forcing one rigid
    transform onto that (as a plain RANSAC fit does) makes the estimate jump to
    whichever strip of rows happens to agree best, frame to frame.

    Tukey-weighted IRLS rejects things that move on their own (a person).
    Returns ``(A, offsets, inliers)``: ``A`` the 2x3 similarity for the CENTRE
    row, ``offsets`` (K x 2) each node's translation minus the centre's - the
    change in intra-frame bend between the two frames - or None if unfit."""
    n = len(p0)
    k = max(3, int(bands))
    if n < 2 * k:
        return None
    cx, cy, L = w / 2.0, h / 2.0, w / 2.0
    xc = (p0[:, 0] - cx) / L
    yc = (p0[:, 1] - cy) / L
    nodes = band_nodes(h, k)
    Wt = _hat_weights(p0[:, 1], nodes)
    z = np.zeros((n, k))
    A = np.vstack([np.hstack([xc[:, None], -yc[:, None], Wt, z]),
                   np.hstack([yc[:, None], xc[:, None], z, Wt])])
    d = (p1 - p0).astype(np.float64)
    b = np.concatenate([d[:, 0], d[:, 1]])
    D2 = np.zeros((k - 2, k))
    for i in range(k - 2):
        D2[i, i:i + 3] = (1.0, -2.0, 1.0)
    R = np.zeros((2 * (k - 2), 2 + 2 * k))
    R[:k - 2, 2:2 + k] = D2
    R[k - 2:, 2 + k:] = D2
    wts = np.ones(n)
    x = None
    for _ in range(max(1, iters)):
        ww = np.concatenate([wts, wts])
        if ww.sum() < 2 * k:
            return None
        sw = np.sqrt(ww)
        lam = math.sqrt(reg * ww.sum() / k)
        M = np.vstack([A * sw[:, None], lam * R])
        rhs = np.concatenate([b * sw, np.zeros(len(R))])
        x = np.linalg.lstsq(M, rhs, rcond=None)[0]
        res = A @ x - b
        rr = np.hypot(res[:n], res[n:])
        live = wts > 0
        sig = max(1.4826 * float(np.median(rr[live])), floor_px)
        c = 4.685 * sig
        wts = np.where(rr < c, (1.0 - (rr / c) ** 2) ** 2, 0.0)
    s, th = x[0] / L, x[1] / L
    T = np.stack([x[2:2 + k], x[2 + k:]], 1)
    tc = _hat_weights(np.array([cy]), nodes)[0] @ T
    lin = np.array([[1.0 + s, -th], [th, 1.0 + s]])
    Aff = np.hstack([lin, (np.array([cx, cy]) + tc - lin @ np.array([cx, cy]))[:, None]])
    return Aff, T - tc, wts > 0


def track_points(prev: np.ndarray, gray: np.ndarray, bands: int, per_band: int,
                 fb_max: float = 0.7):
    """Corners spread over every band (a flat sky or wall must not leave a
    band unmeasured), tracked forward and back; a track that does not return
    to within ``fb_max`` px of where it started is dropped."""
    h, w = prev.shape[:2]
    edges = np.linspace(0, h, max(1, bands) + 1).astype(int)
    pts = []
    for y0, y1 in zip(edges[:-1], edges[1:]):
        if y1 - y0 < 8:
            continue
        p = cv2.goodFeaturesToTrack(prev[y0:y1], per_band, 0.005, 6, blockSize=5)
        if p is not None:
            p = p.reshape(-1, 2)
            p[:, 1] += y0
            pts.append(p)
    if not pts:
        return None, None
    p0 = np.concatenate(pts).astype(np.float32).reshape(-1, 1, 2)
    lk = dict(winSize=(15, 15), maxLevel=3,
              criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 20, 0.03))
    p1, st, _ = cv2.calcOpticalFlowPyrLK(prev, gray, p0, None, **lk)
    pb, sb, _ = cv2.calcOpticalFlowPyrLK(gray, prev, p1, None, **lk)
    fb = np.linalg.norm((pb - p0).reshape(-1, 2), axis=1)
    good = (st.reshape(-1) == 1) & (sb.reshape(-1) == 1) & (fb < fb_max)
    return p0.reshape(-1, 2)[good], p1.reshape(-1, 2)[good]


def measure_motion(prev: np.ndarray, gray: np.ndarray, bands: int = 12,
                   per_band: int = 30, reg: float = 0.1):
    """``(A, offsets)`` between two grey frames in THEIR pixels, or None."""
    if cv2 is None:
        return None
    p0, p1 = track_points(prev, gray, bands, per_band)
    if p0 is None or len(p0) < 3 * bands:
        return None
    fit = fit_banded(p0, p1, gray.shape[1], gray.shape[0], bands, reg)
    if fit is None or int(fit[2].sum()) < 2 * bands:
        return None
    return fit[0], fit[1]


class RsWarp:
    """Global correction ``M`` (raw -> out, 2x3) plus a per-row shear.

    ``offsets`` (K x 2, raw px) is how far each node row's content sits from
    where a rigid frame would put it; the output samples the raw frame that far
    over, which straightens the bend. The bend is a function of the RAW row, and
    the global roll is at most a few degrees, so it is evaluated once per output
    row - two broadcast adds per frame instead of a full per-pixel map."""

    def __init__(self) -> None:
        self._key = None
        self._mx = self._my = self._xs = self._ys = None

    def maps(self, M: np.ndarray, offsets: np.ndarray, h: int, out_w: int, out_h: int):
        if self._key != (out_w, out_h):
            self._key = (out_w, out_h)
            self._xs = np.arange(out_w, dtype=np.float32)
            self._ys = np.arange(out_h, dtype=np.float32)
            self._mx = np.empty((out_h, out_w), np.float32)
            self._my = np.empty((out_h, out_w), np.float32)
        Mi = cv2.invertAffineTransform(np.asarray(M, np.float64))
        ys = self._ys
        nodes = band_nodes(h, len(offsets))
        v_mid = Mi[1, 0] * (out_w / 2.0) + Mi[1, 1] * ys + Mi[1, 2]
        dx = np.interp(v_mid, nodes, offsets[:, 0]).astype(np.float32)
        dy = np.interp(v_mid, nodes, offsets[:, 1]).astype(np.float32)
        np.add((Mi[0, 0] * self._xs)[None, :],
               (Mi[0, 1] * ys + Mi[0, 2] + dx)[:, None], out=self._mx)
        np.add((Mi[1, 0] * self._xs)[None, :],
               (Mi[1, 1] * ys + Mi[1, 2] + dy)[:, None], out=self._my)
        return self._mx, self._my

    def warp(self, frame: np.ndarray, M: np.ndarray, offsets: Optional[np.ndarray],
             out_w: int, out_h: int, dst: Optional[np.ndarray] = None) -> np.ndarray:
        if offsets is None or float(np.abs(offsets).max()) < 0.2:
            return cv2.warpAffine(frame, M, (out_w, out_h), dst=dst, flags=cv2.INTER_LINEAR,
                                  borderMode=cv2.BORDER_REPLICATE)
        mx, my = self.maps(M, offsets, frame.shape[0], out_w, out_h)
        return cv2.remap(frame, mx, my, cv2.INTER_LINEAR, dst=dst,
                         borderMode=cv2.BORDER_REPLICATE)


def to3(A: np.ndarray) -> np.ndarray:
    return np.vstack([np.asarray(A, np.float64), [0.0, 0.0, 1.0]])


def apply_affine(A: np.ndarray, x: float, y: float) -> tuple[float, float]:
    return (A[0, 0] * x + A[0, 1] * y + A[0, 2],
            A[1, 0] * x + A[1, 1] * y + A[1, 2])


def affine_scale(A: np.ndarray) -> float:
    """Isotropic scale of an affine's linear part."""
    return math.sqrt(max(abs(A[0, 0] * A[1, 1] - A[0, 1] * A[1, 0]), 1e-12))


@dataclass
class StabResult:
    motion: np.ndarray      # 2x3, raw frame n-1 -> raw frame n (frame pixels)
    matrix: np.ndarray      # 2x3, raw frame n -> published frame
    ok: bool                # motion was measured (False = assumed still)
    rs: Optional[np.ndarray] = None   # K x 2 rolling-shutter unbend (raw px)


class VideoStabilizer:
    """Stateful per-camera stabiliser. Not thread-safe - one capture thread."""

    def __init__(self, *, enabled: bool = True, mode: str = "similarity",
                 out_size: tuple[int, int] = (1280, 720), smooth_hz: float = 2.0,
                 knee: float = 0.6, max_angle_deg: float = 2.5,
                 analysis_step: int = 4, max_features: int = 120,
                 min_inliers: int = 14, fps: float = 30.0,
                 rs_bands: int = 0, rs_hp_hz: float = 0.5,
                 rs_per_band: int = 16) -> None:
        self.enabled = bool(enabled)
        self.mode = "translate" if str(mode).lower() == "translate" else "similarity"
        self.out_w, self.out_h = int(out_size[0]), int(out_size[1])
        # Clamped: a corner below ~0.3 Hz lets the correction wander to the
        # margin during any slow pan; above ~6 Hz there is nothing left to cut.
        self.smooth_hz = min(max(float(smooth_hz), 0.3), 6.0)
        self.knee = min(max(float(knee), 0.1), 0.95)
        self.max_angle = math.radians(max(0.0, float(max_angle_deg)))
        self.step_px = max(1, int(analysis_step))
        self.max_features = max(20, int(max_features))
        self.min_inliers = max(6, int(min_inliers))
        self.fps = max(1.0, float(fps))
        # Rolling-shutter unbend. 0 = off (rigid estimate, as before). The bend
        # is measured as a CHANGE between frames, so it is integrated - through
        # a leak at rs_hp_hz, or measurement noise would random-walk into a
        # permanent shear. Vibration (5-15 Hz) sits far above the leak.
        self.rs_bands = 0 if int(rs_bands) < 3 else int(rs_bands)
        self.rs_per_band = max(6, int(rs_per_band))
        self._rs_leak = math.exp(-2.0 * math.pi * max(0.05, float(rs_hp_hz)) / self.fps)
        self._rsw = RsWarp()
        self._dst: Optional[np.ndarray] = None
        self.reset()

    def reset(self) -> None:
        self._prev_gray: Optional[np.ndarray] = None
        self._prev_pts = None
        self._raw = np.zeros(3)       # accumulated camera path  (x, y, roll)
        self._smooth = np.zeros(3)    # where the camera "should" be
        self._vel = np.zeros(3)
        self._frames = 0
        self._ok_ema = 1.0
        self._sat_ema = 0.0
        self._cost_ms = 0.0
        self._corr = (0.0, 0.0, 0.0)
        self._rs_meas: Optional[np.ndarray] = None
        self._rs = np.zeros((max(self.rs_bands, 2), 2))

    # -- measurement ------------------------------------------------------
    def estimate(self, frame: np.ndarray) -> Optional[np.ndarray]:
        """Inter-frame affine (previous -> this frame) in frame pixels, or None
        on the first frame or when too little of the scene could be tracked
        (fog, a lens-filling blur, a blank sky)."""
        if cv2 is None:
            return None
        s = self.step_px
        small = np.ascontiguousarray(frame[::s, ::s])
        gray = cv2.cvtColor(small, cv2.COLOR_BGR2GRAY) if small.ndim == 3 else small
        prev, self._prev_gray = self._prev_gray, gray
        self._rs_meas = None
        if prev is None or prev.shape != gray.shape:
            self._prev_pts = None
            return None
        if self.rs_bands:
            r = measure_motion(prev, gray, self.rs_bands, self.rs_per_band)
            if r is None:
                return None
            A = r[0].astype(np.float64)
            A[:, 2] *= s
            self._rs_meas = r[1] * s
            return A
        pts = self._prev_pts
        if pts is None or len(pts) < self.max_features // 2:
            pts = cv2.goodFeaturesToTrack(prev, self.max_features, 0.01, 8, blockSize=5)
        self._prev_pts = None
        if pts is None or len(pts) < self.min_inliers:
            return None
        nxt, st, _ = cv2.calcOpticalFlowPyrLK(
            prev, gray, pts, None, winSize=(15, 15), maxLevel=3,
            criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 20, 0.03))
        good = st.reshape(-1) == 1
        p0, p1 = pts[good], nxt[good]
        if len(p0) < self.min_inliers:
            return None
        A, inl = cv2.estimateAffine2D(p0, p1, method=cv2.RANSAC,
                                      ransacReprojThreshold=1.0, maxIters=200,
                                      confidence=0.98)
        if A is None or inl is None or int(inl.sum()) < self.min_inliers:
            return None
        # Carry the surviving background corners into the next frame instead of
        # re-detecting every time; top up (above) once too many have been lost.
        keep = p1[inl.reshape(-1) == 1]
        self._prev_pts = keep.reshape(-1, 1, 2) if len(keep) >= self.max_features // 2 else None
        A = A.astype(np.float64)
        A[:, 2] *= s
        return A

    # -- per frame ----------------------------------------------------------
    def step(self, frame: np.ndarray) -> StabResult:
        started = time.perf_counter()
        h, w = frame.shape[:2]
        A = self.estimate(frame) if self.enabled else None
        ok = A is not None
        if ok:
            dx, dy, da = similarity_params(A, w / 2.0, h / 2.0)
            # A jump this large is a tracking failure or a scene cut, not a
            # shake. Believing it would throw the picture across the margin.
            if abs(dx) > 0.25 * w or abs(dy) > 0.25 * h or abs(da) > math.radians(10):
                A, ok = None, False
        if not ok:
            A, dx, dy, da = IDENTITY.copy(), 0.0, 0.0, 0.0
        self._frames += 1
        self._ok_ema += 0.02 * ((1.0 if ok else 0.0) - self._ok_ema)

        self._raw += (dx, dy, da)
        # Critically damped follower (zeta = 1): no overshoot, so the picture
        # never swings past where the camera actually went.
        wn = 2.0 * math.pi * self.smooth_hz
        dt = 1.0 / self.fps
        acc = wn * wn * (self._raw - self._smooth) - 2.0 * wn * self._vel
        self._vel += acc * dt
        self._smooth += self._vel * dt
        corr = self._smooth - self._raw

        margin_x = max(0.0, (w - self.out_w) / 2.0)
        margin_y = max(0.0, (h - self.out_h) / 2.0)
        rs = None
        if self.rs_bands and self.mode == "similarity":
            self._rs *= self._rs_leak
            if ok and self._rs_meas is not None:
                self._rs += self._rs_meas
            cap = 0.25 * min(margin_x, margin_y)
            np.clip(self._rs, -cap, cap, out=self._rs)
            rs = self._rs.copy()
            # The unbend moves rows too - pay for it out of the margin.
            reserve = float(np.abs(rs).max())
            margin_x, margin_y = max(0.0, margin_x - reserve), max(0.0, margin_y - reserve)
        if self.mode == "similarity" and self.max_angle > 0.0:
            ang = soft_limit(corr[2], self.max_angle, self.knee)
            # A rotated crop swings its corners outward - pay for that out of
            # the translation margin, or the corners show at full roll.
            sa, ca = abs(math.sin(ang)), 1.0 - math.cos(ang)
            lim_x = max(0.0, margin_x - sa * self.out_h / 2.0 - ca * self.out_w / 2.0)
            lim_y = max(0.0, margin_y - sa * self.out_w / 2.0 - ca * self.out_h / 2.0)
        else:
            ang, lim_x, lim_y = 0.0, margin_x, margin_y
        cx = soft_limit(corr[0], lim_x, self.knee)
        cy = soft_limit(corr[1], lim_y, self.knee)
        saturated = abs(corr[0]) > lim_x or abs(corr[1]) > lim_y
        self._sat_ema += 0.02 * ((1.0 if saturated else 0.0) - self._sat_ema)
        # Write the limited correction back so the filter cannot wind up beyond
        # what can be displayed and then take seconds to unwind.
        self._smooth = self._raw + np.array([cx, cy, ang if self.mode == "similarity" else 0.0])
        self._corr = (cx, cy, ang)

        if self.mode == "translate":
            x0 = int(round(min(max(margin_x - cx, 0.0), max(w - self.out_w, 0))))
            y0 = int(round(min(max(margin_y - cy, 0.0), max(h - self.out_h, 0))))
            M = np.array([[1.0, 0.0, -x0], [0.0, 1.0, -y0]], np.float64)
        else:
            M = correction_matrix(cx, cy, ang, w, h, self.out_w, self.out_h)
        ms = (time.perf_counter() - started) * 1000.0
        self._cost_ms = ms if self._cost_ms == 0.0 else 0.9 * self._cost_ms + 0.1 * ms
        return StabResult(motion=A, matrix=M, ok=ok, rs=rs)

    def render(self, frame: np.ndarray, M: np.ndarray,
               rs: Optional[np.ndarray] = None) -> np.ndarray:
        """Produce the published frame. Always a fresh buffer the caller may
        draw on - never a view into the capture buffer."""
        h, w = frame.shape[:2]
        if w < self.out_w or h < self.out_h or cv2 is None:
            return np.ascontiguousarray(frame)
        if self.mode == "translate" or (abs(M[0, 1]) < 1e-9 and abs(M[1, 0]) < 1e-9
                                        and float(M[0, 2]).is_integer()
                                        and float(M[1, 2]).is_integer()):
            x0, y0 = int(-M[0, 2]), int(-M[1, 2])
            return np.ascontiguousarray(frame[y0:y0 + self.out_h, x0:x0 + self.out_w])
        shape = (self.out_h, self.out_w) + frame.shape[2:]
        if self._dst is None or self._dst.shape != shape:
            self._dst = np.empty(shape, np.uint8)
        if rs is not None:
            return self._rsw.warp(frame, M, rs, self.out_w, self.out_h, dst=self._dst)
        return cv2.warpAffine(frame, M, (self.out_w, self.out_h), dst=self._dst,
                              flags=cv2.INTER_LINEAR, borderMode=cv2.BORDER_REPLICATE)

    def stats(self) -> dict:
        cx, cy, ang = self._corr
        return {
            "mode": self.mode,
            "ok_pct": round(100.0 * self._ok_ema, 0),
            "sat_pct": round(100.0 * self._sat_ema, 0),
            "corr_px": [round(cx, 1), round(cy, 1)],
            "corr_deg": round(math.degrees(ang), 2),
            "cost_ms": round(self._cost_ms, 2),
            "rs_px": round(float(np.abs(self._rs).max()), 1) if self.rs_bands else None,
        }


def stabilizer_from_config(section: dict | None, out_size: tuple[int, int],
                           fps: float) -> Optional[VideoStabilizer]:
    """Build from the ``cameras.stabilize`` block; None when disabled."""
    section = section or {}
    if not bool(section.get("enabled", False)):
        return None
    return VideoStabilizer(
        enabled=True,
        mode=str(section.get("mode", "similarity")),
        out_size=out_size,
        smooth_hz=float(section.get("smooth_hz", 2.0)),
        knee=float(section.get("knee", 0.6)),
        max_angle_deg=float(section.get("max_angle_deg", 2.5)),
        analysis_step=int(section.get("analysis_step", 4)),
        max_features=int(section.get("max_features", 120)),
        fps=fps,
        rs_bands=int(section.get("rs_bands", 0)),
        rs_hp_hz=float(section.get("rs_hp_hz", 0.5)),
        rs_per_band=int(section.get("rs_per_band", 16)),
    )
