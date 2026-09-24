"""Post-flight replay renderer: zero-phase stabilisation + lock HUD -> H.264.

Run by ``FlightRecorder`` as a niced subprocess after disarm::

    python -m drone_stack.gcs.replay_render --src flight.mjpg --index flight.jsonl \
        --out latest.tmp.mp4 --poster latest.tmp.jpg

Why after landing, and why a separate process
---------------------------------------------
In flight the recorder only stores what is cheap: the full-resolution capture
frames as JPEG, plus one JSON line per frame holding the camera motion the live
stabiliser measured anyway and the lock box the live tracker drew. Warping
2304x1296 -> 1920x1080 in flight would cost another core the aircraft does not
have to spare.

Deferring it buys quality as well as CPU. The live stabiliser can only smooth
the past; here the whole path is known, so a centred (zero-phase) Gaussian
smooths with no lag at all - measured roughly 2x steadier again than the causal
filter on the 2026-09-14 flight data.

A separate process keeps a per-frame Python loop off the GCS's GIL, and a CPU
budget (``--cpu-cores``) paces it so that rendering a flight never pushes the
Pi past the load the operator set as the ceiling: it just takes longer.

Motion is re-measured here, from the recorded frames themselves, rather than
trusted from the index. The live estimate is one rigid RANSAC fit per frame,
and on a rolling-shutter sensor under vibration the rows of one frame do not
move together - that fit jumps between whichever strip of rows agrees best and
adds jitter of its own. ``stabilizer.measure_motion`` fits a roll + zoom plus a
translation that varies smoothly down the frame; its centre-row motion drives
the path, and its per-row part (``--rs-bands``) is high-passed into a per-row
shear that straightens the "jello" wobble. Measured on flight footage
2026-09-23: see ``render``. The index motion is the fallback for any frame the
measurement cannot fit.

Colour: frames decode to BGR, warp in BGR, and ffmpeg converts to limited-range
BT.709 4:2:0, tagged as such so browsers do not guess.
"""
from __future__ import annotations

import argparse
import json
import math
import os
import subprocess
import sys
import tempfile
import time
from pathlib import Path
from typing import Optional

import numpy as np

from drone_stack.gcs.stabilizer import (RsWarp, correction_matrix, measure_motion,
                                        similarity_params)


def load_index(path: Path) -> tuple[dict, list]:
    header: dict = {}
    frames: list = []
    with open(path, "r") as fp:
        for line in fp:
            line = line.strip()
            if not line:
                continue
            try:
                rec = json.loads(line)
            except ValueError:
                break            # a torn last line from a power cut
            if "hdr" in rec:
                header = rec["hdr"]
            elif "o" in rec and "n" in rec:
                frames.append(rec)
    return header, frames


def gauss_smooth(X: np.ndarray, sigma: float) -> np.ndarray:
    if sigma <= 0 or len(X) < 3:
        return X.copy()
    r = int(math.ceil(3 * sigma))
    k = np.exp(-0.5 * (np.arange(-r, r + 1) / sigma) ** 2)
    k /= k.sum()
    pad = np.pad(X, ((r, r), (0, 0)), mode="edge")
    return np.stack([np.convolve(pad[:, i], k, mode="valid") for i in range(X.shape[1])], 1)


def _soft(c: np.ndarray, lim: np.ndarray, knee: float) -> np.ndarray:
    lim = np.maximum(lim, 1e-9)
    k = knee * lim
    a = np.abs(c)
    comp = k + (lim - k) * np.tanh((a - k) / np.maximum(lim - k, 1e-9))
    return np.where(a > k, np.sign(c) * comp, c)


def plan_corrections(motions: list, w: int, h: int, out_w: int, out_h: int,
                     sigma: float = 5.0, max_angle_deg: float = 2.5,
                     knee: float = 0.6, reserve_px: float = 0.0) -> list:
    """One raw->output 2x3 matrix per frame from the per-frame camera motion.

    ``motions[i]`` maps frame i-1 to frame i in full-resolution pixels (None =
    not measured, treated as still). ``reserve_px`` is margin held back for
    the rolling-shutter shear, which samples beyond where ``M`` alone would."""
    n = len(motions)
    if n == 0:
        return []
    params = np.zeros((n, 3))
    for i, A in enumerate(motions):
        if A is None:
            continue
        dx, dy, da = similarity_params(np.asarray(A, np.float64), w / 2.0, h / 2.0)
        if abs(dx) > 0.25 * w or abs(dy) > 0.25 * h or abs(da) > math.radians(10):
            continue
        params[i] = (dx, dy, da)
    X = np.cumsum(params, axis=0)
    mx = max(0.0, (w - out_w) / 2.0 - reserve_px)
    my = max(0.0, (h - out_h) / 2.0 - reserve_px)
    amax = math.radians(max(0.0, max_angle_deg))

    def limit(corr: np.ndarray) -> np.ndarray:
        ang = _soft(corr[:, 2], np.full(n, amax), knee) if amax > 0 else np.zeros(n)
        sa, ca = np.abs(np.sin(ang)), 1.0 - np.cos(ang)
        lim_x = np.maximum(0.0, mx - sa * out_h / 2.0 - ca * out_w / 2.0)
        lim_y = np.maximum(0.0, my - sa * out_w / 2.0 - ca * out_h / 2.0)
        return np.stack([_soft(corr[:, 0], lim_x, knee), _soft(corr[:, 1], lim_y, knee), ang], 1)

    corr = limit(gauss_smooth(X, sigma) - X)
    # Second, shorter pass over the LIMITED path: where the margin clamped, the
    # correction bends abruptly, and re-smoothing turns that bend into a glide.
    corr = limit(gauss_smooth(X + corr, max(1.0, sigma / 3.0)) - X)
    return [correction_matrix(c[0], c[1], c[2], w, h, out_w, out_h) for c in corr]


def plan_shear(offsets: list, sigma: float, limit_px: float):
    """Per-frame rolling-shutter shear (n x K x 2, raw px) from each frame
    pair's change in intra-frame bend.

    Summed over the flight, the bend changes give every row's drift relative
    to the centre row. Only the fast part is vibration; the slow part is real
    geometry (a gentle pitch changes perspective), so a Gaussian high-pass of
    ``sigma`` frames splits them. Returns ``(shear, reserve)`` or
    ``(None, 0.0)`` when there is no bend worth correcting."""
    rows = [o for o in offsets if o is not None]
    if not rows or sigma <= 0:
        return None, 0.0
    k = rows[0].shape[0]
    n = len(offsets)
    B = np.zeros((n, k, 2))
    for i, o in enumerate(offsets):
        if o is not None and o.shape == (k, 2):
            B[i] = o
    R = np.cumsum(B, axis=0).reshape(n, -1)
    D = (R - gauss_smooth(R, sigma)).reshape(n, k, 2)
    # Margin for the shear: the 99th percentile, so a single bad fit cannot
    # shrink the stabiliser's room for the whole flight, capped by the caller.
    reserve = min(float(np.percentile(np.abs(D), 99)), max(0.0, limit_px))
    if reserve < 0.5:
        return None, 0.0
    return np.clip(D, -reserve, reserve), reserve


def measure_flight(src: Path, frames: list, bands: int, per_band: int,
                   analysis_w: int, pace=None):
    """Re-measure camera motion from the recorded JPEGs.

    Decodes each frame straight to a small grey image (libjpeg's DCT scaling:
    about 1/16 of the work of a full decode) and returns
    ``(motions, offsets, scale)``: per-frame 2x3 similarity and K x 2 bend
    change in ANALYSIS pixels (None where unmeasured), and analysis -> full
    resolution scale."""
    import cv2

    try:
        import simplejpeg

        def gray(buf):
            return simplejpeg.decode_jpeg(buf, colorspace="GRAY", fastdct=True,
                                          min_width=analysis_w)
    except Exception:  # noqa: BLE001
        def gray(buf):
            return cv2.imdecode(np.frombuffer(buf, np.uint8), cv2.IMREAD_REDUCED_GRAYSCALE_4)

    motions, offsets = [], []
    prev = None
    size = None
    with open(src, "rb") as fp:
        for i, fr in enumerate(frames):
            A = off = None
            try:
                fp.seek(int(fr["o"]))
                g = gray(fp.read(int(fr["n"])))
                if g is None:
                    raise ValueError("undecodable")
                if g.ndim == 3:
                    g = g[:, :, 0]
                if size is None:
                    size = g.shape[:2]
                elif g.shape[:2] != size:
                    raise ValueError("frame size changed")
            except Exception:  # noqa: BLE001 - a corrupt frame breaks the chain
                g = None
            if g is not None and prev is not None:
                r = measure_motion(prev, np.ascontiguousarray(g), bands, per_band)
                if r is not None:
                    A, off = r
            motions.append(A)
            offsets.append(off)
            prev = g
            if pace is not None and i % 10 == 0:
                pace()
            if i % 30 == 0:
                print(json.dumps({"progress": round(0.5 * i / len(frames), 3),
                                  "stage": "measure"}), flush=True)
    return motions, offsets, size


class CpuPacer:
    """Keeps this process plus one child under ``cores`` of CPU on average.

    Sleeping the producer also starves the encoder downstream of it, so pacing
    the Python side alone bounds the pair."""

    def __init__(self, cores: float, child_pid: Optional[int]) -> None:
        self.cores = float(cores)
        self.pid = child_pid
        try:
            self._tick = float(os.sysconf("SC_CLK_TCK"))
        except (AttributeError, ValueError, OSError):
            self._tick = 100.0
        self.t0 = time.monotonic()
        self.c0 = time.process_time()
        self.k0 = self._child()

    def _child(self) -> float:
        if not self.pid:
            return 0.0
        try:
            with open(f"/proc/{self.pid}/stat") as fp:
                f = fp.read().rsplit(")", 1)[1].split()
            return (int(f[11]) + int(f[12])) / self._tick
        except (OSError, IndexError, ValueError):
            return 0.0

    def pace(self) -> float:
        """Sleep if ahead of budget; returns the seconds slept."""
        if self.cores <= 0:
            return 0.0
        used = (time.process_time() - self.c0) + (self._child() - self.k0)
        wall = time.monotonic() - self.t0
        over = used - wall * self.cores
        if over <= 0:
            return 0.0
        nap = min(over / self.cores, 2.0)
        time.sleep(nap)
        return nap


# Detections that are not the lock: thin cyan, so they can never be mistaken
# for the lock HUD's green/amber corner brackets.
DET_BGR = (255, 200, 0)
# A det overlapping the lock box this much IS the locked person - the HUD
# already marks it, and a second box on top only makes it harder to read.
_DET_LOCK_IOU = 0.5


def draw_dets_bgr(dst: np.ndarray, dets, M: np.ndarray, k: float,
                  lock_box=None) -> int:
    """Draw every recorded YOLO detection (index key ``d``: RAW-px
    ``[x1, y1, x2, y2, score]``) through the replay's own correction ``M``.
    Returns how many were drawn."""
    if not dets:
        return 0
    import cv2

    from drone_stack.gcs.person_lock import box_through, iou

    h, w = dst.shape[:2]
    t = max(1, int(round(h / 540.0)))
    fs = 0.45 * h / 720.0
    drawn = 0
    for d in dets:
        if len(d) < 4:
            continue
        if lock_box and iou(d[:4], lock_box[:4]) > _DET_LOCK_IOU:
            continue
        x1, y1, x2, y2 = box_through(M, [v * k for v in d[:4]])[:4]
        if x2 <= 0 or y2 <= 0 or x1 >= w or y1 >= h:
            continue
        p, q = (int(round(x1)), int(round(y1))), (int(round(x2)), int(round(y2)))
        cv2.rectangle(dst, p, q, DET_BGR, t)
        if len(d) >= 5:
            cv2.putText(dst, f"person {float(d[4]) * 100:.0f}%",
                        (p[0], max(int(12 * fs / 0.45), p[1] - 4)),
                        cv2.FONT_HERSHEY_SIMPLEX, fs, DET_BGR, t, cv2.LINE_AA)
        drawn += 1
    return drawn


def render(src: Path, index: Path, out: Path, poster: Optional[Path], out_w: int,
           out_h: int, sigma: float, preset: str, crf: int, threads: int,
           filter_cfg: dict, max_angle_deg: float = 2.5, cpu_cores: float = 1.0,
           measure: bool = True, rs_bands: int = 12, rs_sigma: float = 3.0,
           rs_max_px: float = 40.0, analysis_w: int = 576) -> int:
    import cv2

    from drone_stack.gcs.frame_filter import filter_from_config
    from drone_stack.gcs.person_lock import box_through, draw_hud_bgr, hud_layout

    try:
        import simplejpeg

        def decode(buf):
            return simplejpeg.decode_jpeg(buf, colorspace="BGR", fastdct=True,
                                          fastupsample=True)
    except Exception:  # noqa: BLE001
        def decode(buf):
            img = cv2.imdecode(np.frombuffer(buf, np.uint8), cv2.IMREAD_COLOR)
            if img is None:
                raise ValueError("undecodable frame")
            return img

    cv2.setNumThreads(1)
    header, frames = load_index(index)
    if len(frames) < 2:
        print(json.dumps({"error": "fewer than 2 frames in index"}), flush=True)
        return 2
    main_w, main_h = (int(v) for v in header.get("main", (0, 0)))
    if not main_w:
        with open(src, "rb") as fp:
            fp.seek(int(frames[0]["o"]))
            main_h, main_w = decode(fp.read(int(frames[0]["n"]))).shape[:2]
    live_w = int(header.get("lores", (main_w, main_h))[0]) or main_w
    k = main_w / float(live_w)
    out_w, out_h = min(out_w, main_w) & ~1, min(out_h, main_h) & ~1

    motions = []
    for fr in frames:
        A = fr.get("A")
        if A is None:
            motions.append(None)
            continue
        A = np.asarray(A, np.float64).reshape(2, 3).copy()
        A[:, 2] *= k
        motions.append(A)
    # One pacer for the whole job (measure + render), so the average over the
    # run - not just each half - stays under the budget.
    pacer = CpuPacer(cpu_cores, None)
    shear, reserve, remeasured = None, 0.0, 0
    if measure:
        bands = max(3, int(rs_bands)) if rs_bands else 12
        mm, oo, size = measure_flight(src, frames, bands, 30, analysis_w, pacer.pace)
        if size is not None:
            s = main_w / float(size[1])
            for i, A in enumerate(mm):
                if A is None:
                    continue           # keep the live index motion for this one
                A = A.copy()
                A[:, 2] *= s
                motions[i] = A
                oo[i] = oo[i] * s
                remeasured += 1
            if rs_bands:
                # Never more than a third of the smaller margin: the global
                # path's shake needs most of it.
                lim = min(rs_max_px, (min(main_w - out_w, main_h - out_h) / 2.0) / 3.0)
                shear, reserve = plan_shear(oo, rs_sigma, lim)
    mats = plan_corrections(motions, main_w, main_h, out_w, out_h, sigma, max_angle_deg,
                            reserve_px=reserve)

    t0, t1 = float(frames[0].get("t", 0.0)), float(frames[-1].get("t", 0.0))
    fps = (len(frames) - 1) / (t1 - t0) if t1 > t0 else float(header.get("fps", 30.0))
    fps = min(max(fps, 5.0), 60.0)

    err = tempfile.TemporaryFile()
    cmd = ["ffmpeg", "-y", "-hide_banner", "-nostdin", "-loglevel", "error",
           "-f", "rawvideo", "-pix_fmt", "bgr24", "-s", f"{out_w}x{out_h}",
           "-framerate", f"{fps:.3f}", "-i", "-",
           "-vf", "scale=out_color_matrix=bt709:out_range=tv,format=yuv420p",
           "-c:v", "libx264", "-preset", preset, "-crf", str(crf),
           "-threads", str(threads),
           "-colorspace", "bt709", "-color_primaries", "bt709",
           "-color_trc", "bt709", "-color_range", "tv",
           "-movflags", "+faststart", "-an", "-f", "mp4", str(out)]
    proc = subprocess.Popen(cmd, stdin=subprocess.PIPE, stderr=err)
    pacer.pid = proc.pid
    pacer.k0 = pacer._child()
    rsw = RsWarp()
    filt = filter_from_config(dict(filter_cfg, temporal=False)) \
        if (filter_cfg or {}).get("enabled", False) else None
    dst = np.empty((out_h, out_w, 3), np.uint8)
    have_frame = False
    n = len(frames)
    try:
        with open(src, "rb") as fp:
            for i, fr in enumerate(frames):
                try:
                    fp.seek(int(fr["o"]))
                    img = decode(fp.read(int(fr["n"])))
                    if img.shape[:2] != (main_h, main_w):
                        raise ValueError("frame size changed")
                except Exception:  # noqa: BLE001 - a corrupt frame repeats the last one
                    if have_frame:
                        proc.stdin.write(dst.data)
                    continue
                if filt is not None:
                    img = filt.apply(img)
                M = mats[i]
                rsw.warp(img, M, None if shear is None else shear[i], out_w, out_h, dst=dst)
                box = fr.get("box")
                # Under the lock HUD, so the locked person reads as the lock.
                draw_dets_bgr(dst, fr.get("d"), M, k, box)
                if box:
                    bo = box_through(M, [v * k for v in box[:4]])
                    draw_hud_bgr(dst, hud_layout(bo, fr.get("st", "lock"),
                                                 float(fr.get("sc", 0.0)), out_w, out_h))
                proc.stdin.write(dst.data)
                if not have_frame and poster is not None:
                    cv2.imwrite(str(poster), dst, [cv2.IMWRITE_JPEG_QUALITY, 88])
                have_frame = True
                if i % 10 == 0:
                    pacer.pace()
                if i % 30 == 0:
                    done = 0.5 + 0.5 * i / n if measure else i / n
                    print(json.dumps({"progress": round(done, 3)}), flush=True)
        proc.stdin.close()
    except BrokenPipeError:
        pass
    rc = proc.wait()
    if rc != 0:
        err.seek(0)
        print(json.dumps({"error": err.read().decode(errors="replace")[-400:]}), flush=True)
        return rc or 1
    print(json.dumps({"progress": 1.0, "fps": round(fps, 3), "width": out_w,
                      "height": out_h, "frames": n, "remeasured": remeasured,
                      "rs_reserve_px": round(reserve, 1)}), flush=True)
    return 0


def main(argv: Optional[list] = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--src", required=True, type=Path)
    ap.add_argument("--index", required=True, type=Path)
    ap.add_argument("--out", required=True, type=Path)
    ap.add_argument("--poster", type=Path, default=None)
    ap.add_argument("--width", type=int, default=1920)
    ap.add_argument("--height", type=int, default=1080)
    ap.add_argument("--sigma", type=float, default=5.0)
    ap.add_argument("--max-angle-deg", type=float, default=2.5)
    ap.add_argument("--preset", default="superfast")
    ap.add_argument("--crf", type=int, default=22)
    ap.add_argument("--threads", type=int, default=2)
    ap.add_argument("--cpu-cores", type=float, default=1.0)
    ap.add_argument("--filter", default="{}", help="cameras.filter block as JSON")
    ap.add_argument("--no-measure", action="store_true",
                    help="trust the live index motion instead of re-measuring")
    ap.add_argument("--rs-bands", type=int, default=12,
                    help="rolling-shutter nodes down the frame; 0 = no unbend")
    ap.add_argument("--rs-sigma", type=float, default=3.0,
                    help="high-pass (frames) splitting vibration bend from real geometry")
    ap.add_argument("--rs-max-px", type=float, default=40.0)
    a = ap.parse_args(argv)
    return render(a.src, a.index, a.out, a.poster, a.width, a.height, a.sigma,
                  a.preset, a.crf, a.threads, json.loads(a.filter), a.max_angle_deg,
                  a.cpu_cores, not a.no_measure, a.rs_bands, a.rs_sigma, a.rs_max_px)


if __name__ == "__main__":
    sys.exit(main())
