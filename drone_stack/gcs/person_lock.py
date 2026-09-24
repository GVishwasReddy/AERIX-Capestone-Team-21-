"""Single-target person lock on the Hailo-8 NPU.

What the operator asked for
---------------------------
Find the single most confident person in view, put ONE box on them, and keep
that box on that person for as long as they are in frame - no flicker, no box
hopping to someone else, no box that blinks out for a frame and comes back.

Why a tracker and not just "draw the detections"
-----------------------------------------------
The model (``person_detect/aerix_person_y8m_1280_pass2_biascorr.hef``, YOLOv8m
at 1280x1280) is heavy. Measured on this Hailo-8 it runs at ~18 fps with a
52 ms hardware latency at batch 1 - it physically cannot label every frame of
a 30 fps stream. Drawing raw detections would therefore flash on and off, jump
by the int8 box jitter, and lag the video by 2-3 frames. So:

* **Detection runs asynchronously** on the NPU (``HailoPersonDetector``). The
  camera never waits for it: each frame is offered, and accepted only when the
  NPU is idle. Newest frame wins, nothing queues, latency cannot grow.
* **A Kalman lock (``TargetLock``) owns the box.** It advances every video
  frame, so the box moves at 30 fps; detections correct it when they arrive.
* **Camera motion is removed from the target's motion.** The stabiliser
  already measures how the whole scene moved between frames; the lock applies
  that to its state first, so a vibrating airframe does not look to the
  filter like a sprinting person. A detection that took 3 frames to come back
  is carried forward through those 3 frames of camera motion before it is used.

Lock policy (hysteresis is what removes the flicker)
----------------------------------------------------
* acquire: the highest-scoring person >= ``acquire_conf``, confirmed again on
  a following detection before any box is shown (one-frame false positives
  never draw).
* keep: once locked, a match needs only ``keep_conf`` (down to the HEF's 0.20
  floor) and must pass a motion gate - so a partly occluded target keeps its
  lock while a different person elsewhere in frame cannot steal it.
* hold: missed detections coast on the filter for up to ``hold_s``.
* drop: after ``hold_s`` without a match, or once the target has been outside
  the published frame for ``exit_s``. Then search again.
"""
from __future__ import annotations

import math
import threading
import time
from collections import deque
from pathlib import Path
from typing import Callable, Optional, Sequence

import numpy as np

from drone_stack.gcs.stabilizer import IDENTITY, affine_scale, apply_affine, to3
from drone_stack.utils.logging_setup import get_logger

_log = get_logger("gcs.person_lock")

try:
    import cv2
except Exception:  # noqa: BLE001
    cv2 = None  # type: ignore

Box = tuple  # (x1, y1, x2, y2, score)

LOCK_BGR = (0, 235, 255)
HOLD_BGR = (0, 150, 190)


# --------------------------------------------------------------------------- #
# geometry
# --------------------------------------------------------------------------- #
def iou(a: Sequence[float], b: Sequence[float]) -> float:
    ix = max(0.0, min(a[2], b[2]) - max(a[0], b[0]))
    iy = max(0.0, min(a[3], b[3]) - max(a[1], b[1]))
    inter = ix * iy
    union = (a[2] - a[0]) * (a[3] - a[1]) + (b[2] - b[0]) * (b[3] - b[1]) - inter
    return inter / union if union > 0 else 0.0


def host_nms(dets: list, iou_th: float) -> list:
    """Second-pass NMS on the host, highest score first.

    The HEF's own NMS only merges boxes overlapping by IoU >= 0.70, and int8
    rounding leaves near-duplicates on one person that it never merges
    (person_detect/pi_detect.py measured precision 42% -> 57% at 0.5).
    """
    dets = sorted(dets, key=lambda d: -d[4])
    if iou_th <= 0 or len(dets) < 2:
        return dets
    keep: list = []
    for d in dets:
        if all(iou(d, k) < iou_th for k in keep):
            keep.append(d)
    return keep


def box_through(A: np.ndarray, box: Sequence[float]) -> tuple:
    """Map an axis-aligned box through a (near-similarity) affine: move the
    centre exactly, scale the size. Rotations here are a degree or two, so the
    bounding box of the rotated corners would only inflate the box."""
    cx, cy = (box[0] + box[2]) / 2.0, (box[1] + box[3]) / 2.0
    s = affine_scale(A)
    hw, hh = (box[2] - box[0]) * s / 2.0, (box[3] - box[1]) * s / 2.0
    nx, ny = apply_affine(A, cx, cy)
    return (nx - hw, ny - hh, nx + hw, ny + hh) + tuple(box[4:])

def _sigmoid(x):
    return 1.0 / (1.0 + np.exp(-np.clip(x, -50, 50)))


def parse_multihead_yolo(outs: dict, geom: tuple, score_thr: float) -> list:
    """Decode multi-head YOLO output (separate box/score tensors per scale).

    The new visdronebest.hef outputs 6 tensors (3 scales × {boxes, scores}):
      conv61 (80×80×4) + conv64 (80×80×1)  — large objects
      conv77 (40×40×4) + conv80 (40×40×1)  — medium objects
      conv91 (20×20×4) + conv94 (20×20×1)  — small objects

    Box format is raw DFL regression (cx, cy, w, h in grid-relative coords).
    Score is raw logit (needs sigmoid).
    """
    w_in, h_in, pad_x, pad_y, scale, fw, fh = geom

    # Sort outputs into box (4-ch) and score (1-ch) by matching grid size
    box_tensors = {}  # grid_size -> tensor
    score_tensors = {}  # grid_size -> tensor
    for name, tensor in outs.items():
        if tensor.ndim == 3:
            gh, gw, ch = tensor.shape
        elif tensor.ndim == 4:
            _, gh, gw, ch = tensor.shape
            tensor = tensor.reshape(gh, gw, ch)
        else:
            continue
        grid_key = (gh, gw)
        if ch == 4:
            box_tensors[grid_key] = tensor
        elif ch == 1:
            score_tensors[grid_key] = tensor

    dets = []

    for grid_key, box_t in sorted(box_tensors.items(), key=lambda x: -x[0][0]):
        score_t = score_tensors.get(grid_key)
        if score_t is None:
            continue

        gh, gw = grid_key
        # Stride is the input size divided by the grid width, so this holds for
        # any square input (640 -> 8/16/32, 1280 -> 8/16/32) without a
        # hard-coded table that silently breaks if the model is recompiled at
        # a different resolution.
        stride = w_in // gw

        scores_flat = _sigmoid(score_t.reshape(-1))
        boxes_flat = box_t.reshape(-1, 4)

        # Quick filter: only process cells above threshold
        mask = scores_flat >= score_thr
        if not np.any(mask):
            continue

        indices = np.where(mask)[0]
        for idx in indices:
            gy_idx = idx // gw
            gx_idx = idx % gw
            conf = float(scores_flat[idx])

            # YOLOv8/YOLO26 anchor-free head: the 4 box channels are the DFL-
            # decoded distances (left, top, right, bottom) from the cell CENTRE
            # to each edge, measured in stride units. Verified on this HEF: the
            # 4 channels are always positive (0-6), which is only consistent
            # with edge distances, never with a (cx,cy,w,h) encoding. The old
            # code read them as (x_off, y_off, w, h), which put every box in the
            # wrong place and half its size.
            l, t, r, b = boxes_flat[idx]
            ax = (gx_idx + 0.5) * stride          # cell-centre anchor, in px
            ay = (gy_idx + 0.5) * stride

            # De-letterbox to original frame coords
            x1 = (ax - l * stride - pad_x) / scale
            y1 = (ay - t * stride - pad_y) / scale
            x2 = (ax + r * stride - pad_x) / scale
            y2 = (ay + b * stride - pad_y) / scale

            x1 = min(max(x1, 0.0), fw - 1.0)
            y1 = min(max(y1, 0.0), fh - 1.0)
            x2 = min(max(x2, 0.0), fw - 1.0)
            y2 = min(max(y2, 0.0), fh - 1.0)
            if x2 > x1 + 2 and y2 > y1 + 2:
                dets.append((x1, y1, x2, y2, conf))

    return dets


def parse_nms_buffer(out_tensor: np.ndarray, geom: tuple, score_thr: float) -> list:
    """Standard Hailo NMS-by-class FLOAT32 layout (on-chip NMS models).
    Coordinates are returned as float pixels in the frame that was submitted."""
    w_in, h_in, pad_x, pad_y, scale, fw, fh = geom
    out = []
    flat = out_tensor.reshape(-1)
    if flat.size < 1:
        return out
    cnt = int(flat[0])
    for i in range(cnt):
        base = 1 + 5 * i
        if base + 5 > flat.size:
            break
        ymin, xmin, ymax, xmax, score = (float(v) for v in flat[base:base + 5])
        if score < score_thr:
            continue
        x1 = min(max((xmin * w_in - pad_x) / scale, 0.0), fw - 1.0)
        y1 = min(max((ymin * h_in - pad_y) / scale, 0.0), fh - 1.0)
        x2 = min(max((xmax * w_in - pad_x) / scale, 0.0), fw - 1.0)
        y2 = min(max((ymax * h_in - pad_y) / scale, 0.0), fh - 1.0)
        if x2 > x1 and y2 > y1:
            out.append((x1, y1, x2, y2, score))
    return out


# --------------------------------------------------------------------------- #
# NPU worker
# --------------------------------------------------------------------------- #
class HailoPersonDetector:
    """Asynchronous person detector on the Hailo-8. At most one frame is ever
    in flight; ``try_submit`` refuses (cheaply) while the NPU is busy."""

    def __init__(self, hef_path: str, *, score_thr: float = 0.20,
                 nms_iou: float = 0.5) -> None:
        self.hef_path = str(hef_path)
        self.score_thr = max(0.0, float(score_thr))
        self.nms_iou = float(nms_iou)
        self.state = "idle"          # idle | loading | ready | unavailable
        self._lock = threading.Lock()
        self._evt = threading.Event()
        self._stop = threading.Event()
        self._thread: Optional[threading.Thread] = None
        self._busy = False
        self._job: Optional[tuple] = None
        self._result: Optional[tuple] = None
        self._canvas: Optional[np.ndarray] = None
        self._in_hw = (0, 0)
        self._geom_key = None
        self._infer_ms = 0.0
        self._hz = 0.0
        self._last_done = 0.0
        self.errors = 0

    def start(self) -> None:
        if self._thread is not None and self._thread.is_alive():
            return
        self._stop.clear()
        self.state = "loading"
        self._thread = threading.Thread(target=self._run, name="npu-person", daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._stop.set()
        self._evt.set()

    def _run(self) -> None:
        model = None
        try:
            from drone_stack.gcs import hailo_infer as hi
            if not Path(self.hef_path).exists():
                raise FileNotFoundError(self.hef_path)
            model = hi._Model(self.hef_path, "person_lock")
        except Exception as exc:  # noqa: BLE001
            _log.warning("person lock: Hailo model unavailable (%s)", exc)
        if model is None or not model.ok:
            self.state = "unavailable"
            return
        h_in, w_in = int(model._in_hw[0]), int(model._in_hw[1])
        self._canvas = np.full((h_in, w_in, 3), 114, np.uint8)
        self._in_hw = (h_in, w_in)
        # Detect if this is a multi-head model (separate box/score outputs)
        is_multihead = len(model._out_names) > 1
        self.state = "ready"
        _log.info("person lock: %s on Hailo-8 (input %dx%d, %s)",
                  Path(self.hef_path).name, w_in, h_in,
                  f"{len(model._out_names)}-head" if is_multihead else "NMS")
        while not self._stop.is_set():
            if not self._evt.wait(0.5):
                continue
            self._evt.clear()
            job = self._job
            if job is None:
                continue
            seq, geom = job
            started = time.perf_counter()
            outs = model._run(self._canvas)
            ms = (time.perf_counter() - started) * 1000.0
            dets: list = []
            if outs is None:
                self.errors += 1
            elif is_multihead:
                # Multi-head output (e.g. visdronebest.hef with 6 tensors)
                dets = host_nms(parse_multihead_yolo(outs, geom, self.score_thr), self.nms_iou)
            else:
                # Single output with on-chip NMS (e.g. yolov8n.hef)
                out_tensor = next(iter(outs.values()))
                dets = host_nms(parse_nms_buffer(out_tensor, geom, self.score_thr), self.nms_iou)
            now = time.monotonic()
            with self._lock:
                self._result = (seq, dets)
                self._job = None
                self._busy = False
                self._infer_ms = ms if self._infer_ms == 0.0 else 0.9 * self._infer_ms + 0.1 * ms
                if self._last_done:
                    hz = 1.0 / max(now - self._last_done, 1e-3)
                    self._hz = hz if self._hz == 0.0 else 0.9 * self._hz + 0.1 * hz
                self._last_done = now

    def try_submit(self, seq: int, frame_bgr: np.ndarray) -> bool:
        """Letterbox ``frame_bgr`` into the model input and start inference.

        Runs on the camera thread and costs one colour conversion (~1.5 ms for
        a 1280-wide frame, which needs no resize). Must be called BEFORE any
        HUD is drawn on the frame, or the model would see its own box.
        """
        if self.state != "ready" or self._busy or cv2 is None:
            return False
        h_in, w_in = self._in_hw
        fh, fw = frame_bgr.shape[:2]
        scale = min(w_in / fw, h_in / fh)
        nw, nh = int(round(fw * scale)), int(round(fh * scale))
        pad_x, pad_y = (w_in - nw) // 2, (h_in - nh) // 2
        if self._geom_key != (fw, fh):
            self._canvas[:] = 114
            self._geom_key = (fw, fh)
        src = frame_bgr if (nw, nh) == (fw, fh) else \
            cv2.resize(frame_bgr, (nw, nh), interpolation=cv2.INTER_LINEAR)
        dst = self._canvas[pad_y:pad_y + nh, pad_x:pad_x + nw]
        if dst.flags["C_CONTIGUOUS"]:
            rgb = cv2.cvtColor(src, cv2.COLOR_BGR2RGB, dst=dst)
            if not np.shares_memory(rgb, dst):
                dst[...] = rgb
        else:
            dst[...] = cv2.cvtColor(src, cv2.COLOR_BGR2RGB)
        with self._lock:
            self._busy = True
            self._job = (seq, (w_in, h_in, pad_x, pad_y, scale, fw, fh))
        self._evt.set()
        return True

    def poll(self) -> Optional[tuple]:
        """(seq, detections) for the most recent finished frame, once."""
        with self._lock:
            result, self._result = self._result, None
        return result

    def stats(self) -> dict:
        return {"npu": self.state, "det_hz": round(self._hz, 1),
                "infer_ms": round(self._infer_ms, 1)}


# --------------------------------------------------------------------------- #
# the lock
# --------------------------------------------------------------------------- #
class TargetLock:
    """Kalman single-target lock in raw camera-frame pixels."""

    SEARCH, ACQUIRE, LOCKED = "search", "acquire", "locked"

    def __init__(self, *, acquire_conf: float = 0.40, keep_conf: float = 0.20,
                 confirm_hits: int = 2, hold_s: float = 1.5, exit_s: float = 0.3,
                 gate_iou: float = 0.10, gate_dist: float = 1.0,
                 min_box_px: float = 6.0, reacquire_s: float = 0.0,
                 reacquire_dist: float = 3.0) -> None:
        self.acquire_conf = float(acquire_conf)
        self.keep_conf = min(float(keep_conf), self.acquire_conf)
        self.confirm_hits = max(1, int(confirm_hits))
        self.hold_s = max(0.1, float(hold_s))
        self.exit_s = max(0.0, float(exit_s))
        self.gate_iou = float(gate_iou)
        self.gate_dist = float(gate_dist)
        self.min_box_px = float(min_box_px)
        # Re-acquisition memory. When a lock is lost the target is kept in
        # mind for this long, and a detection matching that memory re-locks
        # the SAME person at once, instead of the generic acquire path
        # picking whoever happens to be most confident. 0 disables it.
        self.reacquire_s = max(0.0, float(reacquire_s))
        self.reacquire_dist = float(reacquire_dist)
        self._mem = None
        self.locks = 0
        self.reacquires = 0
        # Where the recipient's phone says the right person is, as
        # (x, y, sigma) in RAW px, or None. Only consulted when the acquire
        # path has two or more people to choose between - see _pick_candidate.
        self.hint: Optional[tuple] = None
        self.reset()

    def reset(self) -> None:
        self.state = self.SEARCH
        self._cand: Optional[tuple] = None
        self._cand_t = 0.0
        self._hits = 0
        self._x = np.zeros(4)            # cx, cy, vx, vy  (px, px/frame)
        self._P = np.eye(4)
        self._wh = np.zeros(2)
        self._score = 0.0
        self._last_seen = 0.0
        self._lock_t = 0.0
        self._out_since: Optional[float] = None
        self._misses = 0

    def forget(self) -> None:
        """Drop the lock AND the memory of who it was.

        ``reset()`` keeps ``_mem`` so the same person can be re-locked after an
        occlusion. A re-lock onto someone else (the phone is on a different
        person) or a fresh scan must not have the old target re-acquired at
        once, so both are cleared here."""
        self._mem = None
        self.reset()

    # -- helpers --------------------------------------------------------
    def box(self) -> tuple:
        cx, cy = self._x[0], self._x[1]
        w, h = self._wh
        return (cx - w / 2.0, cy - h / 2.0, cx + w / 2.0, cy + h / 2.0, self._score)

    def _associate(self, ref: Sequence[float], dets: list, min_score: float) -> Optional[tuple]:
        rw, rh = max(ref[2] - ref[0], 1.0), max(ref[3] - ref[1], 1.0)
        rcx, rcy = (ref[0] + ref[2]) / 2.0, (ref[1] + ref[3]) / 2.0
        # The gate widens while coasting: the longer since the last match, the
        # further the real person may be from where the filter thinks.
        gate = self.gate_dist * (1.0 + 0.5 * min(self._misses, 3))
        best, best_cost = None, math.inf
        for d in dets:
            if d[4] < min_score:
                continue
            dw, dh = d[2] - d[0], d[3] - d[1]
            size_ratio = math.sqrt(max(dw * dh, 1.0) / (rw * rh))
            if not 0.5 <= size_ratio <= 2.0:
                continue
            ov = iou(ref, d)
            dist = math.hypot((d[0] + d[2]) / 2.0 - rcx, (d[1] + d[3]) / 2.0 - rcy) / max(rw, rh, 8.0)
            if ov < self.gate_iou and dist > gate:
                continue
            cost = (1.0 - ov) + 0.5 * dist - 0.25 * d[4]
            if cost < best_cost:
                best, best_cost = d, cost
        return best

    def _reacquire_pick(self, dets: list, now: float) -> Optional[tuple]:
        """Which detection (if any) is the person we just lost?

        ``self._mem`` holds the remembered target:
            box    (x1, y1, x2, y2) where it was last seen, in frame pixels
            wh     np.array([w, h]), its smoothed size
            score  its smoothed confidence when the lock was lost
            t      time.monotonic() at the moment it was lost
            id     the lock id it carried

        ``dets`` are this frame's detections as (x1, y1, x2, y2, score),
        already filtered to at least ``min_box_px``. ``now`` is monotonic.

        Return the chosen detection tuple to re-lock it immediately, or
        None to fall through to the normal acquire path.

        Available here: ``iou(a, b)``, ``self.reacquire_dist``,
        ``self.reacquire_s``, ``self.keep_conf``, ``self.acquire_conf``,
        plus ``math`` and ``np``.
        """
        mem = self._mem
        if mem is None or not dets:
            return None
        mw, mh = max(float(mem["wh"][0]), 1.0), max(float(mem["wh"][1]), 1.0)
        mcx = (mem["box"][0] + mem["box"][2]) / 2.0
        mcy = (mem["box"][1] + mem["box"][3]) / 2.0
        # The radius grows over the first second after the loss (a walking
        # person drifts) and then stops at 2x. At 5 m a person is ~100 px of a
        # 1280 px frame, so 2 x 3.0 box-heights is ~3 m on the ground; any
        # wider and, with 2-3 people under the scan, 'nearest' stops meaning
        # 'same'. Someone who walked further goes back through the normal
        # acquire path and its confirm_hits check.
        age = max(0.0, now - float(mem["t"]))
        radius = self.reacquire_dist * max(mw, mh) * (1.0 + min(age, 1.0))
        best, best_d = None, math.inf
        for d in dets:
            # The memory is the prior, so the occlusion floor is enough here;
            # demanding acquire_conf would throw away exactly the faint frames
            # a dropout is made of.
            if d[4] < self.keep_conf:
                continue
            dw, dh = max(d[2] - d[0], 1.0), max(d[3] - d[1], 1.0)
            if not 0.5 <= math.sqrt((dw * dh) / (mw * mh)) <= 2.0:
                continue
            dist = math.hypot((d[0] + d[2]) / 2.0 - mcx, (d[1] + d[3]) / 2.0 - mcy)
            # Nearest wins, NOT most confident: the confident one is what the
            # generic acquire already picks, and it may be a bystander.
            if dist <= radius and dist < best_d:
                best, best_d = d, dist
        return best

    def _pick_candidate(self, dets: list) -> Optional[tuple]:
        """Who to start acquiring: the most confident person, unless a phone
        hint is set and there is a choice to make.

        With a hint, each eligible detection costs 0.5*(d/sigma)^2 - log(score):
        a Gaussian distance to where the phone is plus the detector's own
        confidence, so a clearly better-scored box can still win against a
        faint one sitting on the hint."""
        eligible = [d for d in dets if d[4] >= self.acquire_conf]
        if not eligible:
            return None
        if self.hint is None or len(eligible) < 2:
            return max(eligible, key=lambda d: d[4])
        hx, hy, sig = self.hint
        sig = max(float(sig), 1.0)

        def cost(d):
            dx = (d[0] + d[2]) / 2.0 - hx
            dy = (d[1] + d[3]) / 2.0 - hy
            return 0.5 * (dx * dx + dy * dy) / (sig * sig) - math.log(max(d[4], 1e-3))
        return min(eligible, key=cost)

    def _should_switch(self, current_score: float, challenger: tuple) -> bool:
        """May a different person take over an existing lock?

        Called only while locked, for the most confident detection that did NOT
        match the locked target. Returning True drops the current lock and
        starts acquiring ``challenger`` instead.
        """
        return False

    def _start_lock(self, d: tuple, now: float) -> None:
        w, h = d[2] - d[0], d[3] - d[1]
        r = (0.05 * max(h, 8.0)) ** 2
        self._x = np.array([(d[0] + d[2]) / 2.0, (d[1] + d[3]) / 2.0, 0.0, 0.0])
        self._P = np.diag([r, r, (0.1 * h) ** 2, (0.1 * h) ** 2])
        self._wh = np.array([w, h], np.float64)
        self._score = float(d[4])
        self._last_seen = self._lock_t = now
        self._out_since = None
        self._misses = 0
        self._cand = None
        self.state = self.LOCKED
        self.locks += 1

    def _update(self, d: tuple, now: float) -> None:
        h = max(self._wh[1], 8.0)
        z = np.array([(d[0] + d[2]) / 2.0, (d[1] + d[3]) / 2.0])
        R = np.eye(2) * (0.05 * h) ** 2
        S = self._P[:2, :2] + R
        K = self._P[:, :2] @ np.linalg.inv(S)
        self._x = self._x + K @ (z - self._x[:2])
        H = np.zeros((2, 4)); H[0, 0] = H[1, 1] = 1.0
        self._P = (np.eye(4) - K @ H) @ self._P
        # Size is smoothed separately and more heavily: int8 box edges jitter
        # by several pixels, and a breathing box reads as a glitch.
        self._wh += 0.3 * (np.array([d[2] - d[0], d[3] - d[1]]) - self._wh)
        self._score += 0.4 * (float(d[4]) - self._score)
        self._last_seen = now
        self._misses = 0

    # -- per frame ------------------------------------------------------
    def begin_frame(self, motion: Optional[np.ndarray]) -> None:
        """Apply this frame's camera motion, then predict one frame ahead."""
        A = IDENTITY if motion is None else motion
        if self._cand is not None:
            self._cand = box_through(A, self._cand)
        if self.state != self.LOCKED:
            return
        cx, cy = apply_affine(A, self._x[0], self._x[1])
        v = A[:, :2] @ self._x[2:4]
        self._wh *= affine_scale(A)
        F = np.array([[1, 0, 1, 0], [0, 1, 0, 1], [0, 0, 1, 0], [0, 0, 0, 1]], np.float64)
        self._x = F @ np.array([cx, cy, v[0], v[1]])
        h = max(self._wh[1], 8.0)
        Q = np.diag([(0.01 * h) ** 2, (0.01 * h) ** 2, (0.02 * h) ** 2, (0.02 * h) ** 2])
        self._P = F @ self._P @ F.T + Q
        if self._misses:
            self._x[2:4] *= 0.9      # coasting: do not let a stale velocity fly off

    def observe(self, dets: list, now: float, lag_frames: int = 0) -> None:
        """Fuse one detection result (boxes already in this frame's pixels)."""
        dets = [d for d in dets
                if d[2] - d[0] >= self.min_box_px and d[3] - d[1] >= self.min_box_px]
        if self.state == self.LOCKED:
            if lag_frames > 0:
                sx, sy = self._x[2] * lag_frames, self._x[3] * lag_frames
                dets = [(d[0] + sx, d[1] + sy, d[2] + sx, d[3] + sy, d[4]) for d in dets]
            match = self._associate(self.box(), dets, self.keep_conf)
            if match is not None:
                self._update(match, now)
            else:
                self._misses += 1
            others = [d for d in dets if d is not match and d[4] >= self.acquire_conf]
            if others:
                challenger = max(others, key=lambda d: d[4])
                if self._should_switch(self._score, challenger):
                    self.reset()
                    self._cand, self._cand_t, self._hits = challenger, now, 1
                    self.state = self.ACQUIRE
            return

        # -- re-acquisition ---------------------------------------------
        # Before the generic 'most confident wins' acquire, give the person
        # we just lost first refusal. This is what makes the lock sticky
        # across an occlusion or a walk out of frame: the SAME target comes
        # back, instead of a different one taking the slot.
        if self._mem is not None:
            if now - self._mem["t"] > self.reacquire_s:
                self._mem = None
            else:
                again = self._reacquire_pick(dets, now)
                if again is not None:
                    self._start_lock(again, now)
                    self._mem = None
                    self.reacquires += 1
                    return

        confirm_score = (self.acquire_conf + self.keep_conf) / 2.0
        if self.state == self.ACQUIRE and self._cand is not None:
            match = self._associate(self._cand, dets, confirm_score)
            if match is not None:
                self._hits += 1
                if self._hits >= self.confirm_hits:
                    self._start_lock(match, now)
                    return
                self._cand, self._cand_t = match, now
                return
        best = self._pick_candidate(dets)
        if best is None:
            self.state, self._cand, self._hits = self.SEARCH, None, 0
            return
        if self.confirm_hits <= 1:
            self._start_lock(best, now)
            return
        self.state, self._cand, self._cand_t, self._hits = self.ACQUIRE, best, now, 1

    def _remember(self, now: float) -> None:
        """Snapshot the locked target so ``_reacquire_pick`` can find it again.

        Called on EVERY path that loses a lock. ``reset()`` deliberately does
        NOT clear ``_mem``: losing the track and forgetting who it was are two
        different things, and keeping them apart is exactly what lets the same
        person be re-locked rather than replaced.
        """
        if self.state != self.LOCKED:
            return
        self._mem = {
            "box": tuple(self.box()[:4]),
            "wh": self._wh.copy(),
            "score": float(self._score),
            "t": float(now),
            "id": int(self.locks),
        }

    def end_frame(self, now: float, visible: Callable[[tuple], bool]) -> None:
        if self.state == self.ACQUIRE and now - self._cand_t > 1.0:
            self.state, self._cand, self._hits = self.SEARCH, None, 0
        if self.state != self.LOCKED:
            return
        if now - self._last_seen > self.hold_s:
            self._remember(now)
            self.reset()
            return
        if visible(self.box()):
            self._out_since = None
        elif self._out_since is None:
            self._out_since = now
        elif now - self._out_since >= self.exit_s:
            self._remember(now)
            self.reset()

    def snapshot(self, now: float) -> dict:
        if self.state != self.LOCKED:
            return {"state": self.state, "box": None, "score": 0.0}
        fresh = now - self._last_seen < 0.35
        return {"state": "lock" if fresh else "hold", "box": self.box()[:4],
                "score": round(self._score, 3), "id": self.locks,
                "age_s": round(now - self._lock_t, 1)}


# --------------------------------------------------------------------------- #
# HUD
# --------------------------------------------------------------------------- #
def hud_layout(box: Sequence[float], state: str, score: float,
               width: int, height: int) -> Optional[dict]:
    """Where the lock HUD goes, independent of how it is painted - so the live
    BGR stream and the post-flight I420 replay draw the identical box."""
    if cv2 is None:
        return None
    x1 = int(round(min(max(box[0], 0), width - 1)))
    y1 = int(round(min(max(box[1], 0), height - 1)))
    x2 = int(round(min(max(box[2], 0), width - 1)))
    y2 = int(round(min(max(box[3], 0), height - 1)))
    if x2 - x1 < 2 or y2 - y1 < 2:
        return None
    t = max(2, int(round(height / 360.0)))
    arm = max(8, int(0.25 * min(x2 - x1, y2 - y1)))
    segs = [((x1, y1), (x1 + arm, y1)), ((x1, y1), (x1, y1 + arm)),
            ((x2, y1), (x2 - arm, y1)), ((x2, y1), (x2, y1 + arm)),
            ((x1, y2), (x1 + arm, y2)), ((x1, y2), (x1, y2 - arm)),
            ((x2, y2), (x2 - arm, y2)), ((x2, y2), (x2, y2 - arm))]
    locked = state == "lock"
    label = f"LOCK {score * 100:.0f}%" if locked else "HOLD"
    fs = 0.5 * height / 720.0
    ft = max(1, int(round(height / 720.0)))
    (tw, th), base = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, fs, ft)
    pad = max(3, t)
    ly2 = y1 - t if y1 - th - 2 * pad - t >= 0 else y2 + th + 2 * pad + t
    ly1 = ly2 - th - 2 * pad
    lx1 = x1
    lx2 = min(width - 1, x1 + tw + 2 * pad)
    return {"color": LOCK_BGR if locked else HOLD_BGR, "thickness": t,
            "segments": segs, "label": label, "label_rect": (lx1, ly1, lx2, ly2),
            "text_org": (lx1 + pad, ly2 - pad), "font_scale": fs, "font_thickness": ft}


def draw_hud_bgr(frame: np.ndarray, layout: Optional[dict]) -> np.ndarray:
    if layout is None or cv2 is None:
        return frame
    c, t = layout["color"], layout["thickness"]
    for p, q in layout["segments"]:
        cv2.line(frame, p, q, c, t, cv2.LINE_AA)
    lx1, ly1, lx2, ly2 = layout["label_rect"]
    cv2.rectangle(frame, (lx1, ly1), (lx2, ly2), c, -1)
    cv2.putText(frame, layout["label"], layout["text_org"], cv2.FONT_HERSHEY_SIMPLEX,
                layout["font_scale"], (0, 0, 0), layout["font_thickness"], cv2.LINE_AA)
    return frame


# --------------------------------------------------------------------------- #
# pipeline glue
# --------------------------------------------------------------------------- #
class PersonLockPipeline:
    """Per-frame glue between the stabiliser, the NPU worker and the lock.

    Coordinates: the lock lives in RAW capture pixels (before stabilisation),
    detections arrive in PUBLISHED pixels of the frame they were run on, and
    the HUD is drawn in PUBLISHED pixels of the current frame. The per-frame
    history of (camera motion, stabiliser matrix) is what connects the three.
    """

    def __init__(self, detector: Optional[HailoPersonDetector], lock: TargetLock,
                 history: int = 90, hfov_deg: float = 66.0) -> None:
        self.detector = detector
        self.lock = lock
        # Horizontal field of view of the RAW frame, for the normalised image
        # plane the navigator works in (nx = tan of the angle right of axis).
        self.hfov_deg = float(hfov_deg)
        # The navigator's phone hint, (nx, ny, sigma) or None, and the latest
        # re-lock request seq. Written from the bus thread, read on the capture
        # thread: each is one attribute swap, and every lock mutation they
        # cause happens in process(), never here.
        self._hint: Optional[tuple] = None
        self._relock_seq = 0
        self._relock_done = 0
        self._forget_pending = False
        self._hist: deque = deque(maxlen=max(8, int(history)))
        self._seq = 0
        self._snap: dict = {"state": TargetLock.SEARCH, "box": None, "score": 0.0}
        self._enabled = False   # gated by mission phase; enabled on HOVER
        # Every detection of the latest NPU result, RAW px, carried frame to
        # frame by camera motion like the lock box. Recorded so the replay can
        # show what YOLO saw, not only the one person it locked onto. Dropped
        # after DETS_TTL_S: a box that no newer result has confirmed is a
        # guess, and the lock box already covers the person that matters.
        self._dets: list = []
        self._dets_t = 0.0

    DETS_TTL_S = 0.5

    # -- mission-phase gate --------------------------------------------------
    @property
    def enabled(self) -> bool:
        return self._enabled

    @enabled.setter
    def enabled(self, val: bool) -> None:
        if val != self._enabled:
            self._enabled = val
            self._dets = []
            if val:
                # A new hover is a new search: whoever was locked on the last
                # one (or on the ground) must not be re-acquired from memory.
                self._forget_pending = True
                self._hint = None
            _log.info("person lock %s", "ENABLED" if val else "DISABLED")

    def set_hint(self, hint) -> None:
        """Take a PhoneHint from the navigator (any object with valid, nx, ny,
        sigma, relock, seq). ``relock`` with a new seq drops the current lock
        so the hinted person can be acquired instead."""
        if not getattr(hint, "valid", False):
            self._hint = None
            return
        self._hint = (float(hint.nx), float(hint.ny), max(float(hint.sigma), 1e-3))
        if getattr(hint, "relock", False):
            self._relock_seq = max(self._relock_seq, int(getattr(hint, "seq", 0)))

    def _fx(self, width: int) -> float:
        return (width / 2.0) / math.tan(math.radians(self.hfov_deg) / 2.0)

    def start(self) -> None:
        if self.detector is not None:
            self.detector.start()

    def stop(self) -> None:
        if self.detector is not None:
            self.detector.stop()

    def _to_current(self, seq_k: int, dets: list) -> Optional[list]:
        entries = list(self._hist)
        idx = next((i for i, e in enumerate(entries) if e[0] == seq_k), None)
        if idx is None:
            return None              # older than the history - stale, ignore
        Minv = np.linalg.inv(to3(entries[idx][2]))[:2]
        T = np.eye(3)
        for _, A, _ in entries[idx + 1:]:
            T = to3(A) @ T
        C = (T @ to3(Minv))[:2]
        return [box_through(C, d) for d in dets]

    def process(self, frame_out: np.ndarray, motion: Optional[np.ndarray],
                matrix: Optional[np.ndarray], now: Optional[float] = None,
                raw_shape: Optional[tuple] = None) -> dict:
        """Advance one video frame and draw the HUD onto ``frame_out`` in place.

        ``raw_shape`` is the RAW capture frame's shape (the lock's pixel
        space); it defaults to ``frame_out``'s, which is right whenever the
        stabiliser renders at capture size."""
        if not self._enabled:
            return {"state": TargetLock.SEARCH, "box": None, "score": 0.0}
        now = time.monotonic() if now is None else now
        raw_h, raw_w = (raw_shape or frame_out.shape)[:2]
        fx = self._fx(raw_w)
        rcx, rcy = raw_w / 2.0, raw_h / 2.0
        if self._forget_pending:
            self._forget_pending = False
            self.lock.forget()
        if self._relock_seq != self._relock_done:
            self._relock_done = self._relock_seq
            self.lock.forget()
            _log.info("person lock: re-lock requested (phone is on another person)")
        hint = self._hint
        self.lock.hint = None if hint is None else (
            rcx + hint[0] * fx, rcy - hint[1] * fx, hint[2] * fx)
        A = IDENTITY if motion is None else motion
        M = IDENTITY if matrix is None else matrix
        self._seq += 1
        seq = self._seq
        self._hist.append((seq, A, M))
        self.lock.begin_frame(A)
        if self._dets:
            self._dets = [box_through(A, d) for d in self._dets]
        result = self.detector.poll() if self.detector is not None else None
        if result is not None:
            mapped = self._to_current(result[0], result[1])
            if mapped is not None:
                self.lock.observe(mapped, now, lag_frames=seq - result[0])
                self._dets, self._dets_t = list(mapped), now
        if self._dets and now - self._dets_t > self.DETS_TTL_S:
            self._dets = []
        out_h, out_w = frame_out.shape[:2]

        def visible(raw_box):
            cx, cy = apply_affine(M, (raw_box[0] + raw_box[2]) / 2.0,
                                  (raw_box[1] + raw_box[3]) / 2.0)
            return 0.0 <= cx < out_w and 0.0 <= cy < out_h

        self.lock.end_frame(now, visible)
        if self.detector is not None:
            self.detector.try_submit(seq, frame_out)      # before the HUD
        snap = self.lock.snapshot(now)
        # [x1, y1, x2, y2, score], RAW px - the recorder's index format.
        snap["dets"] = [[round(float(v), 1) for v in d[:4]] + [round(float(d[4]), 2)]
                        for d in self._dets if len(d) >= 5]
        # The same people on the normalised image plane, for the navigator.
        snap["people_xy"] = [
            [round(((d[0] + d[2]) / 2.0 - rcx) / fx, 4),
             round(-((d[1] + d[3]) / 2.0 - rcy) / fx, 4), round(float(d[4]), 3)]
            for d in self._dets if len(d) >= 5]
        if snap["box"] is not None:
            b = snap["box"]
            snap["nx"] = round(((b[0] + b[2]) / 2.0 - rcx) / fx, 4)
            snap["ny"] = round(-((b[1] + b[3]) / 2.0 - rcy) / fx, 4)
        if snap["box"] is not None:
            bo = box_through(M, snap["box"])
            snap["box_out"] = tuple(round(v, 1) for v in bo[:4])
            draw_hud_bgr(frame_out, hud_layout(bo, snap["state"], snap["score"], out_w, out_h))
        self._snap = snap
        return snap

    def stats(self) -> dict:
        out = dict(self.detector.stats()) if self.detector is not None else {"npu": "off"}
        out["lock"] = self._snap.get("state")
        out["score"] = self._snap.get("score", 0.0)
        return out


def person_lock_from_config(section: dict | None) -> Optional[PersonLockPipeline]:
    """Build from the ``cameras.person_lock`` block; None when disabled.

    Nothing touches the Hailo here - the model loads on the worker thread when
    the camera starts, so constructing a CameraManager (tests, sim) is free.
    """
    section = section or {}
    if not bool(section.get("enabled", False)):
        return None
    hef = Path(str(section.get("hef", "")))
    if not hef.is_absolute() and not hef.exists():
        repo_root = Path(__file__).resolve().parents[2]
        if (repo_root / hef).exists():
            hef = repo_root / hef
    detector = HailoPersonDetector(str(hef), score_thr=float(section.get("score_floor", 0.20)),
                                   nms_iou=float(section.get("nms_iou", 0.5)))
    lock = TargetLock(acquire_conf=float(section.get("acquire_conf", 0.40)),
                      keep_conf=float(section.get("keep_conf", 0.20)),
                      confirm_hits=int(section.get("confirm_hits", 2)),
                      hold_s=float(section.get("hold_s", 1.5)),
                      exit_s=float(section.get("exit_s", 0.3)),
                      reacquire_s=float(section.get("reacquire_s", 0.0)),
                      reacquire_dist=float(section.get("reacquire_dist", 3.0)))
    return PersonLockPipeline(detector, lock, hfov_deg=float(section.get("hfov_deg", 66.0)))
