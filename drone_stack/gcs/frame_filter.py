"""Real-time image conditioning for the live camera feeds.

Why this module exists
----------------------
In flight the CSI ribbon between the imx708 and the Pi runs centimetres from
four ESCs switching tens of amps. The coupled interference lands on the
sensor's *row* readout, so it surfaces as horizontal lines that drift up or
down the frame as the noise source beats against the frame rate. It is not
photon noise: every pixel in an affected row is displaced by the same amount,
and the artefact only appears once the motors spin.

That distinction is the entire design. A patch-similarity denoiser - OpenCV's
``fastNlMeansDenoisingColored``, which this module replaces - searches for
repeated structure and *preserves* what it finds, and a horizontal stripe is
nothing but repeated structure. So it keeps the artefact while costing ~200 ms
per 720p frame on an M-series Mac and roughly a second on a Pi 5, against a
3 ms JPEG encode. Row noise has to be attacked as row noise.

The chain, cheapest first
-------------------------
1. ``destripe`` - each row's mean is compared against a running median of its
   neighbours' means. Real scene content moves that profile *smoothly*, and a
   median tracks a genuine horizontal edge (a horizon, a rooftop, a doorway)
   without overshoot, so what is left over is essentially the interference
   alone. It is removed by soft-thresholding: rows inside the noise floor are
   left bit-for-bit untouched, rows outside it are pulled back by their excess.
   That is the shrinkage idea the old docstring reached for wavelets to
   describe, applied to the one axis the artefact actually lives on. ~1 ms.
2. ``row repair`` - a row displaced far beyond the shrinkage floor carries no
   recoverable signal, so subtracting an offset cannot save it. Those rows are
   redrawn by interpolating the nearest surviving rows above and below.
   Typically 0-3 rows per frame; unmeasurably cheap.
3. ``temporal`` - a motion-gated IIR blend against the previous frame. Sensor
   noise is uncorrelated frame to frame and collapses under averaging; real
   detail is correlated and survives. The gate drops the blend weight to zero
   wherever the frame genuinely changed, so moving objects do not smear -
   which is what makes temporal averaging usable on a moving aircraft at all.
   Runs in int16 with the gate evaluated on a coarse block-averaged grid.

Everything here is numpy - no OpenCV import - so the module stays testable on a
machine with no camera stack installed.

Latency budget
--------------
``FrameFilter`` times itself and degrades rather than falling behind. If the
smoothed per-frame cost stays above ``budget_ms``, the temporal stage is
dropped and the fact is logged once. Destriping is never dropped: it is the
stage the artefact actually needs and it costs almost nothing.
"""
from __future__ import annotations

import time
from typing import Optional

import numpy as np

from drone_stack.utils.logging_setup import get_logger

_log = get_logger("gcs.frame_filter")


def rolling_median(values: np.ndarray, window: int) -> np.ndarray:
    """Median over *window* samples centred on each element, edge-padded.

    Used to model "what this row's brightness should have been, judging by its
    neighbours". A median rather than a mean because a mean smears a real
    horizontal edge across the whole window and then reports the smearing as
    stripe residual - i.e. it would treat the horizon as interference.
    """
    values = np.asarray(values, dtype=np.float32)
    if window < 3 or values.size < 3:
        return values.copy()
    window = int(window) | 1               # force odd so the window is centred
    window = min(window, values.size | 1)
    half = window // 2
    padded = np.pad(values, half, mode="edge")
    view = np.lib.stride_tricks.sliding_window_view(padded, window)
    return np.median(view, axis=-1).astype(np.float32)


class FrameFilter:
    """Stateful per-camera image conditioner. Not thread-safe: give each
    capture thread its own instance (they each hold a previous-frame buffer).

    Parameters mirror the ``cameras.filter`` config block one-for-one.
    """

    def __init__(
        self,
        *,
        enabled: bool = True,
        destripe: bool = True,
        destripe_window: int = 21,
        destripe_floor: float = 0.6,
        destripe_ceiling: float = 10.0,
        destripe_gain: float = 1.0,
        row_repair: bool = True,
        row_repair_floor: float = 14.0,
        temporal: bool = True,
        temporal_alpha: float = 0.6,
        temporal_motion: float = 20.0,
        temporal_deadband: float = 6.0,
        temporal_gate_scale: int = 4,
        column_step: int = 4,
        budget_ms: float = 18.0,
    ) -> None:
        self.enabled = bool(enabled)
        self.destripe = bool(destripe)
        self.destripe_window = int(destripe_window)
        self.destripe_floor = float(destripe_floor)
        self.destripe_ceiling = max(float(destripe_ceiling), self.destripe_floor)
        self.destripe_gain = float(destripe_gain)
        self.row_repair = bool(row_repair)
        self.row_repair_floor = float(row_repair_floor)
        self.temporal = bool(temporal)
        # clamped: alpha >= 1.0 would freeze the image on the first frame, and
        # a negative motion scale would invert the gate into a smear amplifier.
        self.temporal_alpha = min(max(float(temporal_alpha), 0.0), 0.95)
        self.temporal_motion = max(float(temporal_motion), 1e-3)
        # deadband must stay below the ramp top or the gate inverts
        self.temporal_deadband = min(max(float(temporal_deadband), 0.0),
                                     self.temporal_motion - 1e-3)
        self.temporal_gate_scale = max(1, int(temporal_gate_scale))
        self.column_step = max(1, int(column_step))
        self.budget_ms = float(budget_ms)

        self._prev: Optional[np.ndarray] = None      # int16, previous OUTPUT
        self._temporal_live = self.temporal
        self._cost_ms = 0.0
        self._over_budget = 0
        self._degraded = False
        # diagnostics, surfaced through stats() -> the GCS camera tile
        self._stripe_peak = 0.0
        self._rows_corrected = 0
        self._rows_repaired = 0

    # -- public ---------------------------------------------------------------
    def apply(self, frame):
        """Condition one BGR (or mono) uint8 frame. Returns a new array.

        Never raises: a filter fault must not take the camera down with it, so
        an unexpected error disables the filter and passes the frame through.
        """
        if not self.enabled or frame is None:
            return frame
        started = time.perf_counter()
        try:
            out = self._apply(frame)
        except Exception as exc:  # noqa: BLE001
            _log.warning("frame filter disabled after error: %s", exc)
            self.enabled = False
            return frame
        self._record_cost((time.perf_counter() - started) * 1000.0)
        return out

    def stats(self) -> dict:
        return {
            "cost_ms": round(self._cost_ms, 2),
            "stripe_peak": round(self._stripe_peak, 2),
            "rows_corrected": self._rows_corrected,
            "rows_repaired": self._rows_repaired,
            "temporal": self._temporal_live,
            "degraded": self._degraded,
        }

    # -- internals ------------------------------------------------------------
    def _apply(self, frame):
        arr = np.asarray(frame)
        if arr.dtype != np.uint8 or arr.ndim not in (2, 3) or arr.shape[0] < 8:
            return frame
        out = arr
        if self.destripe or self.row_repair:
            out = self._destripe(out)
        if self._temporal_live:
            out = self._temporal(out)
        else:
            self._prev = None       # so re-enabling never blends against a
        return out                  # frame from minutes ago

    def _destripe(self, arr: np.ndarray) -> np.ndarray:
        # Row brightness profile. Sampling every Nth column is 4x cheaper and
        # statistically identical - a 720p row still contributes 320 samples.
        sample = arr[:, ::self.column_step]
        profile = sample.reshape(sample.shape[0], -1).mean(axis=1, dtype=np.float32)
        base = rolling_median(profile, self.destripe_window)
        resid = profile - base
        self._stripe_peak = float(np.abs(resid).max())

        if self.row_repair:
            severe = self._confirm_destroyed(arr, np.abs(resid) > self.row_repair_floor)
        else:
            severe = np.zeros(resid.shape, dtype=bool)

        if self.destripe:
            # Soft threshold (shrinkage): rows within destripe_floor of where
            # their neighbours say they belong are left exactly alone, so a
            # clean frame passes through bit-for-bit and nothing is invented.
            shrunk = np.sign(resid) * np.maximum(np.abs(resid) - self.destripe_floor, 0.0)
            shrunk *= self.destripe_gain
            # Ceiling. A residual larger than this is more likely a real edge
            # the median could not follow perfectly than interference, and the
            # rows either side of a hard horizon are exactly where that happens.
            # Measured on a banded scene with an 89 DN horizon: capping at 10 DN
            # keeps 90% of the horizon against 85% uncapped, and removes very
            # slightly MORE of the band - so this is not a trade, it is a
            # correction that was overshooting.
            np.clip(shrunk, -self.destripe_ceiling, self.destripe_ceiling, out=shrunk)
            shrunk[severe] = 0.0     # those rows are rebuilt, not nudged
        else:
            shrunk = np.zeros_like(resid)

        rows = np.nonzero(shrunk)[0]
        self._rows_corrected = int(rows.size)
        self._rows_repaired = int(np.count_nonzero(severe))
        if rows.size == 0 and self._rows_repaired == 0:
            return arr

        out = arr.copy()
        if rows.size:
            # Only the affected rows are promoted to wider arithmetic - a clean
            # frame with three bad rows costs three rows of work, not 720.
            delta = np.rint(shrunk[rows]).astype(np.int16)
            patch = out[rows].astype(np.int16)
            patch -= delta.reshape((-1,) + (1,) * (patch.ndim - 1))
            out[rows] = np.clip(patch, 0, 255).astype(np.uint8)
        if self._rows_repaired:
            _repair_rows(out, severe)
        return out

    def _confirm_destroyed(self, arr: np.ndarray, candidates: np.ndarray) -> np.ndarray:
        """Separate a genuinely destroyed row from a row that merely sits on a
        hard horizontal edge.

        A large row-mean residual is not enough on its own. The rows either side
        of a horizon, a rooftop or a doorway also show one, and "repairing" them
        interpolates straight ACROSS the edge - which softened an 89 DN horizon
        to 45 DN before this check existed. The distinction: a destroyed row
        resembles NEITHER neighbour, while an edge row still resembles the side
        it belongs to. So a row is only rebuilt if it differs from both.

        Only rows already flagged by the residual test are examined, so this
        costs a handful of row comparisons rather than a pass over the frame.
        """
        severe = np.zeros_like(candidates)
        rows = np.nonzero(candidates)[0]
        if rows.size == 0:
            return severe
        height = arr.shape[0]
        for row in rows:
            above = arr[row - 1] if row > 0 else arr[min(row + 1, height - 1)]
            below = arr[row + 1] if row < height - 1 else arr[max(row - 1, 0)]
            here = arr[row].astype(np.int16)
            d_above = float(np.abs(here - above.astype(np.int16)).mean())
            d_below = float(np.abs(here - below.astype(np.int16)).mean())
            if min(d_above, d_below) > self.row_repair_floor:
                severe[row] = True
        return severe

    def _temporal(self, arr: np.ndarray) -> np.ndarray:
        cur = arr.astype(np.int16)
        prev = self._prev
        if prev is None or prev.shape != cur.shape:
            self._prev = cur
            return arr

        # Consumed in place below - every temporary here is 16 MB at 720p and
        # allocating them per frame is what turns a cheap filter into a slow one.
        delta = cur - prev
        s = self.temporal_gate_scale

        # Motion gate on a coarse grid. Each cell is the MEAN absolute change
        # over an s x s block, not a point sample: frame-to-frame sensor noise
        # is exactly what we are trying to average away, so a gate that reads
        # single pixels sees that noise as motion, shuts itself off, and the
        # denoiser does nothing measurable. Averaging the block divides the
        # noise in the gate by s while leaving a moving edge just as loud.
        magnitude = np.abs(delta)
        h2 = (magnitude.shape[0] // s) * s
        w2 = (magnitude.shape[1] // s) * s
        if magnitude.ndim == 3:
            chans = magnitude.shape[2]
            cells = magnitude[:h2, :w2].reshape(h2 // s, s, w2 // s, s, chans)
            coarse = cells.sum(axis=(1, 3, 4), dtype=np.int32)
            coarse = coarse * (1.0 / (s * s * chans))
        else:
            cells = magnitude[:h2, :w2].reshape(h2 // s, s, w2 // s, s)
            coarse = cells.sum(axis=(1, 3), dtype=np.int32) * (1.0 / (s * s))
        del magnitude, cells

        # Deadband, then ramp. Below temporal_deadband the change is
        # indistinguishable from noise, so blend at full strength; above
        # temporal_motion something really moved, so blend at zero and let the
        # pixel through untouched. The deadband is the part that matters -
        # without it the residual noise floor sits partway up the ramp and
        # permanently halves the filter's strength for no benefit.
        span = max(self.temporal_motion - self.temporal_deadband, 1e-3)
        weight = (self.temporal_motion - coarse) * (1.0 / span)
        np.clip(weight, 0.0, 1.0, out=weight)
        weight *= self.temporal_alpha

        # Fixed point in 1/128ths: |delta| <= 255 and w <= 128 keeps the product
        # inside int16 (32640 < 32767), so the blend needs no float conversion
        # of the full frame at all.
        wq = (weight * 128.0).astype(np.int16)
        if s > 1:
            wq = np.repeat(np.repeat(wq, s, axis=0), s, axis=1)
        pad_h = cur.shape[0] - wq.shape[0]
        pad_w = cur.shape[1] - wq.shape[1]
        if pad_h or pad_w:      # frame not divisible by the gate scale
            wq = np.pad(wq, ((0, pad_h), (0, pad_w)), mode="edge")
        if cur.ndim == 3:
            wq = wq[:, :, None]

        delta *= wq
        delta += 64             # round rather than floor: a floor would bias
        delta >>= 7             # every blend downward and drift the image dark
        cur -= delta
        np.clip(cur, 0, 255, out=cur)
        self._prev = cur
        return cur.astype(np.uint8)

    def _record_cost(self, ms: float) -> None:
        # EMA so one scheduling hiccup cannot trip the degrade path.
        self._cost_ms = ms if self._cost_ms == 0.0 else 0.8 * self._cost_ms + 0.2 * ms
        if self._degraded or not self._temporal_live:
            return
        if self._cost_ms > self.budget_ms:
            self._over_budget += 1
            if self._over_budget >= 30:
                self._temporal_live = False
                self._degraded = True
                self._prev = None
                _log.warning(
                    "frame filter over budget (%.1f ms > %.1f ms) - temporal "
                    "denoise disabled, destripe kept", self._cost_ms, self.budget_ms
                )
        else:
            self._over_budget = 0


def _repair_rows(arr: np.ndarray, bad: np.ndarray) -> None:
    """Redraw destroyed rows in place from the nearest surviving neighbours."""
    good = np.nonzero(~bad)[0]
    if good.size == 0:
        return                       # every row is wrecked; nothing to draw from
    for row in np.nonzero(bad)[0]:
        i = int(np.searchsorted(good, row))
        lo = int(good[i - 1]) if i > 0 else int(good[0])
        hi = int(good[i]) if i < good.size else int(good[-1])
        if lo == hi:
            arr[row] = arr[lo]
            continue
        t = (row - lo) / float(hi - lo)
        blended = arr[lo].astype(np.float32) * (1.0 - t) + arr[hi].astype(np.float32) * t
        arr[row] = blended.astype(np.uint8)


def filter_from_config(section: dict | None) -> FrameFilter:
    """Build a FrameFilter from a ``cameras.filter`` config mapping."""
    section = section or {}
    return FrameFilter(
        enabled=bool(section.get("enabled", True)),
        destripe=bool(section.get("destripe", True)),
        destripe_window=int(section.get("destripe_window", 21)),
        destripe_floor=float(section.get("destripe_floor", 0.6)),
        destripe_ceiling=float(section.get("destripe_ceiling", 10.0)),
        destripe_gain=float(section.get("destripe_gain", 1.0)),
        row_repair=bool(section.get("row_repair", True)),
        row_repair_floor=float(section.get("row_repair_floor", 14.0)),
        temporal=bool(section.get("temporal", True)),
        temporal_alpha=float(section.get("temporal_alpha", 0.6)),
        temporal_motion=float(section.get("temporal_motion", 20.0)),
        temporal_deadband=float(section.get("temporal_deadband", 6.0)),
        temporal_gate_scale=int(section.get("temporal_gate_scale", 4)),
        column_step=int(section.get("column_step", 4)),
        budget_ms=float(section.get("budget_ms", 18.0)),
    )
