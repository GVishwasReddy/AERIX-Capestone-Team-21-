"""Hailo-8 AI HAT+ inference for the Drone GCS camera feeds.

All neural inference runs on the **Hailo-8 (26 TOPS) NPU**, never on the Pi CPU.
A single scheduler-backed ``VDevice`` is shared by every model so the two camera
threads can run concurrently and the HailoRT scheduler time-slices the device.

Three model wrappers:

* ``Detector``  - ``yolov8n.hef`` (single class = person, on-chip NMS). Returns
                  boxes in the *original* frame's pixel coordinates and draws
                  bounding boxes + labels. Used on the **Pi camera**.
* ``Segmenter`` - legacy binary ``terrain.hef`` (384x640 -> 384x640x1 logit
                  map). Sigmoid + threshold -> translucent colour overlay.
* ``MultiClassSegmenter`` - genuinely multi-class terrain model (e.g.
                  ``fabseg.hef``, 7 classes matching
                  ``drone_stack.novelty.types.TerrainClass``). Per-pixel
                  argmax over the output's class-logit channels -> an int8
                  class-index map. No sigmoid/threshold - argmax has none.

Everything degrades gracefully: if HailoRT / a HEF is unavailable the wrappers
become no-ops (``ok == False``) and the camera simply serves the raw frame, so
the GCS keeps running in sim / on machines with no accelerator.

Design goals honoured here:
* inference on the NPU only -> CPU/RAM stay free,
* thread-safe (each model is driven by exactly one camera thread; the VDevice
  is process-global and created once),
* no per-frame allocations in the hot path (bindings + buffers are reused).
"""
from __future__ import annotations

import threading
import time
from typing import List, Optional, Tuple

import numpy as np

from drone_stack.utils.logging_setup import get_logger

_log = get_logger("gcs.hailo")

try:  # OpenCV is used for resize / letterbox / drawing only (cheap, CPU).
    import cv2
except Exception as exc:  # noqa: BLE001  pragma: no cover
    cv2 = None  # type: ignore
    _log.warning("hailo_infer: OpenCV unavailable (%s)", exc)

try:
    import hailo_platform as hp
except Exception as exc:  # noqa: BLE001  pragma: no cover
    hp = None  # type: ignore
    _log.warning("hailo_infer: hailo_platform unavailable (%s)", exc)


# --------------------------------------------------------------------------- #
# Shared VDevice (scheduler) - created exactly once, lazily.
# --------------------------------------------------------------------------- #
_vdevice = None
_vdevice_lock = threading.Lock()


def _get_vdevice():
    """Return the process-global scheduler VDevice, creating it on first use."""
    global _vdevice
    if hp is None:
        return None
    with _vdevice_lock:
        if _vdevice is None:
            params = hp.VDevice.create_params()
            params.scheduling_algorithm = hp.HailoSchedulingAlgorithm.ROUND_ROBIN
            _vdevice = hp.VDevice(params)
            _log.info("Hailo VDevice created (scheduler=ROUND_ROBIN)")
        return _vdevice


class _Model:
    """Thin wrapper around a configured InferModel kept resident on the NPU."""

    def __init__(self, hef_path: str, name: str) -> None:
        self.name = name
        self.ok = False
        self._cim = None
        self._cm = None
        self._bindings = None
        self._in_hw: Tuple[int, int, int] = (0, 0, 0)  # h, w, c
        self._out_bufs: dict = {}
        self._out_names: List[str] = []
        if hp is None or cv2 is None:
            return
        vdev = _get_vdevice()
        if vdev is None:
            return
        try:
            im = vdev.create_infer_model(hef_path)
            im.input().set_format_type(hp.FormatType.UINT8)
            for on in im.output_names:
                im.output(on).set_format_type(hp.FormatType.FLOAT32)
            self._out_names = list(im.output_names)
            self._in_hw = tuple(im.input().shape)  # (h, w, c)
            # Configure (activate) and keep it resident for the process lifetime.
            self._cm = im.configure()
            self._cim = self._cm.__enter__()
            self._bindings = self._cim.create_bindings()
            for on in self._out_names:
                buf = np.zeros(im.output(on).shape, dtype=np.float32)
                self._out_bufs[on] = buf
                self._bindings.output(on).set_buffer(buf)
            self.ok = True
            _log.info("Hailo model '%s' ready (in=%s outs=%s)",
                      name, self._in_hw, self._out_names)
        except Exception as exc:  # noqa: BLE001
            _log.warning("Hailo model '%s' failed to load: %s", name, exc)
            self.ok = False

    def _run(self, chw_uint8: np.ndarray) -> Optional[dict]:
        """Feed one HxWxC uint8 buffer, return dict{out_name: float32 ndarray}."""
        if not self.ok:
            return None
        try:
            self._bindings.input().set_buffer(np.ascontiguousarray(chw_uint8))
            self._cim.run([self._bindings], 2000)
            return self._out_bufs
        except Exception as exc:  # noqa: BLE001
            _log.warning("Hailo model '%s' inference error: %s", self.name, exc)
            return None


def _sigmoid(x: np.ndarray) -> np.ndarray:
    return 1.0 / (1.0 + np.exp(-x))


class Detector(_Model):
    """yolov8n person detector (on-chip NMS). Boxes in frame pixel coords."""

    # Hailo NMS-by-class FLOAT32 flat buffer layout, per class:
    #   [count, (y_min, x_min, y_max, x_max, score) * count, ...padding]
    # coords are normalised 0..1 relative to the (letterboxed) 640x640 input.

    def __init__(self, hef_path: str, name: str = "yolov8n",
                 score_thr: float = 0.25, label: str = "PERSON") -> None:
        super().__init__(hef_path, name)
        self.score_thr = score_thr
        self.label = label

    def infer(self, frame_bgr: np.ndarray) -> List[Tuple[int, int, int, int, float]]:
        if not self.ok:
            return []
        h_in, w_in, _ = self._in_hw
        fh, fw = frame_bgr.shape[:2]
        # letterbox (preserve aspect ratio) -> w_in x h_in
        scale = min(w_in / fw, h_in / fh)
        nw, nh = int(round(fw * scale)), int(round(fh * scale))
        pad_x, pad_y = (w_in - nw) // 2, (h_in - nh) // 2
        resized = cv2.resize(frame_bgr, (nw, nh), interpolation=cv2.INTER_LINEAR)
        canvas = np.full((h_in, w_in, 3), 114, dtype=np.uint8)
        canvas[pad_y:pad_y + nh, pad_x:pad_x + nw] = resized
        rgb = cv2.cvtColor(canvas, cv2.COLOR_BGR2RGB)
        outs = self._run(rgb)
        if outs is None:
            return []
        flat = next(iter(outs.values())).reshape(-1)
        dets: List[Tuple[int, int, int, int, float]] = []
        i = 0
        cnt = int(flat[i]); i += 1
        for _ in range(cnt):
            if i + 5 > flat.size:
                break
            ymin, xmin, ymax, xmax, score = flat[i:i + 5]; i += 5
            if score < self.score_thr:
                continue
            # de-letterbox back to original frame pixels
            x1 = (xmin * w_in - pad_x) / scale
            y1 = (ymin * h_in - pad_y) / scale
            x2 = (xmax * w_in - pad_x) / scale
            y2 = (ymax * h_in - pad_y) / scale
            x1 = max(0, min(fw - 1, int(round(x1))))
            y1 = max(0, min(fh - 1, int(round(y1))))
            x2 = max(0, min(fw - 1, int(round(x2))))
            y2 = max(0, min(fh - 1, int(round(y2))))
            if x2 > x1 and y2 > y1:
                dets.append((x1, y1, x2, y2, float(score)))
        return dets

    def draw(self, frame_bgr: np.ndarray,
             dets: List[Tuple[int, int, int, int, float]]) -> np.ndarray:
        for (x1, y1, x2, y2, score) in dets:
            cv2.rectangle(frame_bgr, (x1, y1), (x2, y2), (0, 235, 255), 2)
            tag = f"{self.label} {score * 100:.0f}%"
            (tw, th), _ = cv2.getTextSize(tag, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1)
            cv2.rectangle(frame_bgr, (x1, y1 - th - 6), (x1 + tw + 4, y1),
                          (0, 235, 255), -1)
            cv2.putText(frame_bgr, tag, (x1 + 2, y1 - 4),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 1, cv2.LINE_AA)
        return frame_bgr

    def process(self, frame_bgr: np.ndarray) -> np.ndarray:
        return self.draw(frame_bgr, self.infer(frame_bgr))


class Segmenter(_Model):
    """terrain segmentation (single-channel logits) -> translucent overlay."""

    def __init__(self, hef_path: str, name: str = "terrain",
                 thr: float = 0.5, alpha: float = 0.30,
                 colour: Tuple[int, int, int] = (0, 200, 60)) -> None:
        super().__init__(hef_path, name)
        self.thr = thr
        # sigmoid(logit) >= thr  <=>  logit >= ln(thr/(1-thr)). Thresholding the
        # raw logits avoids an exp() over ~250k pixels every frame (CPU saver).
        thr = min(max(thr, 1e-6), 1 - 1e-6)
        self._logit_thr = float(np.log(thr / (1.0 - thr)))
        self.alpha = alpha
        self.colour = np.array(colour, dtype=np.uint8)  # BGR

    def infer_mask(self, frame_bgr: np.ndarray) -> Optional[np.ndarray]:
        """Return a bool mask at the *model* resolution, or None."""
        if not self.ok:
            return None
        h_in, w_in, _ = self._in_hw
        resized = cv2.resize(frame_bgr, (w_in, h_in), interpolation=cv2.INTER_LINEAR)
        rgb = cv2.cvtColor(resized, cv2.COLOR_BGR2RGB)
        outs = self._run(rgb)
        if outs is None:
            return None
        logits = outs[self._out_names[0]].reshape(h_in, w_in)
        return logits >= self._logit_thr

    def process(self, frame_bgr: np.ndarray) -> np.ndarray:
        mask = self.infer_mask(frame_bgr)
        if mask is None:
            return frame_bgr
        fh, fw = frame_bgr.shape[:2]
        mask_u8 = cv2.resize(mask.astype(np.uint8), (fw, fh),
                             interpolation=cv2.INTER_NEAREST)
        mask_full = mask_u8.astype(bool)
        # translucent tint over the terrain region ...
        overlay = frame_bgr.copy()
        overlay[mask_full] = self.colour
        cv2.addWeighted(overlay, self.alpha, frame_bgr, 1 - self.alpha, 0, frame_bgr)
        # ... plus a bright contour so the boundary reads as a HUD, not a wash.
        contours, _ = cv2.findContours(mask_u8, cv2.RETR_EXTERNAL,
                                       cv2.CHAIN_APPROX_SIMPLE)
        cv2.drawContours(frame_bgr, contours, -1, (0, 255, 120), 2)
        cv2.putText(frame_bgr, "TERRAIN", (8, fh - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 120), 2, cv2.LINE_AA)
        return frame_bgr


class MultiClassSegmenter(_Model):
    """Genuinely multi-class terrain segmentation (e.g. ``fabseg.hef``).

    Unlike ``Segmenter`` (one sigmoid logit channel -> boolean mask), the
    output here has one channel per class; the class at each pixel is
    ``argmax`` over those channels, not a threshold. ``num_classes`` must
    match the class-channel count the .hef was exported with (verified
    against ``config/novelty/models.yaml``'s ``class_map`` length by
    ``MultiClassSegmenterAdapter`` - see ``drone_stack/novelty/perception/
    adapters.py``).
    """

    def __init__(self, hef_path: str, name: str = "terrain_mc",
                 num_classes: int = 7) -> None:
        super().__init__(hef_path, name)
        self.num_classes = num_classes

    def infer_class_map(self, frame_bgr: np.ndarray) -> Optional[np.ndarray]:
        """Return an int8 (h, w) class-index map at MODEL resolution, or None."""
        if not self.ok:
            return None
        h_in, w_in, _ = self._in_hw
        resized = cv2.resize(frame_bgr, (w_in, h_in), interpolation=cv2.INTER_LINEAR)
        rgb = cv2.cvtColor(resized, cv2.COLOR_BGR2RGB)
        outs = self._run(rgb)
        if outs is None:
            return None
        logits = outs[self._out_names[0]]
        # HailoRT may report NHWC or NCHW for the output tensor depending on
        # how the .hef was compiled - pick whichever axis actually holds one
        # entry per class rather than assuming a fixed layout.
        if logits.ndim >= 1 and logits.shape[-1] == self.num_classes:
            class_map = np.argmax(logits, axis=-1)
        elif logits.ndim >= 1 and logits.shape[0] == self.num_classes:
            class_map = np.argmax(logits, axis=0)
        else:
            _log.warning(
                "MultiClassSegmenter(%s): output shape %s has no axis of "
                "size num_classes=%d - cannot argmax", self.name,
                logits.shape, self.num_classes)
            return None
        return class_map.reshape(h_in, w_in).astype(np.int8)
