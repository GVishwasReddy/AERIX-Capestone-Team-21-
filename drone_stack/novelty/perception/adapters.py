"""The ONLY module in the novelty layer that imports
``drone_stack.gcs.hailo_infer`` - every algorithmic module (landing_zone,
recipient_auth, motion_monitor) depends on :class:`~drone_stack.novelty.
perception.model_registry.VisionModel` instead, never on Hailo directly.

Wraps the existing, already-working ``Detector``/``Segmenter`` wrappers
(reused as-is, not reimplemented - see ``docs/novelty/landing_zone.md``)
behind the ``VisionModel`` protocol. Both existing classes degrade
gracefully when HailoRT / a .hef is unavailable OR fails to *load* (they
catch their own configure/activate errors internally and set ``ok=False``) -
but ``hailo_infer._get_vdevice()`` itself is called OUTSIDE that try/except
(hailo_infer.py L86), so if the shared VDevice can't be *created* at all -
e.g. ``HAILO_OUT_OF_PHYSICAL_DEVICES`` because ``aerix-gcs.service`` already
holds the one physical Hailo-8 - the exception propagates instead of
degrading. That happens during ``Supervisor.add()`` (bringup.py), which is
NOT inside any NodeBase resilience loop, so an uncaught exception here would
crash the whole stack's boot, not just the novelty layer. Both adapters
below wrap construction in a broad try/except for exactly this reason, so a
busy/absent Hailo device degrades to ``ok=False`` like every other failure
mode already documented in hailo_infer.py's own docstring.
"""
from __future__ import annotations

import numpy as np

from drone_stack.novelty.types import ModelOutput, PersonDetection, PixelBox, SegmentationFrame, TerrainClass
from drone_stack.utils.logging_setup import get_logger

_log = get_logger("novelty.perception.adapters")


class DetectorAdapter:
    """Wraps ``drone_stack.gcs.hailo_infer.Detector`` (yolov8n person
    detector) behind :class:`VisionModel`."""

    def __init__(
        self,
        hef_path: str,
        name: str,
        input_shape: tuple[int, int, int],
        score_thr: float,
        version: str,
    ) -> None:
        from drone_stack.gcs.hailo_infer import Detector

        self._input_shape = input_shape
        self._version = version
        self._det = None
        try:
            self._det = Detector(hef_path, name=name, score_thr=score_thr)
        except Exception as exc:  # noqa: BLE001 - see module docstring
            _log.warning("DetectorAdapter(%s): Hailo unavailable (%s) - ok=False", name, exc)

    @property
    def input_shape(self) -> tuple[int, int, int]:
        return self._input_shape

    @property
    def version(self) -> str:
        return self._version

    @property
    def ok(self) -> bool:
        return self._det is not None and bool(self._det.ok)

    def infer(self, frame_bgr: np.ndarray) -> ModelOutput:
        if self._det is None:
            return ModelOutput(kind="detections", detections=[], model_version=self._version)
        raw = self._det.infer(frame_bgr)  # list[(x1,y1,x2,y2,score)]
        detections = [
            PersonDetection(bbox=PixelBox(x1, y1, x2, y2), score=float(score))
            for (x1, y1, x2, y2, score) in raw
        ]
        return ModelOutput(kind="detections", detections=detections, model_version=self._version)


class SegmenterAdapter:
    """Wraps ``drone_stack.gcs.hailo_infer.Segmenter`` (today: single-channel
    binary terrain segmentation) behind :class:`VisionModel`.

    The wrapped model is binary (walkable / not-walkable); ``safe_class`` /
    ``unsafe_class`` (from ``config/novelty/models.yaml``'s
    ``binary_fallback``) name which two :class:`TerrainClass` values that
    maps onto, so landing_zone.py always sees the same 7-class vocabulary
    regardless of whether the underlying model is binary or multi-class. A
    genuinely multi-class .hef (e.g. ``fabseg.hef``) uses
    :class:`MultiClassSegmenterAdapter` instead (selected by
    ``model_registry.py`` from whether the spec sets ``class_map`` or
    ``binary_fallback``) - landing_zone.py never has to know which one ran.
    """

    def __init__(
        self,
        hef_path: str,
        name: str,
        input_shape: tuple[int, int, int],
        threshold: float,
        version: str,
        safe_class: TerrainClass,
        unsafe_class: TerrainClass,
    ) -> None:
        from drone_stack.gcs.hailo_infer import Segmenter

        self._input_shape = input_shape
        self._version = version
        self._safe_class = safe_class
        self._unsafe_class = unsafe_class
        self._seg = None
        try:
            self._seg = Segmenter(hef_path, name=name, thr=threshold)
        except Exception as exc:  # noqa: BLE001 - see module docstring
            _log.warning("SegmenterAdapter(%s): Hailo unavailable (%s) - ok=False", name, exc)

    @property
    def input_shape(self) -> tuple[int, int, int]:
        return self._input_shape

    @property
    def version(self) -> str:
        return self._version

    @property
    def ok(self) -> bool:
        return self._seg is not None and bool(self._seg.ok)

    def infer(self, frame_bgr: np.ndarray) -> ModelOutput:
        if self._seg is None:
            return ModelOutput(kind="terrain", segmentation=None, model_version=self._version)
        mask = self._seg.infer_mask(frame_bgr)  # bool (h, w) at MODEL resolution, or None
        if mask is None:
            return ModelOutput(kind="terrain", segmentation=None, model_version=self._version)
        class_indices = mask.astype(np.int8)  # True(1) -> safe_class, False(0) -> unsafe_class
        frame = SegmentationFrame(
            class_indices=class_indices,
            index_to_class={0: self._unsafe_class, 1: self._safe_class},
        )
        return ModelOutput(kind="terrain", segmentation=frame, model_version=self._version)


class MultiClassSegmenterAdapter:
    """Wraps ``drone_stack.gcs.hailo_infer.MultiClassSegmenter`` (a genuine
    N-class terrain model, e.g. ``fabseg.hef``) behind :class:`VisionModel`.

    ``class_map`` (from ``config/novelty/models.yaml``'s ``models.<name>.
    class_map``) is the .hef's own output-channel order, index-for-index -
    channel ``i`` is ``class_map[i]``. This adapter's only job is turning a
    raw per-pixel argmax into the same ``SegmentationFrame`` /
    ``TerrainClass`` vocabulary :class:`SegmenterAdapter` already produces,
    so ``landing_zone.py`` never has to know which adapter produced the
    frame it's scoring.
    """

    def __init__(
        self,
        hef_path: str,
        name: str,
        input_shape: tuple[int, int, int],
        version: str,
        class_map: list[TerrainClass],
    ) -> None:
        from drone_stack.gcs.hailo_infer import MultiClassSegmenter

        self._input_shape = input_shape
        self._version = version
        self._index_to_class = {i: cls for i, cls in enumerate(class_map)}
        self._seg = None
        try:
            self._seg = MultiClassSegmenter(hef_path, name=name, num_classes=len(class_map))
        except Exception as exc:  # noqa: BLE001 - see module docstring
            _log.warning("MultiClassSegmenterAdapter(%s): Hailo unavailable (%s) - ok=False", name, exc)

    @property
    def input_shape(self) -> tuple[int, int, int]:
        return self._input_shape

    @property
    def version(self) -> str:
        return self._version

    @property
    def ok(self) -> bool:
        return self._seg is not None and bool(self._seg.ok)

    def infer(self, frame_bgr: np.ndarray) -> ModelOutput:
        if self._seg is None:
            return ModelOutput(kind="terrain", segmentation=None, model_version=self._version)
        class_map = self._seg.infer_class_map(frame_bgr)  # int8 (h, w) at MODEL resolution, or None
        if class_map is None:
            return ModelOutput(kind="terrain", segmentation=None, model_version=self._version)
        frame = SegmentationFrame(
            class_indices=class_map,
            index_to_class=dict(self._index_to_class),
        )
        return ModelOutput(kind="terrain", segmentation=frame, model_version=self._version)
