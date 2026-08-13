"""§2.6 - model interface abstraction.

Every consumer of a vision model (landing_zone.py, recipient_auth.py,
motion_monitor.py) depends ONLY on the :class:`VisionModel` protocol and
:class:`~drone_stack.novelty.types.ModelOutput` defined here - never on
Hailo, hailort, or a specific architecture. New weights are a
``config/novelty/models.yaml`` edit, not a code change; if a new model ever
needs a code change, that is a bug in this abstraction, not in the caller
(see the project brief, §9).

``ModelRegistry.from_config`` is the only place that imports
``drone_stack.gcs.hailo_infer`` (the existing Detector/Segmenter wrappers -
reused, not reimplemented) - that import boundary lives in
``perception/adapters.py``.
"""
from __future__ import annotations

import hashlib
from datetime import datetime, timezone
from pathlib import Path
from typing import Protocol

import numpy as np

from drone_stack.novelty.config import ModelsConfig
from drone_stack.novelty.types import ModelOutput
from drone_stack.utils.logging_setup import get_logger

_log = get_logger("novelty.perception")


def _repo_root() -> Path:
    # drone_stack/novelty/perception/model_registry.py -> perception ->
    # novelty -> drone_stack -> <repo root>
    return Path(__file__).resolve().parents[3]


def _resolve_hef_path(hef_path: str) -> str:
    """``models.yaml`` paths are repo-root-relative (e.g. ``models/x.hef``)
    so the registry works the same regardless of the process's cwd - the
    existing ``gcs/cameras.py`` resolves its own hardcoded paths the same
    way via ``HAILO_MODELS_DIR``."""
    path = Path(hef_path)
    return str(path) if path.is_absolute() else str(_repo_root() / path)


class VisionModel(Protocol):
    def infer(self, frame: np.ndarray) -> ModelOutput: ...

    @property
    def input_shape(self) -> tuple[int, int, int]: ...

    @property
    def version(self) -> str: ...

    @property
    def ok(self) -> bool:
        """True once the model is loaded and ready. False (never raises) if
        HailoRT / the .hef is unavailable - matches the graceful-degradation
        contract already used by drone_stack/gcs/hailo_infer.py."""
        ...


def compute_model_version(hef_path: str | Path) -> str:
    """``<sha256[:12]>@<file-mtime-date>`` - stands in for "model hash +
    training date" (brief §2.6) until weights carry real training metadata.
    Returns "unavailable" if the file does not exist (sim / dev machine /
    weights not yet delivered)."""
    path = Path(hef_path)
    if not path.exists():
        return "unavailable"
    try:
        digest = hashlib.sha256(path.read_bytes()).hexdigest()[:12]
        mtime = datetime.fromtimestamp(path.stat().st_mtime, tz=timezone.utc).date().isoformat()
        return f"{digest}@{mtime}"
    except OSError as exc:  # noqa: BLE001
        _log.warning("compute_model_version(%s) failed: %s", path, exc)
        return "unavailable"


class ModelRegistry:
    """Holds every configured :class:`VisionModel`, keyed by the name in
    ``config/novelty/models.yaml``."""

    def __init__(self, models: dict[str, VisionModel]) -> None:
        self._models = dict(models)

    def get(self, name: str) -> VisionModel:
        try:
            return self._models[name]
        except KeyError:
            raise KeyError(
                f"no model named {name!r} in registry "
                f"(configured: {sorted(self._models)})"
            ) from None

    def names(self) -> list[str]:
        return sorted(self._models)

    def versions(self) -> dict[str, str]:
        """Every model's version string - logged once at flight start by
        EvidenceLogger.set_model_versions so every decision record is
        attributable to the exact weights that produced it."""
        return {name: model.version for name, model in self._models.items()}

    @classmethod
    def from_config(cls, config: ModelsConfig) -> "ModelRegistry":
        # Imported here (not at module scope) so a pure-algorithm test suite
        # that only needs the registry API never has to import hailo_infer.
        from drone_stack.novelty.perception.adapters import DetectorAdapter, SegmenterAdapter

        models: dict[str, VisionModel] = {}
        for name, spec in config.models.items():
            hef_path = _resolve_hef_path(spec.hef_path)
            version = compute_model_version(hef_path)
            if spec.kind == "detector":
                models[name] = DetectorAdapter(
                    hef_path=hef_path,
                    name=name,
                    input_shape=spec.input_shape,
                    score_thr=spec.score_thr or 0.25,
                    version=version,
                )
            elif spec.kind == "segmenter":
                assert spec.binary_fallback is not None  # enforced by ModelSpecConfig
                models[name] = SegmenterAdapter(
                    hef_path=hef_path,
                    name=name,
                    input_shape=spec.input_shape,
                    threshold=spec.seg_threshold or 0.5,
                    version=version,
                    safe_class=spec.binary_fallback.safe_class,
                    unsafe_class=spec.binary_fallback.unsafe_class,
                )
            else:  # pragma: no cover - pydantic Literal already rejects this
                raise ValueError(f"unknown model kind: {spec.kind!r}")
        return cls(models)
