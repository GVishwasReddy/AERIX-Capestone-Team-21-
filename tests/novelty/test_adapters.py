"""novelty/perception/adapters.py - VisionModel wiring, no Hailo hardware
required. Degrade-gracefully paths are exercised through the real Hailo
wrapper classes (no .hef on disk here); the class-map -> SegmentationFrame
wiring itself is exercised by REPLACING adapter._seg wholesale with a stub,
not by mutating attributes on whatever the real construction produced -
see _make_stub_seg's docstring for why that distinction matters."""
from __future__ import annotations

from types import SimpleNamespace

import numpy as np

from drone_stack.novelty.perception.adapters import (
    MultiClassSegmenterAdapter,
    SegmenterAdapter,
)
from drone_stack.novelty.types import TerrainClass

_FULL_CLASS_MAP = [
    TerrainClass.GRASS,
    TerrainClass.PAVEMENT,
    TerrainClass.DIRT,
    TerrainClass.WATER,
    TerrainClass.VEGETATION,
    TerrainClass.OBSTACLE,
    TerrainClass.UNKNOWN,
]


def _make_multiclass_adapter(class_map: list[TerrainClass]) -> MultiClassSegmenterAdapter:
    return MultiClassSegmenterAdapter(
        hef_path="models/does_not_exist.hef",
        name="terrain_mc_test",
        input_shape=(384, 640, 3),
        version="test@2026-08-14",
        class_map=class_map,
    )


def test_multiclass_adapter_degrades_gracefully_without_hailo():
    adapter = _make_multiclass_adapter(_FULL_CLASS_MAP)
    assert adapter.ok is False
    out = adapter.infer(np.zeros((10, 10, 3), dtype=np.uint8))
    assert out.kind == "terrain"
    assert out.segmentation is None


def _make_stub_seg(infer_class_map):
    """A stand-in for the wrapped MultiClassSegmenter, assigned wholesale
    onto adapter._seg rather than mutated in place. Real Hailo construction
    leaves _seg in one of TWO different shapes depending on how it failed:
    a real (non-None) object with ok=False if hailo_platform is not even
    importable (hailo_infer._Model.__init__ returns early - the common case
    on a machine with no Hailo hardware at all), or None if hailo_platform
    IS installed but the shared VDevice can't be acquired right now - e.g.
    HAILO_OUT_OF_PHYSICAL_DEVICES because another process already holds the
    one physical Hailo-8 (see perception/adapters.py's own module
    docstring) - a real condition on the Pi whenever aerix-gcs.service is
    running. Replacing the whole attribute sidesteps needing to know or
    care which of those two _seg already is."""
    return SimpleNamespace(ok=True, infer_class_map=infer_class_map)


def test_multiclass_adapter_builds_index_to_class_from_class_map_order():
    adapter = _make_multiclass_adapter(_FULL_CLASS_MAP)

    fake_class_map = np.array([[0, 3], [5, 6]], dtype=np.int8)
    adapter._seg = _make_stub_seg(lambda frame_bgr: fake_class_map)

    out = adapter.infer(np.zeros((2, 2, 3), dtype=np.uint8))

    assert out.kind == "terrain"
    assert out.segmentation is not None
    np.testing.assert_array_equal(out.segmentation.class_indices, fake_class_map)
    assert out.segmentation.index_to_class[0] == TerrainClass.GRASS
    assert out.segmentation.index_to_class[3] == TerrainClass.WATER
    assert out.segmentation.index_to_class[5] == TerrainClass.OBSTACLE
    assert out.segmentation.index_to_class[6] == TerrainClass.UNKNOWN
    assert len(out.segmentation.index_to_class) == len(_FULL_CLASS_MAP)


def test_multiclass_adapter_returns_none_segmentation_when_wrapped_model_returns_none():
    adapter = _make_multiclass_adapter(_FULL_CLASS_MAP)
    adapter._seg = _make_stub_seg(lambda frame_bgr: None)

    out = adapter.infer(np.zeros((2, 2, 3), dtype=np.uint8))
    assert out.kind == "terrain"
    assert out.segmentation is None


def test_binary_adapter_still_degrades_gracefully_without_hailo():
    """Regression guard: adding the multi-class adapter must not disturb the
    existing binary SegmenterAdapter path."""
    adapter = SegmenterAdapter(
        hef_path="models/does_not_exist.hef",
        name="terrain_test",
        input_shape=(384, 640, 1),
        threshold=0.5,
        version="test@2026-08-14",
        safe_class=TerrainClass.GRASS,
        unsafe_class=TerrainClass.UNKNOWN,
    )
    assert adapter.ok is False
    out = adapter.infer(np.zeros((10, 10, 3), dtype=np.uint8))
    assert out.kind == "terrain"
    assert out.segmentation is None
