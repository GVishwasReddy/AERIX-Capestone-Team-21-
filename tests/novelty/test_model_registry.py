"""novelty/perception/model_registry.py - registry loads from YAML with no
real Hailo hardware required (adapters.py degrades to ok=False)."""
from __future__ import annotations

from drone_stack.novelty.config import NoveltyConfig
from drone_stack.novelty.perception.model_registry import (
    ModelRegistry,
    compute_model_version,
)


def test_registry_builds_from_config_without_hardware(valid_config_dir):
    cfg = NoveltyConfig.load(valid_config_dir)
    registry = ModelRegistry.from_config(cfg.models)

    assert registry.names() == ["terrain", "yolov8n"]


def test_registry_models_degrade_gracefully_without_hailo(missing_hef_config_dir):
    """With no weights on disk, both adapters report ok=False and never raise.

    Uses ``missing_hef_config_dir``, not ``valid_config_dir``: the latter's
    repo-relative hef paths are real files on the Pi, so this asserted the
    machine rather than the contract.
    """
    cfg = NoveltyConfig.load(missing_hef_config_dir)
    registry = ModelRegistry.from_config(cfg.models)

    for name in registry.names():
        model = registry.get(name)
        assert model.ok is False


def test_registry_versions_have_expected_shape(valid_config_dir):
    """The fixture's hef paths (models/terrain.hef, models/yolov8n.hef) are
    repo-root-relative, so this resolves to a REAL file wherever the repo
    actually ships the .hef weights (e.g. on the Pi) and to a missing one
    elsewhere (e.g. a Mac dev checkout with no models/) - either is valid,
    both are covered by compute_model_version's own contract (tested below)."""
    cfg = NoveltyConfig.load(valid_config_dir)
    registry = ModelRegistry.from_config(cfg.models)

    versions = registry.versions()
    assert set(versions) == {"terrain", "yolov8n"}
    for v in versions.values():
        assert v == "unavailable" or "@" in v


def test_registry_get_unknown_name_raises_key_error(valid_config_dir):
    cfg = NoveltyConfig.load(valid_config_dir)
    registry = ModelRegistry.from_config(cfg.models)

    try:
        registry.get("not_a_model")
    except KeyError as exc:
        assert "not_a_model" in str(exc)
    else:
        raise AssertionError("expected KeyError")


def test_registry_infer_on_degraded_model_returns_empty_output(missing_hef_config_dir):
    """A degraded model still answers - with an empty result, not an exception.

    This is the property the whole camera tap leans on: inference is called on
    every kept frame, and a missing or busy Hailo must cost an empty output,
    never a raise into the frame loop.
    """
    import numpy as np

    cfg = NoveltyConfig.load(missing_hef_config_dir)
    registry = ModelRegistry.from_config(cfg.models)

    detector_out = registry.get("yolov8n").infer(np.zeros((640, 640, 3), dtype=np.uint8))
    assert detector_out.kind == "detections"
    assert detector_out.detections == []

    seg_out = registry.get("terrain").infer(np.zeros((384, 640, 3), dtype=np.uint8))
    assert seg_out.kind == "terrain"
    assert seg_out.segmentation is None


def test_compute_model_version_missing_file_is_unavailable(tmp_path):
    assert compute_model_version(tmp_path / "does_not_exist.hef") == "unavailable"


def test_compute_model_version_hashes_real_file(tmp_path):
    hef = tmp_path / "fake.hef"
    hef.write_bytes(b"not a real hef but has bytes")
    version = compute_model_version(hef)
    assert version != "unavailable"
    assert "@" in version
    # Same bytes -> same hash, deterministic.
    assert compute_model_version(hef) == version


def test_registry_builds_multiclass_adapter_from_class_map_spec(tmp_path):
    """A models.yaml entry with class_map (e.g. fabseg.hef) must select
    MultiClassSegmenterAdapter, not the legacy binary SegmenterAdapter -
    and still degrade gracefully with no real Hailo hardware present."""
    import copy

    import numpy as np

    from drone_stack.novelty.perception.adapters import MultiClassSegmenterAdapter
    from tests.novelty.conftest import MODELS, write_config_dir

    models = copy.deepcopy(MODELS)
    del models["models"]["terrain"]["binary_fallback"]
    del models["models"]["terrain"]["seg_threshold"]
    models["models"]["terrain"]["class_map"] = [
        "grass", "pavement", "dirt", "water", "vegetation", "obstacle", "unknown",
    ]
    # Point at weights that cannot exist, so the degradation half of this test
    # holds on the Pi too (which ships real models/*.hef). See
    # conftest.missing_hef_config_dir.
    for name, spec in models["models"].items():
        spec["hef_path"] = str(tmp_path / "no_such_dir" / f"{name}.hef")
    write_config_dir(tmp_path, overrides={"models.yaml": models})

    cfg = NoveltyConfig.load(tmp_path)
    registry = ModelRegistry.from_config(cfg.models)

    terrain = registry.get("terrain")
    assert isinstance(terrain, MultiClassSegmenterAdapter)
    assert terrain.ok is False  # no weights on disk, on any machine

    out = terrain.infer(np.zeros((384, 640, 3), dtype=np.uint8))
    assert out.kind == "terrain"
    assert out.segmentation is None
