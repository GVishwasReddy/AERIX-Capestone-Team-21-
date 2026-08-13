"""novelty/config.py - fail-loud validation of the five config/novelty/*.yaml files."""
from __future__ import annotations

import copy

import pytest

from drone_stack.novelty.config import NoveltyConfig, NoveltyConfigError
from drone_stack.novelty.mission_fsm import TIMED_STATES
from tests.novelty.conftest import (
    LANDING_ZONE,
    MISSION_FSM,
    write_config_dir,
)


def test_valid_config_loads(valid_config_dir):
    cfg = NoveltyConfig.load(valid_config_dir)
    assert cfg.landing_zone.weights.w1_surface == 0.4
    assert cfg.recipient_auth.sync_window_ms == 2000
    assert cfg.motion_monitor.max_recipient_velocity_mps == 1.2
    assert cfg.models.camera.fx == 950.0
    assert set(cfg.mission_fsm.state_timeout_s) == {s.value for s in TIMED_STATES}


def test_repo_shipped_configs_load():
    """The actual config/novelty/*.yaml checked into the repo must itself be
    valid - this is the config every real flight loads."""
    cfg = NoveltyConfig.load()  # default_novelty_config_dir()
    assert cfg.landing_zone.top_k >= 1


def test_missing_file_fails_loud(tmp_path):
    # Write only 4 of 5 files.
    write_config_dir(tmp_path)
    (tmp_path / "models.yaml").unlink()
    with pytest.raises(NoveltyConfigError, match="models.yaml"):
        NoveltyConfig.load(tmp_path)


def test_missing_required_key_fails_loud(tmp_path):
    landing_zone = copy.deepcopy(LANDING_ZONE)
    del landing_zone["min_score"]  # required, no default
    write_config_dir(tmp_path, overrides={"landing_zone.yaml": landing_zone})
    with pytest.raises(NoveltyConfigError, match="landing_zone.yaml"):
        NoveltyConfig.load(tmp_path)


def test_unknown_key_fails_loud(tmp_path):
    """extra='forbid' - a typo'd key must not be silently ignored."""
    landing_zone = copy.deepcopy(LANDING_ZONE)
    landing_zone["mispelled_threshold"] = 1.0
    write_config_dir(tmp_path, overrides={"landing_zone.yaml": landing_zone})
    with pytest.raises(NoveltyConfigError, match="landing_zone.yaml"):
        NoveltyConfig.load(tmp_path)


def test_surface_suitability_missing_class_fails_loud(tmp_path):
    landing_zone = copy.deepcopy(LANDING_ZONE)
    del landing_zone["surface_suitability"]["water"]
    write_config_dir(tmp_path, overrides={"landing_zone.yaml": landing_zone})
    with pytest.raises(NoveltyConfigError, match="water"):
        NoveltyConfig.load(tmp_path)


def test_mission_fsm_missing_state_timeout_fails_loud(tmp_path):
    mission_fsm = copy.deepcopy(MISSION_FSM)
    # Drop one required state.
    some_state = next(iter(mission_fsm["state_timeout_s"]))
    del mission_fsm["state_timeout_s"][some_state]
    write_config_dir(tmp_path, overrides={"mission_fsm.yaml": mission_fsm})
    with pytest.raises(NoveltyConfigError, match=some_state):
        NoveltyConfig.load(tmp_path)


def test_mission_fsm_unknown_state_fails_loud(tmp_path):
    mission_fsm = copy.deepcopy(MISSION_FSM)
    mission_fsm["state_timeout_s"]["NOT_A_REAL_STATE"] = 5.0
    write_config_dir(tmp_path, overrides={"mission_fsm.yaml": mission_fsm})
    with pytest.raises(NoveltyConfigError, match="NOT_A_REAL_STATE"):
        NoveltyConfig.load(tmp_path)


def test_rssi_only_threshold_must_be_wider(tmp_path):
    from tests.novelty.conftest import RECIPIENT_AUTH

    recipient_auth = copy.deepcopy(RECIPIENT_AUTH)
    recipient_auth["max_position_disagreement_m_rssi_only"] = 1.0  # narrower - invalid
    write_config_dir(tmp_path, overrides={"recipient_auth.yaml": recipient_auth})
    with pytest.raises(NoveltyConfigError, match="recipient_auth.yaml"):
        NoveltyConfig.load(tmp_path)


def test_segmenter_without_binary_fallback_fails_loud(tmp_path):
    from tests.novelty.conftest import MODELS

    models = copy.deepcopy(MODELS)
    del models["models"]["terrain"]["binary_fallback"]
    write_config_dir(tmp_path, overrides={"models.yaml": models})
    with pytest.raises(NoveltyConfigError, match="models.yaml"):
        NoveltyConfig.load(tmp_path)
