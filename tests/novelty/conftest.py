"""Shared fixtures for the novelty test suite.

``valid_config_dir`` writes a minimal-but-complete set of the five
config/novelty/*.yaml files to a tmp_path, independent of whatever
thresholds are checked into the repo - tests assert on STRUCTURE/behaviour,
never on the specific (GUESSED, flight-tunable) numbers shipped in the real
YAML files.
"""
from __future__ import annotations

from pathlib import Path

import pytest
import yaml

from drone_stack.novelty.mission_fsm import TIMED_STATES

LANDING_ZONE = {
    "weights": {"w1_surface": 0.4, "w2_slope": 0.25, "w3_clutter": 0.2, "w4_area": 0.15},
    "surface_suitability": {
        "grass": 0.95, "dirt": 0.75, "pavement": 0.55,
        "vegetation": 0.3, "water": 0.0, "obstacle": 0.0, "unknown": 0.1,
    },
    "r_min_m": 2.0,
    "r_max_m": 12.0,
    "min_score": 0.35,
    "max_slope_deg": 12.0,
    "min_safe_radius_m": 1.5,
    "top_k": 3,
    "cell_size_m": 0.5,
    "clutter_norm": 4.0,
}

RECIPIENT_AUTH = {
    "sync_window_ms": 2000,
    "max_position_disagreement_m": 4.0,
    "max_position_disagreement_m_rssi_only": 10.0,
    "vision_retry_timeout_s": 15.0,
    "ble_retry_timeout_s": 15.0,
    "disagreement_retry_timeout_s": 10.0,
    "disambiguation_margin": 0.15,
    "disambiguation_weights": {
        "alpha_ble_position": 0.6, "beta_rssi_consistency": 0.4, "gamma_motion_cue": 0.0,
    },
    "rssi": {"tx_power_dbm": -59.0, "path_loss_exponent": 2.7},
}

MISSION_FSM = {
    "state_timeout_s": {s.value: 10.0 for s in TIMED_STATES},
    "search_radius_expansion_m": 5.0,
    "max_search_radius_m": 30.0,
    "max_hover_retries": 1,
}

MOTION_MONITOR = {
    "max_recipient_velocity_mps": 1.2,
    "abort_altitude_ceiling_m": 8.0,
    "zone_intrusion_radius_m": 3.0,
    "track_history_len": 5,
    "min_track_frames": 3,
}

MODELS = {
    "camera": {"fx": 950.0, "fy": 950.0, "cx": 640.0, "cy": 360.0, "tilt_from_nadir_deg": 0.0},
    "models": {
        "terrain": {
            "kind": "segmenter",
            "hef_path": "models/terrain.hef",
            "input_shape": [384, 640, 1],
            "seg_threshold": 0.5,
            "binary_fallback": {"safe_class": "grass", "unsafe_class": "unknown"},
        },
        "yolov8n": {
            "kind": "detector",
            "hef_path": "models/yolov8n.hef",
            "input_shape": [640, 640, 3],
            "score_thr": 0.25,
        },
    },
}

ALL_CONFIGS = {
    "landing_zone.yaml": LANDING_ZONE,
    "recipient_auth.yaml": RECIPIENT_AUTH,
    "mission_fsm.yaml": MISSION_FSM,
    "motion_monitor.yaml": MOTION_MONITOR,
    "models.yaml": MODELS,
}


def write_config_dir(base: Path, overrides: dict | None = None) -> Path:
    """Write a complete, valid config/novelty/ directory to *base*.

    ``overrides`` is ``{filename: replacement_dict}`` for the files that
    should differ from the defaults above (the rest are written unchanged).
    """
    overrides = overrides or {}
    for filename, default_data in ALL_CONFIGS.items():
        data = overrides.get(filename, default_data)
        (base / filename).write_text(yaml.safe_dump(data), encoding="utf-8")
    return base


@pytest.fixture
def valid_config_dir(tmp_path: Path) -> Path:
    return write_config_dir(tmp_path)
