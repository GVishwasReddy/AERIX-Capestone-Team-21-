"""Unit tests for messages, config loading and geometry helpers."""
from __future__ import annotations

import math
import os

from drone_stack.msg import (
    DiagLevel,
    Obstacle,
    ObstacleArray,
    ObstacleClass,
    to_dict,
)
from drone_stack.utils import geometry as geo
from drone_stack.utils.config import Config, _deep_merge


# --- messages --------------------------------------------------------------
def test_to_dict_serialises_enums_and_nested():
    arr = ObstacleArray(
        obstacles=[Obstacle(id=1, classification=ObstacleClass.PERSON, distance_m=2.0)]
    )
    d = to_dict(arr)
    assert d["obstacles"][0]["classification"] == "person"
    assert d["obstacles"][0]["distance_m"] == 2.0
    assert isinstance(d["stamp"], float)


def test_enum_is_json_string():
    assert DiagLevel.ERROR == "ERROR"
    assert to_dict(DiagLevel.OK) == "OK"


def test_obstacle_array_nearest():
    arr = ObstacleArray(
        obstacles=[Obstacle(distance_m=5.0), Obstacle(distance_m=2.0)]
    )
    assert arr.nearest().distance_m == 2.0


# --- config ----------------------------------------------------------------
def test_deep_merge_overrides_nested():
    base = {"a": {"x": 1, "y": 2}, "b": 3}
    override = {"a": {"y": 9}}
    merged = _deep_merge(base, override)
    assert merged == {"a": {"x": 1, "y": 9}, "b": 3}
    assert base["a"]["y"] == 2  # original untouched


def test_config_defaults_are_sim():
    cfg = Config.load()
    assert cfg.mode == "sim"
    assert cfg.get("mavlink.connection")
    assert cfg.section("lidar")["baud"] == 460800


def test_env_override(monkeypatch):
    monkeypatch.setenv("DRONE_WEB_PORT", "9123")
    monkeypatch.setenv("DRONE_MAVLINK_CONNECTION", "/dev/ttyACM9")
    cfg = Config.load()
    assert cfg.get("web.port") == 9123
    assert cfg.get("mavlink.connection") == "/dev/ttyACM9"


# --- geometry --------------------------------------------------------------
def test_wrap_pi():
    # 3*pi and -3*pi both wrap to +/-pi (equivalent); check magnitude and range.
    assert math.isclose(abs(geo.wrap_pi(3 * math.pi)), math.pi, abs_tol=1e-9)
    assert math.isclose(abs(geo.wrap_pi(-3 * math.pi)), math.pi, abs_tol=1e-9)
    assert -math.pi <= geo.wrap_pi(10.0) <= math.pi
    assert math.isclose(geo.wrap_pi(0.5), 0.5, abs_tol=1e-9)


def test_enu_geodetic_round_trip():
    ref_lat, ref_lon = 47.397742, 8.545594
    east, north = geo.geodetic_to_enu(47.398, 8.546, ref_lat, ref_lon)
    lat, lon = geo.enu_to_geodetic(east, north, ref_lat, ref_lon)
    assert math.isclose(lat, 47.398, abs_tol=1e-6)
    assert math.isclose(lon, 8.546, abs_tol=1e-6)


def test_haversine_known_distance():
    # one degree of latitude is ~111 km
    d = geo.haversine_m(0.0, 0.0, 1.0, 0.0)
    assert 110_000 < d < 112_000
