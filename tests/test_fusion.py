"""Unit tests for sensor fusion (Phase 4)."""
from __future__ import annotations

import math

from drone_stack.msg import Altitude, Attitude, GpsFix, Velocity
from drone_stack.nodes.fusion_node import ComplementaryEstimator, SensorSnapshot
from drone_stack.utils.config import Config


def _estimator() -> ComplementaryEstimator:
    return ComplementaryEstimator(Config.load())


def test_fusion_requires_attitude_and_position():
    est = _estimator()
    empty = est.update(0.1, SensorSnapshot())
    assert not empty.valid

    snap = SensorSnapshot(
        attitude=Attitude(roll=0.0, pitch=0.0, yaw=0.1),
        altitude=Altitude(relative_m=5.0),
    )
    state = est.update(0.1, snap)
    assert state.valid
    assert math.isclose(state.z, 5.0, abs_tol=1e-6)


def test_ned_to_enu_velocity_mapping():
    est = _estimator()
    snap = SensorSnapshot(
        attitude=Attitude(),
        velocity=Velocity(vx=1.0, vy=2.0, vz=-0.5),  # N, E, Down
    )
    state = est.update(0.1, snap)
    assert math.isclose(state.vx, 2.0)   # east <- NED east
    assert math.isclose(state.vy, 1.0)   # north <- NED north
    assert math.isclose(state.vz, 0.5)   # up <- -down


def test_home_set_from_first_fix_and_covariance():
    est = _estimator()
    home = GpsFix(fix_type=3, lat=47.397742, lon=8.545594, alt_amsl_m=488.0, eph=0.8, epv=1.2)
    state = est.update(0.1, SensorSnapshot(attitude=Attitude(), gps=home))
    assert "gps" in state.sources
    assert math.isclose(state.x, 0.0, abs_tol=0.5)
    assert math.isclose(state.y, 0.0, abs_tol=0.5)
    assert state.covariance[0] > 0.0  # horizontal covariance populated


def test_attitude_tracks_measurement():
    est = _estimator()
    snap = SensorSnapshot(attitude=Attitude(roll=0.2, pitch=-0.1, yaw=1.0))
    # Several updates should converge the complementary filter toward the measurement.
    state = est.update(0.05, snap)
    for _ in range(500):
        state = est.update(0.05, snap)
    assert math.isclose(state.roll, 0.2, abs_tol=1e-2)
    assert math.isclose(state.yaw, 1.0, abs_tol=1e-2)
