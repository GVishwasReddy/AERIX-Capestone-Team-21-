"""FusionNode - Phase 4.

Fuses attitude, IMU, GPS, altitude and velocity into a single best-estimate
:class:`~drone_stack.msg.FusedState` in a local ENU frame.

The estimation is deliberately hidden behind the :class:`StateEstimator`
interface so a full EKF can be dropped in later without touching any publisher
or subscriber. The shipped :class:`ComplementaryEstimator` blends the autopilot
attitude with integrated gyro (a complementary filter) and dead-reckons/updates
position from GPS + velocity, populating covariance from GPS accuracy.
"""
from __future__ import annotations

import abc
import threading
import time
from dataclasses import dataclass

from drone_stack.bus import MessageBus
from drone_stack.bus.topics import Topics
from drone_stack.msg import (
    Altitude,
    Attitude,
    FusedState,
    GpsFix,
    Imu,
    LaserScan,
    Velocity,
)
from drone_stack.utils.config import Config
from drone_stack.utils.geometry import clamp, geodetic_to_enu, wrap_pi
from drone_stack.utils.node import NodeBase


@dataclass
class SensorSnapshot:
    """Latest reading from each sensor (any may be ``None``)."""

    attitude: Attitude | None = None
    imu: Imu | None = None
    gps: GpsFix | None = None
    altitude: Altitude | None = None
    velocity: Velocity | None = None
    scan: LaserScan | None = None


class StateEstimator(abc.ABC):
    """Interface every estimator (complementary filter today, EKF tomorrow) fulfils."""

    @abc.abstractmethod
    def update(self, dt: float, sensors: SensorSnapshot) -> FusedState:
        ...


class ComplementaryEstimator(StateEstimator):
    """Complementary-filter attitude + GPS/velocity position estimator."""

    def __init__(self, config: Config) -> None:
        section = config.section("fusion")
        self._alpha = float(section.get("attitude_alpha", 0.98))
        self._use_gps = bool(section.get("use_gps", True))
        self._home_lat: float | None = None
        self._home_lon: float | None = None
        self._home_alt: float | None = None
        self._roll = 0.0
        self._pitch = 0.0
        self._yaw = 0.0

    def set_home(self, lat: float, lon: float, alt: float) -> None:
        self._home_lat, self._home_lon, self._home_alt = lat, lon, alt

    def update(self, dt: float, sensors: SensorSnapshot) -> FusedState:
        self._update_attitude(dt, sensors)

        state = FusedState(
            roll=self._roll, pitch=self._pitch, yaw=self._yaw, sources=[]
        )

        if sensors.attitude is not None:
            state.sources.append("attitude")
        if sensors.imu is not None:
            state.sources.append("imu")

        # Establish home from the first valid GPS fix.
        gps = sensors.gps
        if self._use_gps and gps is not None and gps.has_fix:
            if self._home_lat is None:
                self.set_home(gps.lat, gps.lon, gps.alt_amsl_m)
            east, north = geodetic_to_enu(
                gps.lat, gps.lon, self._home_lat, self._home_lon
            )
            state.x, state.y = east, north
            state.lat, state.lon = gps.lat, gps.lon
            state.alt_amsl_m = gps.alt_amsl_m
            state.sources.append("gps")
            # Horizontal/vertical covariance straight from GPS accuracy.
            hcov = max(0.1, gps.eph) ** 2
            vcov = max(0.1, gps.epv) ** 2
            state.covariance[0] = hcov
            state.covariance[1] = hcov
            state.covariance[2] = vcov

        if sensors.altitude is not None:
            state.alt_rel_m = sensors.altitude.relative_m
            state.z = sensors.altitude.relative_m
            state.sources.append("altitude")

        if sensors.velocity is not None:
            # NED -> ENU
            state.vx = sensors.velocity.vy   # east
            state.vy = sensors.velocity.vx   # north
            state.vz = -sensors.velocity.vz  # up
            state.sources.append("velocity")

        state.valid = sensors.attitude is not None and (
            sensors.gps is not None or sensors.altitude is not None
        )
        return state

    def _update_attitude(self, dt: float, sensors: SensorSnapshot) -> None:
        # Prediction step: integrate gyro rates.
        if sensors.imu is not None and dt > 0:
            self._roll = wrap_pi(self._roll + sensors.imu.gx * dt)
            self._pitch = wrap_pi(self._pitch + sensors.imu.gy * dt)
            self._yaw = wrap_pi(self._yaw + sensors.imu.gz * dt)
        # Correction step: blend toward the autopilot's fused attitude.
        att = sensors.attitude
        if att is not None:
            a = clamp(self._alpha, 0.0, 1.0)
            self._roll = wrap_pi(self._complementary(self._roll, att.roll, a))
            self._pitch = wrap_pi(self._complementary(self._pitch, att.pitch, a))
            self._yaw = wrap_pi(self._complementary(self._yaw, att.yaw, a))

    @staticmethod
    def _complementary(predicted: float, measured: float, alpha: float) -> float:
        # Blend on the shortest angular path to avoid wrap discontinuities.
        delta = wrap_pi(measured - predicted)
        return predicted + (1.0 - alpha) * delta


class FusionNode(NodeBase):
    """Publishes a fused vehicle state estimate at a fixed rate."""

    def __init__(
        self,
        bus: MessageBus,
        config: Config,
        estimator: StateEstimator | None = None,
    ) -> None:
        section = config.section("fusion")
        super().__init__("fusion", bus, config, rate_hz=section.get("rate_hz", 30))
        self.estimator = estimator or ComplementaryEstimator(config)
        self._snapshot = SensorSnapshot()
        self._lock = threading.Lock()
        self._last_update = 0.0

        self.subscribe(Topics.ATTITUDE, self._make_setter("attitude"))
        self.subscribe(Topics.IMU, self._make_setter("imu"))
        self.subscribe(Topics.GPS, self._make_setter("gps"))
        self.subscribe(Topics.ALTITUDE, self._make_setter("altitude"))
        self.subscribe(Topics.VELOCITY, self._make_setter("velocity"))
        self.subscribe(Topics.SCAN, self._make_setter("scan"))

    def _make_setter(self, attr: str):
        def _setter(msg) -> None:
            with self._lock:
                setattr(self._snapshot, attr, msg)
        return _setter

    def step(self) -> None:
        now = time.monotonic()
        dt = (now - self._last_update) if self._last_update else 0.0
        self._last_update = now
        with self._lock:
            snapshot = SensorSnapshot(**vars(self._snapshot))
        state = self.estimator.update(dt, snapshot)
        self.publish(Topics.FUSED_STATE, state)
