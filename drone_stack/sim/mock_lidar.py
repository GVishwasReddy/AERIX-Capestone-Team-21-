"""MockLidar - a simulated RPLIDAR C1 (Phase 7).

Ray-casts the shared :class:`~drone_stack.sim.world.SimWorld` once per beam to
produce a :class:`~drone_stack.msg.LaserScan` in exactly the same layout the real
driver emits (360 beams, angle 0 = forward), so downstream nodes cannot tell the
difference between simulation and hardware.
"""
from __future__ import annotations

import math
import random
from typing import Any

from drone_stack.interfaces.lidar_interface import (
    DEFAULT_BINS,
    LidarInterface,
    build_empty_ranges,
)
from drone_stack.msg import LaserScan
from drone_stack.sim.world import SimWorld
from drone_stack.utils.logging_setup import get_logger


class MockLidar(LidarInterface):
    """Simulated 2-D LiDAR backed by a shared SimWorld."""

    def __init__(self, world: SimWorld, config: dict[str, Any]) -> None:
        self.log = get_logger("sim.lidar")
        self._world = world
        self._frame_id = config.get("frame_id", "lidar_link")
        self._min_range = float(config.get("min_range_m", 0.15))
        self._max_range = float(config.get("max_range_m", 12.0))
        self._offset = math.radians(float(config.get("angle_offset_deg", 0.0)))
        self._invert = bool(config.get("invert", False))
        self._noise_m = 0.02
        self._bins = DEFAULT_BINS
        self._connected = False

    @property
    def connected(self) -> bool:
        return self._connected

    def connect(self) -> bool:
        self._connected = True
        self.log.info("mock RPLIDAR connected (simulation)")
        return True

    def close(self) -> None:
        self._connected = False

    def read_scan(self) -> LaserScan | None:
        if not self._connected:
            return None
        inc = 2.0 * math.pi / self._bins
        ranges = build_empty_ranges(self._bins)
        intensities = [0.0] * self._bins
        yaw = self._world.state.yaw
        for i in range(self._bins):
            sensor_angle = i * inc + self._offset
            if self._invert:
                sensor_angle = -sensor_angle
            world_angle = yaw + sensor_angle
            distance = self._world.raycast(world_angle, self._max_range)
            if math.isfinite(distance):
                distance += random.gauss(0.0, self._noise_m)
                if self._min_range <= distance <= self._max_range:
                    ranges[i] = round(distance, 3)
                    intensities[i] = 47.0
        return LaserScan(
            frame_id=self._frame_id,
            angle_min=0.0,
            angle_max=2.0 * math.pi,
            angle_increment=inc,
            range_min=self._min_range,
            range_max=self._max_range,
            ranges=ranges,
            intensities=intensities,
        )
