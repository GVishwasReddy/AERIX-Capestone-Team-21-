"""ObstacleNode - Phase 5.

Turns a raw :class:`~drone_stack.msg.LaserScan` into a list of classified
:class:`~drone_stack.msg.Obstacle` objects with distance and relative bearing.

Pipeline:
    1. Convert beams to Cartesian points (sensor/body frame, x forward, y left).
    2. Cluster angularly-adjacent returns whose range gap is small (handling the
       0/360 wrap-around).
    3. For each cluster estimate width (chord), radial depth and linearity.
    4. Classify (wall / building / vehicle / pole / person / tree) from those
       geometric features against configurable thresholds.

Bearing convention: 0 deg = straight ahead, positive to the right, range
[-180, 180].
"""
from __future__ import annotations

import math
import threading

from drone_stack.bus import MessageBus
from drone_stack.bus.topics import Topics
from drone_stack.msg import (
    LaserScan,
    Obstacle,
    ObstacleArray,
    ObstacleClass,
)
from drone_stack.utils.config import Config
from drone_stack.utils.geometry import polar_to_cartesian, wrap_180
from drone_stack.utils.node import NodeBase


class _Cluster:
    __slots__ = ("indices", "points", "ranges", "angles")

    def __init__(self) -> None:
        self.indices: list[int] = []
        self.points: list[tuple[float, float]] = []
        self.ranges: list[float] = []
        self.angles: list[float] = []

    def add(self, idx: int, rng: float, angle: float) -> None:
        self.indices.append(idx)
        self.ranges.append(rng)
        self.angles.append(angle)
        self.points.append(polar_to_cartesian(rng, angle))

    def __len__(self) -> int:
        return len(self.indices)


class ObstacleNode(NodeBase):
    """Detects and classifies obstacles from LaserScans."""

    def __init__(self, bus: MessageBus, config: Config) -> None:
        section = config.section("obstacles")
        super().__init__("obstacles", bus, config, rate_hz=section.get("rate_hz", 10))
        self._gap = float(section.get("cluster_gap_m", 0.30))
        self._min_points = int(section.get("min_points", 3))
        self._danger = float(section.get("danger_distance_m", 2.0))
        self._person_range = tuple(section.get("person_width_range_m", [0.25, 0.80]))
        self._pole_max = float(section.get("pole_width_max_m", 0.35))
        self._vehicle_range = tuple(section.get("vehicle_width_range_m", [1.20, 4.00]))
        self._wall_min = float(section.get("wall_min_length_m", 2.50))

        self._latest_scan: LaserScan | None = None
        self._lock = threading.Lock()
        self.subscribe(Topics.SCAN, self._on_scan)

    def _on_scan(self, msg) -> None:
        if isinstance(msg, LaserScan):
            with self._lock:
                self._latest_scan = msg

    def step(self) -> None:
        with self._lock:
            scan = self._latest_scan
        if scan is None or scan.count == 0:
            return
        obstacles = self._detect(scan)
        self.publish(
            Topics.OBSTACLES,
            ObstacleArray(frame_id="base_link", obstacles=obstacles),
        )

    # -- detection -----------------------------------------------------------
    def _detect(self, scan: LaserScan) -> list[Obstacle]:
        clusters = self._cluster(scan)
        obstacles: list[Obstacle] = []
        for i, cluster in enumerate(clusters):
            if len(cluster) < self._min_points:
                continue
            obstacles.append(self._describe(i, cluster))
        obstacles.sort(key=lambda o: o.distance_m)
        return obstacles

    def _cluster(self, scan: LaserScan) -> list[_Cluster]:
        clusters: list[_Cluster] = []
        current = _Cluster()
        prev_range: float | None = None
        angle = scan.angle_min
        inc = scan.angle_increment
        for idx, rng in enumerate(scan.ranges):
            a = angle
            angle += inc
            if not math.isfinite(rng):
                if len(current):
                    clusters.append(current)
                    current = _Cluster()
                prev_range = None
                continue
            if prev_range is not None and abs(rng - prev_range) > self._gap:
                if len(current):
                    clusters.append(current)
                current = _Cluster()
            current.add(idx, rng, a)
            prev_range = rng
        if len(current):
            clusters.append(current)

        # Merge wrap-around: first and last clusters are physically adjacent.
        if len(clusters) >= 2:
            first, last = clusters[0], clusters[-1]
            if (
                first.indices
                and last.indices
                and last.indices[-1] == len(scan.ranges) - 1
                and first.indices[0] == 0
                and abs(first.ranges[0] - last.ranges[-1]) <= self._gap
            ):
                merged = _Cluster()
                for c in (last, first):
                    for idx, rng, ang in zip(c.indices, c.ranges, c.angles):
                        merged.add(idx, rng, ang)
                clusters = [merged] + clusters[1:-1]
        return clusters

    def _describe(self, obstacle_id: int, cluster: _Cluster) -> Obstacle:
        nearest_i = min(range(len(cluster)), key=lambda i: cluster.ranges[i])
        distance = cluster.ranges[nearest_i]
        nx, ny = cluster.points[nearest_i]

        # Chord width between the two extreme points of the cluster.
        (x0, y0), (x1, y1) = cluster.points[0], cluster.points[-1]
        width = math.hypot(x1 - x0, y1 - y0)
        radial_depth = max(cluster.ranges) - min(cluster.ranges)
        angular_width = math.degrees(abs(cluster.angles[-1] - cluster.angles[0]))

        # Circular mean bearing of the cluster (handles the 359/0 wrap so an
        # obstacle straight ahead is not reported off to the side).
        sin_sum = sum(math.sin(a) for a in cluster.angles)
        cos_sum = sum(math.cos(a) for a in cluster.angles)
        mean_angle = math.atan2(sin_sum, cos_sum)
        bearing = wrap_180(-math.degrees(mean_angle))

        classification, confidence = self._classify(width, radial_depth, len(cluster))
        return Obstacle(
            id=obstacle_id,
            distance_m=round(distance, 3),
            bearing_deg=round(bearing, 2),
            angular_width_deg=round(angular_width, 2),
            width_m=round(width, 3),
            x_m=round(nx, 3),
            y_m=round(ny, 3),
            classification=classification,
            confidence=round(confidence, 2),
            danger=distance <= self._danger,
            num_points=len(cluster),
        )

    def _classify(
        self, width: float, radial_depth: float, points: int
    ) -> tuple[ObstacleClass, float]:
        """Heuristic classification from geometric features."""
        linear = radial_depth < 0.5  # a flat, wall-like surface

        if width >= 2.0 * self._wall_min and linear:
            return ObstacleClass.BUILDING, 0.8
        if width >= self._wall_min and linear:
            return ObstacleClass.WALL, 0.85
        if self._vehicle_range[0] <= width <= self._vehicle_range[1]:
            return ObstacleClass.VEHICLE, 0.7
        if width <= self._pole_max:
            return ObstacleClass.POLE, 0.75
        if self._person_range[0] <= width <= self._person_range[1]:
            # People are compact with some depth (not a flat surface).
            confidence = 0.7 if not linear else 0.5
            return ObstacleClass.PERSON, confidence
        if width <= self._vehicle_range[0]:
            return ObstacleClass.TREE, 0.6
        return ObstacleClass.UNKNOWN, 0.4
