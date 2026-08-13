"""Geometry / coordinate helpers used by fusion, obstacles, navigation and sim.

Local frame convention used throughout the stack:
    * ENU local frame relative to "home": x = East, y = North, z = Up (metres).
    * Body/vehicle frame for obstacles: x = forward, y = left, bearing 0 = front,
      positive bearing to the right.
Angles are radians unless a name ends in ``_deg``.
"""
from __future__ import annotations

import math

EARTH_RADIUS_M = 6_378_137.0


def clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


def wrap_pi(angle: float) -> float:
    """Wrap radians to [-pi, pi]."""
    return math.atan2(math.sin(angle), math.cos(angle))


def wrap_180(angle_deg: float) -> float:
    """Wrap degrees to [-180, 180]."""
    return math.degrees(wrap_pi(math.radians(angle_deg)))


def wrap_360(angle_deg: float) -> float:
    """Wrap degrees to [0, 360)."""
    return angle_deg % 360.0


def haversine_m(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
    """Great-circle distance in metres between two lat/lon points (degrees)."""
    p1, p2 = math.radians(lat1), math.radians(lat2)
    dphi = math.radians(lat2 - lat1)
    dlmb = math.radians(lon2 - lon1)
    a = math.sin(dphi / 2) ** 2 + math.cos(p1) * math.cos(p2) * math.sin(dlmb / 2) ** 2
    return 2 * EARTH_RADIUS_M * math.asin(min(1.0, math.sqrt(a)))


def bearing_deg(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
    """Initial compass bearing (degrees, 0 = North) from point 1 to point 2."""
    p1, p2 = math.radians(lat1), math.radians(lat2)
    dl = math.radians(lon2 - lon1)
    x = math.sin(dl) * math.cos(p2)
    y = math.cos(p1) * math.sin(p2) - math.sin(p1) * math.cos(p2) * math.cos(dl)
    return wrap_360(math.degrees(math.atan2(x, y)))


def geodetic_to_enu(
    lat: float, lon: float, ref_lat: float, ref_lon: float
) -> tuple[float, float]:
    """Equirectangular projection of (lat, lon) into local ENU (east, north) m.

    Accurate to well under a metre over the ranges relevant to a small drone.
    """
    d_east = math.radians(lon - ref_lon) * EARTH_RADIUS_M * math.cos(math.radians(ref_lat))
    d_north = math.radians(lat - ref_lat) * EARTH_RADIUS_M
    return d_east, d_north


def enu_to_geodetic(
    east: float, north: float, ref_lat: float, ref_lon: float
) -> tuple[float, float]:
    """Inverse of :func:`geodetic_to_enu`."""
    lat = ref_lat + math.degrees(north / EARTH_RADIUS_M)
    lon = ref_lon + math.degrees(east / (EARTH_RADIUS_M * math.cos(math.radians(ref_lat))))
    return lat, lon


def polar_to_cartesian(range_m: float, angle_rad: float) -> tuple[float, float]:
    """Convert a LiDAR (range, angle) beam to (x, y). angle 0 = +x (forward)."""
    return range_m * math.cos(angle_rad), range_m * math.sin(angle_rad)


def distance_2d(x1: float, y1: float, x2: float, y2: float) -> float:
    return math.hypot(x2 - x1, y2 - y1)
