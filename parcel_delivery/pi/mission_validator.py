"""Pure validation functions for incoming delivery requests.

No hardware or network I/O here — everything is a pure function so it can be
unit tested without a Pixhawk, a lidar, or Firebase.
"""
from __future__ import annotations

import math
from dataclasses import dataclass


@dataclass(frozen=True)
class ValidationResult:
    ok: bool
    reason: str | None = None


def validate_lat_lon_bounds(lat: float, lon: float) -> ValidationResult:
    if not isinstance(lat, (int, float)) or not isinstance(lon, (int, float)):
        return ValidationResult(False, "lat/lon must be numeric")
    if math.isnan(lat) or math.isnan(lon):
        return ValidationResult(False, "lat/lon must not be NaN")
    if not (-90.0 <= lat <= 90.0):
        return ValidationResult(False, f"latitude {lat} out of range [-90, 90]")
    if not (-180.0 <= lon <= 180.0):
        return ValidationResult(False, f"longitude {lon} out of range [-180, 180]")
    return ValidationResult(True)


def haversine_distance_m(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
    """Great-circle distance between two lat/lon points, in meters."""
    r = 6371000.0  # mean Earth radius, meters
    phi1, phi2 = math.radians(lat1), math.radians(lat2)
    dphi = math.radians(lat2 - lat1)
    dlambda = math.radians(lon2 - lon1)
    a = math.sin(dphi / 2) ** 2 + math.cos(phi1) * math.cos(phi2) * math.sin(dlambda / 2) ** 2
    return 2 * r * math.asin(math.sqrt(a))


def validate_geofence(
    lat: float, lon: float, home_lat: float, home_lon: float, radius_m: float
) -> ValidationResult:
    distance = haversine_distance_m(home_lat, home_lon, lat, lon)
    if distance > radius_m:
        return ValidationResult(
            False,
            f"destination is {distance:.1f}m from home, exceeds geofence radius {radius_m:.1f}m",
        )
    return ValidationResult(True)


def validate_altitude(alt_agl_m: float, min_altitude_m: float, max_altitude_m: float) -> ValidationResult:
    if not isinstance(alt_agl_m, (int, float)) or math.isnan(alt_agl_m):
        return ValidationResult(False, "altitude must be numeric")
    if not (min_altitude_m <= alt_agl_m <= max_altitude_m):
        return ValidationResult(
            False,
            f"altitude {alt_agl_m}m out of allowed range [{min_altitude_m}, {max_altitude_m}]m",
        )
    return ValidationResult(True)


def validate_delivery_request(
    lat: float,
    lon: float,
    alt_agl_m: float,
    home_lat: float,
    home_lon: float,
    geofence_radius_m: float,
    min_altitude_m: float,
    max_altitude_m: float,
) -> ValidationResult:
    """Run all checks in order, short-circuiting on the first failure."""
    for result in (
        validate_lat_lon_bounds(lat, lon),
        validate_altitude(alt_agl_m, min_altitude_m, max_altitude_m),
        validate_geofence(lat, lon, home_lat, home_lon, geofence_radius_m),
    ):
        if not result.ok:
            return result
    return ValidationResult(True)
