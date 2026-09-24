"""Configuration loading for the parcel delivery stack.

All tunables come from environment variables (loaded from a local .env via
python-dotenv). No secrets or coordinates are hardcoded.
"""
from __future__ import annotations

import os
from dataclasses import dataclass

from dotenv import load_dotenv


@dataclass(frozen=True)
class Config:
    # Home / geofence
    home_lat: float
    home_lon: float
    geofence_radius_m: float
    min_altitude_m: float
    max_altitude_m: float
    hover_altitude_m: float
    arrival_tolerance_m: float

    # Failsafes
    hover_timeout_s: float
    battery_failsafe_pct: float

    # MAVLink
    mavlink_connection: str
    # Separate endpoint for the lidar bridge. MAVSDK and pymavlink cannot both
    # hold the same serial device — see load_config() and the README.
    lidar_mavlink_connection: str

    # Lidar
    lidar_port: str
    lidar_baud: int
    # Scanned window, centred on the nose. Mirrors lidar.fov_deg in
    # drone_stack's config/*.yaml — keep the two in step or the GCS radar and
    # this bridge disagree about what the aircraft can see.
    lidar_fov_enabled: bool
    lidar_fov_deg: float

    # Firebase
    firebase_credentials_path: str
    firebase_db_url: str

    # Logging
    log_dir: str
    log_level: str


def load_config(env_path: str | None = None) -> Config:
    load_dotenv(dotenv_path=env_path, override=False)

    def _f(name: str, default: float) -> float:
        return float(os.environ.get(name, default))

    def _b(name: str, default: bool) -> bool:
        # Fail loud on a typo rather than silently disabling a safety mask:
        # bool("false") is True, which is exactly the trap here.
        raw = os.environ.get(name)
        if raw is None or raw == "":
            return default
        lowered = raw.strip().lower()
        if lowered in ("1", "true", "yes", "on"):
            return True
        if lowered in ("0", "false", "no", "off"):
            return False
        raise RuntimeError(f"{name} must be a boolean, got {raw!r}")

    def _req(name: str) -> str:
        value = os.environ.get(name)
        if not value:
            raise RuntimeError(f"Missing required environment variable: {name}")
        return value

    mavlink_connection = os.environ.get("MAVLINK_CONNECTION", "udp://:14540")

    return Config(
        home_lat=_f("HOME_LAT", 0.0),
        home_lon=_f("HOME_LON", 0.0),
        geofence_radius_m=_f("GEOFENCE_RADIUS_M", 300.0),
        min_altitude_m=_f("MIN_ALTITUDE_M", 2.0),
        max_altitude_m=_f("MAX_ALTITUDE_M", 50.0),
        hover_altitude_m=_f("HOVER_ALTITUDE_M", 5.0),
        arrival_tolerance_m=_f("ARRIVAL_TOLERANCE_M", 2.0),
        hover_timeout_s=_f("HOVER_TIMEOUT_S", 120.0),
        battery_failsafe_pct=_f("BATTERY_FAILSAFE_PCT", 20.0),
        mavlink_connection=mavlink_connection,
        lidar_mavlink_connection=os.environ.get(
            "LIDAR_MAVLINK_CONNECTION", mavlink_connection
        ),
        lidar_port=os.environ.get("LIDAR_PORT", "/dev/ttyUSB0"),
        lidar_baud=int(os.environ.get("LIDAR_BAUD", "460800")),
        lidar_fov_enabled=_b("LIDAR_FOV_ENABLED", True),
        lidar_fov_deg=_f("LIDAR_FOV_DEG", 250.0),
        firebase_credentials_path=_req("FIREBASE_CREDENTIALS_PATH"),
        firebase_db_url=_req("FIREBASE_DB_URL"),
        log_dir=os.environ.get("LOG_DIR", "logs"),
        log_level=os.environ.get("LOG_LEVEL", "INFO"),
    )
