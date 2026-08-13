"""Canonical topic names.

Using constants instead of raw strings prevents typos and gives a single place
to see every channel in the system (the equivalent of a ROS topic list).
"""
from __future__ import annotations


class Topics:
    """Namespaced topic-name constants."""

    # --- Telemetry (published by MavlinkNode) -------------------------------
    HEARTBEAT = "/telemetry/heartbeat"
    GPS = "/telemetry/gps"
    ATTITUDE = "/telemetry/attitude"
    IMU = "/telemetry/imu"
    BATTERY = "/telemetry/battery"
    ALTITUDE = "/telemetry/altitude"
    VELOCITY = "/telemetry/velocity"
    FLIGHT_MODE = "/telemetry/mode"
    ARMED = "/telemetry/armed"
    SYS_STATUS = "/telemetry/sys_status"
    RC = "/telemetry/rc"
    LINK = "/telemetry/link"

    # --- LiDAR (published by LidarNode) -------------------------------------
    SCAN = "/scan"
    CLOUD = "/cloud"

    # --- Perception / state -------------------------------------------------
    FUSED_STATE = "/state/fused"
    OBSTACLES = "/obstacles"
    AVOIDANCE = "/avoidance"
    SCAN_STATE = "/scan/state"

    # --- Navigation ---------------------------------------------------------
    MISSION_STATE = "/mission/state"
    MISSION_PLAN = "/mission/plan"        # the loaded Mission (waypoints)
    MISSION_CMD = "/mission/cmd"          # request a mission action
    MAVLINK_CMD = "/cmd/mavlink"          # NavCommand -> MavlinkNode

    # --- Diagnostics --------------------------------------------------------
    DIAGNOSTICS = "/diagnostics"
    DIAG_NODES = "/diagnostics/nodes"
    DIAG_LIDAR = "/diagnostics/lidar"
    DIAG_MAVLINK = "/diagnostics/mavlink"

    @classmethod
    def all(cls) -> list[str]:
        """Return every declared topic name (useful for introspection/tests)."""
        return [
            v for k, v in vars(cls).items()
            if not k.startswith("_") and isinstance(v, str)
        ]
