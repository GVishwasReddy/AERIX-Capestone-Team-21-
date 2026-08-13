"""Message dataclasses shared across the whole stack.

All fields have defaults so messages are trivial to construct in tests and so
dataclass inheritance ordering is never a problem. ``stamp`` is wall-clock time
in seconds (``time.time()``).
"""
from __future__ import annotations

import time
from dataclasses import asdict, dataclass, field, is_dataclass
from enum import Enum
from typing import Any


# ---------------------------------------------------------------------------
# Enumerations (str-based so they are JSON-serialisable and compare as strings)
# ---------------------------------------------------------------------------
class DiagLevel(str, Enum):
    OK = "OK"
    WARN = "WARN"
    ERROR = "ERROR"
    STALE = "STALE"


class ObstacleClass(str, Enum):
    UNKNOWN = "unknown"
    WALL = "wall"
    TREE = "tree"
    BUILDING = "building"
    POLE = "pole"
    PERSON = "person"
    VEHICLE = "vehicle"


class ScanState(str, Enum):
    STOPPED = "STOPPED"
    SCANNING = "SCANNING"
    PAUSED = "PAUSED"


class MissionPhase(str, Enum):
    IDLE = "IDLE"
    ARMING = "ARMING"
    TAKEOFF = "TAKEOFF"
    NAVIGATE = "NAVIGATE"
    AVOID = "AVOID"
    HOLD = "HOLD"
    MANUAL = "MANUAL"
    RTL = "RTL"
    LAND = "LAND"
    EMERGENCY = "EMERGENCY"
    DISARMED = "DISARMED"
    COMPLETE = "COMPLETE"


# ---------------------------------------------------------------------------
# Base
# ---------------------------------------------------------------------------
@dataclass
class Message:
    """Base class for every message; carries a wall-clock timestamp.

    ``stamp`` is keyword-only so that subclasses keep their own fields as the
    leading positional arguments (e.g. ``NavCommand("arm")`` sets ``command``,
    not ``stamp``).
    """

    stamp: float = field(default_factory=time.time, kw_only=True)

    def age(self) -> float:
        """Seconds since this message was stamped."""
        return max(0.0, time.time() - self.stamp)


# ---------------------------------------------------------------------------
# Phase 2 - MAVLink telemetry
# ---------------------------------------------------------------------------
@dataclass
class Heartbeat(Message):
    autopilot: int = 0
    vehicle_type: int = 0
    base_mode: int = 0
    custom_mode: int = 0
    system_status: int = 0
    mavlink_version: int = 0


@dataclass
class GpsFix(Message):
    fix_type: int = 0            # 0-1 none, 2 = 2D, 3 = 3D, 4+ = DGPS/RTK
    satellites: int = 0
    lat: float = 0.0            # degrees
    lon: float = 0.0            # degrees
    alt_amsl_m: float = 0.0
    eph: float = 0.0            # horizontal dilution (m)
    epv: float = 0.0            # vertical dilution (m)
    ground_speed_ms: float = 0.0
    course_deg: float = 0.0

    @property
    def has_fix(self) -> bool:
        return self.fix_type >= 3


@dataclass
class Attitude(Message):
    roll: float = 0.0           # rad
    pitch: float = 0.0          # rad
    yaw: float = 0.0            # rad
    rollspeed: float = 0.0      # rad/s
    pitchspeed: float = 0.0
    yawspeed: float = 0.0


@dataclass
class Imu(Message):
    ax: float = 0.0             # m/s^2
    ay: float = 0.0
    az: float = 0.0
    gx: float = 0.0             # rad/s
    gy: float = 0.0
    gz: float = 0.0
    mx: float = 0.0             # gauss (optional)
    my: float = 0.0
    mz: float = 0.0


@dataclass
class Battery(Message):
    voltage_v: float = 0.0
    current_a: float = 0.0
    remaining_pct: float = 0.0
    consumed_mah: float = 0.0
    temperature_c: float = 0.0


@dataclass
class Altitude(Message):
    relative_m: float = 0.0     # above home
    amsl_m: float = 0.0         # above mean sea level
    terrain_m: float = 0.0      # above terrain (if available)
    climb_ms: float = 0.0


@dataclass
class Velocity(Message):
    vx: float = 0.0             # NED north, m/s
    vy: float = 0.0             # NED east, m/s
    vz: float = 0.0             # NED down, m/s
    ground_speed_ms: float = 0.0
    heading_deg: float = 0.0


@dataclass
class FlightMode(Message):
    mode_name: str = "UNKNOWN"
    base_mode: int = 0
    custom_mode: int = 0


@dataclass
class ArmedStatus(Message):
    armed: bool = False


@dataclass
class SystemStatus(Message):
    load_pct: float = 0.0            # autopilot CPU load
    drop_rate_pct: float = 0.0       # comm drop rate
    errors_comm: int = 0
    sensors_present: int = 0         # bitmask
    sensors_enabled: int = 0         # bitmask
    sensors_health: int = 0          # bitmask
    healthy: bool = True


@dataclass
class RcChannels(Message):
    channels: list[int] = field(default_factory=list)   # microseconds
    rssi: int = 0
    count: int = 0


@dataclass
class LinkQuality(Message):
    connected: bool = False
    packets_received: int = 0
    packets_dropped: int = 0
    drop_rate_pct: float = 0.0
    last_heartbeat_age_s: float = 0.0
    connection_string: str = ""


# ---------------------------------------------------------------------------
# Phase 3 - LiDAR
# ---------------------------------------------------------------------------
@dataclass
class LaserScan(Message):
    """A full 360-degree scan. ``ranges`` are metres, indexed by beam.

    beam *i* is at angle ``angle_min + i * angle_increment`` (radians), measured
    counter-clockwise. ``inf`` marks an out-of-range / no-return beam.
    """

    frame_id: str = "lidar_link"
    angle_min: float = 0.0
    angle_max: float = 0.0
    angle_increment: float = 0.0
    range_min: float = 0.0
    range_max: float = 0.0
    ranges: list[float] = field(default_factory=list)
    intensities: list[float] = field(default_factory=list)

    @property
    def count(self) -> int:
        return len(self.ranges)


@dataclass
class PointCloud(Message):
    """Cartesian points (x, y, z) in metres, in the sensor frame."""

    frame_id: str = "lidar_link"
    points: list[tuple[float, float, float]] = field(default_factory=list)

    @property
    def count(self) -> int:
        return len(self.points)


# ---------------------------------------------------------------------------
# Phase 4 - sensor fusion
# ---------------------------------------------------------------------------
@dataclass
class FusedState(Message):
    """Best-estimate vehicle state produced by the fusion node."""

    # Local ENU position relative to home (metres)
    x: float = 0.0
    y: float = 0.0
    z: float = 0.0
    # Local ENU velocity (m/s)
    vx: float = 0.0
    vy: float = 0.0
    vz: float = 0.0
    # Orientation (rad)
    roll: float = 0.0
    pitch: float = 0.0
    yaw: float = 0.0
    # Global position
    lat: float = 0.0
    lon: float = 0.0
    alt_amsl_m: float = 0.0
    alt_rel_m: float = 0.0
    # Diagonal covariance [x,y,z,vx,vy,vz,roll,pitch,yaw]
    covariance: list[float] = field(default_factory=lambda: [0.0] * 9)
    sources: list[str] = field(default_factory=list)
    valid: bool = False


# ---------------------------------------------------------------------------
# Phase 5 - obstacles
# ---------------------------------------------------------------------------
@dataclass
class Obstacle(Message):
    id: int = 0
    distance_m: float = 0.0          # nearest range to the cluster
    bearing_deg: float = 0.0         # 0 = front, +right, in [-180, 180]
    angular_width_deg: float = 0.0
    width_m: float = 0.0             # estimated physical width
    x_m: float = 0.0                 # body frame, forward
    y_m: float = 0.0                 # body frame, left(+)
    classification: ObstacleClass = ObstacleClass.UNKNOWN
    confidence: float = 0.0
    danger: bool = False
    num_points: int = 0


@dataclass
class ObstacleArray(Message):
    frame_id: str = "base_link"
    obstacles: list[Obstacle] = field(default_factory=list)

    @property
    def count(self) -> int:
        return len(self.obstacles)

    def nearest(self) -> Obstacle | None:
        return min(self.obstacles, key=lambda o: o.distance_m, default=None)


@dataclass
class AvoidanceStatus(Message):
    """Summary of the collision-avoidance engine for the GCS avoidance panel."""

    enabled: bool = True
    status: str = "CLEAR"           # CLEAR | SLOW | BRAKE
    sending: bool = False           # are override commands being sent?
    direction: str = "none"         # left | right | front | none
    count: int = 0                  # obstacles considered
    closest_m: float = 0.0
    ttc_s: float = 0.0              # time to collision (s)
    cpa_m: float = 0.0              # closest point of approach (m)
    command: str = "none"
    reason: str = ""


# ---------------------------------------------------------------------------
# Phase 6 - navigation
# ---------------------------------------------------------------------------
@dataclass
class Waypoint(Message):
    seq: int = 0
    lat: float = 0.0
    lon: float = 0.0
    alt_m: float = 0.0               # relative altitude
    x_m: float = 0.0                 # local ENU (optional alternative to lat/lon)
    y_m: float = 0.0
    hold_s: float = 0.0
    radius_m: float = 1.5
    kind: str = "nav"                # nav | takeoff | land | rtl


@dataclass
class Mission(Message):
    name: str = "mission"
    waypoints: list[Waypoint] = field(default_factory=list)

    @property
    def count(self) -> int:
        return len(self.waypoints)


@dataclass
class MissionState(Message):
    phase: MissionPhase = MissionPhase.IDLE
    current_wp: int = 0
    total_wp: int = 0
    distance_to_wp_m: float = 0.0
    armed: bool = False
    mode: str = "UNKNOWN"
    avoiding: bool = False
    message: str = ""


@dataclass
class NavCommand(Message):
    """A command destined for the autopilot via MavlinkNode."""

    command: str = "noop"    # arm|disarm|set_mode|takeoff|goto|rtl|land|brake|velocity
    params: dict[str, Any] = field(default_factory=dict)


# ---------------------------------------------------------------------------
# Phase 9 - diagnostics
# ---------------------------------------------------------------------------
@dataclass
class Diagnostic(Message):
    name: str = ""
    level: DiagLevel = DiagLevel.OK
    message: str = ""
    values: dict[str, Any] = field(default_factory=dict)


@dataclass
class NodeHealth(Message):
    name: str = ""
    alive: bool = False
    running: bool = False
    restarts: int = 0
    heartbeat_age_s: float = 0.0


@dataclass
class SystemDiagnostics(Message):
    cpu_pct: float = 0.0
    mem_pct: float = 0.0
    temp_c: float = 0.0
    uptime_s: float = 0.0
    level: DiagLevel = DiagLevel.OK
    diagnostics: list[Diagnostic] = field(default_factory=list)
    nodes: list[NodeHealth] = field(default_factory=list)


# ---------------------------------------------------------------------------
# Serialisation helper
# ---------------------------------------------------------------------------
def to_dict(obj: Any) -> Any:
    """Recursively convert a message (or any nested structure) to plain types.

    dataclasses -> dict, Enums -> their value, tuples -> lists, so the result is
    directly ``json.dumps``-able for the web dashboard.
    """
    if is_dataclass(obj) and not isinstance(obj, type):
        return {k: to_dict(v) for k, v in asdict(obj).items()}
    if isinstance(obj, Enum):
        return obj.value
    if isinstance(obj, dict):
        return {k: to_dict(v) for k, v in obj.items()}
    if isinstance(obj, (list, tuple)):
        return [to_dict(v) for v in obj]
    if isinstance(obj, float):
        # keep payloads compact and JSON-clean (inf -> null handled by caller)
        return obj
    return obj
