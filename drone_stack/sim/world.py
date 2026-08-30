"""SimWorld - the shared simulated environment (Phase 7).

Holds a simple kinematic vehicle and a set of rectangular static obstacles in a
local ENU frame relative to "home". The mock Pixhawk drives the vehicle through
this world and reads telemetry from it; the mock LiDAR ray-casts the same world.
Because both mocks share one world instance, the simulated LiDAR sees obstacles
move consistently as the simulated vehicle flies its mission.

Nothing here talks to hardware - it is pure math.
"""
from __future__ import annotations

import math
import threading
import time
from dataclasses import dataclass

from drone_stack.msg import (
    Altitude,
    ArmedStatus,
    Attitude,
    Battery,
    FlightMode,
    GpsFix,
    Heartbeat,
    Imu,
    Message,
    NavCommand,
    RcChannels,
    SystemStatus,
    Velocity,
)
from drone_stack.utils.config import Config
from drone_stack.utils.geometry import (
    clamp,
    enu_to_geodetic,
    geodetic_to_enu,
    wrap_pi,
)

_GRAVITY = 9.80665


@dataclass
class SimObstacle:
    kind: str
    x: float          # ENU east (m), centre
    y: float          # ENU north (m), centre
    width: float      # extent along x (m)
    depth: float      # extent along y (m)
    vx: float = 0.0   # velocity (m/s) - non-zero for moving obstacles
    vy: float = 0.0


@dataclass
class VehicleState:
    x: float = 0.0
    y: float = 0.0
    z: float = 0.0            # altitude above home (m)
    vx: float = 0.0          # ENU east velocity
    vy: float = 0.0          # ENU north velocity
    vz: float = 0.0          # up velocity
    yaw: float = 0.0         # rad
    roll: float = 0.0
    pitch: float = 0.0
    armed: bool = False
    mode: str = "GUIDED"


class SimWorld:
    """Thread-safe simulated vehicle + obstacle field."""

    def __init__(self, config: Config) -> None:
        sim = config.section("sim")
        nav = config.section("navigation")
        start = sim.get("start_position", {})
        self._home_lat = float(start.get("lat", 47.397742))
        self._home_lon = float(start.get("lon", 8.545594))
        self._home_alt = float(start.get("alt_amsl_m", 488.0))

        self._cruise_speed = float(nav.get("cruise_speed_ms", 3.0))
        self._climb_rate = 1.5
        self._max_accel = 4.0

        self.state = VehicleState(yaw=math.radians(float(sim.get("start_heading_deg", 0.0))))
        self._battery_v = float(sim.get("battery_capacity_v", 16.8))
        self._battery_full = self._battery_v
        self._battery_empty = 13.2
        self._battery_drain = float(sim.get("battery_drain_per_min_v", 0.05))

        self.obstacles = [
            SimObstacle(
                kind=o.get("type", "unknown"),
                x=float(o.get("x", 0.0)),
                y=float(o.get("y", 0.0)),
                width=float(o.get("width", 0.5)),
                depth=float(o.get("depth", 0.5)),
            )
            for o in sim.get("obstacles", [])
        ]
        # A moving "person" that walks back and forth, to exercise avoidance/TTC.
        self._mover = SimObstacle(
            kind="person", x=6.0, y=-4.0, width=0.5, depth=0.4, vx=0.0, vy=0.9
        )
        self._mover_y_range = (-4.0, 4.0)
        self.obstacles.append(self._mover)

        # Command targets
        self._target: tuple[float, float, float] | None = None
        self._cmd_vel: tuple[float, float, float] | None = None
        self._cmd_vel_expiry = 0.0

        self._lock = threading.RLock()
        self._last_step = time.monotonic()

    # -- commands ------------------------------------------------------------
    def command(self, cmd: NavCommand) -> None:
        with self._lock:
            name, p = cmd.command, cmd.params
            s = self.state
            if name == "arm":
                s.armed = True
            elif name == "disarm":
                s.armed = False
                self._cmd_vel = None
                self._target = None
            elif name == "set_mode":
                mode = str(p.get("mode", s.mode)).upper()
                s.mode = mode
                if mode in ("RTL", "SMART_RTL"):
                    self._target = (0.0, 0.0, s.z if s.z > 0.5 else 0.0)
                    self._cmd_vel = None
                elif mode == "LAND":
                    self._target = (s.x, s.y, 0.0)
                    self._cmd_vel = None
                elif mode in ("POSHOLD", "LOITER", "BRAKE", "ALT_HOLD"):
                    # Park exactly where we are.
                    #
                    # WARNING - this is only true of BRAKE. Real ArduPilot
                    # POSHOLD/LOITER/ALT_HOLD take their ALTITUDE from the
                    # pilot's throttle stick: parked is what they do for a
                    # pilot holding throttle at centre, and a full-rate descent
                    # is what they do for the untouched transmitter of an
                    # autonomous flight. This model has no stick to read, so it
                    # cannot show that, and for months it let a POSHOLD
                    # drop-point hover pass in sim while the same hover flew
                    # the real aircraft into the ground at 2.4 m/s.
                    #
                    # Nothing in the delivery path should be entering these
                    # modes now (NavigationNode._STICK_ALTITUDE_MODES rejects
                    # them), so do not read a green sim run here as evidence
                    # that a stick-driven hold is safe.
                    self._target = (s.x, s.y, s.z)
                    self._cmd_vel = None
            elif name == "takeoff":
                s.mode = "GUIDED"
                self._target = (s.x, s.y, float(p.get("altitude", 5.0)))
                self._cmd_vel = None
            elif name == "goto":
                east, north = geodetic_to_enu(
                    float(p["lat"]), float(p["lon"]), self._home_lat, self._home_lon
                )
                self._target = (east, north, float(p.get("alt", s.z or 5.0)))
                self._cmd_vel = None
                s.mode = "GUIDED"
            elif name in ("rtl", "smart_rtl"):
                s.mode = "SMART_RTL" if name == "smart_rtl" else "RTL"
                self._target = (0.0, 0.0, s.z if s.z > 0.5 else 0.0)
                self._cmd_vel = None
            elif name == "land":
                s.mode = "LAND"
                self._target = (s.x, s.y, 0.0)
                self._cmd_vel = None
            elif name == "brake":
                s.mode = "BRAKE"
                self._target = (s.x, s.y, s.z)
                self._cmd_vel = None
            elif name == "velocity":
                self._cmd_vel = (
                    float(p.get("vx", 0.0)),
                    float(p.get("vy", 0.0)),
                    float(p.get("vz", 0.0)),
                )
                self._cmd_vel_expiry = time.monotonic() + 0.5
            elif name == "yaw":
                # direction +1 = right/CW; our ENU yaw is CCW-positive, so a
                # right turn decreases yaw.
                direction = 1 if int(p.get("direction", 1)) >= 0 else -1
                delta = math.radians(abs(float(p.get("angle", 0.0)))) * direction
                s.yaw = wrap_pi(s.yaw - delta)
            elif name == "set_speed":
                self._cruise_speed = max(0.2, float(p.get("speed", self._cruise_speed)))
            elif name == "set_home":
                self._home_lat = float(p.get("lat", self._home_lat))
                self._home_lon = float(p.get("lon", self._home_lon))
            elif name in ("upload_mission", "clear_mission", "noop"):
                # Mission upload is an autopilot-storage concern; the simulated
                # aircraft is flown by goto setpoints, same as the real one in
                # GUIDED, so there is nothing to store.
                pass

    # -- physics -------------------------------------------------------------
    def step(self, dt: float | None = None) -> None:
        with self._lock:
            now = time.monotonic()
            if dt is None:
                dt = now - self._last_step
            self._last_step = now
            dt = clamp(dt, 0.0, 0.2)  # guard against long stalls
            if dt <= 0.0:
                return
            self._integrate(dt)
            self._drain_battery(dt)
            self._update_movers(dt)

    def _update_movers(self, dt: float) -> None:
        m = self._mover
        m.x += m.vx * dt
        m.y += m.vy * dt
        lo, hi = self._mover_y_range
        if m.y <= lo or m.y >= hi:
            m.vy = -m.vy
            m.y = clamp(m.y, lo, hi)

    def _integrate(self, dt: float) -> None:
        s = self.state
        if not s.armed:
            s.vx = s.vy = s.vz = 0.0
            s.roll = s.pitch = 0.0
            return

        des_vx = des_vy = des_vz = 0.0
        if self._cmd_vel is not None and time.monotonic() < self._cmd_vel_expiry:
            bvx, bvy, bvz = self._cmd_vel
            # body (x forward, y left) -> ENU using yaw
            des_vx = bvx * math.cos(s.yaw) - bvy * math.sin(s.yaw)
            des_vy = bvx * math.sin(s.yaw) + bvy * math.cos(s.yaw)
            des_vz = bvz
        elif self._target is not None:
            tx, ty, tz = self._target
            dx, dy, dz = tx - s.x, ty - s.y, tz - s.z
            horiz = math.hypot(dx, dy)
            if horiz > 1e-3:
                speed = min(self._cruise_speed, horiz / dt)
                des_vx = dx / horiz * speed
                des_vy = dy / horiz * speed
                s.yaw = wrap_pi(math.atan2(dy, dx))
            des_vz = clamp(dz / dt, -self._climb_rate, self._climb_rate)

        # Simple acceleration limit for smoothness.
        s.vx += clamp(des_vx - s.vx, -self._max_accel * dt, self._max_accel * dt)
        s.vy += clamp(des_vy - s.vy, -self._max_accel * dt, self._max_accel * dt)
        s.vz += clamp(des_vz - s.vz, -self._max_accel * dt, self._max_accel * dt)

        s.x += s.vx * dt
        s.y += s.vy * dt
        s.z = max(0.0, s.z + s.vz * dt)

        # Tilt roughly proportional to horizontal acceleration demand.
        s.pitch = clamp(-math.hypot(s.vx, s.vy) / 20.0, -0.3, 0.3)
        s.roll = 0.0

        self._handle_arrival()

    def _handle_arrival(self) -> None:
        s = self.state
        if s.mode in ("RTL", "SMART_RTL"):
            # A real return-to-launch is two stages: fly home at altitude, then
            # descend and disarm. Without the second stage the aircraft hovers
            # over home forever and the mission never reports COMPLETE.
            if (
                math.hypot(s.x, s.y) < 1.0
                and self._target is not None
                and self._target[2] > 0.0
            ):
                self._target = (0.0, 0.0, 0.0)
        if s.mode in ("RTL", "SMART_RTL", "LAND") and s.z <= 0.05:
            dist_home = math.hypot(s.x, s.y)
            if s.mode == "LAND" or dist_home < 1.5:
                s.armed = False
                s.vx = s.vy = s.vz = 0.0

    def _drain_battery(self, dt: float) -> None:
        if self.state.armed:
            self._battery_v = max(
                self._battery_empty * 0.9,
                self._battery_v - self._battery_drain * dt / 60.0,
            )

    # -- telemetry -----------------------------------------------------------
    def battery_pct(self) -> float:
        span = self._battery_full - self._battery_empty
        if span <= 0:
            return 100.0
        return clamp((self._battery_v - self._battery_empty) / span * 100.0, 0.0, 100.0)

    def get_messages(self) -> list[Message]:
        with self._lock:
            s = self.state
            lat, lon = enu_to_geodetic(s.x, s.y, self._home_lat, self._home_lon)
            base_mode = 0
            if s.armed:
                base_mode |= 128  # MAV_MODE_FLAG_SAFETY_ARMED
            ground_speed = math.hypot(s.vx, s.vy)
            heading = (math.degrees(s.yaw)) % 360.0
            return [
                Heartbeat(base_mode=base_mode, system_status=4 if s.armed else 3),
                ArmedStatus(armed=s.armed),
                FlightMode(mode_name=s.mode, base_mode=base_mode),
                GpsFix(
                    fix_type=3,
                    satellites=12,
                    lat=lat,
                    lon=lon,
                    alt_amsl_m=self._home_alt + s.z,
                    eph=0.8,
                    epv=1.2,
                    ground_speed_ms=ground_speed,
                    course_deg=heading,
                ),
                Attitude(
                    roll=s.roll,
                    pitch=s.pitch,
                    yaw=wrap_pi(s.yaw),
                ),
                Imu(
                    ax=math.sin(s.pitch) * -_GRAVITY,
                    ay=math.sin(s.roll) * _GRAVITY,
                    az=-_GRAVITY,
                    gx=0.0,
                    gy=0.0,
                    gz=0.0,
                    mx=math.cos(s.yaw),
                    my=math.sin(s.yaw),
                    mz=0.0,
                ),
                Battery(
                    voltage_v=round(self._battery_v, 2),
                    current_a=8.0 if s.armed else 0.5,
                    remaining_pct=round(self.battery_pct(), 1),
                ),
                Altitude(relative_m=s.z, amsl_m=self._home_alt + s.z, climb_ms=s.vz),
                Velocity(
                    vx=s.vy,          # NED north = ENU north
                    vy=s.vx,          # NED east  = ENU east
                    vz=-s.vz,         # NED down  = -up
                    ground_speed_ms=ground_speed,
                    heading_deg=heading,
                ),
                SystemStatus(load_pct=25.0, healthy=True),
                RcChannels(channels=[1500] * 8, rssi=200, count=8),
            ]

    # -- ray casting for the mock LiDAR --------------------------------------
    def raycast(self, world_angle: float, max_range: float) -> float:
        """Nearest obstacle distance along ``world_angle`` (rad) from the vehicle."""
        with self._lock:
            px, py = self.state.x, self.state.y
        dx, dy = math.cos(world_angle), math.sin(world_angle)
        best = math.inf
        for ob in self.obstacles:
            t = _ray_aabb(px, py, dx, dy, ob)
            if t is not None and 0.0 <= t < best:
                best = t
        return best if best <= max_range else math.inf

    @property
    def home(self) -> tuple[float, float, float]:
        return self._home_lat, self._home_lon, self._home_alt


def _ray_aabb(
    px: float, py: float, dx: float, dy: float, ob: SimObstacle
) -> float | None:
    """Ray/axis-aligned-box intersection; returns nearest t>=0 or None."""
    minx, maxx = ob.x - ob.width / 2.0, ob.x + ob.width / 2.0
    miny, maxy = ob.y - ob.depth / 2.0, ob.y + ob.depth / 2.0
    tmin, tmax = -math.inf, math.inf
    for origin, direction, lo, hi in (
        (px, dx, minx, maxx),
        (py, dy, miny, maxy),
    ):
        if abs(direction) < 1e-9:
            if origin < lo or origin > hi:
                return None
            continue
        t1 = (lo - origin) / direction
        t2 = (hi - origin) / direction
        if t1 > t2:
            t1, t2 = t2, t1
        tmin = max(tmin, t1)
        tmax = min(tmax, t2)
        if tmin > tmax:
            return None
    if tmax < 0:
        return None
    return tmin if tmin >= 0 else tmax
