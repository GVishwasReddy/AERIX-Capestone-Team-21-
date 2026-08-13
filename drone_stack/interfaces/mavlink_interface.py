"""MAVLink interface abstraction + real (pymavlink) implementation.

``MavlinkInterface`` is the contract shared by the real autopilot link and the
simulated one (:class:`drone_stack.sim.mock_pixhawk.MockMavlink`). A node holds
one of these and never cares which it is.

pymavlink is imported lazily so a simulation-only environment does not need it.
"""
from __future__ import annotations

import abc
import math
import time
from typing import Any

from drone_stack.msg import (
    Altitude,
    ArmedStatus,
    Attitude,
    Battery,
    FlightMode,
    GpsFix,
    Heartbeat,
    Imu,
    LinkQuality,
    Message,
    NavCommand,
    RcChannels,
    SystemStatus,
    Velocity,
)
from drone_stack.utils.logging_setup import get_logger

try:  # pymavlink is optional in a pure-simulation install
    from pymavlink import mavutil
except Exception:  # noqa: BLE001 - any import problem means "not available"
    mavutil = None

_GRAVITY = 9.80665


class MavlinkInterface(abc.ABC):
    """Abstract MAVLink link. Implementations must be safe to reconnect."""

    @property
    @abc.abstractmethod
    def connected(self) -> bool:
        ...

    @abc.abstractmethod
    def connect(self) -> bool:
        """Attempt to (re)connect. Return True on success. Never raises."""

    @abc.abstractmethod
    def close(self) -> None:
        ...

    @abc.abstractmethod
    def receive(self) -> list[Message]:
        """Return telemetry messages available since the last call."""

    @abc.abstractmethod
    def send_command(self, command: NavCommand) -> bool:
        """Send a command to the autopilot. Return True if accepted/sent."""

    @abc.abstractmethod
    def link_quality(self) -> LinkQuality:
        ...


class RealMavlink(MavlinkInterface):
    """pymavlink-backed link to a physical Pixhawk (or SITL over UDP)."""

    def __init__(self, config: dict[str, Any]) -> None:
        self.log = get_logger("mavlink.link")
        self._cfg = config
        self._conn_str: str = config.get("connection", "udp:127.0.0.1:14550")
        self._baud: int = int(config.get("baud", 115200))
        self._source_system: int = int(config.get("source_system", 255))
        self._source_component: int = int(config.get("source_component", 190))
        self._target_system: int = int(config.get("target_system", 1))
        self._target_component: int = int(config.get("target_component", 1))
        self._stream_rate: int = int(config.get("request_stream_rate_hz", 10))
        self._hb_timeout: float = float(config.get("heartbeat_timeout_s", 5.0))

        self._master = None
        self._connected = False
        self._last_heartbeat = 0.0
        self._packets_received = 0
        self._drop_rate = 0.0
        self._mode_name = "UNKNOWN"
        # Aux outputs we've already switched to "Disabled" (SERVOx_FUNCTION=0)
        # so MAV_CMD_DO_SET_SERVO can drive them. Done lazily, once per channel.
        self._servo_ready: set[int] = set()

    # -- connection ----------------------------------------------------------
    @property
    def connected(self) -> bool:
        if not self._connected:
            return False
        # Consider the link dead if no heartbeat within the timeout window.
        if self._last_heartbeat and (time.time() - self._last_heartbeat) > self._hb_timeout:
            self._connected = False
        return self._connected

    def connect(self) -> bool:
        if mavutil is None:
            self.log.error(
                "pymavlink is not installed - install requirements-hardware.txt"
            )
            return False
        try:
            self.log.info("connecting to %s", self._conn_str)
            self._master = mavutil.mavlink_connection(
                self._conn_str,
                baud=self._baud,
                source_system=self._source_system,
                source_component=self._source_component,
                autoreconnect=True,
            )
            heartbeat = self._master.wait_heartbeat(timeout=self._hb_timeout)
            if heartbeat is None:
                self.log.warning("no heartbeat within %.1fs", self._hb_timeout)
                self.close()
                return False
            self._target_system = self._master.target_system or self._target_system
            self._target_component = (
                self._master.target_component or self._target_component
            )
            self._request_streams()
            self._connected = True
            self._last_heartbeat = time.time()
            self.log.info(
                "connected (sys=%d comp=%d)",
                self._target_system,
                self._target_component,
            )
            return True
        except Exception as exc:  # noqa: BLE001
            # A missing/unavailable device is an expected condition (we retry),
            # so log a concise warning rather than a full stack trace.
            self.log.warning("connection to %s failed: %s", self._conn_str, exc)
            self.close()
            return False

    def _request_streams(self) -> None:
        try:
            self._master.mav.request_data_stream_send(
                self._target_system,
                self._target_component,
                mavutil.mavlink.MAV_DATA_STREAM_ALL,
                self._stream_rate,
                1,
            )
        except Exception:  # noqa: BLE001
            self.log.exception("failed to request data streams")

    def close(self) -> None:
        self._connected = False
        if self._master is not None:
            try:
                self._master.close()
            except Exception:  # noqa: BLE001
                pass
            self._master = None

    # -- receive -------------------------------------------------------------
    def receive(self) -> list[Message]:
        out: list[Message] = []
        if self._master is None:
            return out
        # Drain everything currently buffered, with a safety budget.
        for _ in range(500):
            try:
                msg = self._master.recv_match(blocking=False)
            except Exception:  # noqa: BLE001
                self.log.exception("recv_match failed")
                self._connected = False
                break
            if msg is None:
                break
            self._packets_received += 1
            parsed = self._parse(msg)
            out.extend(parsed)
        return out

    def _parse(self, msg) -> list[Message]:  # noqa: C901 - explicit per-type mapping
        mtype = msg.get_type()
        now = time.time()
        result: list[Message] = []
        if mtype == "HEARTBEAT":
            self._last_heartbeat = now
            armed = bool(
                msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED
            )
            try:
                self._mode_name = self._master.flightmode
            except Exception:  # noqa: BLE001
                pass
            result.append(
                Heartbeat(
                    autopilot=msg.autopilot,
                    vehicle_type=msg.type,
                    base_mode=msg.base_mode,
                    custom_mode=msg.custom_mode,
                    system_status=msg.system_status,
                    mavlink_version=msg.mavlink_version,
                )
            )
            result.append(ArmedStatus(armed=armed))
            result.append(
                FlightMode(
                    mode_name=self._mode_name,
                    base_mode=msg.base_mode,
                    custom_mode=msg.custom_mode,
                )
            )
        elif mtype == "ATTITUDE":
            result.append(
                Attitude(
                    roll=msg.roll,
                    pitch=msg.pitch,
                    yaw=msg.yaw,
                    rollspeed=msg.rollspeed,
                    pitchspeed=msg.pitchspeed,
                    yawspeed=msg.yawspeed,
                )
            )
        elif mtype == "GPS_RAW_INT":
            result.append(
                GpsFix(
                    fix_type=msg.fix_type,
                    satellites=msg.satellites_visible,
                    lat=msg.lat / 1e7,
                    lon=msg.lon / 1e7,
                    alt_amsl_m=msg.alt / 1000.0,
                    eph=(msg.eph / 100.0) if msg.eph != 65535 else 0.0,
                    epv=(msg.epv / 100.0) if msg.epv != 65535 else 0.0,
                    ground_speed_ms=(msg.vel / 100.0) if msg.vel != 65535 else 0.0,
                    course_deg=(msg.cog / 100.0) if msg.cog != 65535 else 0.0,
                )
            )
        elif mtype == "GLOBAL_POSITION_INT":
            result.append(
                Altitude(
                    relative_m=msg.relative_alt / 1000.0,
                    amsl_m=msg.alt / 1000.0,
                )
            )
            result.append(
                Velocity(
                    vx=msg.vx / 100.0,
                    vy=msg.vy / 100.0,
                    vz=msg.vz / 100.0,
                    ground_speed_ms=math.hypot(msg.vx / 100.0, msg.vy / 100.0),
                    heading_deg=(msg.hdg / 100.0) if msg.hdg != 65535 else 0.0,
                )
            )
        elif mtype in ("SCALED_IMU", "SCALED_IMU2", "SCALED_IMU3"):
            result.append(
                Imu(
                    ax=msg.xacc * _GRAVITY / 1000.0,
                    ay=msg.yacc * _GRAVITY / 1000.0,
                    az=msg.zacc * _GRAVITY / 1000.0,
                    gx=msg.xgyro / 1000.0,
                    gy=msg.ygyro / 1000.0,
                    gz=msg.zgyro / 1000.0,
                    mx=msg.xmag / 1000.0,
                    my=msg.ymag / 1000.0,
                    mz=msg.zmag / 1000.0,
                )
            )
        elif mtype == "HIGHRES_IMU":
            result.append(
                Imu(
                    ax=msg.xacc, ay=msg.yacc, az=msg.zacc,
                    gx=msg.xgyro, gy=msg.ygyro, gz=msg.zgyro,
                    mx=msg.xmag, my=msg.ymag, mz=msg.zmag,
                )
            )
        elif mtype == "SYS_STATUS":
            result.append(
                Battery(
                    voltage_v=msg.voltage_battery / 1000.0,
                    current_a=(msg.current_battery / 100.0)
                    if msg.current_battery != -1
                    else 0.0,
                    remaining_pct=float(max(0, msg.battery_remaining)),
                )
            )
            result.append(
                SystemStatus(
                    load_pct=msg.load / 10.0,
                    drop_rate_pct=msg.drop_rate_comm / 100.0,
                    errors_comm=msg.errors_comm,
                    sensors_present=msg.onboard_control_sensors_present,
                    sensors_enabled=msg.onboard_control_sensors_enabled,
                    sensors_health=msg.onboard_control_sensors_health,
                    healthy=(
                        msg.onboard_control_sensors_health
                        & msg.onboard_control_sensors_enabled
                    )
                    == msg.onboard_control_sensors_enabled,
                )
            )
            self._drop_rate = msg.drop_rate_comm / 100.0
        elif mtype == "BATTERY_STATUS":
            voltages = [v for v in msg.voltages if v != 65535]
            volts = sum(voltages) / 1000.0 if voltages else 0.0
            result.append(
                Battery(
                    voltage_v=volts,
                    current_a=(msg.current_battery / 100.0)
                    if msg.current_battery != -1
                    else 0.0,
                    remaining_pct=float(max(0, msg.battery_remaining)),
                    consumed_mah=float(max(0, msg.current_consumed)),
                )
            )
        elif mtype == "VFR_HUD":
            result.append(
                Altitude(amsl_m=msg.alt, climb_ms=msg.climb)
            )
        elif mtype == "ALTITUDE":
            result.append(
                Altitude(
                    relative_m=msg.altitude_relative,
                    amsl_m=msg.altitude_amsl,
                    terrain_m=msg.altitude_terrain,
                )
            )
        elif mtype == "RC_CHANNELS":
            channels = [
                getattr(msg, f"chan{i}_raw", 0) for i in range(1, 19)
            ]
            result.append(
                RcChannels(
                    channels=channels[: msg.chancount] if msg.chancount else channels,
                    rssi=msg.rssi,
                    count=msg.chancount,
                )
            )
        elif mtype == "STATUSTEXT":
            # Surface autopilot messages (PreArm/Arm failures, EKF, GPS, etc.)
            # to the console so the operator sees *why* a command was refused.
            text = getattr(msg, "text", "")
            if isinstance(text, bytes):
                text = text.decode("utf-8", "replace")
            text = text.strip("\x00").strip()
            if text:
                sev = getattr(msg, "severity", 6)
                if sev <= 3:
                    self.log.error("FC: %s", text)
                elif sev <= 5:
                    self.log.warning("FC: %s", text)
                else:
                    self.log.info("FC: %s", text)
        elif mtype == "COMMAND_ACK":
            try:
                cmd = msg.command
                res = msg.result
                names = {
                    mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM: "arm/disarm",
                    mavutil.mavlink.MAV_CMD_NAV_TAKEOFF: "takeoff",
                    mavutil.mavlink.MAV_CMD_DO_SET_MODE: "set_mode",
                }
                if cmd in names:
                    ok = res == mavutil.mavlink.MAV_RESULT_ACCEPTED
                    reasons = {
                        0: "accepted", 1: "temporarily rejected", 2: "denied",
                        3: "unsupported", 4: "failed", 5: "in progress",
                    }
                    label = reasons.get(res, f"result {res}")
                    if ok:
                        self.log.info("FC ack: %s %s", names[cmd], label)
                    else:
                        self.log.warning("FC ack: %s %s", names[cmd], label)
            except Exception:  # noqa: BLE001
                pass
        return result

    # -- commands ------------------------------------------------------------
    def send_command(self, command: NavCommand) -> bool:
        if self._master is None or not self._connected:
            self.log.warning("cannot send '%s': link down", command.command)
            return False
        try:
            return self._dispatch(command)
        except Exception:  # noqa: BLE001
            self.log.exception("failed to send command '%s'", command.command)
            return False

    def _dispatch(self, command: NavCommand) -> bool:  # noqa: C901
        cmd = command.command
        p = command.params
        mav = self._master.mav
        tgt_s, tgt_c = self._target_system, self._target_component
        if cmd in ("arm", "disarm"):
            mav.command_long_send(
                tgt_s, tgt_c,
                mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0,
                1 if cmd == "arm" else 0, 0, 0, 0, 0, 0, 0,
            )
            return True
        if cmd == "set_mode":
            return self._set_mode(str(p.get("mode", "GUIDED")))
        if cmd == "rtl":
            return self._set_mode("RTL")
        if cmd == "land":
            return self._set_mode("LAND")
        if cmd == "brake":
            return self._set_mode("BRAKE")
        if cmd == "takeoff":
            self._set_mode("GUIDED")
            mav.command_long_send(
                tgt_s, tgt_c, mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0,
                0, 0, 0, 0, 0, 0, float(p.get("altitude", 5.0)),
            )
            return True
        if cmd == "goto":
            type_mask = 0b0000111111111000  # position only
            mav.set_position_target_global_int_send(
                0, tgt_s, tgt_c,
                mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
                type_mask,
                int(float(p["lat"]) * 1e7), int(float(p["lon"]) * 1e7),
                float(p.get("alt", 5.0)),
                0, 0, 0, 0, 0, 0, 0, 0,
            )
            return True
        if cmd == "velocity":
            type_mask = 0b0000111111000111  # velocity only
            mav.set_position_target_local_ned_send(
                0, tgt_s, tgt_c,
                mavutil.mavlink.MAV_FRAME_BODY_NED,
                type_mask,
                0, 0, 0,
                float(p.get("vx", 0.0)), float(p.get("vy", 0.0)),
                float(p.get("vz", 0.0)),
                0, 0, 0, 0, 0,
            )
            return True
        if cmd == "yaw":
            angle = abs(float(p.get("angle", 0.0)))
            direction = 1 if int(p.get("direction", 1)) >= 0 else -1
            mav.command_long_send(
                tgt_s, tgt_c, mavutil.mavlink.MAV_CMD_CONDITION_YAW, 0,
                angle, float(p.get("rate", 25.0)), direction, 1, 0, 0, 0,
            )
            return True
        if cmd == "set_speed":
            mav.command_long_send(
                tgt_s, tgt_c, mavutil.mavlink.MAV_CMD_DO_CHANGE_SPEED, 0,
                1, float(p.get("speed", 3.0)), -1, 0, 0, 0, 0,
            )
            return True
        if cmd == "set_servo":
            ch = int(p.get("channel", 9))
            pwm = max(800, min(2200, int(p.get("pwm", 1500))))
            # ArduPilot only lets DO_SET_SERVO drive an output that isn't
            # assigned a flight function. Set SERVO{ch}_FUNCTION=0 (Disabled)
            # once per channel so AUX1 (ch9) responds. Harmless for an unused
            # aux; persists on the FC.
            if ch not in self._servo_ready:
                try:
                    self._master.mav.param_set_send(
                        tgt_s, tgt_c,
                        f"SERVO{ch}_FUNCTION".encode(), 0.0,
                        mavutil.mavlink.MAV_PARAM_TYPE_INT8,
                    )
                    self.log.info("set SERVO%d_FUNCTION=0 (payload servo)", ch)
                except Exception:  # noqa: BLE001
                    self.log.exception("could not set SERVO%d_FUNCTION", ch)
                self._servo_ready.add(ch)
            mav.command_long_send(
                tgt_s, tgt_c, mavutil.mavlink.MAV_CMD_DO_SET_SERVO, 0,
                ch, pwm, 0, 0, 0, 0, 0,
            )
            self.log.info("DO_SET_SERVO ch=%d pwm=%dus", ch, pwm)
            return True
        if cmd == "noop":
            return True
        self.log.warning("unknown command '%s'", cmd)
        return False

    def _set_mode(self, mode_name: str) -> bool:
        mode_name = mode_name.upper()
        mapping = self._master.mode_mapping() or {}
        if mode_name not in mapping:
            self.log.error("mode '%s' not available on this vehicle", mode_name)
            return False
        self._master.set_mode(mapping[mode_name])
        return True

    # -- quality -------------------------------------------------------------
    def link_quality(self) -> LinkQuality:
        age = (time.time() - self._last_heartbeat) if self._last_heartbeat else 0.0
        return LinkQuality(
            connected=self.connected,
            packets_received=self._packets_received,
            drop_rate_pct=self._drop_rate,
            last_heartbeat_age_s=round(age, 3),
            connection_string=self._conn_str,
        )
