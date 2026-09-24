"""RPLIDAR C1 -> MAVLink OBSTACLE_DISTANCE bridge.

Runs continuously and independently of the delivery state machine: obstacle
data should stream whenever the drone is powered, not just during a mission.

The Pi does NO path planning. It only publishes what the lidar sees — and only
the part of it that is worth seeing: the rear wedge is masked out, because it
contains the airframe itself. See :func:`sector_keep_mask`.

ArduPilot's onboard object avoidance (OA_TYPE = BendyRuler/Dijkstra, with
PRX_TYPE = MAVLink) does the actual rerouting. See the README.

MAVSDK does not expose sending OBSTACLE_DISTANCE, so this uses pymavlink
directly on its own MAVLink connection.

Driver note: the RPLIDAR **C1 is not supported by the `rplidar` PyPI package**.
That library's handshake fails with "Descriptor length mismatch" because the C1
free-runs its scan on power-up and floods the serial buffer. Verified on this
airframe's own unit. We therefore speak the SLAMTEC serial protocol directly:
STOP + flush to silence the free-run, pull DTR low to spin the motor, request a
standard scan, and parse the 5-byte legacy measurement nodes ourselves.
"""
from __future__ import annotations

import logging
import threading
import time
from typing import Iterable, Sequence

logger = logging.getLogger("lidar_bridge")

# OBSTACLE_DISTANCE carries a fixed 72-element array.
SECTOR_COUNT = 72
SECTOR_WIDTH_DEG = 360 / SECTOR_COUNT  # 5 degrees per sector
DISTANCE_UNKNOWN = 65535  # UINT16_MAX -> "no reading in this sector"

MAV_DISTANCE_SENSOR_LASER = 0
MAV_FRAME_BODY_FRD = 12

#: Total scanned arc kept, centred on the nose, matching ``lidar.fov_deg`` in
#: drone_stack's config/real.yaml. 125 deg left + 125 deg right; the remaining
#: 110 deg directly behind the aircraft is never reported.
DEFAULT_FOV_DEG = 250.0

#: Boundary tolerance when judging a sector against the FOV edge, in degrees.
_EPS = 1e-9

# SLAMTEC serial commands (prefixed with 0xA5).
_CMD_STOP = 0x25
_CMD_RESET = 0x40
_CMD_SCAN = 0x20

_MOTOR_SPINUP_S = 5.0  # the C1 needs several seconds to reach scan speed


def wrap_180(deg: float) -> float:
    """Fold an angle onto [-180, 180), so left and right are symmetric.

    Only ``abs()`` of this is ever used below, so which end of the range 180 deg
    lands on does not matter to the mask - but it is [-180, 180) here, where
    drone_stack's ``utils.geometry.wrap_180`` is closed at both ends.
    """
    return (deg + 180.0) % 360.0 - 180.0


def sector_keep_mask(
    fov_deg: float = DEFAULT_FOV_DEG, enabled: bool = True
) -> list[bool]:
    """Which of the 72 sectors fall inside the scanned window.

    The C1 spins a full circle, but the rear wedge of that circle is not usable
    data on this airframe: it always contains the aircraft's own tail and legs,
    plus whatever it happens to be standing next to. Those returns never move,
    so streaming them as OBSTACLE_DISTANCE makes ArduPilot brake for obstacles
    that are effectively bolted to the vehicle — which is exactly what a rooftop
    takeoff looks like to an unmasked scan.

    Masked sectors are left at ``DISTANCE_UNKNOWN`` rather than at some large
    distance. Be clear about what that buys and what it does not: ArduPilot's
    proximity database reads *unknown* as **clear**, so this does not stop
    BendyRuler routing backwards into ground the sensor has never seen. It only
    stops the FC being fed the airframe. Not turning your back on unscanned
    ground is a separate mechanism — ``WP_YAW_BEHAVIOR = 1`` plus drone_stack's
    yaw gate; see ``docs/31aug_status.md`` section 7.

    Judged on each sector's centre, so the kept set is exactly symmetric about
    the nose: the default 250 deg window keeps 50 sectors (0-24 and 47-71) and
    leaves the 22 sectors of the rear 110 deg permanently unknown. This is the
    same rule as drone_stack's ``proximity_node.sector_keep_mask``, so the two
    stacks hand the flight controller the same picture.

    ``enabled=False`` (or a 360 deg window) returns an all-true mask: a full
    circle is not a mask.
    """
    if not enabled or fov_deg >= 360.0:
        return [True] * SECTOR_COUNT

    half_deg = max(0.0, min(360.0, fov_deg)) / 2.0
    keep: list[bool] = []
    for i in range(SECTOR_COUNT):
        centre = i * SECTOR_WIDTH_DEG + SECTOR_WIDTH_DEG / 2.0
        keep.append(abs(wrap_180(centre)) <= half_deg + _EPS)
    return keep


def scan_to_distances(
    scan: Iterable[tuple[float, float]],
    min_distance_cm: int,
    max_distance_cm: int,
    angle_offset_deg: float = 0.0,
    keep: Sequence[bool] | None = None,
) -> list[int]:
    """Bucket a lidar scan into the 72-sector distance array OBSTACLE_DISTANCE wants.

    Args:
        scan: iterable of (angle_deg, distance_m) pairs. Angles are measured
            clockwise from the vehicle's nose, as the RPLIDAR reports them.
        min_distance_cm / max_distance_cm: sensor limits, in centimetres.
        angle_offset_deg: mounting offset applied to every beam. Note this is
            *added* to the raw clockwise angle, whereas drone_stack negates the
            raw angle first (it works counter-clockwise) and then adds its
            offset. The two therefore rotate opposite ways, and the number is
            not portable between them. It is 0.0 on this airframe.
        keep: 72-element mask from :func:`sector_keep_mask`. Beams landing in a
            masked sector are dropped, leaving it ``DISTANCE_UNKNOWN``. ``None``
            keeps the full circle, which is the pre-mask behaviour.

    Returns:
        A 72-element list of centimetre distances. Sectors with no valid
        return hold DISTANCE_UNKNOWN. Where several beams land in one sector,
        the closest wins — the conservative choice for avoidance.
    """
    distances = [DISTANCE_UNKNOWN] * SECTOR_COUNT

    for angle_deg, distance_m in scan:
        if distance_m is None or distance_m <= 0:
            continue

        distance_cm = int(distance_m * 100)
        if distance_cm < min_distance_cm or distance_cm > max_distance_cm:
            continue

        corrected = (angle_deg + angle_offset_deg) % 360.0
        sector = int(corrected / SECTOR_WIDTH_DEG) % SECTOR_COUNT

        # Masked before the range compare, so a rear return can never win a
        # sector it is not allowed to report in.
        if keep is not None and not keep[sector]:
            continue

        if distance_cm < distances[sector]:
            distances[sector] = distance_cm

    return distances


def to_pymavlink_connection(connection: str) -> tuple[str, int | None]:
    """Translate a MAVSDK-style connection string into pymavlink's dialect.

    MAVSDK and pymavlink do not agree on connection-string syntax, and the two
    live side by side in this stack (MAVSDK flies, pymavlink sends
    OBSTACLE_DISTANCE). Passing a MAVSDK string straight to pymavlink silently
    misroutes: pymavlink treats *any* device containing a colon as UDP, so
    ``serial:///dev/ttyACM0:115200`` becomes a bogus UDP connection rather than
    a serial one.

    Returns (device, baud) where baud is None for non-serial transports.

        serial:///dev/ttyACM0:115200 -> ("/dev/ttyACM0", 115200)
        udp://:14540                 -> ("udpin:0.0.0.0:14540", None)
        udp://127.0.0.1:14550        -> ("udpout:127.0.0.1:14550", None)
        tcp://:5760                  -> ("tcp:127.0.0.1:5760", None)
    """
    if connection.startswith("serial://"):
        rest = connection[len("serial://") :]
        device, _, baud = rest.rpartition(":")
        if device and baud.isdigit():
            return device, int(baud)
        return rest, None

    if connection.startswith("udp://"):
        host, _, port = connection[len("udp://") :].rpartition(":")
        # No host means "bind here and listen"; a host means "send there".
        if not host:
            return f"udpin:0.0.0.0:{port}", None
        return f"udpout:{host}:{port}", None

    if connection.startswith("tcp://"):
        host, _, port = connection[len("tcp://") :].rpartition(":")
        # pymavlink's TCP transport is always a client, so an omitted host
        # means the local SITL instance rather than a wildcard bind.
        return f"tcp:{host or '127.0.0.1'}:{port}", None

    # Already a pymavlink-native string (or a bare device path).
    return connection, None


def parse_measurement_nodes(buffer: bytes) -> tuple[list[tuple[bool, float, float]], bytes]:
    """Parse 5-byte legacy measurement nodes out of a raw serial buffer.

    Returns (nodes, leftover) where each node is
    (is_scan_start, angle_deg, distance_m). Resyncs automatically if the buffer
    starts mid-packet.
    """
    nodes: list[tuple[bool, float, float]] = []
    i, n = 0, len(buffer)

    while i + 5 <= n:
        b0, b1 = buffer[i], buffer[i + 1]
        # The start flag and its inverse must differ, and the check bit must be
        # set. If not, we are mid-packet — advance one byte and resync.
        if (b0 & 1) == ((b0 >> 1) & 1) or not (b1 & 1):
            i += 1
            continue

        b2, b3, b4 = buffer[i + 2], buffer[i + 3], buffer[i + 4]
        is_start = bool(b0 & 1)
        angle_deg = ((b1 >> 1) | (b2 << 7)) / 64.0
        distance_m = (b3 | (b4 << 8)) / 4.0 / 1000.0

        if angle_deg <= 360.0:
            nodes.append((is_start, angle_deg, distance_m))
        i += 5

    return nodes, buffer[i:]


class RPLidarC1:
    """Minimal raw-serial driver for the Slamtec RPLIDAR C1."""

    def __init__(self, port: str, baud: int = 460800, timeout: float = 0.5) -> None:
        self.port = port
        self.baud = baud
        self.timeout = timeout
        self._ser = None

    def _cmd(self, byte: int) -> None:
        self._ser.write(b"\xA5" + bytes([byte]))

    def open(self) -> None:
        import serial  # lazy import: hardware-only dependency

        self.close()  # idempotent; guarantees no stale fd on reconnect
        logger.info("opening RPLIDAR C1 on %s @ %d", self.port, self.baud)
        self._ser = serial.Serial(self.port, self.baud, timeout=self.timeout)

        self._cmd(_CMD_STOP)          # silence any free-running scan
        time.sleep(0.2)
        self._ser.reset_input_buffer()

        try:
            self._ser.setDTR(False)   # DTR low spins the scan motor
        except Exception:  # noqa: BLE001 - not all adapters expose DTR
            pass

        self._cmd(_CMD_SCAN)
        descriptor = self._ser.read(7)
        if len(descriptor) != 7 or descriptor[0] != 0xA5 or descriptor[1] != 0x5A:
            raise RuntimeError(f"bad scan descriptor: {descriptor.hex()}")

        logger.info("RPLIDAR C1 scanning (motor spin-up %.0fs)", _MOTOR_SPINUP_S)
        time.sleep(_MOTOR_SPINUP_S)
        self._ser.reset_input_buffer()

    def iter_scans(self, stop_event: threading.Event):
        """Yield complete revolutions as lists of (angle_deg, distance_m)."""
        buffer = b""
        current: list[tuple[float, float]] = []

        while not stop_event.is_set():
            chunk = self._ser.read(2048)
            if not chunk:
                continue

            nodes, buffer = parse_measurement_nodes(buffer + chunk)
            for is_start, angle_deg, distance_m in nodes:
                if is_start and current:
                    yield current
                    current = []
                current.append((angle_deg, distance_m))
                if len(current) > 2500:  # motor stalled / garbage — drop it
                    current = []

    def close(self) -> None:
        if self._ser is not None:
            try:
                self._cmd(_CMD_STOP)
                self._ser.setDTR(True)  # stop the motor
                self._ser.close()
            except Exception:  # noqa: BLE001
                pass
            self._ser = None


class LidarBridge:
    def __init__(
        self,
        mavlink_connection: str,
        lidar_port: str,
        lidar_baud: int = 460800,
        min_distance_m: float = 0.15,
        max_distance_m: float = 12.0,
        angle_offset_deg: float = 0.0,
        rate_hz: float = 10.0,
        source_system: int = 255,
        source_component: int = 195,  # MAV_COMP_ID_OBSTACLE_AVOIDANCE
        fov_enabled: bool = True,
        fov_deg: float = DEFAULT_FOV_DEG,
    ) -> None:
        self.mavlink_connection = mavlink_connection
        self.lidar_port = lidar_port
        self.lidar_baud = lidar_baud
        self.min_distance_cm = int(min_distance_m * 100)
        self.max_distance_cm = int(max_distance_m * 100)
        self.angle_offset_deg = angle_offset_deg
        self.interval_s = 1.0 / rate_hz
        self.source_system = source_system
        self.source_component = source_component
        self.fov_enabled = bool(fov_enabled)
        self.fov_deg = float(fov_deg)
        # Built once: the mask is pure geometry and does not change in flight.
        self._keep = sector_keep_mask(self.fov_deg, self.fov_enabled)
        kept = sum(self._keep)
        if kept < SECTOR_COUNT:
            logger.info(
                "lidar FOV mask: %.0f deg window, %d/%d sectors reported, "
                "rear %.0f deg left unknown",
                self.fov_deg,
                kept,
                SECTOR_COUNT,
                360.0 - self.fov_deg,
            )
        else:
            logger.warning(
                "lidar FOV mask DISABLED - the full 360 deg is streamed to the FC, "
                "including the airframe's own tail"
            )

        self._mav = None
        self._lidar: RPLidarC1 | None = None
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None

    # ------------------------------------------------------------ connections

    def connect_mavlink(self) -> None:
        # Imported lazily so scan_to_distances() stays testable without pymavlink.
        from pymavlink import mavutil  # noqa: PLC0415

        device, baud = to_pymavlink_connection(self.mavlink_connection)
        logger.info("lidar bridge connecting to %s (baud=%s)", device, baud)
        kwargs: dict = {
            "source_system": self.source_system,
            "source_component": self.source_component,
        }
        if baud is not None:
            kwargs["baud"] = baud
        self._mav = mavutil.mavlink_connection(device, **kwargs)
        # Bounded: an unbooted or bootloader-mode FC must not wedge the thread.
        if self._mav.wait_heartbeat(timeout=15) is None:
            self._close_mavlink()
            raise RuntimeError(f"no MAVLink heartbeat from {device} within 15s")
        logger.info("lidar bridge got heartbeat from autopilot")

    def _close_mavlink(self) -> None:
        if self._mav is not None:
            try:
                self._mav.close()
            except Exception:  # noqa: BLE001
                pass
            self._mav = None

    # --------------------------------------------------------------- sending

    def send_obstacle_distance(self, distances: Sequence[int]) -> None:
        if self._mav is None:
            raise RuntimeError("MAVLink connection not established")

        try:
            self._mav.mav.obstacle_distance_send(
                int(time.time() * 1e6),          # time_usec
                MAV_DISTANCE_SENSOR_LASER,       # sensor_type
                list(distances),                 # distances (72, cm)
                0,                               # increment (deprecated, use increment_f)
                self.min_distance_cm,            # min_distance (cm)
                self.max_distance_cm,            # max_distance (cm)
                float(SECTOR_WIDTH_DEG),         # increment_f (deg)
                0.0,                             # angle_offset (deg)
                MAV_FRAME_BODY_FRD,              # frame
            )
        except OSError:
            # The autopilot's fd went stale — almost always a USB bus reset
            # took the Pixhawk down with the lidar. Drop it so _run() rebuilds.
            self._close_mavlink()
            raise

    # ----------------------------------------------------------------- runner

    def _run(self) -> None:
        while not self._stop.is_set():
            try:
                # A USB over-current trip re-enumerates every device on the bus,
                # the Pixhawk included, leaving a stale MAVLink fd behind. Rebuild
                # it here so a brown-out costs seconds, not the rest of the flight.
                if self._mav is None:
                    self.connect_mavlink()

                self._lidar = RPLidarC1(self.lidar_port, self.lidar_baud)
                self._lidar.open()

                last_send = 0.0
                for scan in self._lidar.iter_scans(self._stop):
                    if self._stop.is_set():
                        break
                    now = time.time()
                    # 10% tolerance. The C1's native revolution rate (10 Hz) is
                    # exactly the default target, so an exact comparison drops
                    # every other scan on jitter and halves the output rate.
                    if now - last_send < self.interval_s * 0.9:
                        continue
                    last_send = now

                    distances = scan_to_distances(
                        scan,
                        self.min_distance_cm,
                        self.max_distance_cm,
                        self.angle_offset_deg,
                        keep=self._keep,
                    )
                    self.send_obstacle_distance(distances)
            except Exception:  # noqa: BLE001 - the bridge must always recover
                logger.exception("lidar bridge error, reconnecting in 2s")
            finally:
                if self._lidar is not None:
                    self._lidar.close()
                    self._lidar = None
            if not self._stop.is_set():
                self._stop.wait(2.0)

    def start(self) -> None:
        try:
            self.connect_mavlink()
        except Exception:  # noqa: BLE001
            # Not fatal: _run() retries every 2s. Failing here would abort the
            # whole flight stack just because the FC was still booting.
            logger.exception("initial MAVLink connect failed, will retry in thread")
        self._stop.clear()
        self._thread = threading.Thread(target=self._run, name="lidar_bridge", daemon=True)
        self._thread.start()
        logger.info("lidar bridge started")

    def stop(self) -> None:
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=10.0)
            self._thread = None
        self._close_mavlink()
        logger.info("lidar bridge stopped")
