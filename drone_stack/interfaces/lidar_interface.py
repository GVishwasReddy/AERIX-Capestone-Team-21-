"""LiDAR interface abstraction + real RPLIDAR C1 implementation.

``LidarInterface`` is shared by the real driver and the simulated LiDAR
(:class:`drone_stack.sim.mock_lidar.MockLidar`). Scans are normalised into a
fixed-resolution :class:`~drone_stack.msg.LaserScan` (default 360 beams, one per
degree) so every downstream consumer sees a stable layout.

The ``rplidar`` package is imported lazily so simulation never needs it.
"""
from __future__ import annotations

import abc
import math
import threading
import time
from collections import deque
from typing import Any

from drone_stack.msg import LaserScan
from drone_stack.utils.geometry import wrap_360
from drone_stack.utils.logging_setup import get_logger

DEFAULT_BINS = 360

# The C1's CP210x USB-serial link occasionally reports "readable but returned no
# data" when the Pi is under heavy IRQ/CPU pressure (e.g. the Hailo NPU driver's
# per-frame kernel-warning storm). Those are transient, not an unplug, so we
# absorb a burst of them - flushing and retrying - before declaring the link
# dead. ~20 x 50 ms ~= 1 s of tolerance: long enough to ride out a hiccup, short
# enough to still recover quickly from a genuine disconnect.
_READ_RETRY_LIMIT = 20
_READ_RETRY_SLEEP = 0.05


class LidarInterface(abc.ABC):
    """Abstract LiDAR. Implementations must survive unplug/replug via reconnect."""

    @property
    @abc.abstractmethod
    def connected(self) -> bool:
        ...

    @abc.abstractmethod
    def connect(self) -> bool:
        """Attempt to (re)connect and start scanning. Never raises."""

    @abc.abstractmethod
    def close(self) -> None:
        ...

    @abc.abstractmethod
    def read_scan(self) -> LaserScan | None:
        """Return the next full scan, or ``None`` if none is available."""


def build_empty_ranges(bins: int) -> list[float]:
    return [math.inf] * bins


class RealLidar(LidarInterface):
    """Driver for a physical Slamtec RPLIDAR C1 over USB serial."""

    def __init__(self, config: dict[str, Any]) -> None:
        self.log = get_logger("lidar.driver")
        self._cfg = config
        self._port: str = config.get("port", "/dev/ttyUSB0")
        self._baud: int = int(config.get("baud", 460800))
        self._frame_id: str = config.get("frame_id", "lidar_link")
        self._min_range: float = float(config.get("min_range_m", 0.15))
        self._max_range: float = float(config.get("max_range_m", 12.0))
        self._offset_deg: float = float(config.get("angle_offset_deg", 0.0))
        self._invert: bool = bool(config.get("invert", False))
        self._bins = DEFAULT_BINS

        # raw-serial driver state (the pip `rplidar` lib cannot drive the C1)
        self._ser = None
        self._reader: threading.Thread | None = None
        self._stop_evt = threading.Event()
        self._scans: deque = deque(maxlen=3)
        self._buf = b""
        self._cur: list = []
        self._connected = False
        self.info: dict[str, Any] = {}
        self.health: tuple[str, int] = ("Unknown", 0)

    @property
    def connected(self) -> bool:
        return self._connected

    def _cmd(self, byte: int) -> None:
        self._ser.write(b"\xA5" + bytes([byte]))

    def connect(self) -> bool:
        """Open the C1 and start streaming.

        The Slamtec RPLIDAR C1 is not supported by the legacy ``rplidar`` PyPI
        package (its ``get_health`` handshake fails with "Descriptor length
        mismatch" because the C1 free-runs its scan on power-up and floods the
        serial buffer). We speak the SLAMTEC serial protocol directly instead:
        STOP + flush to silence the free-run, enable the motor (DTR low),
        request a standard scan, then parse the 5-byte legacy measurement nodes
        on a dedicated reader thread.
        """
        try:
            import serial  # lazy import (hardware-only dep)
        except Exception:  # noqa: BLE001
            self.log.error("pyserial not installed - install requirements-hardware.txt")
            return False
        # Always release any previous handle first. A reconnect that re-opened
        # the port while the old fd was still open produced two openers on
        # /dev/ttyUSB0 -> "device reports readiness to read but returned no data
        # (... multiple access on port?)" and a permanent flap. close() is
        # idempotent, so this is a no-op on the very first connect.
        self.close()
        try:
            self.log.info("opening RPLIDAR C1 on %s @ %d", self._port, self._baud)
            self._ser = serial.Serial(self._port, self._baud, timeout=0.5)
            self._cmd(0x25)                      # STOP any auto/prior scan
            time.sleep(0.2)
            self._ser.reset_input_buffer()       # flush the free-run flood
            try:
                self._ser.setDTR(False)          # enable scan motor
            except Exception:  # noqa: BLE001
                pass
            self._cmd(0x20)                      # request standard scan
            desc = self._ser.read(7)
            if len(desc) != 7 or desc[0] != 0xA5 or desc[1] != 0x5A:
                raise RuntimeError(f"bad scan descriptor: {desc.hex()}")
            self.health = ("Good", 0)
            self.log.info("RPLIDAR C1 scanning (motor spin-up)")
            time.sleep(5.0)                      # motor needs several s to reach speed
            self._ser.reset_input_buffer()
            self._buf = b""
            self._cur = []
            self._scans.clear()
            self._stop_evt.clear()
            self._reader = threading.Thread(
                target=self._read_loop, name="lidar-reader", daemon=True
            )
            self._reader.start()
            self._connected = True
            return True
        except Exception:  # noqa: BLE001
            self.log.exception("failed to open RPLIDAR C1")
            self.close()
            return False

    def _read_loop(self) -> None:
        """Continuously parse 5-byte legacy nodes and group them into scans."""
        fails = 0
        while not self._stop_evt.is_set():
            try:
                chunk = self._ser.read(2048)
                fails = 0
            except Exception:  # noqa: BLE001 - transient hiccup or real unplug
                fails += 1
                if fails <= _READ_RETRY_LIMIT:
                    # Likely a transient "readable but no data" under CPU/IRQ
                    # load - flush and keep the link instead of tearing it down.
                    try:
                        self._ser.reset_input_buffer()
                    except Exception:  # noqa: BLE001
                        pass
                    if self._stop_evt.wait(_READ_RETRY_SLEEP):
                        return
                    continue
                self.log.warning(
                    "lidar read failed %dx in a row - marking disconnected", fails
                )
                self._connected = False
                return
            if not chunk:
                continue
            buf = self._buf + chunk
            i, n = 0, len(buf)
            while i + 5 <= n:
                b0, b1 = buf[i], buf[i + 1]
                # start-flag must be complementary to its inverse and the
                # check-bit must be 1, otherwise we are mid-packet: resync.
                if (b0 & 1) == ((b0 >> 1) & 1) or not (b1 & 1):
                    i += 1
                    continue
                b2, b3, b4 = buf[i + 2], buf[i + 3], buf[i + 4]
                start = b0 & 1
                quality = b0 >> 2
                angle = ((b1 >> 1) | (b2 << 7)) / 64.0
                dist_mm = (b3 | (b4 << 8)) / 4.0
                if start and self._cur:            # new revolution -> commit scan
                    self._scans.append(self._cur)
                    self._cur = []
                if angle <= 360.0:
                    self._cur.append((float(quality), angle, dist_mm))
                if len(self._cur) > 2500:          # safety: motor stalled
                    self._cur = []
                i += 5
            self._buf = buf[i:]

    def close(self) -> None:
        self._connected = False
        self._stop_evt.set()
        if self._reader is not None:
            self._reader.join(timeout=1.0)
            self._reader = None
        if self._ser is not None:
            try:
                self._cmd(0x25)                  # STOP
            except Exception:  # noqa: BLE001
                pass
            try:
                self._ser.setDTR(True)           # motor off
            except Exception:  # noqa: BLE001
                pass
            try:
                self._ser.close()
            except Exception:  # noqa: BLE001
                pass
            self._ser = None
        self._scans.clear()

    def read_scan(self) -> LaserScan | None:
        if not self._connected or not self._scans:
            return None
        # Return the freshest completed scan and drop any backlog so the radar
        # never lags behind the sensor (deque holds newest-last).
        try:
            measurements = self._scans[-1]
        except IndexError:
            return None
        self._scans.clear()
        return self._to_laserscan(measurements)

    def _to_laserscan(self, measurements) -> LaserScan:
        ranges = build_empty_ranges(self._bins)
        intensities = [0.0] * self._bins
        bin_width = 360.0 / self._bins
        for quality, angle_deg, distance_mm in measurements:
            angle = angle_deg + self._offset_deg
            if self._invert:
                angle = -angle
            idx = int(wrap_360(angle) / bin_width) % self._bins
            rng = distance_mm / 1000.0
            if rng < self._min_range or rng > self._max_range:
                continue
            # keep the nearest return that falls in this bin
            if rng < ranges[idx]:
                ranges[idx] = rng
                intensities[idx] = float(quality)
        return LaserScan(
            frame_id=self._frame_id,
            angle_min=0.0,
            angle_max=2.0 * math.pi,
            angle_increment=math.radians(bin_width),
            range_min=self._min_range,
            range_max=self._max_range,
            ranges=ranges,
            intensities=intensities,
        )
