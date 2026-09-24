"""LiDAR interface abstraction + real RPLIDAR C1 implementation.

``LidarInterface`` is shared by the real driver and the simulated LiDAR
(:class:`drone_stack.sim.mock_lidar.MockLidar`). Scans are normalised into a
fixed-resolution :class:`~drone_stack.msg.LaserScan` (default 360 beams, one per
degree) so every downstream consumer sees a stable layout.

The RPLIDAR reports its beam angle **clockwise** looking down on the unit,
while :class:`~drone_stack.msg.LaserScan` - and everything built on it -
measures **counter-clockwise** (x forward, y left). ``lidar.clockwise``
performs that handedness conversion; without it every return lands on the
wrong side and the radar, the obstacle bearings and the avoidance direction
all come out mirrored left/right.

Beam 0 is the LiDAR's **front reference** (the face pointing at the nose of
the airframe, see ``lidar.angle_offset_deg``). :class:`FovMask` keeps only the
270 deg arc centred on it - 135 deg left, 135 deg right - and blanks the 90 deg
wedge directly behind, which is permanently occupied by the airframe and
whatever the drone is parked next to.

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
from drone_stack.utils.geometry import wrap_180, wrap_360
from drone_stack.utils.logging_setup import get_logger

DEFAULT_BINS = 360
# Total arc kept, centred on the LiDAR's front reference (see FovMask).
DEFAULT_FOV_DEG = 250.0

#: Minimum per-sample quality the C1 must report for a return to be believed.
#:
#: Chosen from a measured survival curve, not taste (bench, 2026-09-21, 202
#: revolutions, 64314 valid samples). For each candidate threshold: how much of
#: the signal from solid bins survives, versus how much of the flicker does, and
#: how many flickering bins are left afterwards.
#:
#:   thresh   solid kept   flicker kept   flicker bins
#:        2       100.0%          83.2%             28
#:        3        99.9%          60.7%             16
#:        4        99.8%          54.2%             11   <- shipped
#:        6        95.1%          43.4%             17
#:        8        86.2%          38.4%             18
#:       10        72.1%          37.8%             33
#:
#: 4 is the knee: essentially all of the real signal for a two-thirds cut in
#: flickering bins. ⚠️ Raising it further is NOT safer, and the table shows why
#: - past the knee the gate starts destroying returns off real surfaces, those
#: bins stop being solid, and the flicker-bin count climbs back to where it
#: started. Same shape as the VFH safety radius in CLAUDE.md 12c: bigger blocks
#: more of the picture without buying margin.
#:
#: This gate halves the noise but does not remove it - 11 flickering bins
#: survive it - which is why ScanFilter exists as a second, different-in-kind
#: stage. Set 0 to disable.
DEFAULT_QUALITY_MIN = 4

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


class FovMask:
    """Front-referenced angular window - which beams of a scan are usable.

    The LiDAR is mounted with one face pointing at the nose of the airframe.
    **That face is the 0 deg reference**; ``lidar.angle_offset_deg`` rotates the
    device's raw angles onto it, so by the time a beam reaches this mask its
    angle is already measured from the front line, positive to the left.

    ``lidar.fov_deg`` (default 270) is the total arc kept, centred on that
    reference: 135 deg to the left and 135 deg to the right. The remaining
    ``360 - fov_deg`` (default 90 deg) - the wedge directly behind the aircraft
    - is blanked to ``inf``. That sector permanently contains the airframe's own
    tail and whatever the drone is standing next to; those returns never move,
    so feeding them to ObstacleNode makes the avoidance logic brake for
    obstacles that are effectively bolted to the aircraft.

    Blanking happens *here*, in the interface, so every consumer downstream -
    PointCloud, ObstacleNode, NavigationNode avoidance and the GCS radar - sees
    the same view and no path can accidentally re-admit the rear sector.

    Beams landing exactly on the +/-135 deg boundary are kept (a closed
    window), so a 360-bin scan keeps 271 bins and blanks the 89 that fall
    strictly inside the 90 deg blind wedge.
    """

    _EPS = 1e-9

    def __init__(self, config: dict[str, Any] | None = None) -> None:
        cfg = config or {}
        self.enabled = bool(cfg.get("fov_enabled", True))
        fov = float(cfg.get("fov_deg", DEFAULT_FOV_DEG))
        self.fov_deg = min(360.0, max(0.0, fov))
        if self.fov_deg >= 360.0:
            self.enabled = False       # a full circle is not a mask
        self.half_deg = self.fov_deg / 2.0
        self.blind_deg = 360.0 - self.fov_deg
        self._cache: dict[int, list[bool]] = {}

    def keeps(self, angle_deg: float) -> bool:
        """True if a beam ``angle_deg`` off the front line is inside the FOV."""
        if not self.enabled:
            return True
        return abs(wrap_180(angle_deg)) <= self.half_deg + self._EPS

    def keep_bins(self, bins: int) -> list[bool]:
        """Per-bin keep flags for a ``bins``-beam scan (beam i at i*360/bins)."""
        cached = self._cache.get(bins)
        if cached is None:
            width = 360.0 / bins
            cached = [self.keeps(i * width) for i in range(bins)]
            self._cache[bins] = cached
        return cached

    def apply(
        self, ranges: list[float], intensities: list[float] | None = None
    ) -> None:
        """Blank every out-of-FOV beam of an assembled scan, in place."""
        if not self.enabled:
            return
        for i, ok in enumerate(self.keep_bins(len(ranges))):
            if not ok:
                ranges[i] = math.inf
                if intensities is not None:
                    intensities[i] = 0.0

    def describe(self) -> dict[str, Any]:
        """FOV geometry for diagnostics and the GCS radar overlay (degrees)."""
        return {
            "enabled": self.enabled,
            "fov_deg": self.fov_deg if self.enabled else 360.0,
            "half_deg": self.half_deg if self.enabled else 180.0,
            "blind_deg": self.blind_deg if self.enabled else 0.0,
        }



class ScanFilter:
    """Temporal persistence gate - rejects returns that do not come back.

    The C1's noise on this airframe is **not** range jitter. Measured on the
    bench (2026-09-21, 202 revolutions at 10.1 Hz): bins that a real surface
    occupies report a median range spread of 2.4 cm, and only 0.1% of
    consecutive-frame pairs move further than ``obstacles.cluster_gap_m``. The
    ranges are excellent.

    What the sensor does instead is **flicker**: 12.6% of the bins that ever
    hold a return hold one in a quarter of revolutions or fewer, and 1.6% of
    all returns are isolated single bins with both angular neighbours empty.
    Those are what reach ObstacleNode as clusters, get a track, and steer the
    aircraft. So the instrument that fixes it is persistence, not smoothing -
    smoothing would average a range that is already good.

    ``lidar.quality_min`` removes roughly half of those samples at source
    (see ``RealLidar._to_laserscan``). This stage removes what survives: a
    return is published only if the recent past **corroborates** it.

    Why corroboration is angular, not per-bin
    -----------------------------------------
    ⚠️ A strict per-bin median is a pole-deleter, and a static bench cannot
    show you that. ``avoidance_steer_rate_deg_s`` is 10.0 deg/s, so across a
    depth-3 window at 10 Hz the airframe rotates 3 deg - three 1 deg bins. A
    0.3 m pole at 8 m subtends 2.1 deg, about two bins. It therefore slides
    out of its own bin between revolutions while remaining a perfectly real
    obstacle, and a per-bin test would erase it exactly when the aircraft is
    turning - which is when it is avoiding something.

    ``angular_tol_bins`` is the width of that grace, and ``range_tol_m``
    (defaulting to the clusterer's own ``cluster_gap_m``) is what keeps the
    grace honest: a neighbouring bin only corroborates if it agrees about the
    *distance* too, so a near return is never propped up by a far wall three
    bins over.

    Direction matters as well. Like ``proximity_node.SectorFilter``, this
    filter is asymmetric: something closing is the case worth spending
    latency on, something receding is not. With ``fast_approach`` a return
    that one previous revolution already corroborates *and* that is closer
    than the gate would otherwise publish goes out immediately, at the
    conservative (farther) of the pair.

    The filter can only ever **suppress** a return - it never invents range
    or clearance. Its failure mode is therefore late, not blind, and the
    numbers above size that lateness: worst case ``min_support`` revolutions,
    200 ms at 10 Hz, 0.20 m at the 1.0 m/s cruise, against an avoidance band
    that opens at 10.0 m.
    """

    def __init__(self, config: dict[str, Any] | None = None) -> None:
        cfg = config or {}
        self.enabled = bool(cfg.get("filter_enabled", True))
        #: previous revolutions retained as evidence
        self.depth = max(1, int(cfg.get("filter_depth", 3)))
        #: how many of them must corroborate before a return is published
        self.min_support = max(1, int(cfg.get("filter_min_support", 2)))
        #: angular grace, in bins, for a corroborating return (see class doc)
        self.angular_tol = max(0, int(cfg.get("filter_angular_tol_bins", 3)))
        #: a corroborating return must also agree about distance
        self.range_tol = float(cfg.get("filter_range_tol_m", 0.30))
        #: publish a corroborated closing return without waiting for the gate
        self.fast_approach = bool(cfg.get("filter_fast_approach", True))
        self._history: deque[list[float]] = deque(maxlen=self.depth)
        #: last-scan counters, surfaced through LidarNode diagnostics
        self.passed = 0
        self.rejected = 0

    def reset(self) -> None:
        self._history.clear()
        self.passed = 0
        self.rejected = 0

    def _corroborations(self, idx: int, rng: float, bins: int) -> int:
        """How many retained revolutions back up a return of ``rng`` at ``idx``."""
        votes = 0
        for frame in self._history:
            for off in range(-self.angular_tol, self.angular_tol + 1):
                other = frame[(idx + off) % bins]
                if math.isfinite(other) and abs(other - rng) <= self.range_tol:
                    votes += 1
                    break
        return votes

    def _nearest_within_tol(
        self, frame: list[float], idx: int, bins: int
    ) -> float:
        """Closest finite return in ``frame`` within ``angular_tol`` of ``idx``."""
        best = math.inf
        for off in range(-self.angular_tol, self.angular_tol + 1):
            value = frame[(idx + off) % bins]
            if math.isfinite(value) and value < best:
                best = value
        return best

    def _is_supported(self, idx: int, rng: float, bins: int) -> bool:
        """Decide whether history corroborates this return well enough to publish.

        Returns True to publish the beam, False to blank it to ``inf``.

        Available state:
          ``self._history``     - up to ``self.depth`` previous scans, oldest
                                  first, each a full ``bins``-long range list
                                  (``math.inf`` where that bin was empty).
          ``self._corroborations(idx, rng, bins)`` - how many of those
                                  revolutions hold a finite return within
                                  ``self.angular_tol`` bins of ``idx`` whose
                                  range is within ``self.range_tol`` of ``rng``.
          ``self.min_support``, ``self.depth``, ``self.fast_approach``.
        """
        if self._corroborations(idx, rng, bins) >= self.min_support:
            return True
        if not self.fast_approach:
            return False
        # Fast path for something closing. Only the NEWEST revolution says
        # anything about direction, so only it is consulted. It has to
        # corroborate on its own terms - within range_tol, not merely hold
        # some return at this bearing - or a wall at 10 m would vouch for a
        # speckle at 2 m sitting in front of it. Combined with `rng < prev`
        # that is the SectorFilter rule: seen twice and getting nearer
        # publishes now instead of waiting for a third revolution. One frame
        # on its own still never passes.
        prev = self._nearest_within_tol(self._history[-1], idx, bins)
        return math.isfinite(prev) and 0.0 < prev - rng <= self.range_tol

    def apply(self, ranges: list[float]) -> list[float]:
        """Return a filtered copy of ``ranges``; record this scan as evidence."""
        if not self.enabled:
            return ranges
        bins = len(ranges)
        if len(self._history) < self.depth:
            # Warming up. Pass the raw scan through so the radar and the
            # avoidance are fed from the first revolution rather than staring
            # at an empty world for depth frames; it settles within ~300 ms.
            self._history.append(list(ranges))
            self.passed = sum(1 for r in ranges if math.isfinite(r))
            self.rejected = 0
            return ranges

        out = build_empty_ranges(bins)
        passed = rejected = 0
        for idx, rng in enumerate(ranges):
            if not math.isfinite(rng):
                continue
            if self._is_supported(idx, rng, bins):
                out[idx] = rng
                passed += 1
            else:
                rejected += 1
        self._history.append(list(ranges))
        self.passed, self.rejected = passed, rejected
        return out

    def describe(self) -> dict[str, Any]:
        """Filter geometry + last-scan counters, for diagnostics and the GCS."""
        return {
            "enabled": self.enabled,
            "depth": self.depth,
            "min_support": self.min_support,
            "angular_tol_bins": self.angular_tol,
            "range_tol_m": self.range_tol,
            "fast_approach": self.fast_approach,
            "passed": self.passed,
            "rejected": self.rejected,
        }

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
        # True for every Slamtec unit: the device numbers its beams clockwise
        # looking down on it, the stack works counter-clockwise. See
        # _to_laserscan for why this is applied before the mounting offset.
        self._clockwise: bool = bool(config.get("clockwise", True))
        self._invert: bool = bool(config.get("invert", False))
        # Per-sample confidence gate. The C1 reports a 6-bit quality with every
        # measurement node; until 2026-09-21 the driver parsed it, stored it in
        # `intensities` and never acted on it. Measured on the bench over 202
        # revolutions, it separates signal from noise cleanly: samples landing
        # in bins a real surface occupies (>=90% of revolutions) have a median
        # quality of 25, while samples in flickering bins (<=25%) have a median
        # of 2. See DEFAULT_QUALITY_MIN for how the threshold was chosen.
        self._quality_min: int = int(config.get("quality_min", DEFAULT_QUALITY_MIN))
        self._bins = DEFAULT_BINS
        # 270 deg front-referenced window; the rear 90 deg never enters a scan.
        self._fov = FovMask(config)

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
        keep = self._fov.keep_bins(self._bins)
        for quality, angle_deg, distance_mm in measurements:
            if quality < self._quality_min:
                # Weak return - a glint, dust, a beam clipping an edge, or the
                # tail of a surface seen at a steep angle. Rejected here, at the
                # sample, so it never even competes for a bin.
                continue
            # Handedness first: negate the device's clockwise reading to get
            # the stack's counter-clockwise angle. THEN rotate by the mounting
            # offset - the offset names a direction in the device's own frame
            # (the raw angle that points at the nose), so it must not be
            # flipped along with the beam.
            angle = -angle_deg if self._clockwise else angle_deg
            angle += self._offset_deg
            if self._invert:
                angle = -angle
            idx = int(wrap_360(angle) / bin_width) % self._bins
            if not keep[idx]:
                # Rear blind sector - dropped at the driver so no consumer
                # (cloud, obstacles, avoidance, radar) can ever see it.
                continue
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
