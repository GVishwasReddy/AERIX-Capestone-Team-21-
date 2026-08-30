"""ProximityNode - streams OBSTACLE_DISTANCE so the FC can avoid for itself.

Everything else in this stack avoids obstacles from the *ground* side: the
navigator watches ``/obstacles`` and brakes or sidesteps by commanding the
aircraft. That only works while the navigator is the one flying. The moment the
pilot takes the sticks, the navigator stands down - by design, because a GCS
that keeps commanding through a pilot takeover is what put this airframe in the
ground on 2026-08-22 (see tests/test_transmitter_authority.py).

So for avoidance to survive a manual flight it has to run *on the flight
controller*. ArduPilot already has that: give its proximity library a distance
picture and it limits the pilot's own stick demand, refusing to let them fly
into something while never taking the aircraft away from them. This node
supplies that picture and nothing else - it makes no decisions and sends no
flight commands.

Filtering (added 2026-08-29 after the first manual flight)
----------------------------------------------------------
The first version handed the FC the single closest beam in each 5 degree
sector, straight off each revolution. That is the most twitch-prone thing you
can feed an avoidance controller: one stray return - dust, a glint off a wet
surface, a thin wire, a beam catching the frame - reads as a solid wall for one
frame, and the aircraft lurches. The flight was, in the pilot's words, awful.

Two filters fix it, and they are deliberately different in kind:

* **Spatially**, a sector must contain at least ``min_points`` returns, and the
  ``min_points``-th closest one sets the distance. At 1 degree binning a
  5 degree sector holds at most 5 beams, so the default of 2 means a lone
  outlier can never set a sector while a real surface - which lights up every
  beam that crosses it - still reports immediately.

* **Temporally**, each sector is the median of the last ``filter_depth``
  revolutions. A median (not a mean) is what makes a one-frame spike vanish
  entirely rather than be averaged in. At depth 3 and 10 Hz an obstacle has to
  be seen in 2 of 3 revolutions, which costs about 200 ms of latency - cheap
  against a 2 m avoidance margin, and the difference between a steady hover and
  an oscillation.

Both are conservative in the safe direction: they can only ever delay or
suppress a *reaction*, never invent clearance where the sensor saw something
solid twice running.

Required ArduPilot parameters (the FC does nothing with this stream until they
are set)::

    PRX1_TYPE     2      # proximity source = MAVLink (needs an FC REBOOT)
    AVOID_ENABLE  3      # use fence + proximity
    AVOID_MARGIN  2      # metres to hold off obstacles
    AVOID_BEHAVE  1      # 1 = stop dead (calmer), 0 = slide along
    OA_TYPE       0      # no path planner - this is what keeps yaw untouched

Avoidance applies in the pilot-assisted modes - LOITER, POSHOLD, ALT_HOLD - and
in AUTO/GUIDED. It does *not* apply in STABILIZE or ACRO, which have no
position or velocity controller to limit.

Why this node rather than parcel_delivery/pi/lidar_bridge.py, which sends the
same message: only one process can hold the LiDAR's serial port, and the GCS
stack already has it. Reading the scan off the bus keeps a single owner, keeps
the GCS radar live, and - the point of the exercise - inherits the driver's
front-referenced window for free instead of re-deriving it.
"""
from __future__ import annotations

import math
import threading
from collections import deque

from drone_stack.bus import MessageBus
from drone_stack.bus.topics import Topics
from drone_stack.interfaces.lidar_interface import FovMask
from drone_stack.msg import LaserScan, NavCommand
from drone_stack.utils.config import Config
from drone_stack.utils.geometry import wrap_180, wrap_360
from drone_stack.utils.node import NodeBase

#: OBSTACLE_DISTANCE carries a fixed 72-element array, 5 degrees per sector,
#: measured CLOCKWISE from the nose in MAV_FRAME_BODY_FRD.
SECTOR_COUNT = 72
SECTOR_WIDTH_DEG = 360.0 / SECTOR_COUNT
MAV_FRAME_BODY_FRD = 12

#: UINT16_MAX. ArduPilot ignores any sector outside [min_distance, max_distance],
#: so this reads as "nothing measured here" rather than "nothing is here".
DISTANCE_UNKNOWN = 65535

#: Boundary tolerance when bucketing a beam, in sector units.
_EPS = 1e-9


def sector_keep_mask(fov: FovMask) -> list[bool]:
    """Which of the 72 sectors fall inside the scanned window.

    Judged on each sector's centre, so the kept set is exactly symmetric about
    the nose: a 250 deg window keeps 50 sectors (0-24 and 47-71) and leaves the
    22 sectors of the rear 110 deg permanently unknown.
    """
    keep: list[bool] = []
    for i in range(SECTOR_COUNT):
        centre = i * SECTOR_WIDTH_DEG + SECTOR_WIDTH_DEG / 2.0
        keep.append(fov.keeps(wrap_180(centre)))
    return keep


def scan_to_sectors(
    scan: LaserScan,
    min_cm: int,
    max_cm: int,
    keep: list[bool] | None = None,
    min_points: int = 1,
) -> list[int]:
    """Bucket a LaserScan into the OBSTACLE_DISTANCE array.

    The scan is counter-clockwise (x forward, y left) and OBSTACLE_DISTANCE
    runs clockwise from the nose, so the beam angle is negated on the way in.
    Getting this backwards mirrors the picture the FC avoids against, which
    would make it dodge *into* obstacles.

    ``min_points`` is the spatial noise gate: a sector needs that many returns
    before it reports at all, and the ``min_points``-th closest sets its
    distance. At 1 the closest single beam wins, which is what made the first
    manual flight unflyable.
    """
    buckets: list[list[int]] = [[] for _ in range(SECTOR_COUNT)]
    for i, rng in enumerate(scan.ranges):
        if rng is None or not math.isfinite(rng):
            continue
        cm = int(rng * 100)
        if cm < min_cm or cm > max_cm:
            continue
        # Beam angle from the index, per the LaserScan contract, rather than
        # accumulating the increment: 360 additions of a radian increment drift
        # far enough to land a beam one sector off at the boundaries. _EPS then
        # settles the beams that still fall exactly on a boundary.
        a = scan.angle_min + i * scan.angle_increment
        deg = wrap_360(-math.degrees(a))
        sector = int(deg / SECTOR_WIDTH_DEG + _EPS) % SECTOR_COUNT
        if keep is not None and not keep[sector]:
            continue
        buckets[sector].append(cm)

    need = max(1, int(min_points))
    distances = [DISTANCE_UNKNOWN] * SECTOR_COUNT
    for sector, values in enumerate(buckets):
        if len(values) < need:
            continue
        values.sort()
        distances[sector] = values[need - 1]
    return distances


class SectorFilter:
    """Per-sector median over the last N revolutions, biased toward approach.

    Median rather than mean: a single frame of noise is discarded outright
    instead of being blended into the output. A sector has to be occupied in
    more than half the retained frames before the FC ever sees it as close.

    The median is symmetric, though, and the two directions are not equally
    urgent. Something getting *closer* is the case worth spending latency on;
    something getting *further away* is not, and smoothing it costs nothing.
    With ``fast_approach`` the filter is therefore asymmetric: a sector that two
    consecutive revolutions agree has closed in is published immediately, while
    anything receding still has to win the median before the FC believes it.

    Two consecutive frames, not one, is the point. A single close reading is
    exactly the stray return that made the aircraft oscillate on the first
    manual flight, and it still gets rejected. Requiring the pair - and
    publishing the *farther* of the two - keeps that rejection while cutting a
    revolution off the reaction to a real obstacle.
    """

    def __init__(self, depth: int = 3, fast_approach: bool = True) -> None:
        self.depth = max(1, int(depth))
        self.fast_approach = bool(fast_approach)
        self._frames: deque[list[int]] = deque(maxlen=self.depth)

    def _median(self) -> list[int]:
        mid = len(self._frames) // 2
        return [
            sorted(frame[sector] for frame in self._frames)[mid]
            for sector in range(SECTOR_COUNT)
        ]

    def update(self, distances: list[int]) -> list[int]:
        self._frames.append(list(distances))
        if self.depth == 1 or len(self._frames) < self.depth:
            # Warming up. Passing the raw frame through keeps the FC fed from
            # the first revolution; it settles within depth frames (~300 ms).
            return list(distances)

        median = self._median()
        if not self.fast_approach or len(self._frames) < 2:
            return median

        previous = self._frames[-2]
        out: list[int] = []
        for sector in range(SECTOR_COUNT):
            raw = distances[sector]
            prev = previous[sector]
            if raw != DISTANCE_UNKNOWN and prev != DISTANCE_UNKNOWN:
                # The conservative of the pair: both revolutions have to be
                # closer than the median for this to fire, and the farther
                # reading is the one published.
                corroborated = max(raw, prev)
                if corroborated < median[sector]:
                    out.append(corroborated)
                    continue
            out.append(median[sector])
        return out

    def reset(self) -> None:
        self._frames.clear()


class ProximityNode(NodeBase):
    """Publishes the LiDAR scan to the FC as OBSTACLE_DISTANCE."""

    def __init__(self, bus: MessageBus, config: Config) -> None:
        section = config.section("proximity")
        super().__init__("proximity", bus, config, rate_hz=section.get("rate_hz", 10))
        self._min_cm = int(float(section.get("min_distance_m", 0.30)) * 100)
        self._max_cm = int(float(section.get("max_distance_m", 12.0)) * 100)
        self._min_points = int(section.get("min_points", 2))
        self._filter = SectorFilter(
            int(section.get("filter_depth", 3)),
            fast_approach=bool(section.get("fast_approach", True)),
        )

        # Second line of defence on the rear wedge. The driver already blanks
        # it, so in normal operation this changes nothing - but the flight
        # controller is the one consumer that can act on this data without a
        # human in the loop, and it must never be handed a reading from the
        # sector the airframe permanently parks its own clutter in.
        self._fov = FovMask(config.section("lidar"))
        self._keep = sector_keep_mask(self._fov)

        self._latest: LaserScan | None = None
        self._lock = threading.Lock()
        self._sent = 0
        self.subscribe(Topics.SCAN, self._on_scan)

    def on_start(self) -> None:
        kept = sum(self._keep)
        self.log.info(
            "proximity -> FC: %d/%d sectors live (%.0f deg), %d masked (%.0f deg); "
            "noise gate %d returns/sector, median of %d revolutions; "
            "needs PRX1_TYPE=2 and AVOID_ENABLE on the FC to have any effect",
            kept, SECTOR_COUNT, kept * SECTOR_WIDTH_DEG,
            SECTOR_COUNT - kept, (SECTOR_COUNT - kept) * SECTOR_WIDTH_DEG,
            self._min_points, self._filter.depth,
        )

    def _on_scan(self, msg) -> None:
        if isinstance(msg, LaserScan):
            with self._lock:
                self._latest = msg

    def step(self) -> None:
        with self._lock:
            scan = self._latest
            self._latest = None          # only ever send a fresh revolution
        if scan is None or scan.count == 0:
            return
        raw = scan_to_sectors(
            scan, self._min_cm, self._max_cm, self._keep, self._min_points
        )
        distances = self._filter.update(raw)
        self.publish(
            Topics.MAVLINK_CMD,
            NavCommand(
                command="obstacle_distance",
                params={
                    "distances": distances,
                    "min_cm": self._min_cm,
                    "max_cm": self._max_cm,
                    "increment_deg": SECTOR_WIDTH_DEG,
                    "frame": MAV_FRAME_BODY_FRD,
                },
            ),
        )
        self._sent += 1
        if self._sent % 100 == 1:
            live = sum(1 for d in distances if d != DISTANCE_UNKNOWN)
            noisy = sum(1 for d in raw if d != DISTANCE_UNKNOWN) - live
            self.log.debug(
                "OBSTACLE_DISTANCE #%d, %d sectors with returns (%d filtered out)",
                self._sent, live, noisy,
            )
