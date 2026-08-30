"""Frame-to-frame obstacle tracking - Phase 5b.

:class:`~drone_stack.nodes.obstacle_node.ObstacleNode` re-detects clusters from
scratch on every revolution, and an obstacle's ``id`` is its index in a list
that is re-sorted by distance. Identity therefore changes the moment anything
closer appears, so nothing downstream could tell that the cluster 2 m ahead is
the same one that was 3 m ahead a tenth of a second earlier. That is the whole
reason avoidance only ever reacted to things standing still: a moving object was
a brand-new obstacle on every scan and never accumulated a velocity.

This module associates clusters between revolutions and differentiates position.
Two different velocities come out of it, and they answer different questions:

* **Closing speed** (``closing_ms``) - how fast the gap is shrinking, measured
  in the body frame. It needs no knowledge of our own motion, it is available on
  the second revolution a track is seen, and it is what braking distance should
  be derived from: a wall we are flying at closes exactly as dangerously as a
  car driving at us.

* **World velocity** (``vx_m_s``/``vy_m_s``, ENU) - the object's own motion over
  the ground, recovered by subtracting our ego-motion from the measured
  body-frame velocity. This is what separates "a wall, and we are moving" from
  "a person walking toward us". It needs a valid :class:`FusedState`; without
  one the object is reported static rather than guessed at, because a guess here
  would label every wall dynamic the instant the aircraft translated.

Ego-motion has two parts and both matter. Translation is obvious. Rotation is
not: at 0.8 m/s an obstacle 5 m off the nose moves through the body frame faster
from a gentle yaw than from the whole airframe's forward speed, so a tracker
that ignores yaw rate reports a stationary fence post as a 2 m/s crossing
target every time the aircraft turns.

Frames used here (this stack's conventions, see ``polar_to_cartesian`` and
``ObstacleNode._describe``):
    body   x forward, y **left**; bearing_deg 0 = nose, positive to the right
    world  ENU - vx = East, vy = North
    yaw    NED heading in radians: 0 = North, positive clockwise toward East
"""
from __future__ import annotations

import math

from drone_stack.msg import FusedState, Obstacle
from drone_stack.utils.geometry import wrap_pi


def body_to_enu(x_b: float, y_b: float, yaw: float) -> tuple[float, float]:
    """Rotate a body-frame vector (x forward, y left) into ENU."""
    sin_y, cos_y = math.sin(yaw), math.cos(yaw)
    return x_b * sin_y - y_b * cos_y, x_b * cos_y + y_b * sin_y


def enu_to_body(east: float, north: float, yaw: float) -> tuple[float, float]:
    """Rotate an ENU vector into the body frame (x forward, y left)."""
    sin_y, cos_y = math.sin(yaw), math.cos(yaw)
    return east * sin_y + north * cos_y, -east * cos_y + north * sin_y


class Track:
    """One obstacle followed across revolutions."""

    __slots__ = (
        "id", "x", "y", "vx_b", "vy_b", "closing", "vx_e", "vy_n",
        "speed", "is_dynamic", "hits", "misses", "dyn_hits",
        "first_seen", "last_seen",
    )

    def __init__(self, track_id: int, x: float, y: float, now: float) -> None:
        self.id = track_id
        self.x = x                  # body frame, metres
        self.y = y
        self.vx_b = 0.0             # body-frame velocity of the *measurement*
        self.vy_b = 0.0
        self.closing = 0.0          # +ve = range shrinking, m/s
        self.vx_e = 0.0             # world ENU velocity of the object
        self.vy_n = 0.0
        self.speed = 0.0            # |world velocity|
        self.is_dynamic = False
        self.hits = 1
        self.misses = 0
        self.dyn_hits = 0           # consecutive frames measured as moving
        self.first_seen = now
        self.last_seen = now

    @property
    def age_s(self) -> float:
        return self.last_seen - self.first_seen


class ObstacleTracker:
    """Nearest-neighbour tracker over successive ObstacleArrays."""

    def __init__(
        self,
        gate_m: float = 1.2,
        max_misses: int = 3,
        min_hits: int = 2,
        alpha: float = 0.5,
        dynamic_speed_ms: float = 0.5,
        max_dt_s: float = 1.0,
        dynamic_min_hits: int = 3,
        max_speed_ms: float = 6.0,
    ) -> None:
        #: Association radius. A track and a new cluster are the same object if
        #: the cluster falls within this of where the track was predicted to be.
        #: At 10 Hz and 0.8 m/s of own speed, a static object moves 8 cm per
        #: frame, so this is loose enough for a target crossing at ~12 m/s.
        self.gate_m = float(gate_m)
        #: Revolutions a track survives unmatched before it is dropped. The
        #: LiDAR drops returns off dark or oblique surfaces for a frame at a
        #: time; killing a track on the first miss would reset its velocity.
        self.max_misses = int(max_misses)
        #: Matches before a track's velocity is published. One frame gives a
        #: position and no velocity at all; the second gives the first estimate.
        self.min_hits = int(min_hits)
        #: Smoothing on the differentiated velocity. Differentiating position
        #: amplifies range noise, so it is filtered - but only lightly, because
        #: this is the signal that is supposed to be fast.
        self.alpha = float(alpha)
        #: Own-speed threshold above which an object counts as moving.
        self.dynamic_speed_ms = float(dynamic_speed_ms)
        #: Gap after which the tracker restarts rather than differentiating
        #: across it (node stall, LiDAR restart).
        self.max_dt_s = float(max_dt_s)
        #: Consecutive frames a track must measure as moving before it is
        #: called dynamic. Clusters split and merge between revolutions where
        #: returns are sparse, and a split makes the centroid jump - which
        #: differentiates into a large one-frame velocity. A real pedestrian
        #: keeps moving; a split cluster flickers, so requiring consecutive
        #: agreement rejects it without slowing the real case much.
        self.dynamic_min_hits = int(dynamic_min_hits)
        #: Speed above which the estimate is treated as an association error
        #: rather than a target. Nothing this aircraft has to avoid at 1 m/s in
        #: a delivery yard moves faster than this, so a larger number is
        #: evidence the tracker matched two different objects - and acting on
        #: it would inflate the brake distance and stop for nothing.
        self.max_speed_ms = float(max_speed_ms)

        self._tracks: list[Track] = []
        self._next_id = 1
        self._last_t: float | None = None
        self._last_yaw: float | None = None

    @property
    def tracks(self) -> list[Track]:
        return list(self._tracks)

    def reset(self) -> None:
        self._tracks.clear()
        self._last_t = None
        self._last_yaw = None

    def update(
        self,
        obstacles: list[Obstacle],
        now: float,
        ego: FusedState | None = None,
    ) -> list[Obstacle]:
        """Associate ``obstacles`` with existing tracks and annotate them."""
        dt = 0.0 if self._last_t is None else now - self._last_t
        if dt <= 0.0 or dt > self.max_dt_s:
            # First frame, or a gap long enough that differentiating across it
            # would manufacture a velocity. Re-seed instead.
            self._reseed(obstacles, now, ego)
            return obstacles

        yaw = ego.yaw if (ego is not None and ego.valid) else None
        # Yaw *rate* between the two frames, not absolute heading: this is what
        # rotation compensation needs, and wrap_pi keeps it honest across 359->1.
        omega = 0.0
        if yaw is not None and self._last_yaw is not None:
            omega = wrap_pi(yaw - self._last_yaw) / dt

        matched = self._associate(obstacles, dt)
        for obstacle, track in matched:
            self._advance(obstacle, track, dt, now, ego, omega)

        self._retire(now)
        self._last_t = now
        self._last_yaw = yaw
        return obstacles

    # -- internals -----------------------------------------------------------
    def _reseed(
        self, obstacles: list[Obstacle], now: float, ego: FusedState | None
    ) -> None:
        self._tracks = []
        for obstacle in obstacles:
            track = Track(self._new_id(), obstacle.x_m, obstacle.y_m, now)
            self._tracks.append(track)
            obstacle.track_id = track.id
            # hits == 1: position only, no velocity yet. Everything stays at the
            # dataclass defaults (0.0 / False) rather than being invented.
        self._last_t = now
        self._last_yaw = ego.yaw if (ego is not None and ego.valid) else None

    def _new_id(self) -> int:
        track_id = self._next_id
        self._next_id += 1
        return track_id

    def _associate(
        self, obstacles: list[Obstacle], dt: float
    ) -> list[tuple[Obstacle, Track]]:
        """Greedy nearest-neighbour, closest pair first.

        Greedy rather than optimal (Hungarian): with the handful of clusters a
        250 deg scan produces, the cost of a globally optimal assignment buys
        nothing, and taking the closest pair first already prevents a distant
        cluster from stealing a track from the one sitting on top of it.
        """
        pairs: list[tuple[float, int, int]] = []
        for oi, obstacle in enumerate(obstacles):
            for ti, track in enumerate(self._tracks):
                # Predict the track forward so a fast crosser is still gated
                # against where it should be, not where it was.
                px = track.x + track.vx_b * dt
                py = track.y + track.vy_b * dt
                d = math.hypot(obstacle.x_m - px, obstacle.y_m - py)
                if d <= self.gate_m:
                    pairs.append((d, oi, ti))
        pairs.sort()

        used_o: set[int] = set()
        used_t: set[int] = set()
        matched: list[tuple[Obstacle, Track]] = []
        for _, oi, ti in pairs:
            if oi in used_o or ti in used_t:
                continue
            used_o.add(oi)
            used_t.add(ti)
            matched.append((obstacles[oi], self._tracks[ti]))

        for ti, track in enumerate(self._tracks):
            if ti not in used_t:
                track.misses += 1

        now_tracks = self._tracks
        for oi, obstacle in enumerate(obstacles):
            if oi in used_o:
                continue
            # Unmatched cluster: a new object, or one that just came back after
            # more misses than max_misses. Either way it starts fresh.
            track = Track(self._new_id(), obstacle.x_m, obstacle.y_m, 0.0)
            now_tracks.append(track)
            obstacle.track_id = track.id
        return matched

    def _advance(
        self,
        obstacle: Obstacle,
        track: Track,
        dt: float,
        now: float,
        ego: FusedState | None,
        omega: float,
    ) -> None:
        raw_vx = (obstacle.x_m - track.x) / dt
        raw_vy = (obstacle.y_m - track.y) / dt
        if track.hits <= 1:
            # First differentiation: adopt it outright. Blending it toward the
            # zero a fresh track starts at would report half the true velocity,
            # and ego-motion removal subtracts the *full* ego term - so a
            # stationary post during a yaw would come out as a fast crossing
            # target instead of cancelling to nothing.
            track.vx_b, track.vy_b = raw_vx, raw_vy
        else:
            a = self.alpha
            track.vx_b = a * raw_vx + (1.0 - a) * track.vx_b
            track.vy_b = a * raw_vy + (1.0 - a) * track.vy_b

        # Closing speed: the component of the measured body-frame velocity along
        # the line of sight, sign-flipped so +ve means the gap is shrinking.
        # This deliberately includes our own motion - it is the rate the gap
        # actually closes, which is what stopping distance depends on.
        rng = math.hypot(obstacle.x_m, obstacle.y_m)
        if rng > 1e-6:
            track.closing = -(track.vx_b * obstacle.x_m + track.vy_b * obstacle.y_m) / rng
        else:
            track.closing = 0.0

        track.x = obstacle.x_m
        track.y = obstacle.y_m
        track.hits += 1
        track.misses = 0
        track.last_seen = now
        if track.first_seen == 0.0:
            track.first_seen = now

        # Ego-motion removal -> the object's own velocity over the ground.
        if ego is not None and ego.valid:
            v_fwd, v_left = enu_to_body(ego.vx, ego.vy, ego.yaw)
            # A static object's apparent body velocity is -(v_fwd, v_left) from
            # our translation plus omega*(-y, x) from our yaw. Whatever is left
            # after removing both is the object moving under its own power.
            own_x = track.vx_b + v_fwd + omega * track.y
            own_y = track.vy_b + v_left - omega * track.x
            track.vx_e, track.vy_n = body_to_enu(own_x, own_y, ego.yaw)
            track.speed = math.hypot(track.vx_e, track.vy_n)

            # The "is it moving" counter is judged on THIS frame's raw motion,
            # not the smoothed estimate. Smoothing makes a one-frame cluster
            # jump decay over several frames (3.0 -> 1.5 -> 0.75 at alpha=0.5),
            # which would sail past a consecutive-frames test and be reported as
            # a moving object anyway. The raw value is back to zero the very
            # next frame, so an artefact scores exactly one hit and a real
            # pedestrian scores one every frame.
            raw_own_x = raw_vx + v_fwd + omega * track.y
            raw_own_y = raw_vy + v_left - omega * track.x
            raw_speed = math.hypot(raw_own_x, raw_own_y)

            if raw_speed > self.max_speed_ms or track.speed > self.max_speed_ms:
                # Implausible for anything we have to avoid: this is two
                # different objects matched to one track, not a fast one.
                # Drop the estimate rather than braking for an artefact.
                track.vx_e = track.vy_n = track.speed = 0.0
                track.dyn_hits = 0
                track.closing = 0.0
            elif raw_speed >= self.dynamic_speed_ms:
                track.dyn_hits += 1
            else:
                track.dyn_hits = 0
            track.is_dynamic = (
                track.hits >= self.min_hits
                and track.dyn_hits >= self.dynamic_min_hits
            )
        else:
            track.vx_e = track.vy_n = track.speed = 0.0
            track.dyn_hits = 0
            track.is_dynamic = False

        if track.hits >= self.min_hits:
            obstacle.closing_ms = round(track.closing, 2)
            obstacle.vx_m_s = round(track.vx_e, 2)
            obstacle.vy_m_s = round(track.vy_n, 2)
            obstacle.speed_m_s = round(track.speed, 2)
            obstacle.is_dynamic = track.is_dynamic
        obstacle.track_id = track.id
        obstacle.age_s = round(track.age_s, 2)
        obstacle.hits = track.hits

    def _retire(self, now: float) -> None:
        kept: list[Track] = []
        for track in self._tracks:
            if track.misses > self.max_misses:
                continue
            if track.first_seen == 0.0:
                track.first_seen = now
                track.last_seen = now
            kept.append(track)
        self._tracks = kept
