"""§2.4 - descent-abort monitor.

Tracks the confirmed recipient's ground displacement across frames and
signals an abort if either of two independent conditions holds (both are
checked every frame; either alone is sufficient to abort):

1. **velocity-above-threshold AND altitude-below-ceiling** - the tracked
   recipient is moving briskly enough, close enough to touchdown, that
   landing on/near them becomes a real risk. Both conditions are required,
   not either (see ``config/novelty/motion_monitor.yaml``'s own comment) -
   a moving recipient high above touchdown still has time to be re-tracked
   before descent continues, so altitude gates the check.
2. **zone-intrusion** - a DIFFERENT, untracked person enters within
   ``zone_intrusion_radius_m`` of the landing zone centroid during descent,
   regardless of the recipient's own velocity or the current altitude - an
   unrelated bystander wandering into the touchdown footprint is its own
   hazard, independent of the confirmed recipient's motion.

## Track association

``MotionMonitor`` owns ``PersonDetection.track_id`` assignment (see
``types.py``'s own docstring: "filled in by MotionMonitor's tracker") -
nearest-neighbour, frame-to-frame, gated by ``max_association_distance_m``
so two different people are never merged into one track just because a
frame was missed. An unmatched detection starts a new track; a track with
no match this frame is kept (not immediately dropped, so one missed
detection doesn't lose the recipient's identity) but produces no fresh
position until it is matched again.

## Velocity estimation

"simple average displacement over the window, not a single frame-to-frame
delta" (the shipped YAML's own comment) - net displacement between the
OLDEST and NEWEST position currently held (up to ``track_history_len``
frames) divided by the elapsed time between them. This smooths detector
jitter that a raw last-two-frames delta would amplify. No estimate
(``None``) until at least ``min_track_frames`` positions are on record for
that track - see ``docs/novelty/motion_monitor.md``.
"""
from __future__ import annotations

from collections import deque

from drone_stack.novelty.config import MotionMonitorConfig
from drone_stack.novelty.types import GroundPoint, MotionAbortEvent, PersonDetection


class MotionMonitor:
    def __init__(self, cfg: MotionMonitorConfig) -> None:
        self._cfg = cfg
        self._next_track_id = 1
        self._history: dict[int, deque[tuple[float, GroundPoint]]] = {}

    def reset(self) -> None:
        """Clear every track - call when starting a new recipient handoff."""
        self._next_track_id = 1
        self._history.clear()

    def update(self, detections: list[PersonDetection]) -> list[PersonDetection]:
        """Associate this frame's ground-projected detections to existing
        tracks (nearest match within ``max_association_distance_m``),
        assigning ``track_id`` on each detection IN PLACE and appending its
        position to that track's history. Returns the same list. Detections
        with no ground position are left untouched (``track_id`` stays
        whatever it was) - there is no position to track."""
        grounded = [d for d in detections if d.ground is not None]
        unmatched_track_ids = set(self._history)

        for d in grounded:
            best_id: int | None = None
            best_dist: float | None = None
            for track_id in unmatched_track_ids:
                _, last_ground = self._history[track_id][-1]
                dist = last_ground.distance_to(d.ground)
                if dist <= self._cfg.max_association_distance_m and (best_dist is None or dist < best_dist):
                    best_id, best_dist = track_id, dist

            if best_id is None:
                best_id = self._next_track_id
                self._next_track_id += 1
                self._history[best_id] = deque(maxlen=self._cfg.track_history_len)

            unmatched_track_ids.discard(best_id)
            d.track_id = best_id
            self._history[best_id].append((d.stamp, d.ground))

        return detections

    def velocity_mps(self, track_id: int) -> float | None:
        """Average displacement rate over the held history window, or
        ``None`` if the track is unknown or has fewer than
        ``min_track_frames`` positions on record yet."""
        history = self._history.get(track_id)
        if history is None or len(history) < self._cfg.min_track_frames:
            return None
        oldest_stamp, oldest_ground = history[0]
        newest_stamp, newest_ground = history[-1]
        elapsed = newest_stamp - oldest_stamp
        if elapsed <= 0:
            return None
        return oldest_ground.distance_to(newest_ground) / elapsed

    def check_velocity_abort(self, track_id: int, altitude_m: float) -> MotionAbortEvent | None:
        if altitude_m > self._cfg.abort_altitude_ceiling_m:
            return None
        velocity = self.velocity_mps(track_id)
        if velocity is None or velocity <= self._cfg.max_recipient_velocity_mps:
            return None
        return MotionAbortEvent(
            reason="recipient_velocity", track_id=track_id,
            velocity_mps=velocity, altitude_m=altitude_m,
        )

    def check_zone_intrusion(
        self,
        detections: list[PersonDetection],
        recipient_track_id: int,
        landing_zone_centroid: GroundPoint,
    ) -> MotionAbortEvent | None:
        """First OTHER tracked person (not ``recipient_track_id``) found
        within ``zone_intrusion_radius_m`` of the landing zone centroid, or
        ``None`` if the zone is clear."""
        for d in detections:
            if d.track_id == recipient_track_id or d.ground is None:
                continue
            dist = d.ground.distance_to(landing_zone_centroid)
            if dist <= self._cfg.zone_intrusion_radius_m:
                return MotionAbortEvent(
                    reason="zone_intrusion", track_id=d.track_id,
                    intruder_distance_m=dist,
                )
        return None

    def evaluate(
        self,
        detections: list[PersonDetection],
        recipient_track_id: int,
        altitude_m: float,
        landing_zone_centroid: GroundPoint,
    ) -> list[MotionAbortEvent]:
        """One call per frame: update tracks, then run both abort checks.
        Returns every triggered event (0, 1, or both can fire the same
        frame) - the caller (``delivery_node.py``) logs and acts on each."""
        detections = self.update(detections)
        events: list[MotionAbortEvent] = []

        velocity_event = self.check_velocity_abort(recipient_track_id, altitude_m)
        if velocity_event is not None:
            events.append(velocity_event)

        intrusion_event = self.check_zone_intrusion(detections, recipient_track_id, landing_zone_centroid)
        if intrusion_event is not None:
            events.append(intrusion_event)

        return events
