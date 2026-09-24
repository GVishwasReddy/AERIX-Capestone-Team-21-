"""The replay spans ARM -> the end of the flight, and shows what YOLO saw.

* FlightEndDetector: the RC e-stop text this airframe really prints, the
  landed inference (which must never end a clip mid-flight), touchdown time.
* hub.stop_clip / GcsHub._on_armed: the clip is trimmed back to touchdown, and
  an e-stop from before the ARM cannot end the next flight.
* PersonLockPipeline: every detection is carried to the recorder (HOVER only).
* replay_render.draw_dets_bgr: those detections are burned into the replay.

Hardware-free throughout.
"""
from __future__ import annotations

import types

import numpy as np
import pytest

from drone_stack.gcs.flight_state import FlightEndDetector
from drone_stack.gcs.hub import GcsHub, stop_clip


def _tick(fe, t, *, alt=0.0, gs=0.0, vs=0.0, text="", armed=True, status=4):
    return fe.update(armed=armed, mode="STABILIZE", system_status=status,
                     altitude_m=alt, ground_speed_ms=gs, vert_speed_ms=vs,
                     fc_text=text, now=float(t))


def _armed():
    fe = FlightEndDetector({})
    fe.arm_edge(True)
    return fe


# --------------------------------------------------------------------------- #
# FlightEndDetector
# --------------------------------------------------------------------------- #
class TestEmergencyStopText:
    def test_the_switch_this_airframe_uses_ends_the_flight(self):
        """RC8_OPTION=31 prints exactly this - and none of the generic
        'emergency stop' phrases, which is why it never used to match."""
        assert _tick(_armed(), 0, alt=3.0, text="RC8: MotorEStop HIGH") == "emergency stop"

    def test_releasing_the_switch_is_not_a_stop(self):
        assert _tick(_armed(), 0, alt=3.0, text="RC8: MotorEStop LOW") is None


class TestLanded:
    def test_armed_on_the_pad_never_counts_as_landed(self):
        """Low and still is exactly what 'landed' looks like - but it has not
        flown yet. Without the airborne latch the clip ended before takeoff."""
        fe = _armed()
        assert all(_tick(fe, t, alt=0.1) is None for t in range(120))

    def test_came_down_and_sat_still_is_landed_with_touchdown_time(self):
        fe = _armed()
        for t in range(10):
            assert _tick(fe, t, alt=2.0, gs=0.5) is None
        reasons = [_tick(fe, t, alt=0.2) for t in range(10, 40)]
        # It keeps saying so until the hub calls mark_ended(); the first
        # verdict is the one that counts.
        first = next(i for i, r in enumerate(reasons) if r)
        assert reasons[first] == "landed", reasons
        assert first >= fe.land_dwell_s, "believed before the dwell was up"
        assert fe.touchdown_t == 10.0, "touchdown is where the quiet began"

    def test_a_hop_then_a_real_flight_is_not_cut_at_the_hop(self):
        """flight_20260922_150703, at 1 Hz: a 1.3 m hop, ~10.5 s sat armed on
        the ground, then the real flight to 6 m. Ending at the hop lost it."""
        fe = _armed()
        seq = [1.0, 1.3, 0.9] + [0.3] * 11 + [1.3, 3.0, 6.0] * 10
        assert all(_tick(fe, t, alt=a) is None for t, a in enumerate(seq))

    def test_touchdown_survives_mark_ended(self):
        """The hub trims with it right after the verdict; mark_ended clears
        the dwell timer and must not take touchdown with it."""
        fe = _armed()
        _tick(fe, 0, alt=2.0)
        t = 1
        while _tick(fe, t, alt=0.1) is None:
            t += 1
        fe.mark_ended()
        assert fe.touchdown_t == 1.0

    def test_the_next_arm_forgets_the_last_touchdown(self):
        fe = _armed()
        _tick(fe, 0, alt=2.0)
        for t in range(1, 30):
            _tick(fe, t, alt=0.1)
        fe.mark_ended()
        fe.arm_edge(False)
        fe.arm_edge(True)
        assert fe.touchdown_t is None and not fe.airborne and not fe.ended


# --------------------------------------------------------------------------- #
# hub
# --------------------------------------------------------------------------- #
class _Rec:
    def __init__(self):
        self.calls = []

    def stop(self, reason, end_t=None):
        self.calls.append((reason, end_t))
        return {"ok": True, "message": ""}


class _OldRec:
    """The recorder as it was before end_t existed."""

    def __init__(self):
        self.calls = []

    def stop(self, reason):
        self.calls.append(reason)
        return {"ok": True, "message": ""}


class TestStopClip:
    def _landed(self):
        fe = _armed()
        _tick(fe, 100, alt=2.0)
        for t in range(101, 130):
            if _tick(fe, t, alt=0.1):
                break
        return fe

    def test_the_clip_ends_at_touchdown_not_at_the_verdict(self):
        rec = _Rec()
        stop_clip(rec, self._landed(), "landed")
        assert rec.calls == [("landed", 101.0 + 2.0)]

    def test_still_flying_is_not_trimmed(self):
        rec, fe = _Rec(), _armed()
        _tick(fe, 0, alt=3.0, gs=2.0)
        stop_clip(rec, fe, "emergency stop")
        assert rec.calls == [("emergency stop", None)]

    def test_a_recorder_without_end_t_still_stops_once(self):
        rec = _OldRec()
        stop_clip(rec, self._landed(), "landed")
        assert rec.calls == ["landed"]


class TestArmEdge:
    def _hub(self, was_armed):
        order: list = []
        rec = types.SimpleNamespace(
            set_armed=lambda a: order.append(("set_armed", a)),
            start=lambda r: (order.append(("start", r)), {"ok": True, "message": ""})[1],
            stop=lambda r: (order.append(("stop", r)), {"ok": True, "message": ""})[1],
            is_recording=lambda: False,
        )
        fe = FlightEndDetector({})
        hub = types.SimpleNamespace(
            _was_armed=was_armed, recorder=rec, flight_end=fe,
            _fc_texts=__import__("collections").deque(maxlen=20),
            _flight_end_t=None, _push_console=lambda *a: None,
            record=lambda a: order.append(("log", a)),
        )
        return hub, order

    def test_an_estop_from_before_the_arm_cannot_end_the_next_flight(self):
        hub, _ = self._hub(was_armed=False)
        hub._fc_texts.append("RC8: MotorEStop HIGH")
        GcsHub._on_armed(hub, types.SimpleNamespace(armed=True))
        assert list(hub._fc_texts) == []

    def test_disarm_after_an_early_end_does_not_stop_twice(self):
        hub, order = self._hub(was_armed=True)
        hub.flight_end.arm_edge(True)
        hub.flight_end.mark_ended()          # the e-stop already closed it
        GcsHub._on_armed(hub, types.SimpleNamespace(armed=False))
        assert ("stop", "disarmed") not in order, order
        assert order[-1] == ("set_armed", False)


# --------------------------------------------------------------------------- #
# detections: person_lock -> recorder -> replay
# --------------------------------------------------------------------------- #
cv2 = pytest.importorskip("cv2")

from drone_stack.gcs import person_lock as pl  # noqa: E402
from drone_stack.gcs.replay_render import DET_BGR, draw_dets_bgr  # noqa: E402


class _Det:
    """Scripted NPU: each entry is a result list, or None for 'no result
    this frame' (the NPU is slower than the camera)."""

    def __init__(self, script):
        self.script = list(script)
        self._pending = None

    def start(self):
        pass

    def stop(self):
        pass

    def try_submit(self, seq, frame):
        nxt = self.script.pop(0) if self.script else None
        self._pending = None if nxt is None else (seq, nxt)
        return True

    def poll(self):
        out, self._pending = self._pending, None
        return out


def _frames(pipe, n, dt=1 / 30, motion=None):
    return [pipe.process(np.zeros((720, 1280, 3), np.uint8), motion, None, now=i * dt)
            for i in range(n)]


PERSON = (400.0, 200.0, 500.0, 500.0, 0.81)


class TestDetsRecorded:
    def test_every_detection_is_in_the_snapshot(self):
        other = (900.0, 100.0, 960.0, 300.0, 0.44)
        pipe = pl.PersonLockPipeline(_Det([[PERSON, other]] * 5), pl.TargetLock())
        pipe.enabled = True
        snaps = _frames(pipe, 4)
        assert sorted(d[4] for d in snaps[-1]["dets"]) == [0.44, 0.81]

    def test_dets_follow_the_camera_between_results(self):
        pipe = pl.PersonLockPipeline(_Det([[PERSON]] + [None] * 5), pl.TargetLock())
        pipe.enabled = True
        _frames(pipe, 2)
        shift = np.array([[1, 0, 10], [0, 1, 0]], np.float64)
        s = pipe.process(np.zeros((720, 1280, 3), np.uint8), shift, None, now=2 / 30)
        assert s["dets"][0][0] == pytest.approx(410.0)

    def test_a_det_no_newer_result_confirms_expires(self):
        pipe = pl.PersonLockPipeline(_Det([[PERSON]] + [None] * 60), pl.TargetLock())
        pipe.enabled = True
        snaps = _frames(pipe, 40)
        assert snaps[2]["dets"], "expired too early"
        assert snaps[-1]["dets"] == [], "a stale det was still being recorded"

    def test_nothing_is_recorded_outside_hover(self):
        pipe = pl.PersonLockPipeline(_Det([[PERSON]] * 5), pl.TargetLock())
        snaps = _frames(pipe, 4)                 # never enabled
        assert all(not s.get("dets") for s in snaps)


class TestDetsDrawn:
    I = np.array([[1, 0, 0], [0, 1, 0]], np.float64)

    def test_a_det_is_burned_into_the_replay_frame(self):
        img = np.zeros((720, 1280, 3), np.uint8)
        assert draw_dets_bgr(img, [[100, 100, 200, 300, 0.9]], self.I, 1.0) == 1
        assert (img[100, 150] == DET_BGR).all()

    def test_it_is_scaled_from_capture_px_to_the_record_stream(self):
        img = np.zeros((720, 1280, 3), np.uint8)
        draw_dets_bgr(img, [[50, 50, 100, 150, 0.9]], self.I, 2.0)
        assert (img[100, 150] == DET_BGR).all()

    def test_the_locked_person_is_left_to_the_lock_hud(self):
        img = np.zeros((720, 1280, 3), np.uint8)
        box = [100, 100, 200, 300]
        assert draw_dets_bgr(img, [box + [0.9]], self.I, 1.0, lock_box=box) == 0
        assert not img.any()

    def test_old_index_lines_without_dets_draw_nothing(self):
        img = np.zeros((72, 128, 3), np.uint8)
        assert draw_dets_bgr(img, None, self.I, 1.0) == 0
