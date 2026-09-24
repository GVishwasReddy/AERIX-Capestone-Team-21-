"""The replay must show the LATEST flight - not one from days earlier.

The bug these cover (2026-09-23)
--------------------------------
The operator opened REPLAY after the 22 Sep flights and got a 21 Sep bench
clip. Every flight had been recorded; none ever reached latest.mp4:

* The stabilised render runs at ~5.8x real time (22 s clip -> 130 s). Between
  flights the Pi was re-armed or power-cycled first, every time.
* Nothing was fsync'd. 88 MB dirty after 22 s, 0 written back - a battery pull
  lost the job.json and the boot sweep then deleted the frames.
* A deferred OLDER render could land after a newer flight's and os.replace
  over it; boot adoption ordered jobs by mtime on a Pi with no RTC.
* min_seconds=15 judged ARMED time, so a 13 s real hop was thrown away.
(The RC motor e-stop half of the fix lives in hub.py / flight_state.py and
is tested alongside them.)

Every test here was run against the code with its fix patched out first and
confirmed to fail (aerix notes, section 8).
"""
from __future__ import annotations

import json
import os
import queue
import time
from pathlib import Path

import numpy as np

from drone_stack.gcs.recorder import FlightRecorder, _Preempted, _SENTINEL


def _rec(tmp_path: Path, **over) -> FlightRecorder:
    settings = {"enabled": True, "dir": str(tmp_path), "min_seconds": 15.0}
    settings.update(over)
    return FlightRecorder(settings)


def _kept_clip(tmp_path: Path, seq: int, body: bytes = b"NEWER-FLIGHT") -> None:
    (tmp_path / "latest.mp4").write_bytes(body)
    (tmp_path / "latest.json").write_text(json.dumps({
        "name": "latest.mp4", "seq": seq, "duration_s": 30.0, "stabilized": True,
    }))


def _flight(tmp_path: Path, stem: str, seq: int, frames: int = 4,
            with_job: bool = True, torn_tail: bool = False) -> tuple[Path, Path]:
    """A recording with a real index: frame i is 100 bytes at offset 100*i."""
    src = tmp_path / f"{stem}.mjpg"
    index = tmp_path / f"{stem}.jsonl"
    body = b"".join(bytes([65 + i]) * 100 for i in range(frames))
    lines = [json.dumps({"hdr": {"fps": 30.0}, "seq": seq})]
    lines += [json.dumps({"o": 100 * i, "n": 100, "t": 10.0 + i * 0.5, "A": None})
              for i in range(frames)]
    if torn_tail:
        body += b"Z" * 40                       # half a frame reached the card
        lines.append(json.dumps({"o": 100 * frames, "n": 100, "t": 99.0}))
        lines.append('{"o": 5')                 # and a torn index line
    src.write_bytes(body)
    index.write_text("\n".join(lines) + "\n")
    if with_job:
        (tmp_path / f"{stem}.job.json").write_text(json.dumps({
            "src": src.name, "index": index.name, "frames": frames,
            "duration": 30.0, "fps": 30.0, "reason": "armed", "size": None,
            "seq": seq,
        }))
    return src, index


def _stub_encoders(rec: FlightRecorder, seen: list, hd_fails: bool = False) -> None:
    def quick(src, tmp, fps):
        tmp.write_bytes(b"QUICK")
        seen.append("quick")
        return 1152, 648

    def render(src, index, tmp, tmp_poster, frames):
        # Stage 2 starts only AFTER the quick clip is the replay.
        seen.append(("render-sees", (rec.dir / "latest.mp4").read_bytes()))
        if hd_fails:
            raise RuntimeError("ffmpeg died")
        tmp.write_bytes(b"HD")
        return {"width": 1920, "height": 1080, "fps": 30.0}

    rec._ffmpeg_quick = quick
    rec._render = render


class TestAnOlderFlightNeverReplacesANewerOne:

    def test_a_late_older_render_does_not_overwrite_the_newer_clip(self, tmp_path):
        """THE regression test. The 22 Sep sequence: flight #4's render was
        deferred, flight #5 landed and became the clip, then #4's render ran
        on the next disarm and os.replace()d #5 away."""
        src, index = _flight(tmp_path, "flight-old", seq=4)
        rec = _rec(tmp_path)                    # #4 is owed; nothing newer yet
        rec._pending = None
        # ...and while #4's render is running, #5 lands and becomes the clip.
        # (Adoption at boot has its own check; this is the in-flight race.)
        _kept_clip(tmp_path, seq=5)
        rec._load_clip()
        _stub_encoders(rec, [])
        rec._encode_and_promote(src, index, None, None, 4, 30.0, 30.0, "armed",
                                tmp_path / "flight-old.job.json", seq=4)
        assert (tmp_path / "latest.mp4").read_bytes() == b"NEWER-FLIGHT", \
            "an older flight's render replaced the newer flight's clip"
        assert json.loads((tmp_path / "latest.json").read_text())["seq"] == 5
        assert not (tmp_path / "flight-old.job.json").exists(), \
            "the stale job was left for the next boot to adopt again"

    def test_keeping_a_new_flight_supersedes_an_older_owed_render(self, tmp_path):
        """stop() of a kept flight must drop what is still owed from an older
        one, on disk too - it can only ever overwrite the new clip."""
        _flight(tmp_path, "flight-old", seq=3)
        rec = _rec(tmp_path)
        assert rec._pending is not None
        started: list = []
        rec._encode_and_promote = lambda *a, **k: started.append(k.get("seq"))
        rec.start("armed")
        for _ in range(3):
            rec.offer(b"\xff\xd8jpeg\xff\xd9", 64, 48)
            time.sleep(0.02)
        rec.note_altitude(4.0)
        rec.stop("disarmed")
        assert rec._pending is None, "the older flight is still queued"
        assert not (tmp_path / "flight-old.job.json").exists()
        assert not (tmp_path / "flight-old.mjpg").exists()
        assert started and started[0] > 3, "the new flight did not get a newer seq"

    def test_boot_adoption_orders_by_seq_not_mtime(self, tmp_path):
        """No RTC: the clock steps backwards at boot, so the NEWER flight can
        carry the OLDER mtime. mtime picked the wrong one."""
        _flight(tmp_path, "flight-b", seq=9)
        _flight(tmp_path, "flight-a", seq=8)
        past = time.time() - 3600
        os.utime(tmp_path / "flight-b.job.json", (past, past))
        rec = _rec(tmp_path)
        assert rec._pending is not None
        assert rec._pending["src"] == "flight-b.mjpg", "adopted the older flight"
        assert not (tmp_path / "flight-a.job.json").exists()


class TestTheLatestFlightIsShownQuickly:

    def test_the_quick_clip_is_the_replay_before_the_hd_render_runs(self, tmp_path):
        """The stabilised render alone never finished between 22 Sep flights.
        The quick clip must already BE the replay while it runs."""
        src, index = _flight(tmp_path, "flight-new", seq=2)
        _kept_clip(tmp_path, seq=1, body=b"OLD")
        rec = _rec(tmp_path)
        rec._pending = None
        seen: list = []
        _stub_encoders(rec, seen)
        rec._encode_and_promote(src, index, None, None, 4, 30.0, 30.0, "armed",
                                tmp_path / "flight-new.job.json", seq=2)
        assert seen[0] == "quick"
        assert seen[1] == ("render-sees", b"QUICK"), \
            "the HD render ran while the replay still showed the previous flight"
        assert (tmp_path / "latest.mp4").read_bytes() == b"HD"
        clip = json.loads((tmp_path / "latest.json").read_text())
        assert clip["seq"] == 2 and clip["stabilized"] is True
        assert not (tmp_path / "flight-new.job.json").exists()
        assert not src.exists() and not index.exists()

    def test_a_failed_hd_render_keeps_the_quick_clip(self, tmp_path):
        """The flight is already the replay. A stage-2 fault must not salvage
        it away or fall back to showing the previous flight."""
        src, index = _flight(tmp_path, "flight-new", seq=2)
        rec = _rec(tmp_path)
        rec._pending = None
        _stub_encoders(rec, [], hd_fails=True)
        rec._encode_and_promote(src, index, None, None, 4, 30.0, 30.0, "armed",
                                tmp_path / "flight-new.job.json", seq=2)
        assert (tmp_path / "latest.mp4").read_bytes() == b"QUICK"
        assert json.loads((tmp_path / "latest.json").read_text())["seq"] == 2
        assert not list(tmp_path.glob("failed-*")), "a playable flight was salvaged"
        assert not (tmp_path / "flight-new.job.json").exists()

    def test_a_preempted_hd_render_remembers_the_quick_clip_exists(self, tmp_path):
        """Re-arming during stage 2 must not redo stage 1, and must not flag
        the flight as 'REPLAY OWED' - it is already the replay."""
        src, index = _flight(tmp_path, "flight-new", seq=2)
        rec = _rec(tmp_path)
        rec._pending = None
        seen: list = []
        _stub_encoders(rec, seen)
        rec._render = lambda *a, **k: (_ for _ in ()).throw(_Preempted())
        rec._encode_and_promote(src, index, None, None, 4, 30.0, 30.0, "armed",
                                tmp_path / "flight-new.job.json", seq=2)
        assert rec._pending is not None and rec._pending["quick_done"] is True
        on_disk = json.loads((tmp_path / "flight-new.job.json").read_text())
        assert on_disk["quick_done"] is True, "a restart would redo stage 1"
        assert src.exists(), "preemption destroyed the frames"


    def test_a_preempted_render_unwinding_late_does_not_unmark_the_new_one(self, tmp_path):
        """The old render's thread finishes AFTER the next flight's render has
        started. If that clears 'busy', the next ARM no longer preempts the
        running render, and an owed job can start a second one beside it."""
        src, index = _flight(tmp_path, "flight-old", seq=1)
        rec = _rec(tmp_path)
        rec._pending = None
        with rec._lock:
            rec._busy = 1                       # the new flight's render, running
        _stub_encoders(rec, [])
        rec._render = lambda *a, **k: (_ for _ in ()).throw(_Preempted())
        rec._encode_and_promote(src, index, None, None, 4, 30.0, 30.0, "armed",
                                tmp_path / "flight-old.job.json", seq=1,
                                quick_done=True)
        assert rec._busy, "a late-unwinding render cleared the running one's busy mark"


class TestWhatCountsAsAFlight:

    def test_a_short_flight_that_left_the_ground_is_kept(self, tmp_path):
        """22 Sep 14:57: a 13 s GUIDED hop, discarded by the 15 s rule."""
        rec = _rec(tmp_path)
        src = tmp_path / "x.mjpg"
        src.write_bytes(b"x")
        assert rec._should_keep(390, 13.0, src, airborne=True)[0] is True
        assert rec._should_keep(390, 13.0, src, airborne=False)[0] is False

    def test_airborne_latches_only_while_recording_and_above_threshold(self, tmp_path):
        rec = _rec(tmp_path)
        rec.note_altitude(5.0)
        assert rec._airborne is False, "latched with no recording running"
        rec.start("armed")
        rec.note_altitude(0.6)                  # bench baro drift
        assert rec._airborne is False
        rec.note_altitude(1.4)
        rec.note_altitude(0.0)                  # landed again: still a flight
        assert rec._airborne is True
        rec.stop("disarmed")


class TestAPowerCutDoesNotLoseTheFlight:

    def test_a_recording_cut_off_mid_flight_is_recovered(self, tmp_path):
        """The Pi runs off the flight battery. No job.json, a torn last frame
        and a torn index line - but three complete frames the index vouches for."""
        src, index = _flight(tmp_path, "flight-cut", seq=6, frames=3,
                             with_job=False, torn_tail=True)
        rec = _rec(tmp_path)
        assert src.exists(), "the sweep deleted a recoverable flight"
        job = json.loads((tmp_path / "flight-cut.job.json").read_text())
        assert job["frames"] == 3 and job["seq"] == 6
        assert src.stat().st_size == 300, "the torn tail frame was left in"
        assert len(index.read_text().splitlines()) == 4   # header + 3 frames
        assert rec._pending is not None

    def test_the_job_reaches_the_card_before_stop_returns(self, tmp_path, monkeypatch):
        """Atomic is not durable: without fsync the job lived only in RAM for
        up to 30 s after disarm."""
        synced: list = []
        real = os.fsync
        monkeypatch.setattr(os, "fsync", lambda fd: (synced.append(fd), real(fd))[1])
        rec = _rec(tmp_path)
        rec._write_job(tmp_path / "flight-z.mjpg", None, None, None, 10, 20.0,
                       30.0, "armed", seq=1)
        assert (tmp_path / "flight-z.job.json").exists()
        assert synced, "the job was written without an fsync"

    def test_the_index_header_carries_the_seq(self, tmp_path):
        """Recovery reads the flight's place in the order from here - a
        recovered power-cut flight must not sort as the oldest."""
        rec = _rec(tmp_path)
        rec._geometry = {"main": [64, 48], "lores": [32, 24], "fps": 30.0}
        q = queue.Queue()
        q.put(_SENTINEL)
        rec._writer_loop(tmp_path / "f.mjpg", tmp_path / "f.jsonl", q, seq=7)
        assert json.loads((tmp_path / "f.jsonl").read_text().splitlines()[0])["seq"] == 7


class TestTheClipEndsAtTouchdown:
    """stop(reason, end_t): the hub passes touchdown + 2 s so the replay does
    not end on the aircraft idling on the pad."""

    def _stopped(self, tmp_path, end_t, frames=6):
        rec = _rec(tmp_path)
        rec._encode_and_promote = lambda *a, **k: None
        src, index = _flight(tmp_path, "flight-t", seq=1, frames=frames, with_job=False)
        (tmp_path / "flight-t.job.json").unlink(missing_ok=True)
        with rec._lock:
            rec._state = "recording"
            rec._src, rec._index, rec._queue, rec._writer = src, index, None, None
            rec._frames, rec._t0, rec._t_last = frames, 0.0, 30.0
            rec._seq_cur, rec._airborne, rec._t_wall0 = 1, True, 1000.0
        rec.stop("disarmed", end_t=end_t)
        return src, index

    def test_frames_after_end_t_are_cut_from_both_files(self, tmp_path):
        # frames at t = 10.0, 10.5, ... 12.5; keep t <= 11.2 -> 3 frames
        src, index = self._stopped(tmp_path, end_t=11.2)
        lines = index.read_text().splitlines()
        assert '"hdr"' in lines[0], "the header line was dropped"
        assert [json.loads(l)["t"] for l in lines[1:]] == [10.0, 10.5, 11.0]
        assert src.stat().st_size == 300
        job = json.loads((tmp_path / "flight-t.job.json").read_text())
        assert job["frames"] == 3
        assert abs(job["duration"] - 28.5) < 1e-6      # 30 s minus the 1.5 s cut
        assert abs(job["t_end"] - 1028.5) < 1e-6

    def test_an_end_t_that_would_erase_the_flight_is_ignored(self, tmp_path):
        """A wrong clock (wall time instead of monotonic) must never be able
        to reduce a flight to nothing."""
        src, index = self._stopped(tmp_path, end_t=5.0)
        assert src.stat().st_size == 600
        assert len(index.read_text().splitlines()) == 7
        assert json.loads((tmp_path / "flight-t.job.json").read_text())["frames"] == 6

    def test_detections_are_written_to_the_index(self, tmp_path):
        rec = _rec(tmp_path)
        rec._geometry = {"main": [64, 48], "lores": [32, 24], "fps": 30.0}
        rec._encoder = lambda: (lambda *planes: b"\xff\xd8j\xff\xd9")
        q = queue.Queue()
        planes = (np.zeros((48, 64), np.uint8),) * 3
        q.put((planes, {"t": 1.0, "A": None, "dets": [[1.0, 2.0, 3.0, 4.0, 0.9]]}))
        q.put(_SENTINEL)
        rec._writer_loop(tmp_path / "f.mjpg", tmp_path / "f.jsonl", q, seq=1)
        row = json.loads((tmp_path / "f.jsonl").read_text().splitlines()[1])
        assert row["d"] == [[1.0, 2.0, 3.0, 4.0, 0.9]]
