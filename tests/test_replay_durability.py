"""The post-flight replay must survive the GCS restarting on top of it.

The bug these cover (2026-09-19)
--------------------------------
Every armed flight WAS recorded. The footage was then destroyed, silently, in
a three-step sequence nobody had to do anything wrong to trigger::

    10:59:24  recording started (armed)
    11:00:12  recording stopped (disarmed): 1386 frames, 47.9 s - rendering
    11:01:20  bringup in 'real' mode              <- operator restarts the GCS
    11:01:20  discarded unfinished recording flight-20260919-105924.mjpg (273 MB)

``replay_render`` runs as a CHILD of the GCS, so systemd kills it with the
service; and ``_sweep_orphans`` could not tell an interrupted *recording*
(truncated, genuinely garbage) from an interrupted *render* (a complete
flight that merely has not been encoded yet). It deleted both. ``latest.mp4``
was therefore never replaced, and the dashboard spent four days offering a
22-second bench clip from 2026-09-15 in place of the flight just landed.

The fix is a ``.job.json`` sidecar written the moment a recording is judged a
keeper. Its presence is the entire distinction the sweep was missing.

Read section 8 of the aerix notes before adding to this file: a regression
test written against already-fixed code proves nothing. Every test here was
run against the BROKEN recorder first and confirmed to fail.
"""
from __future__ import annotations

import json
from pathlib import Path

import pytest

from drone_stack.gcs.recorder import FlightRecorder, _Preempted


def _wait_for(pred, timeout: float = 3.0) -> bool:
    """_maybe_render_pending hands the render to a thread, so the observable
    effect is asynchronous. Poll rather than sleep a fixed amount."""
    import time
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if pred():
            return True
        time.sleep(0.01)
    return False


def _rec(tmp_path: Path, **over) -> FlightRecorder:
    settings = {"enabled": True, "dir": str(tmp_path), "min_seconds": 15.0}
    settings.update(over)
    return FlightRecorder(settings)


def _finished_flight(tmp_path: Path, stem: str = "flight-20260919-105924",
                     with_job: bool = True, duration: float = 47.9) -> Path:
    """A recording that COMPLETED and is owed a render - the exact on-disk
    state the 11:00:12 disarm above left behind."""
    src = tmp_path / f"{stem}.mjpg"
    src.write_bytes(b"\xff\xd8\xff\xe0jpegbytes" * 64)
    (tmp_path / f"{stem}.jsonl").write_text('{"hdr": {"fps": 30.0}}\n')
    if with_job:
        (tmp_path / f"{stem}.job.json").write_text(json.dumps({
            "src": src.name, "index": f"{stem}.jsonl", "frames": 1386,
            "duration": duration, "fps": 28.9, "reason": "armed", "size": None,
        }))
    return src


class TestTheSweepKeepsWorkItDidNotFinish:
    """The load-bearing pair. One flight kept, one genuinely-broken one not."""

    def test_a_finished_flight_awaiting_render_survives_a_restart(self, tmp_path):
        """THE regression test. Against the broken sweep this frame file was
        deleted at boot and the flight was gone for good."""
        src = _finished_flight(tmp_path)
        _rec(tmp_path)  # construction runs the boot sweep
        assert src.exists(), "the sweep deleted a flight that was only owed a render"
        assert (tmp_path / "flight-20260919-105924.jsonl").exists()
        assert (tmp_path / "flight-20260919-105924.job.json").exists()

    def test_an_interrupted_recording_is_still_swept(self, tmp_path):
        """No job sidecar means the writer never reached the end: the file is
        truncated at an unknown point. That really is garbage, and letting it
        accumulate fills the card at ~11 MB per second of flight."""
        src = _finished_flight(tmp_path, stem="flight-20260919-095024",
                               with_job=False)
        _rec(tmp_path)
        assert not src.exists()
        assert not (tmp_path / "flight-20260919-095024.jsonl").exists()

    def test_a_partial_render_output_is_always_swept(self, tmp_path):
        """latest.tmp.mp4 is half an encode. The resumed render starts from
        frame zero and writes it again, so keeping it would only mislead."""
        _finished_flight(tmp_path)
        (tmp_path / "latest.tmp.mp4").write_bytes(b"\x00" * 128)
        (tmp_path / "latest.tmp.jpg").write_bytes(b"\x00" * 16)
        _rec(tmp_path)
        assert not (tmp_path / "latest.tmp.mp4").exists()
        assert not (tmp_path / "latest.tmp.jpg").exists()


class TestThePendingRenderIsAdoptedAfterARestart:

    def test_the_owed_flight_is_reported_to_the_operator(self, tmp_path):
        """The operator must never be shown an older clip as though it were
        the newest. status() carries the owed flight so the badge can say so."""
        _finished_flight(tmp_path)
        rec = _rec(tmp_path)
        pending = rec.status()["pending"]
        assert pending is not None, "a restart lost the flight silently again"
        assert pending["duration_s"] == pytest.approx(47.9, abs=0.05)
        assert pending["reason"] == "armed"

    def test_only_the_newest_owed_flight_is_kept(self, tmp_path):
        """Exactly one clip is ever kept, so only the newest job can still win
        the output file. Older ones are dead weight on the SD card."""
        old = _finished_flight(tmp_path, stem="flight-20260919-095024", duration=30.0)
        import os, time
        os.utime(tmp_path / "flight-20260919-095024.job.json", (time.time() - 600,) * 2)
        new = _finished_flight(tmp_path, stem="flight-20260919-105924")
        rec = _rec(tmp_path)
        assert new.exists()
        assert not old.exists(), "a superseded job was left filling the card"
        assert rec.status()["pending"]["duration_s"] == pytest.approx(47.9, abs=0.05)

    def test_a_job_whose_frames_are_gone_is_discarded_not_adopted(self, tmp_path):
        """Otherwise every boot adopts a job it can never satisfy."""
        _finished_flight(tmp_path)
        (tmp_path / "flight-20260919-105924.mjpg").unlink()
        rec = _rec(tmp_path)
        assert rec.status()["pending"] is None
        assert not (tmp_path / "flight-20260919-105924.job.json").exists()


class TestArmingAlwaysRecords:
    """'When armed in any mode it should record' - including while the
    previous flight is still rendering."""

    def test_arming_during_a_render_preempts_it_instead_of_refusing(self, tmp_path):
        """start() used to return ok=False 'still encoding the last clip'.
        Arming again within a minute of landing therefore recorded NOTHING,
        which is the one thing a flight recorder may never do."""
        rec = _rec(tmp_path)
        rec._state = "encoding"          # a render is in flight
        preempted = []
        rec._preempt_render = lambda: preempted.append(True)
        result = rec.start("armed")
        assert result["ok"] is True, "arming was refused while a render ran"
        assert rec._state == "recording"
        assert preempted == [True], "the render was not preempted"

    def test_a_preempted_render_keeps_its_job_for_later(self, tmp_path):
        """Preemption costs only the CPU already spent. If it cost the job the
        cure would be no better than the disease."""
        src = _finished_flight(tmp_path)
        job = tmp_path / "flight-20260919-105924.job.json"
        rec = _rec(tmp_path)
        rec._pending = None
        rec._state = "recording"         # the recording that preempted it
        rec._render = lambda *a, **k: (_ for _ in ()).throw(_Preempted())
        rec._encode_and_promote(src, tmp_path / "flight-20260919-105924.jsonl",
                                None, None, 1386, 47.9, 28.9, "armed", job)
        assert job.exists(), "preemption destroyed the job it was meant to defer"
        assert src.exists(), "preemption destroyed the frames"
        assert rec._pending is not None
        assert rec._state == "recording", "the deferred render stole the state"


class TestTheRenderPolicyIsHonoured:
    """_may_render_now is the one place that decides when spending a core on a
    replay is acceptable on an aircraft. These assert the contract the rest of
    the machinery relies on, not any particular policy beyond it."""

    def test_no_render_is_started_while_the_aircraft_is_armed(self, tmp_path):
        """The boot sweep's original comment is still right: this runs on an
        aircraft that is usually about to fly. Refusing is free - the job stays
        on disk and is retried on the next disarm."""
        _finished_flight(tmp_path)
        rec = _rec(tmp_path)
        rec._armed = True
        started = []
        rec._encode_and_promote = lambda *a, **k: started.append(a)
        rec._maybe_render_pending()
        assert not _wait_for(lambda: bool(started), timeout=0.5), \
            "a replay render was started mid-flight"
        assert rec._pending is not None, "the owed flight was dropped, not deferred"

    def test_disarming_retries_the_owed_render(self, tmp_path):
        """Disarm is the moment rendering becomes safe, so it is also the
        retry trigger - otherwise an owed flight waits for the next reboot."""
        _finished_flight(tmp_path)
        rec = _rec(tmp_path)
        rec._armed = True
        rec._maybe_render_pending()
        assert rec._pending is not None
        started: list = []
        rec._encode_and_promote = lambda *a, **k: started.append(a)
        rec.set_armed(False)
        assert _wait_for(lambda: bool(started)), \
            "disarming did not pick the owed replay back up"
        assert rec._pending is None

    def test_a_boot_that_has_never_heard_from_the_fc_does_not_render(self, tmp_path):
        """_armed is False by INITIALISATION on a fresh boot, not because
        anybody observed the aircraft to be disarmed. Reading the first as the
        second burns a core at 11:01:20 - while the aircraft may be seconds
        from arming - which is the whole scenario this module exists to
        survive. The job must be deferred, not consumed."""
        _finished_flight(tmp_path)
        started: list = []
        rec = _rec(tmp_path)
        rec._encode_and_promote = lambda *a, **k: started.append(a)
        rec._maybe_render_pending()
        assert not rec._arm_seen, "the fixture faked an arm observation"
        assert not _wait_for(lambda: bool(started), timeout=0.5), \
            "a replay render was started before the FC said anything"
        assert rec._pending is not None, "the owed flight was dropped, not deferred"
        assert (tmp_path / "flight-20260919-105924.job.json").exists(), \
            "refusing consumed the job it was meant to leave on disk"

    def test_the_first_ground_observation_releases_the_owed_render(self, tmp_path):
        """The other half of the trade. Deferring at boot is only acceptable
        because the FC repeats its arm state on every heartbeat, so the
        observation arrives about a second later and the operator gets their
        footage without flying again. If that feed is ever broken this test
        keeps passing and test_a_repeated_arm_state_still_reaches_the_recorder
        goes red - read them as a pair."""
        _finished_flight(tmp_path)
        rec = _rec(tmp_path)
        started: list = []
        rec._encode_and_promote = lambda *a, **k: started.append(a)
        rec.set_armed(False)             # the level off the next heartbeat
        assert _wait_for(lambda: bool(started)), \
            "the owed replay never started once the ground was confirmed"


class TestTheArmLevelReachesTheRecorder:
    """GcsHub._on_armed filters to EDGES, which is right for start()/stop() -
    they must fire once per transition. The recorder needs the LEVEL: the
    transition it missed while it was not running is never replayed."""

    def _stub(self, was_armed: bool):
        import types
        seen: list = []
        rec = types.SimpleNamespace(set_armed=lambda a: seen.append(a))
        return types.SimpleNamespace(_was_armed=was_armed, recorder=rec), seen

    def test_a_repeated_arm_state_still_reaches_the_recorder(self):
        """The 1 Hz repeat carries the only observation a restarted GCS will
        ever get while the aircraft sits on the ground."""
        from drone_stack.gcs.hub import GcsHub
        import types
        hub, seen = self._stub(was_armed=False)
        GcsHub._on_armed(hub, types.SimpleNamespace(armed=False))
        assert seen == [False], \
            "the heartbeat's arm level was discarded by the edge filter"

    def test_a_repeat_does_not_restart_the_recording(self):
        """...and it must stay a no-op otherwise: start() on every heartbeat
        would truncate the flight once a second."""
        from drone_stack.gcs.hub import GcsHub
        import types
        started: list = []
        hub, seen = self._stub(was_armed=True)
        hub.recorder.start = lambda r: started.append(("rec", r))
        hub.recorder.stop = lambda r: started.append(("rec", r))
        # The telemetry log is edge-driven too - restarting it once a second
        # would leave the Replay button pointing at a one-second log.
        hub.record = lambda a: started.append(("log", a))
        GcsHub._on_armed(hub, types.SimpleNamespace(armed=True))
        assert seen == [True]
        assert started == [], "a repeated heartbeat re-entered the edge path"
        assert hub._was_armed is True

    def test_the_disarm_edge_still_closes_the_clip_before_retrying(self):
        """Ordering is load-bearing: stop() queues the NEW clip, and only then
        does set_armed(False) retry anything still owed. Reversed, the restart
        renders the older flight and the one just landed waits behind it."""
        from drone_stack.gcs.hub import GcsHub
        import types
        order: list = []
        rec = types.SimpleNamespace(
            set_armed=lambda a: order.append(("set_armed", a)),
            stop=lambda r: (order.append(("stop", r)), {"ok": True, "message": ""})[1],
            start=lambda r: (order.append(("start", r)), {"ok": True, "message": ""})[1],
        )
        hub = types.SimpleNamespace(
            _was_armed=True, recorder=rec, _push_console=lambda *a: None,
            record=lambda a: order.append(("log", a)),
        )
        GcsHub._on_armed(hub, types.SimpleNamespace(armed=False))
        assert order == [("log", "stop"), ("stop", "disarmed"),
                         ("set_armed", False)], order
