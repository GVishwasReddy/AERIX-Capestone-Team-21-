"""Flight video recorder - arms with the aircraft, keeps exactly one clip.

Design, and why it is shaped this way
-------------------------------------
**In flight, store only what is cheap.** The Pi 5 has **no hardware video
encoder** (CLAUDE.md section 6), so nothing H.264 runs while the aircraft is in
the air. Two feeds exist:

* **Full-resolution record stream (default since 2026-09-15).** The camera hands
  over the ISP's 2304x1296 YUV420 frame together with the camera motion the live
  stabiliser measured and the person-lock box it drew (``offer_frame``). The
  writer JPEG-encodes the YUV planes directly (no colour conversion, ~12 ms of
  one core) and appends one JSON line per frame to an index. After disarm,
  ``replay_render`` turns that into a 1920x1080 clip stabilised with zero-phase
  smoothing - steadier than anything a live, causal filter can do - with the
  lock box burned in.
* **Stream tap (fallback).** Without a record stream the recorder appends the
  already-encoded live JPEG bytes (``offer``), exactly as before.

**The recorder is not a viewer.** It never calls ``cam.report_delivered()``.
The adaptive-bitrate controller steers by the *worst* delivered rate among open
MJPEG streams; a consumer writing to a local SD card always keeps up, so
reporting would pin the camera to rung 0 and flood the Wi-Fi link the operator
is actually watching through.

**Capture is never blocked.** ``offer``/``offer_frame`` run on the camera
capture thread. They do a non-blocking put onto a bounded queue and drop the
frame if the writer has fallen behind. A dropped frame's camera motion is folded
into the next kept frame, so the replay's motion path stays continuous.

File layout (all inside ``recording.dir``)::

    flight-<stamp>.mjpg    in progress: raw concatenated JPEGs
    flight-<stamp>.jsonl   in progress: per-frame offset/size/motion/lock box
    latest.mp4             the one kept clip
    latest.jpg             its poster frame
    latest.json            its metadata

**Exactly one clip is kept - the latest flight's.** A finished recording
replaces ``latest.mp4`` via ``os.replace``, which is atomic. A recording that
never got airborne and is shorter than ``min_seconds`` is discarded instead, so
a bench arm/disarm fumble cannot destroy a flight; one that flew is always kept.

**Two stages after disarm (2026-09-23).** A quick unstabilised clip is promoted
first (~0.8x flight length), then the stabilised 1080p render replaces it
(~6x). The stabilised render alone never finished between the 09-22 flights -
the Pi was re-armed or power-cycled first - and the replay kept showing 09-21.

**Recordings are numbered** (``seq``, persisted in ``recordings/seq`` and in
every job, index header and clip). The Pi has no RTC, so neither wall time nor
mtimes can order flights; seq can. Every promotion checks it, so an older
flight's render can never replace a newer flight's clip.

**Durable.** The frame file is fdatasync'd every ``flush_s`` while recording,
jobs and metadata are fsync'd, and a recording cut off by a power loss is
recovered from its index at boot instead of being swept.

**Post-flight rendering respects the CPU ceiling.** The renderer is niced and
paces itself to ``render_cpu_cores`` - a long flight takes longer to render
rather than pushing the Pi past the load the GCS and BLE handshake need.
"""
from __future__ import annotations

import json
import os
import queue
import shutil
import subprocess
import sys
import threading
import time
from pathlib import Path
from typing import Optional

import numpy as np

from drone_stack.utils.logging_setup import get_logger

_log = get_logger("gcs.recorder")

# Bounded so a slow SD card costs recorded frames rather than stalling the
# camera thread. 90 encoded frames is ~3 s of 720p30 headroom (~3 MB).
_QUEUE_FRAMES = 90
# Raw YUV frames are 4.5 MB each at 2304x1296, so the queue is much shorter:
# ~0.5 s of slack, ~70 MB worst case.
_QUEUE_RAW_FRAMES = 15

_SENTINEL = object()


class _Preempted(Exception):
    """The render was killed to give a recording the machine - not a fault."""
_REPO_ROOT = Path(__file__).resolve().parents[2]


def _to3(A) -> np.ndarray:
    M = np.eye(3)
    if A is not None:
        M[:2] = np.asarray(A, np.float64).reshape(2, 3)
    return M


class FlightRecorder:
    """Records the Pi camera for one armed flight and renders the replay."""

    def __init__(self, settings: Optional[dict] = None,
                 filter_cfg: Optional[dict] = None) -> None:
        settings = settings or {}
        self.enabled = bool(settings.get("enabled", True))
        self.dir = Path(settings.get("dir", "recordings"))
        # A clip shorter than this is thrown away and the previous keeper is
        # left alone. Guards against arm/disarm fumbles and motor tests.
        self.min_seconds = float(settings.get("min_seconds", 15.0))
        self._crf = int(settings.get("crf", 24))
        self._preset = str(settings.get("preset", "veryfast"))
        self._threads = int(settings.get("encode_threads", 3))
        self._max_seconds = float(settings.get("max_seconds", 1800.0))
        # Full-resolution record stream + stabilised replay render.
        self._quality = int(settings.get("quality", 82))
        self._out_w = int(settings.get("out_width", 1920))
        self._out_h = int(settings.get("out_height", 1080))
        self._sigma = float(settings.get("smooth_sigma", 5.0))
        self._max_angle = float(settings.get("max_angle_deg", 2.5))
        self._cpu_cores = float(settings.get("render_cpu_cores", 1.0))
        self._filter_cfg = dict(filter_cfg or {})
        # A recording in which the aircraft left the ground is ALWAYS kept,
        # whatever its length - "the latest flight" means the latest time it
        # flew, not the latest time it sat armed for 15 s. min_seconds now only
        # decides for recordings that never got airborne (bench arms). 1 m, not
        # less: barometric relative altitude drifts ~0.5 m on a bench.
        self._airborne_alt = float(settings.get("airborne_alt_m", 1.0))
        # Stage 1: a quick, unstabilised clip promoted straight after disarm
        # so the replay shows THIS flight within about one flight-length.
        # Measured 2026-09-23 on a 20 s bench clip: 15.3 s (0.77x real time)
        # at these settings, against 130 s for the capped stabilised render -
        # which no flight since 09-21 ever stayed powered long enough to finish.
        self._quick = bool(settings.get("quick_clip", True))
        self._quick_w = int(settings.get("quick_width", 1152))
        self._quick_h = int(settings.get("quick_height", 648))
        self._quick_preset = str(settings.get("quick_preset", "veryfast"))
        self._quick_crf = int(settings.get("quick_crf", 23))
        self._quick_threads = int(settings.get("quick_threads", 2))
        # Dirty-page bound while recording. The kernel's own expiry is 30 s,
        # measured: 88 MB dirty and 0 KB written back after 22 s of recording.
        # A battery pull lost everything in that window.
        self._flush_s = float(settings.get("flush_s", 2.0))

        self._lock = threading.RLock()
        self._state = "idle"          # idle | recording | encoding | ready | error
        self._message = ""
        self._reason = ""
        self._progress = 0.0
        self._src: Optional[Path] = None
        self._index: Optional[Path] = None
        self._queue: Optional[queue.Queue] = None
        self._writer: Optional[threading.Thread] = None
        self._t0 = 0.0
        self._t_last = 0.0
        self._frames = 0
        self._dropped = 0
        self._bytes = 0
        self._first_jpeg: Optional[bytes] = None
        self._size: Optional[tuple[int, int]] = None
        self._clip: Optional[dict] = None
        self._geometry: Optional[dict] = None
        self._pending_motion: Optional[np.ndarray] = None
        # A finished recording still owing a render. Durable: it is backed by a
        # .job.json on disk, so it survives the GCS restart that used to
        # destroy it (see _adopt_pending).
        self._pending: Optional[dict] = None
        # The running replay_render child, so a recording can preempt it.
        self._render_proc: Optional[subprocess.Popen] = None
        self._preempt = threading.Event()
        # Latest ARM state, fed by GcsHub off the same bus edge that starts and
        # stops recordings. Read by _may_render_now.
        self._armed = False
        # _armed is a GUESS until the FC tells us otherwise. Without
        # _arm_seen there is no way to distinguish "never heard from the
        # flight controller" from "observed disarmed on the ground", and
        # treating the first as the second starts a render during boot -
        # which is the 11:01:20 restart this module exists to survive.
        self._arm_seen = False
        # Monotonic recording number, persisted in the recordings dir. The Pi
        # has NO RTC: its clock can step backwards at boot, so wall time and
        # file mtimes cannot order flights. seq can, and every promotion is
        # checked against it - an older flight can never replace a newer one.
        self._seq = 0
        self._seq_cur = 0             # seq of the recording in progress
        # Seq of the newest recording that stop() decided to KEEP. Any job or
        # render below it is superseded: rendering it could only ever replace
        # a newer flight with an older one (the 2026-09-22 replay bug).
        self._floor = 0
        self._airborne = False
        # Encode threads running (either stage) - a count, so a preempted
        # thread finishing late cannot clear it for a newer one. Separate from
        # _state because stage 2 runs while the quick clip is already "ready".
        self._busy = 0
        self._upgrading: Optional[float] = None
        self._t_wall0 = 0.0

        self.dir.mkdir(parents=True, exist_ok=True)
        self._attach_file_log()
        self._load_clip()
        self._sweep_orphans()
        self._adopt_pending()

    # -- paths ---------------------------------------------------------------
    @property
    def _mp4(self) -> Path:
        return self.dir / "latest.mp4"

    @property
    def _poster(self) -> Path:
        return self.dir / "latest.jpg"

    def _job_path(self, src: Path) -> Path:
        """Sidecar marking a recording as COMPLETE and owed a render.

        Its presence is the whole difference between an interrupted *recording*
        (garbage - the file is truncated and its tail is unknown) and an
        interrupted *render* (a finished flight that merely has not been
        encoded yet). Without it the boot sweep cannot tell them apart, and
        every GCS restart between flights threw the footage away.
        """
        return self.dir / f"{src.stem}.job.json"

    def _poster_sidecar(self, src: Path) -> Path:
        """First JPEG of a stream-tap recording, kept beside the job so the
        fallback poster survives a restart too. The stabilised render writes
        its own poster and never needs this."""
        return self.dir / f"{src.stem}.poster.jpg"

    @property
    def _meta(self) -> Path:
        return self.dir / "latest.json"

    @property
    def _seq_file(self) -> Path:
        return self.dir / "seq"

    # -- durability helpers --------------------------------------------------
    def _attach_file_log(self) -> None:
        """Mirror this module's log lines into recordings/recorder.log.

        journald on the Pi is Storage=volatile and the Pi is powered from the
        flight battery, so every battery swap erased the only record of what
        the recorder did - the 09-22 flights' fate could not be reconstructed.
        Small and rotating: two 256 KB files."""
        import logging
        from logging.handlers import RotatingFileHandler
        path = str((self.dir / "recorder.log").resolve())
        if any(getattr(h, "baseFilename", None) == path for h in _log.handlers):
            return
        try:
            h = RotatingFileHandler(path, maxBytes=256_000, backupCount=1)
            h.setFormatter(logging.Formatter("%(asctime)s %(levelname)s %(message)s"))
            _log.addHandler(h)
        except OSError as exc:  # noqa: BLE001
            _log.warning("no persistent recorder log: %s", exc)

    def _fsync_dir(self) -> None:
        """Make renames/creates in the recordings dir survive a power cut."""
        try:
            fd = os.open(self.dir, os.O_RDONLY)
            try:
                os.fsync(fd)
            finally:
                os.close(fd)
        except OSError:
            pass

    def _write_json_durable(self, path: Path, data: dict) -> None:
        """tmp + fsync + os.replace + dir fsync. Without the fsyncs an atomic
        rename is only atomic in RAM: measured, nothing reaches the SD card for
        up to 30 s, and a battery pulled in that window loses the file."""
        tmp = path.with_suffix(path.suffix + ".tmp")
        with open(tmp, "w") as fp:
            fp.write(json.dumps(data, indent=2))
            fp.flush()
            os.fsync(fp.fileno())
        os.replace(tmp, path)
        self._fsync_dir()

    def _clip_seq(self) -> int:
        return int((self._clip or {}).get("seq", 0) or 0)

    def _next_seq(self) -> int:
        """Next recording number. Seeded from every place a seq is recorded,
        so losing the counter file can never make numbers go backwards."""
        seen = [self._seq, self._clip_seq()]
        try:
            seen.append(int(self._seq_file.read_text().strip() or 0))
        except (OSError, ValueError):
            pass
        for j in self.dir.glob("*.job.json"):
            try:
                seen.append(int(json.loads(j.read_text()).get("seq", 0) or 0))
            except (OSError, ValueError):
                pass
        seq = max(seen) + 1
        try:
            self._seq_file.write_text(str(seq))
        except OSError:
            pass
        self._seq = seq
        return seq

    # -- startup housekeeping ------------------------------------------------
    def _load_clip(self) -> None:
        """Re-adopt the clip left by a previous run, so a GCS restart does not
        make a perfectly good recording disappear from the dashboard."""
        try:
            if self._meta.exists() and self._mp4.exists():
                self._clip = json.loads(self._meta.read_text())
                self._state = "ready"
        except Exception as exc:  # noqa: BLE001
            _log.warning("could not read recording metadata: %s", exc)

    def _sweep_orphans(self) -> None:
        """Delete in-progress files left by a crash or a power cut.

        A recording that FINISHED and is only owed a render is not an orphan:
        it carries a .job.json sidecar and is picked up again by
        _adopt_pending. Sweeping those was the 2026-09-19 data-loss bug - every
        GCS restart between flights deleted the flight that had just landed,
        and the dashboard went on showing a clip from four days earlier.

        Still deliberately NOT rendered from inside this method: it runs in the
        GCS boot path on an aircraft that is usually about to fly. Adoption
        decides separately, and lazily, when spending a core is safe.
        """
        # NB: "failed-*" is deliberately NOT swept. Those are frames a failed
        # encode preserved on purpose; only one set is ever kept.
        owed = {j.name[:-len(".job.json")] for j in self.dir.glob("*.job.json")}
        # A recording with no job is not necessarily garbage: the power went
        # while armed, or before the job reached the card. Its index says
        # exactly which frames are complete, so recover those first.
        for src in self.dir.glob("flight-*.mjpg"):
            stem = src.name.split(".", 1)[0]
            if stem not in owed and self._recover(src):
                owed.add(stem)
        # latest.tmp.* is a PARTIAL render output and is always garbage: the
        # resumed render starts from frame zero and writes it again.
        orphans = list(self.dir.glob("latest.tmp.mp4"))
        orphans += list(self.dir.glob("latest.tmp.jpg"))
        for stale in list(self.dir.glob("flight-*.mjpg")) + \
                list(self.dir.glob("flight-*.jsonl")) + \
                list(self.dir.glob("flight-*.poster.jpg")):
            stem = stale.name.split(".", 1)[0]
            if stem in owed:
                continue
            orphans.append(stale)
        for stale in orphans:
            try:
                mb = stale.stat().st_size / 1e6
                stale.unlink()
                _log.warning(
                    "discarded unfinished recording %s (%.0f MB) - the GCS did "
                    "not shut down cleanly while recording", stale.name, mb
                )
            except OSError:
                pass

    def _recover(self, src: Path) -> bool:
        """Turn a recording cut off by a power loss into an owed render.

        The old sweep called such a file "truncated at an unknown point" and
        deleted it. With an index that is not true: every line records the
        offset and length of a frame, so the complete frames are exactly those
        that end inside the file. Stream-tap recordings (no index) really have
        no such boundary and are still swept.
        """
        index = src.with_suffix(".jsonl")
        if not index.exists():
            return False
        try:
            size = src.stat().st_size
            lines = index.read_text().splitlines()
            kept, frames, seq = [], [], 0
            for n, line in enumerate(lines):
                try:
                    d = json.loads(line)
                except ValueError:
                    break                   # torn last line
                if n == 0 and "hdr" in d:
                    seq = int(d.get("seq", 0) or 0)
                    kept.append(line)
                    continue
                if int(d.get("o", 0)) + int(d.get("n", 0)) > size:
                    break                   # frame bytes never reached the card
                kept.append(line)
                frames.append(d)
            if len(frames) < 2:
                return False
            ts = [float(f.get("t") or 0.0) for f in frames]
            duration = max(0.0, ts[-1] - ts[0])
            # Cut the torn tail frame too: the quick encode reads the frame
            # file as a plain JPEG stream and has no index to stop it.
            end = int(frames[-1]["o"]) + int(frames[-1]["n"])
            if end < size:
                os.truncate(src, end)
            # Rewrite the index to the complete frames only, so neither stage
            # reads past the end of the frame file.
            tmp = index.with_suffix(".jsonl.tmp")
            tmp.write_text("\n".join(kept) + "\n")
            os.replace(tmp, index)
            mtime = src.stat().st_mtime
            job = {
                "src": src.name, "index": index.name, "frames": len(frames),
                "duration": duration, "fps": self._average_fps(len(frames), duration),
                "reason": "recovered", "size": None,
                # The header carries the seq assigned at start(); only an index
                # from before 2026-09-23 lacks it.
                "seq": seq or self._next_seq(),
                "t_start": mtime - duration, "t_end": mtime,
                "quick_done": False,
            }
            self._write_json_durable(self._job_path(src), job)
            _log.warning("recovered %s after a power loss: %d complete frames, "
                         "%.0f s - queued for render", src.name, len(frames), duration)
            return True
        except (OSError, ValueError) as exc:  # noqa: BLE001
            _log.warning("could not recover %s: %s", src.name, exc)
            return False

    # -- pending renders -----------------------------------------------------
    def _adopt_pending(self) -> None:
        """Re-adopt a finished recording whose render was cut short.

        Almost always a GCS restart between test flights: the render is a child
        process, so systemd kills it with the service, and the operator is left
        looking at the previous clip with nothing in the log to say why.
        """
        def order(q: Path) -> tuple:
            # seq first: mtime follows a clock that can step backwards at boot
            # (no RTC), which once let the older job win. mtime only breaks
            # ties between pre-seq jobs.
            try:
                s = int(json.loads(q.read_text()).get("seq", 0) or 0)
            except (OSError, ValueError):
                s = 0
            return (s, q.stat().st_mtime)

        jobs = sorted(self.dir.glob("*.job.json"), key=order)
        if not jobs:
            return
        # Exactly one clip is ever kept, so only the newest job can still win
        # the output file. Older ones are dead weight on the card.
        for stale in jobs[:-1]:
            self._discard_job(stale)
        job_path = jobs[-1]
        try:
            job = json.loads(job_path.read_text())
        except (OSError, ValueError) as exc:  # noqa: BLE001
            _log.warning("unreadable render job %s: %s", job_path.name, exc)
            self._discard_job(job_path)
            return
        seq = int(job.get("seq", 0) or 0)
        self._seq = max(self._seq, seq)
        if seq and seq < self._clip_seq():
            _log.info("render job %s is older than the kept clip - discarding",
                      job_path.name)
            self._discard_job(job_path)
            return
        if not (self.dir / str(job.get("src", ""))).exists():
            _log.warning("render job %s has no frames - discarding", job_path.name)
            self._discard_job(job_path)
            return
        job["job"] = job_path.name
        with self._lock:
            self._pending = job
            if self._state in ("idle", "ready") and not job.get("quick_done"):
                self._state = "pending"
                self._message = ("a previous flight's replay is still to be "
                                 f"rendered ({float(job.get('duration', 0.0)):.0f}s)")
        _log.warning(
            "adopting unrendered replay %s (%.0f s, %s) left by a previous run",
            job.get("src"), float(job.get("duration", 0.0)), job.get("reason", "?"),
        )
        self._maybe_render_pending()

    def _discard_job(self, job_path: Path) -> None:
        """Drop a job and everything it owned. Used when the job is superseded,
        unreadable, or its frames are gone."""
        try:
            job = json.loads(job_path.read_text())
        except (OSError, ValueError):  # noqa: BLE001
            job = {}
        stem = job_path.name[:-len(".job.json")]
        for name in (job.get("src"), job.get("index"), f"{stem}.poster.jpg"):
            if name:
                try:
                    (self.dir / str(name)).unlink(missing_ok=True)
                except OSError:
                    pass
        try:
            job_path.unlink(missing_ok=True)
        except OSError:
            pass

    def _may_render_now(self) -> bool:
        """May the recorder spend a core rendering a pending replay right now?

        Called with self._lock held, from _maybe_render_pending. Returning
        False leaves the job on disk untouched - it is retried on the next
        disarm - so refusing is always safe and never loses footage.

        Available state:
          self._armed    latest ARM edge from the FC - but see self._arm_seen
          self._arm_seen has the FC told us the arm state even once? False
                         until the first set_armed() call, so on a fresh boot
                         self._armed is False by INITIALISATION, not because
                         anyone observed the aircraft to be disarmed
          self._state    "idle" | "ready" | "pending" | "recording" | "encoding"
          self._clip     the kept clip, or None
        """
        # Never on a guess. Until the FC has actually said so, _armed is a
        # constructor default, and reading it as "safely on the ground" is
        # what started a render during boot.
        if not self._arm_seen:
            return False
        if self._armed:
            return False
        # Belt and braces: _maybe_render_pending already refuses these, but the
        # encoder is single-threaded and this is the one place the policy is
        # stated, so state it completely.
        return self._state not in ("recording", "encoding")

    def _maybe_render_pending(self) -> None:
        """Start the owed render if there is one and now is a safe moment."""
        with self._lock:
            if self._pending is None:
                return
            if self._state in ("recording", "encoding") or self._busy:
                return
            if not self._may_render_now():
                return
            job = self._pending
            self._pending = None
            stale = int(job.get("seq", 0) or 0) < max(self._floor, self._clip_seq())
        if stale:
            _log.info("owed render %s is older than the kept clip - dropped", job.get("src"))
            self._discard_job(self.dir / str(job["job"]))
            return
        with self._lock:
            self._progress = 0.0
            self._reason = str(job.get("reason", "armed"))
            if job.get("quick_done") and self._clip is not None:
                # This flight is already the replay; only its HD version is owed.
                self._state = "ready"
                self._message = "clip ready - stabilised HD resuming"
            else:
                self._state = "encoding"
                self._message = "rendering the previous flight's replay"
        src = self.dir / str(job["src"])
        index = self.dir / str(job["index"]) if job.get("index") else None
        poster = self._poster_sidecar(src)
        first = None
        try:
            if poster.exists():
                first = poster.read_bytes()
        except OSError:
            pass
        size = tuple(job["size"]) if job.get("size") else None
        self._preempt.clear()
        threading.Thread(
            target=self._encode_and_promote,
            args=(src, index, first, size, int(job.get("frames", 0)),
                  float(job.get("duration", 0.0)), float(job.get("fps", 30.0)),
                  str(job.get("reason", "armed")), self.dir / str(job["job"])),
            kwargs={"seq": int(job.get("seq", 0) or 0),
                    "quick_done": bool(job.get("quick_done", False)),
                    "t_start": job.get("t_start"), "t_end": job.get("t_end")},
            name="rec-encode-resume", daemon=True,
        ).start()

    def set_armed(self, armed: bool) -> None:
        """Fed by GcsHub off the ARM edge. Disarming is the moment an owed
        render becomes safe, so it is also the retry trigger."""
        self._armed = bool(armed)
        self._arm_seen = True           # this is now an observation, not a guess
        if not armed:
            self._maybe_render_pending()

    def _preempt_render(self) -> None:
        """Stop the running render so a recording can have the machine.

        Costs only the CPU already spent: the job sidecar stays on disk and the
        render restarts from frame zero after the next disarm. Footage always
        outranks pixels.
        """
        self._preempt.set()
        proc = self._render_proc
        if proc is not None and proc.poll() is None:
            try:
                proc.kill()
            except OSError:
                pass

    # -- frame sinks ---------------------------------------------------------
    def set_geometry(self, main: tuple, lores: tuple, fps: float) -> None:
        """Declared once by a camera with a record stream: the record frame size,
        the live frame size its motion and boxes are measured in, and the
        nominal rate."""
        self._geometry = {"main": [int(main[0]), int(main[1])],
                          "lores": [int(lores[0]), int(lores[1])], "fps": float(fps)}

    def offer(self, jpeg: bytes, w: int, h: int) -> None:
        """Camera capture thread hands a published frame over. Never blocks."""
        q = self._queue
        if q is None:
            return
        try:
            q.put_nowait(jpeg)
        except queue.Full:
            self._dropped += 1
            return
        self._t_last = time.monotonic()
        if self._first_jpeg is None:
            # Pinned here, not at encode time: the adaptive ladder can change
            # the frame size mid-flight, and the fallback encoder needs one
            # fixed output geometry to scale everything to.
            self._first_jpeg = jpeg
            self._size = (w, h)

    def offer_frame(self, planes: tuple, meta: dict) -> None:
        """Full-resolution I420 planes + this frame's motion/lock metadata.
        Called on the camera capture thread. Never blocks."""
        q = self._queue
        if q is None:
            return
        A = meta.get("A")
        if self._pending_motion is not None:
            # Frames dropped since the last kept one moved the camera too; the
            # replay stabiliser needs the whole path, not just the kept steps.
            A = (_to3(A) @ self._pending_motion)[:2]
        try:
            q.put_nowait((planes, dict(meta, A=A)))
        except queue.Full:
            self._dropped += 1
            self._pending_motion = _to3(A)
            return
        self._pending_motion = None
        self._t_last = time.monotonic()
        if self._size is None:
            self._size = (int(planes[0].shape[1]), int(planes[0].shape[0]))

    # -- lifecycle -----------------------------------------------------------
    def start(self, reason: str = "manual") -> dict:
        with self._lock:
            if not self.enabled:
                return {"ok": False, "message": "recording disabled by config"}
            if self._state == "recording":
                return {"ok": False, "message": "already recording"}
            # _busy, not _state: the HD upgrade runs while the quick clip is
            # already "ready", and it must yield to an arming aircraft too.
            preempting = bool(self._busy) or self._state == "encoding"

        if preempting:
            # An aircraft that is arming outranks a replay of the last flight.
            # Safe to interrupt only because the render is durable: its job
            # sidecar stays on disk and it resumes after the next disarm.
            # Before this, start() REFUSED here - so arming again within a
            # minute of landing silently recorded nothing at all.
            _log.warning("preempting the running replay render - the aircraft is arming")
            self._preempt_render()

        with self._lock:
            stamp = time.strftime("%Y%m%d-%H%M%S")
            raw = self._geometry is not None
            self._src = self.dir / f"flight-{stamp}.mjpg"
            self._index = self.dir / f"flight-{stamp}.jsonl" if raw else None
            self._queue = queue.Queue(maxsize=_QUEUE_RAW_FRAMES if raw else _QUEUE_FRAMES)
            self._frames = self._dropped = self._bytes = 0
            self._first_jpeg = None
            self._size = None
            self._pending_motion = None
            self._progress = 0.0
            self._t0 = self._t_last = time.monotonic()
            self._t_wall0 = time.time()
            self._airborne = False
            self._seq_cur = self._next_seq()
            self._state = "recording"
            self._reason = reason
            self._message = f"recording ({reason})"
            self._writer = threading.Thread(
                target=self._writer_loop,
                args=(self._src, self._index, self._queue, self._seq_cur),
                name="rec-writer", daemon=True,
            )
            self._writer.start()
            _log.info("recording #%d started (%s) -> %s%s", self._seq_cur, reason,
                      self._src.name, " (full-res + motion index)" if raw else "")
            return {"ok": True, "message": f"recording started ({reason})"}

    def stop(self, reason: str = "manual", end_t: Optional[float] = None) -> dict:
        """end_t: time.monotonic() (the index "t" clock) after which frames are
        not part of the flight - the hub passes touchdown + 2 s so the replay
        does not end on a minute of the aircraft sitting on the pad."""
        with self._lock:
            if self._state != "recording":
                return {"ok": False, "message": "not recording"}
            src, index, q, writer = self._src, self._index, self._queue, self._writer
            # Cleared first so offer() stops feeding the queue the instant the
            # aircraft disarms, before the writer is asked to drain.
            self._queue = None
            duration = max(0.0, self._t_last - self._t0)
            seq, airborne = self._seq_cur, self._airborne
            t_start, t_end = self._t_wall0, self._t_wall0 + duration
            self._state = "encoding"
            self._message = "finalising"

        if q is not None:
            q.put(_SENTINEL)
        if writer is not None:
            writer.join(timeout=10.0)
        frames = self._frames
        first = self._first_jpeg
        size = self._size
        if end_t is not None and index is not None:
            trimmed = self._trim_after(src, index, float(end_t))
            if trimmed is not None:
                frames, cut_s = trimmed
                duration = max(0.0, duration - cut_s)
                t_end = t_start + duration

        keep, why = self._should_keep(frames, duration, src, airborne)
        if not keep:
            self._discard(src, why)
            if index is not None:
                index.unlink(missing_ok=True)
            # A discarded bench arm leaves any owed render of the last real
            # flight alone - it still renders after this disarm.
            return {"ok": True, "message": why}

        # This flight is now THE latest. Anything older that is still owed a
        # render can only ever overwrite it, so it goes - on disk too, or the
        # next boot would adopt it again.
        with self._lock:
            self._floor = max(self._floor, seq)
            older = self._pending
            self._pending = None
        if older is not None and older.get("job"):
            _log.info("superseded unrendered %s by recording #%d",
                      older.get("src"), seq)
            self._discard_job(self.dir / str(older["job"]))

        fps = self._average_fps(frames, duration)
        # Written BEFORE the render starts, never after: the whole point is to
        # survive the render dying, and a job written afterwards would not.
        job_path = self._write_job(src, index, first, size, frames,
                                   duration, fps, reason, seq=seq,
                                   t_start=t_start, t_end=t_end, airborne=airborne)
        _log.info(
            "recording #%d stopped (%s): %d frames, %.1f s, %.1f fps, %s - %s",
            seq, reason, frames, duration, fps,
            "flew" if airborne else "never airborne",
            "quick clip, then stabilised replay" if index is not None else "encoding",
        )
        self._preempt.clear()
        threading.Thread(
            target=self._encode_and_promote,
            args=(src, index, first, size, frames, duration, fps, reason, job_path),
            kwargs={"seq": seq, "t_start": t_start, "t_end": t_end},
            name="rec-encode", daemon=True,
        ).start()
        return {"ok": True, "message": f"recorded {duration:.0f}s - encoding"}

    def _trim_after(self, src: Path, index: Path, end_t: float) -> Optional[tuple]:
        """Drop the frames recorded after end_t. Returns (frames kept, seconds
        cut), or None when nothing was trimmed.

        Frames are appended in time order, so the cut is always a tail: the
        index is rewritten atomically and the mjpg truncated to the last kept
        frame, so the render never even reads the dropped bytes."""
        try:
            lines = index.read_text().splitlines()
        except OSError as exc:
            _log.warning("trim: cannot read %s (%s) - keeping every frame", index.name, exc)
            return None
        head = [ln for ln in lines[:1] if '"hdr"' in ln]
        rows = []
        for ln in lines[len(head):]:
            try:
                rows.append((ln, json.loads(ln)))
            except ValueError:
                break
        if not rows:
            return None
        keep = [(ln, r) for ln, r in rows if float(r.get("t") or 0.0) <= end_t]
        if len(keep) == len(rows):
            return None
        if len(keep) < 2:
            # A wrong clock or an early touchdown estimate must never be able
            # to erase a flight. Keeping too much is harmless; this is not.
            _log.warning("trim: end_t %.2f would leave %d of %d frames - ignored",
                         end_t, len(keep), len(rows))
            return None
        last = keep[-1][1]
        cut_s = float(rows[-1][1].get("t") or 0.0) - float(last.get("t") or 0.0)
        tmp = index.with_name(index.name + ".tmp")
        with open(tmp, "w") as fh:
            fh.write("\n".join(head + [ln for ln, _ in keep]) + "\n")
            fh.flush()
            os.fsync(fh.fileno())
        os.replace(tmp, index)
        with open(src, "r+b") as fh:
            fh.truncate(int(last["o"]) + int(last["n"]))
            os.fsync(fh.fileno())
        self._fsync_dir()
        _log.info("trimmed %d frames (%.1f s) after touchdown from %s",
                  len(rows) - len(keep), cut_s, src.name)
        return len(keep), max(0.0, cut_s)

    def _write_job(self, src: Path, index: Optional[Path], first: Optional[bytes],
                   size: Optional[tuple], frames: int, duration: float,
                   fps: float, reason: str, seq: int = 0,
                   t_start: Optional[float] = None, t_end: Optional[float] = None,
                   airborne: bool = False) -> Optional[Path]:
        """Persist everything _encode_and_promote needs to run without the
        process that recorded the flight."""
        job_path = self._job_path(src)
        try:
            if first:
                self._poster_sidecar(src).write_bytes(first)
            # Durable, not merely atomic: a battery pulled within 30 s of
            # disarm used to lose this file, and the boot sweep then deleted
            # the frames as an "unfinished recording".
            self._write_json_durable(job_path, {
                "src": src.name,
                "index": index.name if index is not None else None,
                "frames": int(frames),
                "duration": float(duration),
                "fps": float(fps),
                "reason": str(reason),
                "size": list(size) if size else None,
                "seq": int(seq),
                "t_start": t_start,
                "t_end": t_end,
                "airborne": bool(airborne),
                "quick_done": False,
            })
            return job_path
        except OSError as exc:  # noqa: BLE001
            # Not fatal: this render still runs. It just will not survive a
            # restart, which is exactly the old behaviour.
            _log.warning("could not write render job (%s) - this replay will "
                         "not survive a GCS restart", exc)
            return None

    def shutdown(self) -> None:
        """Called from GcsHub.stop(). Finishes the file so the sweep on the
        next boot has nothing to throw away."""
        if self._state == "recording":
            self.stop("shutdown")

    # -- retention -----------------------------------------------------------
    def _should_keep(self, frames: int, duration: float,
                     src: Optional[Path], airborne: bool = False) -> tuple[bool, str]:
        """Decide whether this clip is worth replacing the kept one with.

        A recording in which the aircraft FLEW always wins, however short: the
        22 Sep 14:57 hop was 13 s armed and was thrown away by the old 15 s
        rule while the dashboard kept showing 21 Sep. min_seconds now guards
        only against bench arms that never left the ground."""
        if src is None or not src.exists():
            return False, "nothing was recorded"
        if frames < 2:
            return False, "no frames captured - is the camera running?"
        if airborne:
            return True, ""
        if duration < self.min_seconds:
            return False, (f"clip {duration:.0f}s never left the ground and is under "
                           f"the {self.min_seconds:.0f}s minimum - discarded, "
                           f"previous clip kept")
        return True, ""

    def note_altitude(self, alt_m: Optional[float]) -> None:
        """Fed by GcsHub from the ALTITUDE topic (height above home). Latches
        'this recording flew' - mode-agnostic, so a hop in STABILIZE, LOITER,
        GUIDED or AUTO all count the same."""
        if alt_m is None or self._airborne or self._state != "recording":
            return
        try:
            alt = float(alt_m)
        except (TypeError, ValueError):
            return
        if alt >= self._airborne_alt:
            self._airborne = True
            _log.info("recording #%d: airborne (%.1f m) - this clip will be kept",
                      self._seq_cur, alt)

    def _discard(self, src: Optional[Path], why: str) -> None:
        if src is not None:
            try:
                src.unlink(missing_ok=True)
            except OSError:
                pass
        with self._lock:
            self._state = "ready" if self._clip else "idle"
            self._message = why
        _log.info("recording discarded: %s", why)

    def _average_fps(self, frames: int, duration: float) -> float:
        """Real delivered rate, not the configured one."""
        if duration <= 0.0 or frames < 2:
            return 30.0
        return max(1.0, min(60.0, (frames - 1) / duration))

    # -- writer thread -------------------------------------------------------
    def _encoder(self):
        try:
            import simplejpeg

            q = self._quality
            return lambda Y, U, V: simplejpeg.encode_jpeg_yuv_planes(Y, U, V, q, fastdct=True)
        except Exception:  # noqa: BLE001
            import cv2

            def enc(Y, U, V):
                i420 = np.concatenate([Y.reshape(-1), U.reshape(-1), V.reshape(-1)])
                bgr = cv2.cvtColor(i420.reshape(Y.shape[0] * 3 // 2, Y.shape[1]),
                                   cv2.COLOR_YUV2BGR_I420)
                ok, buf = cv2.imencode(".jpg", bgr, [cv2.IMWRITE_JPEG_QUALITY, self._quality])
                return buf.tobytes() if ok else None
            return enc

    def _flusher(self, fds: list, stop: threading.Event) -> None:
        """fdatasync the recording every flush_s, OFF the writer thread.

        On the writer it would stall the 0.5 s raw queue while the SD card
        catches up and cost frames; here it only pushes pages the kernel was
        going to write anyway - 30 s later, which is the window a battery
        pull used to wipe out."""
        while not stop.wait(self._flush_s):
            for fd in fds:
                try:
                    os.fdatasync(fd)
                except OSError:
                    pass

    def _writer_loop(self, path: Path, index: Optional[Path], q: "queue.Queue",
                     seq: int = 0) -> None:
        encode = self._encoder() if index is not None else None
        idx = None
        stop_flush = threading.Event()
        last_flush = time.monotonic()
        try:
            with open(path, "wb", buffering=1 << 20) as fp:
                if index is not None:
                    idx = open(index, "w", buffering=1 << 16)
                    # seq rides in the header so a recording recovered after a
                    # power cut keeps its true place in the flight order.
                    idx.write(json.dumps({"hdr": self._geometry, "seq": int(seq)}) + "\n")
                fds = [fp.fileno()] + ([idx.fileno()] if idx is not None else [])
                flusher = None
                if self._flush_s > 0:
                    flusher = threading.Thread(target=self._flusher, args=(fds, stop_flush),
                                               name="rec-flush", daemon=True)
                    flusher.start()
                while True:
                    item = q.get()
                    if item is _SENTINEL:
                        break
                    if isinstance(item, (bytes, bytearray)):
                        jpeg, meta = item, None
                    else:
                        planes, meta = item
                        jpeg = encode(*planes)
                        if not jpeg:
                            self._dropped += 1
                            continue
                    offset = fp.tell()
                    fp.write(jpeg)
                    if idx is not None and meta is not None:
                        A = meta.get("A")
                        box = meta.get("box")
                        idx.write(json.dumps({
                            "o": offset, "n": len(jpeg),
                            "t": round(float(meta.get("t") or 0.0), 4),
                            "A": None if A is None else
                            [round(float(v), 5) for v in np.asarray(A).reshape(-1)[:6]],
                            "box": None if not box else [round(float(v), 1) for v in box[:4]],
                            # Detector boxes [x1,y1,x2,y2,score] in capture px,
                            # drawn by replay_render. Already rounded upstream.
                            "d": meta.get("dets") or None,
                            "st": meta.get("st"), "sc": meta.get("sc"),
                        }) + "\n")
                    self._frames += 1
                    self._bytes += len(jpeg)
                    now = time.monotonic()
                    if now - last_flush >= self._flush_s:
                        # Python buffers -> page cache only (cheap). The index
                        # buffer holds ~14 s of lines at 30 fps, and recovery can
                        # only use frames whose index line reached the kernel.
                        last_flush = now
                        fp.flush()
                        if idx is not None:
                            idx.flush()
                # Final: everything on the card before stop() writes the job
                # that declares this recording complete.
                stop_flush.set()
                if flusher is not None:
                    flusher.join(timeout=5.0)
                fp.flush()
                os.fsync(fp.fileno())
                if idx is not None:
                    idx.flush()
                    os.fsync(idx.fileno())
        except Exception as exc:  # noqa: BLE001
            _log.error("recording writer failed: %s", exc)
            with self._lock:
                self._state = "error"
                self._message = f"write failed: {exc}"
        finally:
            stop_flush.set()
            if idx is not None:
                idx.close()

    # -- encode --------------------------------------------------------------
    def _render(self, src: Path, index: Path, tmp: Path, tmp_poster: Path,
                frames: int) -> dict:
        """Run replay_render in a niced subprocess; returns its final report."""
        cmd = ["nice", "-n", "10", sys.executable, "-m", "drone_stack.gcs.replay_render",
               "--src", str(src.resolve()), "--index", str(index.resolve()),
               "--out", str(tmp.resolve()), "--poster", str(tmp_poster.resolve()),
               "--width", str(self._out_w), "--height", str(self._out_h),
               "--sigma", str(self._sigma), "--max-angle-deg", str(self._max_angle),
               "--preset", self._preset, "--crf", str(self._crf),
               "--threads", str(self._threads), "--cpu-cores", str(self._cpu_cores),
               "--filter", json.dumps(self._filter_cfg)]
        # Paced to render_cpu_cores, so allow for the slowest plausible rate.
        deadline = max(self._max_seconds, frames * 0.6)
        proc = subprocess.Popen(cmd, cwd=str(_REPO_ROOT), stdout=subprocess.PIPE,
                                stderr=subprocess.PIPE, text=True)
        self._render_proc = proc
        
        timer = threading.Timer(deadline, proc.kill)
        timer.start()
        
        final: dict = {}
        try:
            for line in proc.stdout:
                try:
                    rep = json.loads(line)
                except ValueError:
                    continue
                if "progress" in rep:
                    with self._lock:
                        self._progress = float(rep["progress"])
                        if self._upgrading is not None:
                            # The quick clip is already playable; this is only
                            # the stabilised HD version replacing it.
                            self._upgrading = self._progress
                            if self._state == "ready":
                                self._message = (f"clip ready - stabilised HD "
                                                 f"{self._progress * 100:.0f}%")
                        else:
                            self._message = f"rendering 1080p replay {self._progress * 100:.0f}%"
                final.update(rep)
            rc = proc.wait(timeout=60)
        finally:
            timer.cancel()
            if proc.poll() is None:
                proc.kill()
            self._render_proc = None
        if self._preempt.is_set():
            # Killed on purpose by start(). Distinct from a failure: there is
            # nothing wrong with these frames and they must NOT be downgraded
            # to a plain encode or salvaged - the job resumes them intact.
            raise _Preempted()
        if rc != 0 or "error" in final or not tmp.exists():
            err = final.get("error") or (proc.stderr.read() if proc.stderr else "")
            raise RuntimeError(f"render failed ({rc}): {str(err).strip()[-300:]}")
        return final

    def _ffmpeg_stream(self, src: Path, tmp: Path, size, fps: float) -> tuple[int, int]:
        """The original encode: the recorded JPEG stream straight to H.264."""
        w, h = size or (1280, 720)
        w, h = (w // 2) * 2, (h // 2) * 2
        if w > self._out_w or h > self._out_h:
            w, h = self._out_w & ~1, self._out_h & ~1
        cmd = [
            "nice", "-n", "10", "ffmpeg", "-y", "-hide_banner", "-nostdin",
            "-loglevel", "error",
            "-f", "image2pipe", "-c:v", "mjpeg", "-framerate", f"{fps:.3f}",
            "-i", str(src),
            # Mandatory, not cosmetic: the adaptive ladder can change the frame
            # size mid-recording and ffmpeg refuses to change frame properties
            # on the fly without a scale filter to normalise them.
            "-vf", f"scale={w}:{h}:flags=bilinear,setsar=1,format=yuv420p",
            "-c:v", "libx264", "-preset", self._preset, "-crf", str(self._crf),
            "-threads", str(self._threads),
            "-movflags", "+faststart", "-an", "-f", "mp4", str(tmp),
        ]
        proc = subprocess.run(cmd, capture_output=True, text=True,
                              timeout=max(300.0, self._max_seconds))
        if proc.returncode != 0 or not tmp.exists():
            raise RuntimeError((proc.stderr or "ffmpeg failed").strip()[:300])
        return w, h

    def _ffmpeg_quick(self, src: Path, tmp: Path, fps: float) -> tuple[int, int]:
        """Stage 1: the raw frames straight to a small H.264 clip, fast.

        ``-lowres 1`` makes the MJPEG decoder produce 1152x648 directly from
        the DCT (no full decode then downscale) - that is most of the speed.
        Popen, not run(): registered as _render_proc so an arming aircraft
        can kill it like any render."""
        w, h = self._quick_w & ~1, self._quick_h & ~1
        cmd = [
            "nice", "-n", "10", "ffmpeg", "-y", "-hide_banner", "-nostdin",
            "-loglevel", "error", "-lowres", "1",
            "-f", "image2pipe", "-c:v", "mjpeg", "-framerate", f"{fps:.3f}",
            "-i", str(src),
            "-vf", f"scale={w}:{h}:flags=bilinear,setsar=1,format=yuv420p",
            "-c:v", "libx264", "-preset", self._quick_preset,
            "-crf", str(self._quick_crf), "-threads", str(self._quick_threads),
            "-movflags", "+faststart", "-an", "-f", "mp4", str(tmp),
        ]
        proc = subprocess.Popen(cmd, stdout=subprocess.DEVNULL,
                                stderr=subprocess.PIPE, text=True)
        self._render_proc = proc
        try:
            _, err = proc.communicate(timeout=max(300.0, self._max_seconds))
        except subprocess.TimeoutExpired:
            proc.kill()
            _, err = proc.communicate()
        finally:
            self._render_proc = None
        if self._preempt.is_set():
            raise _Preempted()
        if proc.returncode != 0 or not tmp.exists():
            raise RuntimeError((err or "ffmpeg failed").strip()[-300:])
        return w, h

    def _first_frame(self, src: Path, index: Optional[Path]) -> Optional[bytes]:
        """First recorded JPEG, as the quick clip's poster (the stabilised
        render writes its own). Raw recordings have no _first_jpeg."""
        try:
            if index is not None and index.exists():
                with open(index) as fp:
                    for line in fp:
                        d = json.loads(line)
                        if "o" in d:
                            with open(src, "rb") as f:
                                f.seek(int(d["o"]))
                                return f.read(int(d["n"]))
        except (OSError, ValueError):
            pass
        return None

    def _promote(self, tmp: Path, tmp_poster: Path, first: Optional[bytes],
                 clip: dict) -> bool:
        """Make ``tmp`` the kept clip - unless a NEWER flight already owns it.

        This check is the actual fix for the stale replay. Previously any
        render that finished simply os.replace()d latest.mp4, so an old
        flight's deferred render landing after a new flight's overwrote it."""
        seq = int(clip.get("seq", 0) or 0)
        try:
            with open(tmp, "rb") as fp:          # the mp4 itself, on the card
                os.fsync(fp.fileno())
        except OSError:
            pass
        with self._lock:
            newest = max(self._clip_seq(), self._floor)
            if seq < newest:
                _log.warning("not promoting recording #%d: #%d is newer", seq, newest)
                tmp.unlink(missing_ok=True)
                tmp_poster.unlink(missing_ok=True)
                return False
            # Atomic: there is never a moment where the old clip is deleted
            # and the new one is not yet in place.
            os.replace(tmp, self._mp4)
            if tmp_poster.exists():
                os.replace(tmp_poster, self._poster)
            elif first:
                self._poster.write_bytes(first)
            self._write_json_durable(self._meta, clip)
            self._clip = clip
            if self._state != "recording":      # a new flight may own _state
                self._state = "ready"
                self._message = "clip ready"
            self._progress = 1.0
        return True

    def _encode_and_promote(self, src: Path, index: Optional[Path],
                            first: Optional[bytes], size: Optional[tuple[int, int]],
                            frames: int, duration: float, fps: float, reason: str,
                            job_path: Optional[Path] = None, seq: int = 0,
                            quick_done: bool = False, t_start: Optional[float] = None,
                            t_end: Optional[float] = None) -> None:
        """Two stages. (1) a quick unstabilised clip, promoted as soon as it
        exists - this is what makes the replay show the flight that just
        landed; (2) the stabilised 1080p render, which replaces it in place.
        The job stays on disk until stage 2 is settled, with quick_done noted
        so a restart does not redo stage 1."""
        # ".tmp.mp4", not ".mp4.tmp": ffmpeg picks its MUXER from the output
        # file extension - see the 2026-09-13 note in git history.
        tmp = self.dir / "latest.tmp.mp4"
        tmp_poster = self.dir / "latest.tmp.jpg"
        with self._lock:
            self._busy += 1
        now = time.time()
        t_end = t_end if t_end else now
        t_start = t_start if t_start else t_end - duration

        def clip_meta(w: int, h: int, fps_: float, stabilised: bool) -> dict:
            return {
                "name": self._mp4.name,
                "seq": int(seq),
                # Flight times, not render times: the UI and the operator
                # both mean "when did this flight happen".
                "started": t_start,
                "ended": t_end,
                "duration_s": round(duration, 1),
                "frames": frames,
                "fps": round(fps_, 2),
                "width": w, "height": h,
                "size_bytes": tmp.stat().st_size,
                "reason": reason,
                "dropped": self._dropped,
                "stabilized": stabilised,
            }

        def settle() -> None:
            # The job is only retired once an mp4 of this flight is in place.
            # Ordered this way on purpose: a power cut anywhere above leaves
            # the job intact and the flight is re-encoded on the next boot.
            self._retire_job(job_path, src)
            self._drop_source(src)
            if index is not None:
                self._drop_source(index)

        try:
            if index is not None and index.exists():
                if self._quick and not quick_done:
                    try:
                        w, h = self._ffmpeg_quick(src, tmp, fps)
                        if not self._promote(tmp, tmp_poster,
                                             self._first_frame(src, index) or first,
                                             clip_meta(w, h, fps, False)):
                            settle()            # a newer flight owns the replay
                            return
                        quick_done = True
                        _log.info("quick clip ready: recording #%d, %.0f s, %dx%d "
                                  "- stabilised HD next", seq, duration, w, h)
                        if job_path is not None and job_path.exists():
                            try:
                                job = json.loads(job_path.read_text())
                                job["quick_done"] = True
                                self._write_json_durable(job_path, job)
                            except (OSError, ValueError):
                                pass
                    except _Preempted:
                        raise
                    except Exception as exc:  # noqa: BLE001
                        # Stage 2 still runs; it is the better clip anyway.
                        _log.error("quick clip failed, going straight to HD: %s", exc)
                        tmp.unlink(missing_ok=True)
                if quick_done:
                    with self._lock:
                        self._upgrading = 0.0
                try:
                    rep = self._render(src, index, tmp, tmp_poster, frames)
                    w, h = int(rep.get("width", self._out_w)), int(rep.get("height", self._out_h))
                    fps = float(rep.get("fps", fps))
                    stabilised = True
                except _Preempted:
                    # Must outrun the plain-encode fallback below: a preempted
                    # render has perfectly good frames and is owed a REAL
                    # stabilised render, not a downgrade.
                    raise
                except Exception as exc:  # noqa: BLE001
                    tmp.unlink(missing_ok=True)
                    tmp_poster.unlink(missing_ok=True)
                    if quick_done:
                        # This flight is already the replay. Keep that clip.
                        _log.error("stabilised render failed - keeping the quick "
                                   "clip of recording #%d: %s", seq, exc)
                        settle()
                        return
                    # The raw frames are still a perfectly good recording; an
                    # unstabilised clip beats no clip.
                    _log.error("stabilised render failed, plain encode instead: %s", exc)
                    w, h = self._ffmpeg_stream(src, tmp, size, fps)
                    stabilised = False
            else:
                w, h = self._ffmpeg_stream(src, tmp, size, fps)
                stabilised = False

            clip = clip_meta(w, h, fps, stabilised)
            if self._promote(tmp, tmp_poster, first, clip):
                _log.info(
                    "clip ready: recording #%d, %.0f s, %.1f MB, %dx%d%s "
                    "(previous clip replaced)", seq, duration,
                    clip["size_bytes"] / 1e6, w, h, ", stabilised" if stabilised else "",
                )
            settle()
        except _Preempted:
            # Give the frames back to the pending queue and leave every file
            # on disk. Nothing is lost; the render simply runs later.
            tmp.unlink(missing_ok=True)
            tmp_poster.unlink(missing_ok=True)
            if job_path is not None and job_path.exists():
                job = {
                    "src": src.name,
                    "index": index.name if index is not None else None,
                    "frames": int(frames), "duration": float(duration),
                    "fps": float(fps), "reason": str(reason),
                    "size": list(size) if size else None,
                    "job": job_path.name, "seq": int(seq),
                    "quick_done": bool(quick_done),
                    "t_start": t_start, "t_end": t_end,
                }
                with self._lock:
                    superseded = seq < self._floor or (
                        self._pending is not None
                        and int(self._pending.get("seq", 0) or 0) > seq)
                    if not superseded:
                        self._pending = job
                if superseded:
                    _log.info("preempted render of %s is superseded - dropped", src.name)
                    self._discard_job(job_path)
                else:
                    _log.info("replay render deferred - %s will be rendered after "
                              "the next disarm", src.name)
            # Do NOT touch _state: the recording that preempted this owns it.
        except Exception as exc:  # noqa: BLE001
            _log.error("encode failed: %s", exc)
            tmp.unlink(missing_ok=True)
            tmp_poster.unlink(missing_ok=True)
            kept = self._salvage(src, index)
            # _salvage renamed the frames to failed-*, so the job now points at
            # nothing. Retire it or the next boot adopts a job with no source.
            self._retire_job(job_path, src)
            with self._lock:
                if self._state == "encoding":
                    self._state = "ready" if self._clip else "error"
                self._message = (f"encode failed: {exc}"
                                 + (f" - raw frames kept as {kept}" if kept else ""))
        finally:
            with self._lock:
                # A preempted thread unwinds AFTER the next flight's render has
                # started; zeroing a flag here would make that render
                # un-preemptable and let a second one start beside it.
                self._busy = max(0, self._busy - 1)
                if not self._busy:
                    self._upgrading = None
                if self._state == "encoding" and self._clip is not None:
                    self._state = "ready"

    def _retire_job(self, job_path: Optional[Path], src: Path) -> None:
        """Drop the job sidecar and its poster once the render is settled."""
        for stale in (job_path, self._poster_sidecar(src)):
            if stale is None:
                continue
            try:
                stale.unlink(missing_ok=True)
            except OSError:
                pass

    def _drop_source(self, src: Path) -> None:
        """Delete the raw frames once they are safely inside an mp4 - at
        full resolution they fill the card at ~11 MB per second of flight."""
        try:
            src.unlink(missing_ok=True)
        except OSError:
            pass

    def _salvage(self, src: Path, index: Optional[Path] = None) -> str:
        """Keep the raw frames (and their motion index) when the encode fails,
        so a transient fault does not destroy a flight's footage. Exactly one
        salvage set is kept - the newest."""
        if not src.exists():
            return ""
        try:
            for old_file in list(self.dir.glob("failed-*.mjpg")) + \
                    list(self.dir.glob("failed-*.jsonl")):
                old_file.unlink(missing_ok=True)
            stem = src.stem.replace("flight-", "")
            kept = self.dir / f"failed-{stem}.mjpg"
            src.replace(kept)
            if index is not None and index.exists():
                index.replace(self.dir / f"failed-{stem}.jsonl")
            _log.warning(
                "raw frames kept at %s (%.0f MB) - encode them by hand with: "
                "ffmpeg -f image2pipe -c:v mjpeg -i %s -c:v libx264 out.mp4",
                kept, kept.stat().st_size / 1e6, kept.name,
            )
            return kept.name
        except OSError as exc:
            _log.error("could not salvage raw frames: %s", exc)
            return ""

    # -- introspection -------------------------------------------------------
    def is_recording(self) -> bool:
        """True while frames are being written. Read by the camera's adaptive
        controller and by its grabber, which only copies the full-resolution
        frame out of the ISP while this is True."""
        return self._state == "recording"

    def clip_path(self) -> Optional[Path]:
        return self._mp4 if self._mp4.exists() else None

    def poster_path(self) -> Optional[Path]:
        return self._poster if self._poster.exists() else None

    def status(self) -> dict:
        """Shipped in every GCS frame, so the UI reads config and live state
        rather than holding numbers of its own (CLAUDE.md section 5)."""
        with self._lock:
            recording = self._state == "recording"
            elapsed = (self._t_last - self._t0) if recording else 0.0
            free_mb = 0.0
            try:
                free_mb = shutil.disk_usage(self.dir).free / 1e6
            except OSError:
                pass
            return {
                "enabled": self.enabled,
                "state": self._state,
                "active": recording,
                "reason": self._reason,
                "message": self._message,
                "progress": round(self._progress, 3) if self._state == "encoding" else None,
                # A flight recorded but not yet rendered. Surfaced so the
                # operator is never silently shown an older clip than the
                # flight they just made - the failure this whole mechanism
                # exists to prevent.
                "pending": None if self._pending is None else {
                    "duration_s": round(float(self._pending.get("duration", 0.0)), 1),
                    "reason": str(self._pending.get("reason", "")),
                    "seq": int(self._pending.get("seq", 0) or 0),
                    # True: the flight IS already the replay (quick clip);
                    # only its stabilised HD version is still owed.
                    "quick_done": bool(self._pending.get("quick_done", False)),
                },
                # Stabilised HD render progress (0..1) while the quick clip of
                # the same flight is already playable; None otherwise.
                "upgrading": None if self._upgrading is None else round(self._upgrading, 3),
                "seq": self._seq_cur if recording else None,
                "airborne": self._airborne if recording else None,
                "elapsed_s": round(elapsed, 1),
                "frames": self._frames,
                "dropped": self._dropped,
                "bytes": self._bytes,
                "min_seconds": self.min_seconds,
                "free_mb": round(free_mb, 0),
                "clip": dict(self._clip) if self._clip else None,
            }
