"""Live camera capture for the Drone GCS.

One capture thread owning the physical camera and keeping the *latest* JPEG
frame ready for the MJPEG HTTP endpoint:

* ``picam`` - Raspberry Pi Camera (imx708) via ``picamera2``.

The USB webcam (Logitech C270, cam 0) was REMOVED on 2026-09-11 at the user's
request - it had stopped enumerating ("no UVC capture node found") and was
retrying once a second forever, which is also what left the GCS service in a
failed state. With it went the terrain-segmentation overlay that ran on its
frames; that path was already dead code (``_build_novelty_processors`` returned
early), so nothing that was actually running was lost.

Each camera can carry an optional **processor** - a callable that takes the
freshly captured BGR frame (numpy) and returns an annotated BGR frame. This is
where the Hailo-8 NPU overlay is drawn (person detection on the Pi camera).
All neural inference happens on the Hailo
accelerator, never on the Pi CPU; the processor only does the (cheap) draw +
JPEG encode on the CPU. Processors run **only on kept frames** (after frame
skipping) so the CPU cost is bounded.

Everything degrades gracefully: if a camera (or its library, or the Hailo HAT)
is missing the stream simply reports ``connected: False`` / serves the raw frame,
so the GCS still runs in pure simulation with no hardware attached.

## Novelty layer tap (project plan §9, step 6)

Processors are built by ``_build_novelty_processors`` (not the legacy
``drone_stack.gcs.hailo_infer.Detector``/``Segmenter`` construction
directly) via ``drone_stack.novelty.perception.model_registry.ModelRegistry``
- the SAME frame is never re-captured (the camera device cannot be opened
twice), so this is the ONE place inference can run: once per kept frame,
publishing the structured result (``PersonDetection`` list /
``SegmentationFrame``) on ``NoveltyTopics`` for ``DeliveryNode`` to consume,
then drawing the identical overlay from that already-computed result (no
second inference call). ``CameraManager`` needs the shared ``MessageBus``
for this - see its own ``bus`` constructor parameter.
"""
from __future__ import annotations

import io
import os
import threading
import time
from typing import TYPE_CHECKING, Callable, Optional

from drone_stack.gcs.frame_filter import FrameFilter, filter_from_config
from drone_stack.gcs.person_lock import person_lock_from_config
from drone_stack.gcs.stabilizer import stabilizer_from_config
from drone_stack.utils.logging_setup import get_logger

if TYPE_CHECKING:
    from drone_stack.bus import MessageBus

_log = get_logger("gcs.cameras")

# A processor takes a BGR numpy frame and returns an (annotated) BGR frame.
Processor = Callable[["object"], "object"]

# 1x1 black JPEG used before the first real frame arrives / when disconnected.
_PLACEHOLDER_JPEG = bytes.fromhex(
    "ffd8ffe000104a46494600010100000100010000ffdb004300080606070605080707"
    "07090908"
    "0a0c140d0c0b0b0c1912130f141d1a1f1e1d1a1c1c20242e2720222c231c1c283729"
    "2c30313434341f27393d38323c2e333432ffc0000b080001000101011100ffc40014"
    "0001000000000000000000000000000000000009ffc40014100100000000000000"
    "00000000000000000000ffda0008010100003f00d2cf20ffd9"
)


# How often the adaptive controller re-evaluates. Long enough that a single
# slow second (a browser repaint, a burst of MAVLink) cannot move the rung, and
# short enough that flying behind a building does not cost ten seconds of
# frozen video before the picture softens and recovers.
_ADAPT_PERIOD_S = 2.0

# How long a rung that just failed stays off-limits. After this, the controller
# is allowed to probe one rung sharper again. 25 s is long enough that flying
# behind a building does not immediately re-flood the link on the way out, and
# short enough that a genuine improvement (landing, walking closer) is picked
# up while it still matters.
_ADAPT_COOLDOWN_S = 25.0


class _BaseCamera:
    def __init__(self, cam_id: int, name: str, skip: int = 1,
                 processor: Optional[Processor] = None,
                 frame_filter: Optional[FrameFilter] = None,
                 jpeg_quality: int = 60,
                 adaptive: bool = False,
                 ladder: Optional[list] = None,
                 target_fps: float = 30.0,
                 min_height: int = 0,
                 stabilizer=None,
                 vision=None) -> None:
        self.cam_id = cam_id
        self.name = name
        self.width = 0
        self.height = 0
        self.connected = False
        self._jpeg: bytes = _PLACEHOLDER_JPEG
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._thread: Optional[threading.Thread] = None
        self._frames = 0
        self._fps = 0.0
        self._fps_t = time.monotonic()
        # optional per-frame annotator (Hailo overlay). Applied on kept frames.
        self._processor = processor
        # frame skipping: publish/encode only every Nth captured frame to keep
        # CPU + latency down (frames are still *read* every loop so the buffer
        # never backs up and we always encode the freshest frame).
        self._skip = max(1, int(skip))
        self._skip_ctr = 0
        # Image conditioning (destripe / row repair / temporal denoise). Each
        # camera owns its own instance: the filter carries a previous-frame
        # buffer, so sharing one between capture threads would blend the two
        # cameras' images into each other.
        self._filter = frame_filter
        self._jpeg_quality = int(jpeg_quality)
        # Monotonic frame counter + wakeup, so the MJPEG endpoint can block
        # until a frame actually exists instead of polling on a timer and
        # adding up to a frame period of latency to every frame it serves.
        self._seq = 0
        self._new_frame = threading.Event()
        # Capture-to-published latency, in ms: grab -> filter -> overlay ->
        # encode. This is the number the whole exercise turns on, so it is
        # measured rather than argued about.
        self._latency_ms = 0.0
        # asyncio waiters, one per open MJPEG connection. Registering an
        # asyncio.Event and firing it from the capture thread means a streaming
        # request costs no thread at all while it waits. The earlier version
        # parked each connection on a worker thread via run_in_executor, which
        # works but draws from a pool of ~8 on a Pi 5 - a handful of browser
        # tabs, or stale connections that had not timed out yet, could exhaust
        # it and stall unrelated executor work.
        self._async_waiters: set = set()
        self._waiter_lock = threading.Lock()
        # Optional frame sink (the flight recorder). One callable, set once at
        # hub construction. Deliberately NOT a subscriber list: this is a tap
        # on the already-encoded bytes, not another viewer, and it must never
        # report a delivered rate - see recorder.py's module docstring for why
        # feeding the adaptive controller from a local disk writer would pin
        # the camera to rung 0 and flood the link.
        self._sink = None
        # Optional zero-arg predicate: "is the sink currently recording?".
        # Lets the adaptive controller below tell a live flight recording apart
        # from an idle camera - see _adapt_tick.
        self._sink_recording = None
        # True while the controller is holding rung 0 purely for the recorder.
        self._rec_boost = False
        # The flight recorder, when the camera feeds it full-resolution frames
        # plus motion metadata itself (see set_recorder) rather than the
        # already-encoded stream bytes through the sink above.
        self._recorder = None
        # Published frames never go below this height, whatever the adaptive
        # ladder asks for: the operator's floor is 720p. 0 = no floor.
        self._min_height = max(0, int(min_height))
        # Digital stabiliser (stabilizer.VideoStabilizer) and the NPU person
        # lock (person_lock.PersonLockPipeline). Both optional; both run on the
        # capture thread, and neither touches hardware until start().
        self._stab = stabilizer
        self._vision = vision
        # Called with every person-lock snapshot (capture thread). The
        # CameraManager sets it to put the lock on the bus for the navigator.
        self.lock_sink: Optional[Callable[[dict], None]] = None

        # -- adaptive bitrate ------------------------------------------------
        # A weak link does NOT make this stream lag: the MJPEG endpoint always
        # sends the newest frame and silently drops whatever it missed. What a
        # weak link does is cost frames - 30 fps encoded, 6 fps delivered, and
        # the picture looks frozen. So holding 30 fps on a bad link is entirely
        # a question of making each frame small enough to fit through it.
        #
        # The controller trades sharpness for cadence, in that order, because a
        # soft 30 fps picture is flyable and a sharp 4 fps one is not. Steps are
        # a fixed, inspectable ladder rather than a continuous controller: this
        # rides on an aircraft, and an operator who sees the picture soften
        # should be able to name exactly which rung it is on.
        self._adaptive = bool(adaptive)
        self._ladder = [(float(s), int(q)) for s, q in (ladder or [])] or \
                       [(1.0, int(jpeg_quality))]
        self._rung = 0
        self._target_fps = float(target_fps)
        # stream key -> (delivered fps, monotonic stamp). One entry per open
        # viewer; the controller steers by the WORST of them, since a rung that
        # only suits the fastest viewer starves everyone else.
        self._delivered: dict = {}
        self._adapt_lock = threading.Lock()
        self._adapt_t = time.monotonic()
        self._good_ticks = 0
        # Best (lowest-index) rung the controller is currently allowed to climb
        # to. Borrowed from TCP's ssthresh: the rung we just failed at is known
        # to be too fat for this link, so re-entering it is not exploration, it
        # is a guaranteed stall. It relaxes by one rung per cooldown so a link
        # that genuinely improves is still discovered.
        self._probe_floor = 0
        self._probe_t = time.monotonic()

    def _keep(self) -> bool:
        self._skip_ctr += 1
        return (self._skip_ctr % self._skip) == 0

    def _condition(self, frame):
        """Repair sensor/interference artefacts before anything else sees the
        frame - the overlay is drawn on top of it and the JPEG encodes it, so
        this has to run first or the boxes get denoised along with the image."""
        if self._filter is None:
            return frame
        return self._filter.apply(frame)

    def _annotate(self, frame):
        """Run the Hailo processor on a kept frame; never let it break capture."""
        if self._processor is None:
            return frame
        try:
            return self._processor(frame)
        except Exception as exc:  # noqa: BLE001
            _log.warning("%s processor error: %s", self.name, exc)
            return frame

    # -- lifecycle -----------------------------------------------------------
    def start(self) -> None:
        if self._thread and self._thread.is_alive():
            return
        self._stop.clear()
        if self._vision is not None:
            # Loads the HEF on its own thread; the first frames publish without
            # a box rather than waiting ~2 s for the NPU.
            self._vision.start()
        self._thread = threading.Thread(
            target=self._run, name=f"cam-{self.name}", daemon=True
        )
        self._thread.start()

    def stop(self) -> None:
        self._stop.set()
        if self._vision is not None:
            self._vision.stop()
        if self._thread:
            self._thread.join(timeout=2.0)

    def _stabilize_and_track(self, frame):
        """Stabilise, then run the person lock on the stabilised frame.

        Returns ``(published_frame, stab_result_or_None, lock_snapshot_or_None)``.
        Never raises: a vision fault costs the box, not the picture."""
        result = None
        out = frame
        if self._stab is not None:
            try:
                result = self._stab.step(frame)
                out = self._stab.render(frame, result.matrix, result.rs)
            except Exception as exc:  # noqa: BLE001
                _log.warning("%s stabiliser error: %s", self.name, exc)
                result, out = None, frame
        snap = None
        if self._vision is not None:
            try:
                snap = self._vision.process(
                    out,
                    None if result is None else result.motion,
                    None if result is None else result.matrix,
                    raw_shape=frame.shape,
                )
            except Exception as exc:  # noqa: BLE001
                _log.warning("%s person lock error: %s", self.name, exc, exc_info=True)
            sink = self.lock_sink
            if snap is not None and sink is not None:
                try:
                    sink(snap)
                except Exception as exc:  # noqa: BLE001
                    _log.warning("%s person lock sink error: %s", self.name, exc)
        return out, result, snap

    # -- accessors -----------------------------------------------------------
    def jpeg(self) -> bytes:
        with self._lock:
            return self._jpeg

    def jpeg_seq(self) -> tuple[bytes, int]:
        """Latest frame together with its sequence number, read atomically."""
        with self._lock:
            return self._jpeg, self._seq

    def subscribe(self):
        """Register an asyncio.Event fired on every published frame.

        Must be called from the event loop that will await it. Always pair with
        ``unsubscribe`` in a finally block or the waiter leaks.
        """
        import asyncio

        loop = asyncio.get_running_loop()
        event = asyncio.Event()
        with self._waiter_lock:
            self._async_waiters.add((loop, event))
        return event

    def unsubscribe(self, event) -> None:
        with self._waiter_lock:
            self._async_waiters = {w for w in self._async_waiters if w[1] is not event}

    def set_sink(self, sink, is_recording=None) -> None:
        """Attach a consumer called with (jpeg, width, height) per published
        frame. Must not block: it runs on the capture thread, so anything slow
        here costs the live stream frames. Pass None to detach.

        *is_recording* is an optional zero-arg predicate the adaptive
        controller consults when no viewer is connected.
        """
        self._sink = sink
        self._sink_recording = is_recording

    def set_recorder(self, recorder) -> None:
        """Attach the flight recorder. The base camera has no separate record
        stream, so it records the published JPEG bytes through the sink."""
        self._recorder = recorder
        self.set_sink(recorder.offer, recorder.is_recording)

    def wait_for_frame(self, last_seq: int, timeout: float = 1.0) -> bool:
        """Block until a frame newer than *last_seq* is published.

        Called from the MJPEG endpoint's executor thread. Returns False on
        timeout so a disconnected camera cannot wedge the request forever.
        """
        deadline = time.monotonic() + timeout
        while True:
            with self._lock:
                if self._seq != last_seq:
                    return True
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                return False
            self._new_frame.clear()
            # Re-check after clearing: a frame published between the check
            # above and the clear would otherwise be lost and we would sleep
            # through it.
            with self._lock:
                if self._seq != last_seq:
                    return True
            self._new_frame.wait(min(remaining, 0.25))

    def _publish(self, jpeg: bytes, w: int, h: int,
                 captured_at: float | None = None) -> None:
        if captured_at:
            sample = (time.monotonic() - captured_at) * 1000.0
            # EMA so one scheduling hiccup does not make the reading jump.
            self._latency_ms = (sample if self._latency_ms == 0.0
                                else 0.8 * self._latency_ms + 0.2 * sample)
        with self._lock:
            self._jpeg = jpeg
            self.width, self.height = w, h
            self._seq += 1
        self._new_frame.set()
        with self._waiter_lock:
            waiters = tuple(self._async_waiters)
        for loop, event in waiters:
            try:
                loop.call_soon_threadsafe(event.set)
            except RuntimeError:
                # loop already closed; the stream's finally block removes it
                pass
        sink = self._sink
        if sink is not None:
            try:
                sink(jpeg, w, h)
            except Exception as exc:  # noqa: BLE001
                # A recorder fault must never take the live picture down with
                # it: the stream the pilot is flying by outranks the recording.
                _log.warning("%s frame sink error: %s", self.name, exc)
        self._frames += 1
        now = time.monotonic()
        dt = now - self._fps_t
        if dt >= 1.0:
            self._fps = round(self._frames / dt, 1)
            self._frames = 0
            self._fps_t = now

    # -- adaptive bitrate ----------------------------------------------------
    def report_delivered(self, key, fps: float) -> None:
        """Called once a second by each open MJPEG stream with its real rate."""
        with self._adapt_lock:
            self._delivered[key] = (float(fps), time.monotonic())

    def drop_delivery(self, key) -> None:
        with self._adapt_lock:
            self._delivered.pop(key, None)

    def _worst_delivered(self) -> Optional[float]:
        """Slowest viewer seen in the last 3 s, or None if nobody is watching.

        Stale entries are ignored rather than removed: a stream that stopped
        reporting is usually one that is about to close, and its last reading
        should not pin the whole camera to a low rung forever.
        """
        now = time.monotonic()
        with self._adapt_lock:
            live = [f for f, t in self._delivered.values() if now - t < 3.0]
        return min(live) if live else None

    def _choose_rung(self, rung: int, delivered: float, target: float,
                     n_rungs: int, good_ticks: int) -> tuple:
        """Decide which ladder rung to encode at next.

        Returns ``(new_rung, new_good_ticks)``. Called once every
        ``_ADAPT_PERIOD_S`` seconds, and only while somebody is watching.

        * ``rung``        current index into ``self._ladder``; 0 is the
                          sharpest/fattest, ``n_rungs - 1`` the softest/leanest.
        * ``delivered``   frames per second the slowest viewer actually got.
        * ``target``      frames per second we are trying to hold (30).
        * ``good_ticks``  how many consecutive evaluations have looked healthy;
                          yours to reset or increment as you see fit.
        """
        now = time.monotonic()

        # Asymmetric on purpose. Falling behind is an emergency - the picture is
        # already stuttering by the time we notice - so drop a rung on the very
        # first bad evaluation (2 s).
        if delivered < target * 0.85:
            failed = min(rung + 1, n_rungs - 1)
            # Remember that THIS rung could not be sustained, and refuse to
            # climb back into it until the cooldown expires. Without this the
            # controller walks straight back up, re-floods the link, collapses,
            # and the operator watches the picture breathe every few seconds.
            self._probe_floor = max(self._probe_floor, failed)
            self._probe_t = now
            return failed, 0

        # Relax the ceiling one rung at a time, never in a jump - each probe has
        # to prove itself at the new rung before the next one is unlocked.
        if self._probe_floor > 0 and now - self._probe_t >= _ADAPT_COOLDOWN_S:
            self._probe_floor -= 1
            self._probe_t = now

        if delivered >= target * 0.97:
            good_ticks += 1
            # Four consecutive healthy evaluations (8 s), not three: at three
            # this still climbed on the back of one lucky quiet moment.
            if good_ticks >= 4 and rung > self._probe_floor:
                return rung - 1, 0
            return rung, good_ticks

        # In between: good enough to keep, not good enough to bet on. Hold the
        # rung but reset the streak, so a link hovering at 90% never climbs.
        return rung, 0

    def _adapt_tick(self) -> None:
        """Run the controller. Called from the capture thread after publishing."""
        if not self._adaptive or len(self._ladder) < 2:
            return
        now = time.monotonic()
        if now - self._adapt_t < _ADAPT_PERIOD_S:
            return
        self._adapt_t = now
        delivered = self._worst_delivered()
        if delivered is None:
            # Nobody is watching. Do not chase a number nobody is measuring -
            # and do not spring back to rung 0 either, or the first frame the
            # next viewer sees is the fat one the link already rejected.
            #
            # UNLESS a recording is running. Then the only consumer is the
            # local disk, which always keeps up: there is no link to protect,
            # and leaving the footage at whatever rung a departed viewer's bad
            # link forced would write 640x360 to the card for no reason at all.
            # The ladder exists to fit a radio, not a filesystem.
            if self._rung != 0 and self._sink_recording is not None:
                try:
                    recording = bool(self._sink_recording())
                except Exception:  # noqa: BLE001
                    recording = False
                if recording:
                    _log.info(
                        "%s adaptive: rung %d -> 0 (recording, no viewers)",
                        self.name, self._rung,
                    )
                    self._rung, self._good_ticks = 0, 0
                    self._rec_boost = True
            return

        if self._rec_boost:
            # A viewer came back while the recorder held rung 0. Hand the link
            # straight back to the rung it had already proven it could carry
            # rather than walking down to it one evaluation at a time - that is
            # 2 s of stutter per rung, and _probe_floor already knows the
            # answer. Deliberately pessimistic: the cooldown will let it climb
            # again from there if the link really has improved.
            self._rec_boost = False
            if self._probe_floor > self._rung:
                _log.info("%s adaptive: rung %d -> %d (viewer returned)",
                          self.name, self._rung, self._probe_floor)
                self._rung = min(self._probe_floor, len(self._ladder) - 1)
                self._good_ticks = 0
        rung, good = self._choose_rung(self._rung, delivered, self._target_fps,
                                       len(self._ladder), self._good_ticks)
        rung = max(0, min(int(rung), len(self._ladder) - 1))
        if rung != self._rung:
            _log.info("%s adaptive: rung %d -> %d (%s), delivered %.1f/%.0f fps",
                      self.name, self._rung, rung, self._ladder[rung],
                      delivered, self._target_fps)
        self._rung, self._good_ticks = rung, good

    def _encode_frame(self, cv2, frame):
        """Scale (if the rung calls for it) then encode. Returns (jpeg, frame).

        The frame is returned as well as the JPEG because a scaled rung changes
        the published dimensions, and the camera tile reports what was actually
        sent rather than what the sensor produced.
        """
        if not self._adaptive:
            return _encode_jpeg(cv2, frame, self._jpeg_quality), frame
        scale, quality = self._ladder[self._rung]
        h, w = frame.shape[:2]
        if scale < 0.999 and self._min_height and h * scale < self._min_height:
            # The floor wins over the ladder: below it, only quality is spent.
            scale = min(1.0, self._min_height / float(h))
        if scale < 0.999:
            # Even dimensions: JPEG chroma subsampling works on 2x2 blocks and
            # an odd edge costs a wasted half-block.
            nw, nh = max(2, int(w * scale) & ~1), max(2, int(h * scale) & ~1)
            # INTER_AREA is the correct filter for downscaling - it averages the
            # source pixels rather than sampling them, so it does not alias the
            # sensor noise into the moire the destripe stage just removed.
            frame = cv2.resize(frame, (nw, nh), interpolation=cv2.INTER_AREA)
        return _encode_jpeg(cv2, frame, quality), frame

    def info(self) -> dict:
        out = {
            "id": self.cam_id,
            "name": self.name,
            "res": f"{self.width}x{self.height}" if self.connected else "--",
            "fps": self._fps if self.connected else 0,
            "connected": self.connected,
            # Watch this, not the fps: a camera can hold 30 fps while every
            # frame it serves is a second old.
            "latency_ms": round(self._latency_ms, 1) if self.connected else 0,
        }
        if self._adaptive and len(self._ladder) > 1:
            # On the tile so a soft picture is explainable at a glance instead
            # of being mistaken for a focus or a filter problem.
            scale, quality = self._ladder[self._rung]
            out["adapt"] = {"rung": self._rung, "rungs": len(self._ladder),
                            "scale": scale, "quality": quality,
                            "ceiling": self._probe_floor}
        if self._filter is not None and self._filter.enabled:
            # Surfaced so a stripe problem is visible on the dashboard rather
            # than only in the operator's eyes: rows_repaired climbing the
            # moment the motors spin is the EMI signature.
            out["filter"] = self._filter.stats()
        if self._stab is not None:
            out["stab"] = self._stab.stats()
        if self._vision is not None:
            out["vision"] = self._vision.stats()
        return out

    # -- override ------------------------------------------------------------
    def _run(self) -> None:  # pragma: no cover - hardware loop
        raise NotImplementedError


def _encode_jpeg(cv2, frame, quality: int):
    """Encode and sanity-check one frame.

    A JPEG must end with the EOI marker FFD9. A truncated buffer renders in
    the browser as the top of the image followed by a hard horizontal edge and
    grey below - i.e. exactly the artefact this work is chasing - so a frame
    that does not end cleanly is dropped rather than published. The previous
    good frame stays on screen for one period instead.
    """
    ok, buf = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, int(quality)])
    if not ok or buf is None or len(buf) < 4:
        return None
    if buf[-2] != 0xFF or buf[-1] != 0xD9:
        return None
    return buf.tobytes()


class PiCamera(_BaseCamera):
    """Raspberry Pi camera (imx708) via picamera2.

    Capture and processing are **decoupled**: a
    lightweight grabber thread keeps only the freshest raw frame, while
    ``_run`` annotates (Hailo person-detection overlay) + JPEG-encodes the
    latest frame in parallel.  This stops annotation/encoding time from
    throttling the capture cadence, so the published rate tracks the camera
    sensor instead of ``capture + annotate + encode`` summed.

    Without decoupling, picamera2's internal FIFO queue grows whenever the
    consumer cannot keep up and ``capture_array()`` returns progressively
    OLDER frames - the lag accumulates and produces a visible "water mirror"
    effect during flight.  The grabber thread drains the queue as fast as
    the sensor delivers, so the encoder always sees the freshest frame.
    """

    def __init__(self, cam_id: int, name: str = "PiCam",
                 width: int = 1280, height: int = 720, skip: int = 1,
                 fps: int = 30, processor: Optional[Processor] = None,
                 frame_filter: Optional[FrameFilter] = None,
                 jpeg_quality: int = 60, isp_denoise: str = "fast",
                 ae_mode: str = "short", rotate_180: bool = False,
                 adaptive: bool = False, ladder: Optional[list] = None,
                 target_fps: Optional[float] = None, min_height: int = 0,
                 stabilizer=None, vision=None,
                 capture_size: Optional[tuple] = None,
                 record_size: Optional[tuple] = None,
                 lens_position: Optional[float] = None,
                 ae_custom: Optional[dict] = None) -> None:
        super().__init__(cam_id, name, skip=skip, processor=processor,
                         frame_filter=frame_filter, jpeg_quality=jpeg_quality,
                         adaptive=adaptive, ladder=ladder,
                         target_fps=target_fps if target_fps else float(fps),
                         min_height=min_height, stabilizer=stabilizer,
                         vision=vision)
        self._req_w, self._req_h = width, height
        # The live frame is captured LARGER than it is published when the
        # stabiliser is on: the difference is the margin the correction moves
        # the crop within. ISP-scaled, so it costs nothing on the CPU.
        self._cap_w, self._cap_h = (tuple(int(v) for v in capture_size)
                                    if capture_size else (width, height))
        # A second, full-resolution ISP output used only by the flight
        # recorder (YUV420, so it JPEG-encodes without a colour conversion).
        # None = record the live stream instead.
        self._record_size = tuple(int(v) for v in record_size) if record_size else None
        self._fps_cap = fps
        self._isp_denoise = str(isp_denoise)
        self._ae_mode = str(ae_mode)
        # Camera Module 3 has a motorised lens. Left alone, libcamera parks it
        # at the tuning file's default of 1.0 dioptre = focused at 1 m, which
        # on this lens holds focus only from ~0.7 to ~1.8 m - everything the
        # aircraft actually looks at was soft. A fixed position (not AF: AF
        # hunting in flight breathes the picture and confuses the stabiliser).
        self._lens_position = None if lens_position is None else float(lens_position)
        # ae_mode "custom": an exposure table (shutter us / gain) written into
        # the tuning file's AGC. Lets the shutter stay short and spend gain
        # first, which is what cuts vibration blur.
        self._ae_custom = dict(ae_custom) if ae_custom else None
        # Last ExposureTime / AnalogueGain / LensPosition the ISP reported.
        self._sensor_meta: dict = {}
        # The camera is mounted upside down on the airframe. Prefer libcamera's
        # own Transform: the ISP does the flip in hardware for free, where a
        # CPU rotate would cost a copy of every 720p frame on a Pi that is
        # already sharing cores with the flight stack. _sw_rotate is the
        # fallback for a libcamera too old to expose Transform.
        self._rotate_180 = bool(rotate_180)
        self._sw_rotate = False
        self._raw = None            # latest captured BGR frame
        self._raw_main = None       # its full-resolution YUV420 twin (recording only)
        self._raw_t = 0.0           # when it was captured (monotonic)
        self._raw_lock = threading.Lock()
        self._raw_evt = threading.Event()

    def _grabber(self, picam) -> None:
        """Continuously read the newest frame; never blocks the encoder.

        Dual-stream mode (a record size is configured) reads one request per
        sensor frame and copies out the live ``lores`` image always, but the
        4.5 MB full-resolution ``main`` image ONLY while a recording is running
        - a copy nobody writes is pure waste on a core the flight stack shares.
        """
        dual = self._record_size is not None
        n = 0
        while not self._stop.is_set() and self._grab_ok:
            main = None
            n += 1
            try:
                if dual:
                    req = picam.capture_request()
                    try:
                        frame = req.make_array("lores")
                        rec = self._recorder
                        if rec is not None and rec.is_recording():
                            main = req.make_array("main")
                        if n % 15 == 1:
                            self._note_sensor(req.get_metadata())
                    finally:
                        req.release()
                else:
                    frame = picam.capture_array()
            except Exception:  # noqa: BLE001
                self._grab_ok = False
                self._raw_evt.set()
                return
            if frame is None:
                self._grab_ok = False
                self._raw_evt.set()
                return
            with self._raw_lock:
                self._raw = frame
                self._raw_main = main
                self._raw_t = time.monotonic()
            self._raw_evt.set()

    def set_recorder(self, recorder) -> None:
        """Attach the flight recorder.

        With a record size configured the recorder gets the full-resolution
        ISP frame plus this frame's camera motion and lock box, and renders a
        stabilised 1080p clip after landing. Without one it falls back to
        recording the published stream bytes, as before."""
        self._recorder = recorder
        if self._record_size is None:
            self.set_sink(recorder.offer, recorder.is_recording)
            return
        self._sink = None
        self._sink_recording = recorder.is_recording
        recorder.set_geometry(main=self._record_size, lores=(self._cap_w, self._cap_h),
                              fps=float(self._fps_cap))

    def _yuv_planes(self, arr):
        """Split a picamera2 YUV420 array (h*3/2 rows x stride) into I420
        plane views, honouring a stride wider than the image."""
        import numpy as np

        w, h = self._record_size
        stride = arr.shape[1]
        Y = arr[:h, :w]
        U = arr[h:h + h // 4].reshape(h // 2, stride // 2)[:, :w // 2]
        V = arr[h + h // 4:h + h // 2].reshape(h // 2, stride // 2)[:, :w // 2]
        if self._sw_rotate:
            Y, U, V = Y[::-1, ::-1], U[::-1, ::-1], V[::-1, ::-1]
        return tuple(np.ascontiguousarray(p) for p in (Y, U, V))

    def _sensor_controls(self) -> dict:
        """libcamera controls applied at configure time.

        Two of these are free wins that cost the CPU nothing because the ISP
        does the work in hardware, and neither was being set before:

        ``NoiseReductionMode`` - the ISP's own spatial denoise, running on the
        image pipeline rather than on a core the flight stack needs. It is
        strictly better value than any CPU filter for ordinary sensor noise,
        which is why the CPU chain in ``frame_filter`` deliberately only
        handles what the ISP cannot see: row-correlated interference.

        ``AeExposureMode = Short`` - biases auto-exposure toward a shorter
        exposure and higher gain for the same brightness. On a moving aircraft
        that matters twice over: it cuts motion blur, and because this is a
        rolling-shutter sensor a long exposure also shears the frame (the top
        and bottom rows are sampled milliseconds apart), which reads as the
        image tearing horizontally as the airframe moves. Shorter exposure
        shrinks that skew directly.
        """
        controls: dict = {"FrameRate": self._fps_cap}
        try:
            from libcamera import controls as lc
        except Exception:  # noqa: BLE001 - simulation / no camera stack
            return controls

        if self._lens_position is not None:
            try:
                controls["AfMode"] = lc.AfModeEnum.Manual
                controls["LensPosition"] = max(0.0, self._lens_position)
            except Exception as exc:  # noqa: BLE001 - fixed-focus module
                _log.warning("Pi camera: LensPosition unsupported (%s)", exc)

        nr_name = {"off": "Off", "minimal": "Minimal", "fast": "Fast",
                   "high_quality": "HighQuality"}.get(self._isp_denoise.lower())
        if nr_name:
            try:
                controls["NoiseReductionMode"] = getattr(
                    lc.draft.NoiseReductionModeEnum, nr_name)
            except Exception as exc:  # noqa: BLE001
                _log.warning("Pi camera: NoiseReductionMode=%s unsupported (%s)",
                             nr_name, exc)

        ae_name = {"normal": "Normal", "short": "Short", "long": "Long",
                   "custom": "Custom"}.get(self._ae_mode.lower())
        if ae_name:
            try:
                controls["AeExposureMode"] = getattr(
                    lc.AeExposureModeEnum, ae_name)
            except Exception as exc:  # noqa: BLE001
                _log.warning("Pi camera: AeExposureMode=%s unsupported (%s)",
                             ae_name, exc)
        return controls

    @staticmethod
    def custom_exposure_mode(section: Optional[dict]) -> Optional[dict]:
        """Validate ``ae_custom`` into a tuning-file exposure mode, or None.

        libcamera's AGC walks the table stage by stage - raise the shutter to
        ``shutter[i]``, then the gain to ``gain[i]`` - so both lists must be the
        same length and non-decreasing or the walk is meaningless."""
        if not section:
            return None
        try:
            sh = [int(v) for v in section.get("shutter_us", [])]
            ga = [float(v) for v in section.get("gain", [])]
        except (TypeError, ValueError):
            return None
        if (len(sh) < 2 or len(sh) != len(ga) or sh != sorted(sh) or ga != sorted(ga)
                or sh[0] <= 0 or ga[0] < 1.0):
            return None
        return {"shutter": sh, "gain": ga}

    def _tuning(self):
        """Sensor tuning with the ``custom`` exposure mode filled in, or None
        for the stock file. Falls back to ``short`` if anything is missing, so
        a bad table can never leave AGC pointing at a mode that does not exist."""
        if self._ae_mode.lower() != "custom":
            return None
        mode = self.custom_exposure_mode(self._ae_custom)
        try:
            from picamera2 import Picamera2
            if mode is None:
                raise ValueError("cameras.picam.ae_custom missing or invalid")
            model = Picamera2.global_camera_info()[0]["Model"]
            tuning = Picamera2.load_tuning_file(f"{model}.json")
            agc = Picamera2.find_tuning_algo(tuning, "rpi.agc")
            for ch in agc.get("channels", [agc]):
                ch.setdefault("exposure_modes", {})["custom"] = mode
            _log.info("Pi camera: custom AE shutter %s us, gain %s", mode["shutter"], mode["gain"])
            return tuning
        except Exception as exc:  # noqa: BLE001
            _log.warning("Pi camera: custom exposure unavailable (%s) - using 'short'", exc)
            self._ae_mode = "short"
            return None

    def _note_sensor(self, md: Optional[dict]) -> None:
        if not md:
            return
        self._sensor_meta = {
            "exposure_us": int(md.get("ExposureTime", 0) or 0),
            "gain": round(float(md.get("AnalogueGain", 0.0) or 0.0), 2),
            "lens_dpt": (round(float(md["LensPosition"]), 2)
                         if md.get("LensPosition") is not None else None),
        }

    def info(self) -> dict:
        out = super().info()
        if self._sensor_meta and self.connected:
            # Shutter time is the number that decides vibration blur; the lens
            # position proves the focus setting actually reached the module.
            out["sensor"] = dict(self._sensor_meta)
        return out

    def _run(self) -> None:  # pragma: no cover - hardware loop
        try:
            import cv2
            from picamera2 import Picamera2
        except Exception as exc:  # noqa: BLE001
            _log.warning("Pi camera: picamera2/OpenCV unavailable (%s)", exc)
            return

        backoff = 1.0
        while not self._stop.is_set():
            picam = None
            grab_thread = None
            try:
                picam = Picamera2(tuning=self._tuning())
                # 'RGB888' from picamera2 is delivered in BGR byte order, which
                # is exactly what OpenCV (imencode) and the Hailo wrappers expect.
                # buffer_count=2 is the low-latency floor: picamera2 defaults to
                # ~4-6 queued buffers and delivers them FIFO, so a consumer that
                # briefly falls behind reads progressively OLDER frames and the
                # lag grows without bound. Two buffers bounds staleness to <=1
                # frame while still double-buffering so capture never starves.
                extra = {}
                if self._rotate_180:
                    try:
                        from libcamera import Transform
                        # 180 deg == horizontal flip + vertical flip. Applies to
                        # every ISP output, so the record stream is flipped too.
                        extra["transform"] = Transform(hflip=1, vflip=1)
                    except Exception as exc:  # noqa: BLE001
                        _log.warning("Pi camera: libcamera Transform unavailable "
                                     "(%s) - rotating on the CPU instead", exc)
                        self._sw_rotate = True
                live = {"size": (self._cap_w, self._cap_h), "format": "RGB888"}
                if self._record_size is not None:
                    # main = full-resolution record stream, lores = live stream.
                    # Both are scaled from the same 2304x1296 sensor mode by the
                    # ISP. JPEG-range YCbCr (sYCC) so the recorder can encode the
                    # YUV planes directly with no conversion and no colour shift.
                    try:
                        from libcamera import ColorSpace
                        extra["colour_space"] = ColorSpace.Sycc()
                    except Exception:  # noqa: BLE001
                        pass
                    cfg = picam.create_video_configuration(
                        main={"size": self._record_size, "format": "YUV420"},
                        lores=live, controls=self._sensor_controls(),
                        buffer_count=3, **extra)
                else:
                    cfg = picam.create_video_configuration(
                        main=live, controls=self._sensor_controls(),
                        buffer_count=2, **extra)
                picam.configure(cfg)
                picam.start()
                self.connected = True
                backoff = 1.0
                _log.info("Pi camera (imx708) connected: live %dx%d -> %dx%d, record %s "
                          "(ISP NR=%s, AE=%s, CPU filter=%s, rot180=%s, stab=%s, "
                          "person lock=%s)",
                          self._cap_w, self._cap_h, self._req_w, self._req_h,
                          "x".join(map(str, self._record_size)) if self._record_size else "stream",
                          self._isp_denoise, self._ae_mode,
                          "on" if (self._filter and self._filter.enabled) else "off",
                          "sw" if self._sw_rotate else ("isp" if self._rotate_180 else "off"),
                          self._stab.mode if self._stab else "off",
                          "on" if self._vision else "off")
                # start grabber
                self._grab_ok = True
                self._raw = None
                self._raw_main = None
                self._raw_t = 0.0
                self._raw_evt.clear()
                if self._stab is not None:
                    self._stab.reset()
                grab_thread = threading.Thread(
                    target=self._grabber, args=(picam,),
                    name=f"cam-{self.name}-grab", daemon=True)
                grab_thread.start()
                # process the freshest frame as fast as the pipeline allows
                while not self._stop.is_set():
                    if not self._raw_evt.wait(timeout=2.0):
                        if not self._grab_ok:
                            raise RuntimeError("grabber stalled")
                        continue
                    self._raw_evt.clear()
                    if not self._grab_ok:
                        raise RuntimeError("frame read failed")
                    with self._raw_lock:
                        frame, main, captured_at = self._raw, self._raw_main, self._raw_t
                        self._raw_main = None
                    if frame is None:
                        continue
                    if self._sw_rotate:
                        frame = cv2.rotate(frame, cv2.ROTATE_180)
                    if not self._keep():
                        continue
                    # Order matters: destripe works on sensor ROWS, so it runs
                    # before the stabiliser rotates them; the model must see the
                    # stabilised frame the box is drawn on, before the box is.
                    frame = self._condition(frame)
                    frame, stab, snap = self._stabilize_and_track(frame)
                    frame = self._annotate(frame)
                    jpeg, frame = self._encode_frame(cv2, frame)
                    if jpeg is not None:
                        h, w = frame.shape[:2]
                        self._publish(jpeg, w, h, captured_at)
                    rec = self._recorder
                    if main is not None and rec is not None:
                        try:
                            rec.offer_frame(self._yuv_planes(main), {
                                "t": captured_at,
                                "A": None if stab is None or not stab.ok
                                else stab.motion,
                                "box": snap.get("box") if snap else None,
                                "st": snap.get("state") if snap else None,
                                "sc": snap.get("score") if snap else None,
                                # Every YOLO box, not just the lock (HOVER only).
                                "dets": snap.get("dets") if snap else None,
                            })
                        except Exception as exc:  # noqa: BLE001
                            _log.warning("%s recorder error: %s", self.name, exc)
                    # After publishing, never before: the controller steers on
                    # what viewers received, so it wants the freshest reading.
                    self._adapt_tick()
            except Exception as exc:  # noqa: BLE001
                self.connected = False
                _log.warning("Pi camera error: %s (retry in %.0fs)", exc, backoff)
            finally:
                self._grab_ok = False
                if grab_thread is not None:
                    grab_thread.join(timeout=1.0)
                try:
                    if picam is not None:
                        picam.stop()
                        picam.close()
                except Exception:  # noqa: BLE001
                    pass
            self._stop.wait(backoff)
            backoff = min(backoff * 2, 10.0)

# --------------------------------------------------------------------------- #
# Novelty layer overlay wiring - see module docstring "Novelty layer tap"
# --------------------------------------------------------------------------- #
def _draw_person_detections(frame_bgr, detections: list) -> "object":
    """Draw PERSON boxes from an already-computed detection list - mirrors
    ``hailo_infer.Detector.draw()`` exactly (same colours/label), operating
    on ``PersonDetection`` objects instead of raw tuples since that is what
    the novelty registry's ``DetectorAdapter`` returns."""
    import cv2

    for d in detections:
        x1, y1, x2, y2 = d.bbox.x1, d.bbox.y1, d.bbox.x2, d.bbox.y2
        cv2.rectangle(frame_bgr, (x1, y1), (x2, y2), (0, 235, 255), 2)
        tag = f"PERSON {d.score * 100:.0f}%"
        (tw, th), _ = cv2.getTextSize(tag, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1)
        cv2.rectangle(frame_bgr, (x1, y1 - th - 6), (x1 + tw + 4, y1), (0, 235, 255), -1)
        cv2.putText(frame_bgr, tag, (x1 + 2, y1 - 4),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 1, cv2.LINE_AA)
    return frame_bgr


def _build_novelty_processors(bus: Optional["MessageBus"]):
    return None
    """Return the picam_processor built from the novelty
    ``ModelRegistry`` - see the module docstring's "Novelty layer tap".

    picam -> yolov8n person detection: publishes ``list[PersonDetection]``
             (ground-projection is ``DeliveryNode``'s job, not this one - see
             its own module docstring) on ``NoveltyTopics.PERSON_DETECTIONS``,
             then the same box+label overlay.

    Returns None if the novelty config, registry, or Hailo stack is
    unavailable so the cameras fall back to raw video with no error -
    matches the pre-existing graceful-degradation contract. If *bus* is
    None (no shared MessageBus available), the overlay still draws but
    nothing is published - a camera-only caller with no bus still works.
    """
    try:
        from drone_stack.novelty.config import NoveltyConfig
        from drone_stack.novelty.perception.model_registry import ModelRegistry
        from drone_stack.novelty.topics import NoveltyTopics
    except Exception as exc:  # noqa: BLE001
        _log.warning("novelty overlays disabled (import failed: %s)", exc)
        return None

    try:
        cfg = NoveltyConfig.load()
        registry = ModelRegistry.from_config(cfg.models)
    except Exception as exc:  # noqa: BLE001 - see perception/adapters.py's own
        # docstring: a busy/absent Hailo device can raise OUTSIDE the
        # adapters' own graceful-degradation boundary, so this construction
        # is wrapped here exactly like DeliveryNode wraps its own registry.
        _log.warning("novelty overlays disabled (config/registry failed: %s)", exc)
        return None

    picam_proc = None

    if "yolov8n" in registry.names() and registry.get("yolov8n").ok:
        person_model = registry.get("yolov8n")

        def picam_proc(frame_bgr, _model=person_model):
            out = _model.infer(frame_bgr)
            if bus is not None:
                bus.publish(NoveltyTopics.PERSON_DETECTIONS, out.detections)
            return _draw_person_detections(frame_bgr, out.detections)

        _log.info("Pi camera: yolov8n person-detection overlay ENABLED (novelty registry)")

    return picam_proc


class CameraManager:
    """Owns the Pi camera; lazy-starts capture on first access.

    Was two cameras until 2026-09-11, when the USB webcam was removed. The
    ``_cams`` dict, the ``/api/camera/{id}`` routes and ``infos()`` were always
    keyed by id rather than by position, so dropping one entry needed no
    renumbering - the Pi camera keeps id 1 and its stream URL is unchanged.
    """

    def __init__(self, bus: Optional["MessageBus"] = None,
                 enabled: bool = True,
                 settings: Optional[dict] = None) -> None:
        # *bus* is the shared MessageBus (GcsHub.bus) the novelty-layer
        # overlay processors publish structured detections/segmentation on
        # - see _build_novelty_processors. Optional so a caller with no bus
        # (e.g. a camera-only script) still gets working overlays, just
        # without the NoveltyTopics publish side effect.
        self._bus = bus
        # *settings* is the whole ``cameras`` config block. Everything below has
        # a working default, so a caller that passes nothing gets exactly the
        # behaviour this class had before the block existed.
        settings = settings or {}
        filter_cfg = settings.get("filter") or {}
        jpeg_quality = int(settings.get("jpeg_quality", 60))
        pi_cfg = settings.get("picam") or {}

        # The Pi camera is on the CSI ribbon running past the ESCs, so it is
        # the one that picks up row interference in flight and it is filtered
        # by default.
        pi_filter = (filter_from_config(filter_cfg)
                     if bool(pi_cfg.get("filter", True)) else None)

        picam_proc = _build_novelty_processors(bus)

        out_w = int(pi_cfg.get("width", 1280))
        out_h = int(pi_cfg.get("height", 720))
        fps = int(pi_cfg.get("fps", 30))
        # Stabilisation needs a crop margin, so the live frame is captured
        # larger (ISP-scaled, free) and cropped/warped back to width x height.
        stab_cfg = settings.get("stabilize") or {}
        stabilizer = stabilizer_from_config(stab_cfg, (out_w, out_h), fps)
        capture_size = None
        if stabilizer is not None:
            capture_size = (int(stab_cfg.get("capture_width", 1536)),
                            int(stab_cfg.get("capture_height", 864)))
            if capture_size[0] < out_w or capture_size[1] < out_h:
                capture_size = (out_w, out_h)
        # Built here, loaded later: the HEF only opens on the NPU worker thread
        # when the camera starts, so tests and the simulator never touch it.
        vision = person_lock_from_config(settings.get("person_lock"))
        record_size = None
        if pi_cfg.get("record_width") and pi_cfg.get("record_height"):
            record_size = (int(pi_cfg["record_width"]), int(pi_cfg["record_height"]))
        # The Pi cam is on CSI (no USB/LIDAR bus contention). Removing the C270
        # also gave the RPLIDAR back the USB bandwidth it used to contend for.
        self._cams: dict[int, _BaseCamera] = {
            1: PiCamera(1, name="P1", skip=1,
                        width=out_w, height=out_h,
                        fps=fps, processor=picam_proc,
                        frame_filter=pi_filter,
                        jpeg_quality=int(pi_cfg.get("jpeg_quality", jpeg_quality)),
                        isp_denoise=str(pi_cfg.get("isp_denoise", "fast")),
                        ae_mode=str(pi_cfg.get("ae_mode", "short")),
                        rotate_180=bool(pi_cfg.get("rotate_180", False)),
                        adaptive=bool(pi_cfg.get("adaptive", False)),
                        ladder=pi_cfg.get("adapt_ladder"),
                        target_fps=float(pi_cfg.get("adapt_target_fps", 0)) or None,
                        min_height=int(pi_cfg.get("min_height", 0)),
                        stabilizer=stabilizer, vision=vision,
                        capture_size=capture_size, record_size=record_size,
                        lens_position=pi_cfg.get("lens_position"),
                        ae_custom=pi_cfg.get("ae_custom")),
        }
        self._started = False
        self._enabled = enabled
        self._lock = threading.Lock()

        # Gate person lock by mission phase: enable only when hovering
        # over the target waypoint (camera facing down).
        if bus is not None:
            from drone_stack.bus.topics import Topics
            bus.subscribe(Topics.MISSION_STATE, self._on_mission_state)
            # The drop-point scan: the lock goes out to the navigator, and the
            # navigator's phone hint (which person holds the phone) comes back.
            if vision is not None:
                bus.subscribe(Topics.PHONE_HINT, self._on_phone_hint)
                self._cams[1].lock_sink = self._publish_lock
        self._lock_pub_key = None
        self._lock_pub_t = 0.0

    @property
    def enabled(self) -> bool:
        return self._enabled

    def _on_mission_state(self, msg) -> None:
        """Enable person lock when hovering over the target or idle on the ground."""
        from drone_stack.msg.messages import MissionPhase
        phase = getattr(msg, "phase", None)
        cam = self._cams.get(1)   # Pi camera
        if cam is None or getattr(cam, "_vision", None) is None:
            return
        # Active during HOVER (at target) and IDLE (ground testing).
        # Disabled during transit (NAVIGATE, TAKEOFF, RTL, etc.).
        cam._vision.enabled = phase in (MissionPhase.HOVER, MissionPhase.IDLE)

    def _on_phone_hint(self, msg) -> None:
        cam = self._cams.get(1)
        if cam is not None and getattr(cam, "_vision", None) is not None:
            cam._vision.set_hint(msg)

    LOCK_HEARTBEAT_S = 0.25

    def _publish_lock(self, snap: dict) -> None:
        """PersonLockState on a change of state/lock id/head count, and at
        least every LOCK_HEARTBEAT_S while the lock runs - the navigator treats
        a lock older than ~1 s as gone, so silence must mean "nothing seen"."""
        from drone_stack.bus.topics import Topics
        from drone_stack.msg.messages import PersonLockState
        people = snap.get("people_xy") or []
        key = (snap.get("state"), snap.get("id"), len(people))
        now = time.monotonic()
        if key == self._lock_pub_key and now - self._lock_pub_t < self.LOCK_HEARTBEAT_S:
            return
        self._lock_pub_key, self._lock_pub_t = key, now
        self._bus.publish(Topics.PERSON_LOCK, PersonLockState(
            cam_id=1, state=str(snap.get("state", "search")),
            score=float(snap.get("score", 0.0) or 0.0),
            lock_id=int(snap.get("id", 0) or 0),
            nx=float(snap.get("nx", 0.0)), ny=float(snap.get("ny", 0.0)),
            people=len(people), people_xy=[list(p) for p in people]))

    def start(self) -> None:
        """Bring the camera up. A no-op when cameras are disabled.

        The gate lives here rather than at the call sites because there are two
        of them - GcsHub.start() and the /api/camera/{id}/stream handler, which
        the dashboard hits on page load - and gating only the first left the
        second opening cameras anyway.
        """
        with self._lock:
            if not self._enabled:
                if not self._started:
                    self._started = True     # log the reason once, not per request
                    _log.info("cameras disabled by config - not starting capture")
                return
            if self._started:
                return
            self._started = True
        for cam in self._cams.values():
            cam.start()

    def stop(self) -> None:
        for cam in self._cams.values():
            cam.stop()

    def get(self, cam_id: int) -> Optional[_BaseCamera]:
        return self._cams.get(cam_id)

    def infos(self) -> list[dict]:
        return [self._cams[i].info() for i in sorted(self._cams)]
