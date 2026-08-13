"""Live camera capture for the Drone GCS.

Two independent capture threads, each owning one physical camera and keeping the
*latest* JPEG frame ready for the MJPEG HTTP endpoint:

* ``usb``   - Logitech C270 (or any UVC webcam) via OpenCV / V4L2, MJPG @ 640x480.
* ``picam`` - Raspberry Pi Camera (imx708) via ``picamera2``.

Each camera can carry an optional **processor** - a callable that takes the
freshly captured BGR frame (numpy) and returns an annotated BGR frame. This is
where the Hailo-8 NPU overlays are drawn (terrain segmentation on the webcam,
person detection on the Pi camera). All neural inference happens on the Hailo
accelerator, never on the Pi CPU; the processor only does the (cheap) draw +
JPEG encode on the CPU. Processors run **only on kept frames** (after frame
skipping) so the CPU cost is bounded.

Everything degrades gracefully: if a camera (or its library, or the Hailo HAT)
is missing the stream simply reports ``connected: False`` / serves the raw frame,
so the GCS still runs in pure simulation with no hardware attached.
"""
from __future__ import annotations

import io
import os
import threading
import time
from typing import Callable, Optional

from drone_stack.utils.logging_setup import get_logger

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


class _BaseCamera:
    def __init__(self, cam_id: int, name: str, skip: int = 1,
                 processor: Optional[Processor] = None) -> None:
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

    def _keep(self) -> bool:
        self._skip_ctr += 1
        return (self._skip_ctr % self._skip) == 0

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
        self._thread = threading.Thread(
            target=self._run, name=f"cam-{self.name}", daemon=True
        )
        self._thread.start()

    def stop(self) -> None:
        self._stop.set()
        if self._thread:
            self._thread.join(timeout=2.0)

    # -- accessors -----------------------------------------------------------
    def jpeg(self) -> bytes:
        with self._lock:
            return self._jpeg

    def _publish(self, jpeg: bytes, w: int, h: int) -> None:
        with self._lock:
            self._jpeg = jpeg
            self.width, self.height = w, h
        self._frames += 1
        now = time.monotonic()
        dt = now - self._fps_t
        if dt >= 1.0:
            self._fps = round(self._frames / dt, 1)
            self._frames = 0
            self._fps_t = now

    def info(self) -> dict:
        return {
            "id": self.cam_id,
            "name": self.name,
            "res": f"{self.width}x{self.height}" if self.connected else "--",
            "fps": self._fps if self.connected else 0,
            "connected": self.connected,
        }

    # -- override ------------------------------------------------------------
    def _run(self) -> None:  # pragma: no cover - hardware loop
        raise NotImplementedError


def _find_usb_camera_index(preferred: int = 0) -> int:
    """Locate the UVC webcam's V4L2 *capture* node.

    Device numbers are NOT stable across reboots: when the Pi CSI pipeline
    (rp1-cfe / pispbe) initialises it can claim /dev/video0..7, pushing the USB
    C270 to a higher node (e.g. /dev/video8). Hard-coding an index therefore
    silently breaks after a reboot ("USB cam offline"). We instead scan for a
    uvcvideo device that actually exposes image formats (MJPG/YUYV): this skips
    the CSI devices (different driver) and the webcam metadata node (no
    formats). Falls back to ``preferred`` when nothing matches.
    """
    import glob
    import subprocess

    def _num(path: str) -> int:
        tail = path.rsplit("video", 1)[-1]
        return int(tail) if tail.isdigit() else 999

    for dev in sorted(glob.glob("/dev/video*"), key=_num):
        try:
            info = subprocess.run(["v4l2-ctl", "-d", dev, "--info"],
                                  capture_output=True, text=True,
                                  timeout=3).stdout
            if "uvcvideo" not in info:
                continue
            fmts = subprocess.run(["v4l2-ctl", "-d", dev, "--list-formats"],
                                  capture_output=True, text=True,
                                  timeout=3).stdout
            if "MJPG" in fmts or "YUYV" in fmts:
                idx = _num(dev)
                _log.info("USB camera auto-detected at %s (index=%d)", dev, idx)
                return idx
        except Exception:  # noqa: BLE001
            continue
    _log.warning("USB camera: no UVC capture node found; "
                 "falling back to index=%d", preferred)
    return preferred


class UsbCamera(_BaseCamera):
    """UVC webcam (Logitech C270) via OpenCV V4L2, native MJPG.

    Capture and processing are **decoupled**: a lightweight grabber thread keeps
    only the freshest raw frame, while ``_run`` annotates (Hailo terrain overlay)
    + JPEG-encodes the latest frame in parallel. This stops the ~30 ms overlay
    from throttling the capture cadence, so the published rate tracks the camera
    instead of ``read + overlay`` summed.

    Auto-exposure is pinned to **manual** (``auto_exposure=1``) with a fixed
    exposure/gain. The C270's aperture-priority auto-exposure otherwise stretches
    the exposure time in low light and collapses the frame rate (e.g. 7 fps in a
    dark room), which broke the >=15 fps requirement. Manual exposure keeps the
    frame rate lighting-independent. Tune via ``USB_EXPOSURE`` / ``USB_GAIN``.
    """

    def __init__(self, cam_id: int, index: int, name: str = "USB",
                 width: int = 1280, height: int = 720, skip: int = 1,
                 processor: Optional[Processor] = None,
                 exposure: int = 200, gain: int = 200) -> None:
        super().__init__(cam_id, name, skip=skip, processor=processor)
        self.index = index
        self._req_w, self._req_h = width, height
        self._exposure = int(os.environ.get("USB_EXPOSURE", exposure))
        self._gain = int(os.environ.get("USB_GAIN", gain))
        self._raw = None            # latest captured BGR frame
        self._raw_lock = threading.Lock()
        self._raw_evt = threading.Event()

    def _apply_exposure(self) -> None:
        """Pin manual exposure via v4l2-ctl (must run *after* the device opens;
        opening the device resets UVC controls to their defaults)."""
        import subprocess
        try:
            subprocess.run(
                ["v4l2-ctl", "-d", f"/dev/video{self.index}",
                 "-c", "auto_exposure=1",
                 "-c", f"exposure_time_absolute={self._exposure}",
                 "-c", f"gain={self._gain}"],
                check=False, capture_output=True, timeout=3,
            )
        except Exception as exc:  # noqa: BLE001
            _log.warning("USB camera: could not set manual exposure (%s)", exc)

    def _grabber(self, cap) -> None:
        """Continuously read the newest frame; never blocks the encoder."""
        import cv2  # noqa: F401  (already imported by _run; keeps thread self-contained)
        while not self._stop.is_set() and self._grab_ok:
            ok, frame = cap.read()
            if not ok or frame is None:
                self._grab_ok = False
                self._raw_evt.set()
                return
            with self._raw_lock:
                self._raw = frame
            self._raw_evt.set()

    def _run(self) -> None:  # pragma: no cover - hardware loop
        try:
            import cv2
        except Exception as exc:  # noqa: BLE001
            _log.warning("USB camera: OpenCV unavailable (%s)", exc)
            return
        backoff = 1.0
        while not self._stop.is_set():
            self.index = _find_usb_camera_index(self.index)
            cap = cv2.VideoCapture(self.index, cv2.CAP_V4L2)
            grab_thread = None
            try:
                cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
                cap.set(cv2.CAP_PROP_FRAME_WIDTH, self._req_w)
                cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self._req_h)
                cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
                if not cap.isOpened():
                    raise RuntimeError(f"cannot open /dev/video{self.index}")
                cap.read()                 # force stream negotiation before ...
                self._apply_exposure()     # ... pinning manual exposure
                self.connected = True
                backoff = 1.0
                _log.info("USB camera connected on /dev/video%d "
                          "(exp=%d gain=%d, decoupled capture)",
                          self.index, self._exposure, self._gain)
                # start grabber
                self._grab_ok = True
                self._raw = None
                self._raw_evt.clear()
                grab_thread = threading.Thread(
                    target=self._grabber, args=(cap,),
                    name=f"cam-{self.name}-grab", daemon=True)
                grab_thread.start()
                # process the freshest frame as fast as annotate+encode allows
                while not self._stop.is_set():
                    if not self._raw_evt.wait(timeout=2.0):
                        if not self._grab_ok:
                            raise RuntimeError("grabber stalled")
                        continue
                    self._raw_evt.clear()
                    if not self._grab_ok:
                        raise RuntimeError("frame read failed")
                    with self._raw_lock:
                        frame = self._raw
                    if frame is None:
                        continue
                    frame = self._annotate(frame)   # Hailo terrain overlay (NPU)
                    ok, buf = cv2.imencode(
                        ".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, 60]
                    )
                    if ok:
                        h, w = frame.shape[:2]
                        self._publish(buf.tobytes(), w, h)
            except Exception as exc:  # noqa: BLE001
                self.connected = False
                _log.warning("USB camera error: %s (retry in %.0fs)", exc, backoff)
            finally:
                self._grab_ok = False
                if grab_thread is not None:
                    grab_thread.join(timeout=1.0)
                try:
                    cap.release()
                except Exception:  # noqa: BLE001
                    pass
            self._stop.wait(backoff)
            backoff = min(backoff * 2, 10.0)


class PiCamera(_BaseCamera):
    """Raspberry Pi camera (imx708) via picamera2.

    When a ``processor`` is attached we capture raw frames as numpy arrays so the
    Hailo overlay can be drawn, then JPEG-encode with OpenCV. With no processor
    we still use the array path (uniform code); encoding 720p on the Pi 5 CPU at
    ~15 fps is cheap and keeps the pipeline identical to the USB camera.
    """

    def __init__(self, cam_id: int, name: str = "PiCam",
                 width: int = 1280, height: int = 720, skip: int = 1,
                 fps: int = 32, processor: Optional[Processor] = None) -> None:
        super().__init__(cam_id, name, skip=skip, processor=processor)
        self._req_w, self._req_h = width, height
        self._fps_cap = fps

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
            try:
                picam = Picamera2()
                # 'RGB888' from picamera2 is delivered in BGR byte order, which
                # is exactly what OpenCV (imencode) and the Hailo wrappers expect.
                # buffer_count=2 is the low-latency floor: picamera2 defaults to
                # ~4-6 queued buffers and delivers them FIFO, so a consumer that
                # briefly falls behind reads progressively OLDER frames and the
                # lag grows without bound. Two buffers bounds staleness to <=1
                # frame while still double-buffering so capture never starves.
                cfg = picam.create_video_configuration(
                    main={"size": (self._req_w, self._req_h), "format": "RGB888"},
                    controls={"FrameRate": self._fps_cap},
                    buffer_count=2,
                )
                picam.configure(cfg)
                picam.start()
                self.connected = True
                backoff = 1.0
                _log.info("Pi camera (imx708) connected via capture_array")
                while not self._stop.is_set():
                    frame = picam.capture_array()   # BGR, HxWx3
                    if frame is None:
                        raise RuntimeError("capture_array returned None")
                    if not self._keep():
                        continue
                    frame = self._annotate(frame)   # Hailo person-detection overlay
                    ok, buf = cv2.imencode(
                        ".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, 60]
                    )
                    if ok:
                        h, w = frame.shape[:2]
                        self._publish(buf.tobytes(), w, h)
            except Exception as exc:  # noqa: BLE001
                self.connected = False
                _log.warning("Pi camera error: %s (retry in %.0fs)", exc, backoff)
            finally:
                try:
                    if picam is not None:
                        picam.stop()
                        picam.close()
                except Exception:  # noqa: BLE001
                    pass
            self._stop.wait(backoff)
            backoff = min(backoff * 2, 10.0)


# --------------------------------------------------------------------------- #
# Hailo overlay wiring
# --------------------------------------------------------------------------- #
_MODELS_DIR = os.environ.get(
    "HAILO_MODELS_DIR",
    os.path.join(os.path.dirname(os.path.dirname(os.path.dirname(__file__))), "models"),
)


def _build_hailo_processors():
    """Return (usb_processor, picam_processor).

    usb   -> terrain segmentation overlay,
    picam -> yolov8n person-detection overlay.

    Returns (None, None) if the Hailo stack / models are unavailable so the
    cameras fall back to raw video with no error.
    """
    try:
        from drone_stack.gcs.hailo_infer import Detector, Segmenter
    except Exception as exc:  # noqa: BLE001
        _log.warning("Hailo overlays disabled (import failed: %s)", exc)
        return None, None

    terrain = os.path.join(_MODELS_DIR, "terrain.hef")
    yolo = os.path.join(_MODELS_DIR, "yolov8n.hef")
    usb_proc = picam_proc = None
    try:
        seg = Segmenter(terrain)
        if seg.ok:
            usb_proc = seg.process
            _log.info("USB webcam: terrain segmentation overlay ENABLED (Hailo)")
    except Exception as exc:  # noqa: BLE001
        _log.warning("terrain segmenter init failed: %s", exc)
    try:
        det = Detector(yolo, score_thr=0.40)
        if det.ok:
            picam_proc = det.process
            _log.info("Pi camera: yolov8n person-detection overlay ENABLED (Hailo)")
    except Exception as exc:  # noqa: BLE001
        _log.warning("yolov8n detector init failed: %s", exc)
    return usb_proc, picam_proc


class CameraManager:
    """Owns both cameras; lazy-starts capture on first access."""

    def __init__(self) -> None:
        # NOTE: the USB C270 is capped at 640x480. At 1280x720 its USB power +
        # bandwidth draw starves the RPLIDAR C1 (shared USB bus), stalling the
        # LIDAR motor. 640x480 lets both run - LIDAR holds a full 10 Hz. The Pi
        # camera is on CSI (not USB), so it keeps 1280x720 without contention.
        usb_proc, picam_proc = _build_hailo_processors()
        self._cams: dict[int, _BaseCamera] = {
            0: UsbCamera(0, index=0, name="USB", width=640, height=480, skip=1,
                         processor=usb_proc),
            # Pi cam is on CSI (no USB/LIDAR contention) and detection is ~9 ms,
            # so run every frame at 30 fps for the "excellent" range. USB stays
            # decoupled at ~15 fps (its C270 is hardware-capped at 640x480).
            1: PiCamera(1, name="P1", skip=1, fps=30, processor=picam_proc),
        }
        self._started = False
        self._lock = threading.Lock()

    def start(self) -> None:
        with self._lock:
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
