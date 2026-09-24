"""FastAPI + WebSocket server for the Drone GCS.

Run::

    python -m drone_stack.gcs.server --config config/sim.yaml --port 8000

Environment overrides (also honoured, matching the spec's .env approach):
    MAVLINK_PORT   -> mavlink.connection
    LIDAR_PORT     -> lidar.port
    GCS_PORT       -> HTTP port
"""
from __future__ import annotations

import argparse
import asyncio
import contextlib
import os
import socket
import sys
import time
from pathlib import Path

from drone_stack.gcs.hub import GcsHub
from drone_stack.gcs.ws_flow import FrameGate, PayloadDelta
from drone_stack.utils.config import Config
from drone_stack.utils.logging_setup import get_logger, setup_logging

try:
    import uvicorn
    from fastapi import FastAPI, WebSocket, WebSocketDisconnect
    from fastapi.responses import FileResponse, JSONResponse, StreamingResponse
    from fastapi.staticfiles import StaticFiles
except Exception as exc:  # noqa: BLE001
    raise SystemExit(
        "FastAPI/uvicorn not installed. Run: pip install -r requirements.txt"
    ) from exc

_STATIC = Path(__file__).resolve().parent / "static"
_log = get_logger("gcs.server")

# Changes on every GCS start. Console ids restart at 1 with the process, so a
# page that outlives a restart must know to forget its old high-water mark -
# otherwise it silently discards every new console line until refreshed.
_BOOT_ID = f"{time.time():.3f}"

# Unsent bytes the kernel may hold per connection. Linux otherwise autotunes a
# socket's send buffer up to tcp_wmem max (4 MB here) and never back down; the
# MJPEG stream filled 661 KB of it on a 1.83 Mbit/s link = 2.9 s of camera lag
# that only a refresh cleared. Capping it pushes backpressure up to uvicorn's
# flow control, so the stream's newest-frame-wins loop drops stale frames
# instead of the kernel queueing them. It limits only NOT-YET-SENT data, so
# the congestion window (throughput) is untouched.
_NOTSENT_LOWAT = 16 * 1024


def _apply_env(config: Config) -> Config:
    overrides: dict = {}
    raw = config.raw
    if os.environ.get("MAVLINK_PORT"):
        raw.setdefault("mavlink", {})["connection"] = os.environ["MAVLINK_PORT"]
    if os.environ.get("LIDAR_PORT"):
        raw.setdefault("lidar", {})["port"] = os.environ["LIDAR_PORT"]
    if overrides or os.environ.get("MAVLINK_PORT") or os.environ.get("LIDAR_PORT"):
        return Config(raw)
    return config


def create_app(config: Config) -> "FastAPI":
    hub = GcsHub(config)

    @contextlib.asynccontextmanager
    async def lifespan(app: "FastAPI"):
        hub.start()
        _log.info("GCS ready on http://0.0.0.0 (open the page)")
        try:
            yield
        finally:
            hub.stop()

    app = FastAPI(title="Drone GCS", lifespan=lifespan)
    app.state.hub = hub

    @app.get("/")
    async def index():
        return FileResponse(_STATIC / "index.html")

    @app.get("/api/state")
    async def state():
        """One-shot snapshot of exactly what the WebSocket streams.

        Lets scripts (scripts/inject_order.py) and health checks read live
        state without opening a WebSocket, and makes the delivery panel's data
        inspectable with curl when something looks wrong on the dashboard.
        """
        return JSONResponse(hub.build_payload())

    @app.get("/api/logs")
    async def logs():
        return JSONResponse({"logs": hub.list_logs()})

    @app.get("/api/mission/plan")
    async def download_plan():
        return JSONResponse(hub.export_plan())

    @app.post("/api/mission/plan")
    async def upload_plan(plan: dict):
        return JSONResponse(hub.import_plan(plan))

    @app.post("/api/command")
    async def command(body: dict):
        return JSONResponse(hub.command(body.get("cmd", ""), body.get("params", {})))

    @app.get("/api/recording")
    async def recording():
        """Recorder state + the kept clip's metadata.

        The same dict the WebSocket ships every frame, so a script or a curl
        can read it without opening a socket - matching /api/state.
        """
        return JSONResponse(hub.recorder.status())

    @app.get("/api/recording/video")
    async def recording_video():
        """The one kept clip.

        FileResponse honours Range requests, which is what lets the dashboard's
        <video> element seek instead of only playing straight through.

        no-store is not optional here: the clip lives at a FIXED url and its
        contents change on every flight. Without it the browser would happily
        replay the PREVIOUS flight from cache and look like the recorder had
        silently stopped working - exactly the class of bug the UI no-cache
        middleware was added for on 2026-09-11.
        """
        path = hub.recorder.clip_path()
        if path is None:
            return JSONResponse({"error": "no recording available"},
                                status_code=404)
        return FileResponse(
            path, media_type="video/mp4",
            headers={"Cache-Control": "no-store"},
        )

    @app.get("/api/recording/poster.jpg")
    async def recording_poster():
        """First frame of the kept clip, written straight from the recorded
        bytes - no decode, no re-encode. Gives the replay tile a real thumbnail
        of that flight rather than a generic placeholder."""
        path = hub.recorder.poster_path()
        if path is None:
            return JSONResponse({"error": "no recording available"},
                                status_code=404)
        return FileResponse(
            path, media_type="image/jpeg",
            headers={"Cache-Control": "no-store"},
        )

    @app.get("/api/camera/{cam_id}")
    async def camera(cam_id: int):
        cam = hub.cameras.get(cam_id)
        if cam is None:
            return JSONResponse({"id": cam_id, "connected": False})
        return JSONResponse(cam.info())

    @app.get("/api/camera/{cam_id}/stream")
    async def camera_stream(cam_id: int):
        cam = hub.cameras.get(cam_id)
        if cam is None:
            return JSONResponse({"error": "no such camera"}, status_code=404)
        hub.cameras.start()
        boundary = "frame"

        async def gen():
            # Event-driven, not polled. The original loop woke on a 1/30 s timer
            # and re-checked, so a frame finishing just after a tick waited out
            # the rest of the period before being sent - up to 33 ms added to
            # every frame, plus a wakeup 30 times a second per viewer whether or
            # not anything had changed. This awaits the camera's own publish
            # notification instead, so each frame goes out the instant it is
            # encoded and an idle stream costs nothing at all.
            last_seq = -1
            new_frame = cam.subscribe()
            # Delivered-rate feedback for the adaptive controller. Measured
            # HERE, at the yield, rather than at the encoder: resuming this
            # generator is gated by ASGI backpressure, so the rate this loop
            # actually achieves IS the rate the link sustains. The encoder has
            # no way to see that by itself.
            key = id(new_frame)
            sent, mark = 0, time.monotonic()
            try:
                while True:
                    jpeg, seq = cam.jpeg_seq()
                    if seq != last_seq:
                        last_seq = seq
                        yield (
                            b"--" + boundary.encode() + b"\r\n"
                            b"Content-Type: image/jpeg\r\n"
                            b"Content-Length: " + str(len(jpeg)).encode()
                            + b"\r\n\r\n" + jpeg + b"\r\n"
                        )
                        sent += 1
                        now = time.monotonic()
                        if now - mark >= 1.0:
                            cam.report_delivered(key, sent / (now - mark))
                            sent, mark = 0, now
                        continue
                    new_frame.clear()
                    # Re-read after clearing: a frame published between the read
                    # above and the clear would otherwise be lost and we would
                    # wait a full second through it.
                    if cam.jpeg_seq()[1] != last_seq:
                        continue
                    try:
                        # Bounded, so a camera that stops delivering cannot wedge
                        # the request open forever.
                        await asyncio.wait_for(new_frame.wait(), timeout=1.0)
                    except asyncio.TimeoutError:
                        pass
            except asyncio.CancelledError:
                return
            finally:
                cam.unsubscribe(new_frame)
                cam.drop_delivery(key)

        return StreamingResponse(
            gen(),
            media_type=f"multipart/x-mixed-replace; boundary={boundary}",
            # Defeat every downstream buffer so latency can't accumulate:
            #  - no-store/no-cache: browser never holds frames back
            #  - X-Accel-Buffering: no  -> disables nginx/proxy response buffering
            # The generator itself already emits only the *newest* frame (it re-
            # reads cam.jpeg() each pass), so under a slow link stale frames are
            # dropped rather than queued -> the stream stays live, never lagging.
            headers={
                "Cache-Control": "no-cache, no-store, must-revalidate",
                "Pragma": "no-cache",
                "X-Accel-Buffering": "no",
            },
        )

    @app.websocket("/ws")
    async def ws(socket: WebSocket):
        await socket.accept()
        _log.info("client connected")

        # Per-connection: a credit window so a slow page gets fewer fresh
        # frames instead of an ever-growing queue, and a thinner that stops
        # re-sending what this page already has. See ws_flow.py.
        gate, delta = FrameGate(), PayloadDelta()

        async def sender():
            try:
                while True:
                    await gate.wait_turn()
                    payload = delta.apply(hub.build_payload())
                    payload["seq"] = gate.next_seq()
                    payload["boot"] = _BOOT_ID
                    await socket.send_json(payload)
                    await asyncio.sleep(1.0 / 15.0)
            except (WebSocketDisconnect, RuntimeError):
                pass

        async def receiver():
            try:
                while True:
                    msg = await socket.receive_json()
                    if msg.get("cmd") == "frame_ack":
                        gate.on_ack(msg.get("seq"))
                        continue
                    if msg.get("cmd") == "ping":
                        await socket.send_json({"pong": msg.get("t")})
                        continue
                    result = hub.command(msg.get("cmd", ""), msg.get("params", {}))
                    await socket.send_json({"ack": msg.get("cmd"), "result": result})
            except (WebSocketDisconnect, RuntimeError):
                pass

        send_task = asyncio.create_task(sender())
        recv_task = asyncio.create_task(receiver())
        done, pending = await asyncio.wait(
            {send_task, recv_task}, return_when=asyncio.FIRST_COMPLETED
        )
        for task in pending:
            task.cancel()
        _log.info("client disconnected")

    @app.middleware("http")
    async def _revalidate_ui(request, call_next):
        """Make the browser revalidate the dashboard's own files.

        StaticFiles/FileResponse send ETag + Last-Modified but NO
        Cache-Control, so browsers fall back to *heuristic* caching and will
        serve a stale index.html/app.js for a long time without ever asking
        the server. That is why a UI change that is provably deployed can
        still look absent until a hard refresh - which cost real debugging
        time on 2026-09-11.

        "no-cache" does not mean "do not store": it means "always
        revalidate". The ETag above is preserved, so an unchanged file is
        still answered with a cheap 304 and no body - this costs one
        conditional request per file per load, not a re-download.

        Deliberately scoped to the UI documents only. The MJPEG stream and
        the JSON APIs are already uncacheable, and tagging them would just
        add a header to every frame.
        """
        response = await call_next(request)
        path = request.url.path
        if path == "/" or path.startswith("/static/"):
            response.headers["Cache-Control"] = "no-cache"
        return response

    app.mount("/static", StaticFiles(directory=_STATIC), name="static")
    return app


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="Drone GCS server")
    parser.add_argument("--config", "-c", default=None, help="profile YAML")
    parser.add_argument("--host", default="0.0.0.0")
    parser.add_argument(
        "--port", type=int, default=int(os.environ.get("GCS_PORT", "8000"))
    )
    args = parser.parse_args(argv)

    config = _apply_env(Config.load(profile_path=args.config))
    setup_logging(config)
    app = create_app(config)
    sock = _bounded_listener(args.host, args.port)
    server = uvicorn.Server(uvicorn.Config(app, log_level="warning"))
    server.run(sockets=[sock])
    return 0


def _bounded_listener(host: str, port: int) -> socket.socket:
    """Listening socket whose accepted connections cannot bufferbloat.

    Accepted sockets inherit TCP_NOTSENT_LOWAT from the listener (verified on
    this Pi's kernel), so one option here covers the camera stream, the
    WebSocket and everything else. If the platform lacks the option the
    server still starts, just without the cap.
    """
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    opt = getattr(socket, "TCP_NOTSENT_LOWAT", 25 if sys.platform.startswith("linux") else None)
    if opt is not None:
        try:
            sock.setsockopt(socket.IPPROTO_TCP, opt, _NOTSENT_LOWAT)
        except OSError as exc:
            _log.warning("TCP_NOTSENT_LOWAT unavailable (%s); streams may lag", exc)
    sock.bind((host, port))
    sock.listen(128)
    sock.set_inheritable(True)
    return sock


if __name__ == "__main__":
    raise SystemExit(main())
