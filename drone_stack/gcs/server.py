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
from pathlib import Path

from drone_stack.gcs.hub import GcsHub
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
            last = None
            try:
                while True:
                    jpeg = cam.jpeg()
                    if jpeg is not last:
                        last = jpeg
                        yield (
                            b"--" + boundary.encode() + b"\r\n"
                            b"Content-Type: image/jpeg\r\n"
                            b"Content-Length: " + str(len(jpeg)).encode()
                            + b"\r\n\r\n" + jpeg + b"\r\n"
                        )
                    await asyncio.sleep(1 / 30)
            except asyncio.CancelledError:
                return

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

        async def sender():
            try:
                while True:
                    await socket.send_json(hub.build_payload())
                    await asyncio.sleep(1.0 / 15.0)
            except (WebSocketDisconnect, RuntimeError):
                pass

        async def receiver():
            try:
                while True:
                    msg = await socket.receive_json()
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
    uvicorn.run(app, host=args.host, port=args.port, log_level="warning")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
