#!/usr/bin/env python3
"""Read live flight_mode from the running GCS WebSocket and print on change.
Non-invasive: just another /ws client, no serial-port contention."""
import asyncio
import json
import sys
import time

import websockets

URL = "ws://127.0.0.1:8090/ws"
WINDOW_S = float(sys.argv[1]) if len(sys.argv) > 1 else 25.0


async def main():
    last = None
    t0 = time.time()
    async with websockets.connect(URL, max_size=None) as ws:
        print("connected to %s — FLIP SwC LOW/MID/HIGH now (%.0fs)" % (URL, WINDOW_S))
        while time.time() - t0 < WINDOW_S:
            try:
                raw = await asyncio.wait_for(ws.recv(), timeout=2)
            except asyncio.TimeoutError:
                continue
            try:
                data = json.loads(raw)
            except Exception:
                continue
            mode = (data.get("telemetry") or {}).get("flight_mode")
            if mode is not None and mode != last:
                print("t=%4.1fs  flight_mode = %s" % (time.time() - t0, mode))
                last = mode
    print("final flight_mode:", last)


asyncio.run(main())
