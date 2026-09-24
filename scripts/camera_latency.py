#!/usr/bin/env python3
"""Measure where camera lag actually comes from, stage by stage.

Run it on the Pi while the GCS service is up - it uses only the HTTP API, so it
never touches the camera device and cannot fight the service for it.

    python3 scripts/camera_latency.py            # the Pi camera, 8 s
    python3 scripts/camera_latency.py --cam 1 --seconds 15

What it separates, and why that is the whole point:

* **pipeline latency** - grab -> filter -> overlay -> encode, timed inside the
  capture thread from a stamp taken at capture. This is what a slow filter
  inflates. It is reported by the camera itself.
* **encode rate** - how often the capture thread publishes a new frame.
* **delivered rate** - how often a frame actually arrives over HTTP.

The gap between the last two is the diagnosis:

* encode rate low, delivered rate matches it  -> the Pi is the bottleneck
  (filter, encode, or a starved CPU). Look at pipeline latency and filter cost.
* encode rate fine, delivered rate lower      -> the link is the bottleneck.
  Look at the bandwidth figure and drop `picam.fps` or `jpeg_quality`.
* both fine but the picture still feels late  -> the lag is downstream of here
  (browser, Wi-Fi buffering); compare against a second viewer.
"""
from __future__ import annotations

import argparse
import json
import sys
import time
import urllib.request

DEFAULT_BASE = "http://127.0.0.1:8090"

# The GCS is a device on the local network, never something to reach through a
# proxy. urllib picks up the host's system/environment proxy settings by
# default, which on a laptop makes a perfectly reachable Pi fail with a bare
# "No route to host" while curl to the same address works fine.
_OPENER = urllib.request.build_opener(urllib.request.ProxyHandler({}))


def _get_json(url: str, timeout: float = 5.0):
    with _OPENER.open(url, timeout=timeout) as resp:
        return json.loads(resp.read().decode())


def _sample_stream(url: str, seconds: float):
    """Read the MJPEG stream and time real frame arrivals.

    Frames are located by the JPEG SOI/EOI markers rather than by trusting
    Content-Length, so a truncated frame is counted as truncated instead of
    silently shifting every measurement after it.
    """
    arrivals: list[float] = []
    sizes: list[int] = []
    truncated = 0
    buf = b""
    total = 0
    deadline = time.monotonic() + seconds
    try:
        with _OPENER.open(url, timeout=10.0) as resp:
            while time.monotonic() < deadline:
                chunk = resp.read(16384)
                if not chunk:
                    break
                total += len(chunk)
                buf += chunk
                while True:
                    start = buf.find(b"\xff\xd8\xff")
                    if start < 0:
                        break
                    end = buf.find(b"\xff\xd9", start + 3)
                    if end < 0:
                        if len(buf) > 8 << 20:      # runaway; something is wrong
                            truncated += 1
                            buf = b""
                        break
                    arrivals.append(time.monotonic())
                    sizes.append(end + 2 - start)
                    buf = buf[end + 2:]
    except Exception as exc:  # noqa: BLE001
        print(f"    stream read stopped: {exc}")
    return arrivals, sizes, truncated, total


def _percentile(values: list[float], pct: float) -> float:
    if not values:
        return 0.0
    ordered = sorted(values)
    idx = min(len(ordered) - 1, int(round((pct / 100.0) * (len(ordered) - 1))))
    return ordered[idx]


def report(base: str, cam_id: int, seconds: float) -> int:
    print(f"\n=== camera {cam_id} ===")
    try:
        before = _get_json(f"{base}/api/camera/{cam_id}")
    except Exception as exc:  # noqa: BLE001
        print(f"  cannot reach {base}: {exc}")
        return 1
    if not before.get("connected"):
        print("  NOT CONNECTED - nothing to measure")
        return 1

    print(f"  {before.get('name','?')} @ {before.get('res','?')}"
          f"   reported {before.get('fps', 0)} fps")

    print(f"  sampling the live stream for {seconds:.0f}s ...")
    arrivals, sizes, truncated, total = _sample_stream(
        f"{base}/api/camera/{cam_id}/stream", seconds)
    after = _get_json(f"{base}/api/camera/{cam_id}")

    if len(arrivals) < 3:
        print(f"  only {len(arrivals)} frames arrived - the stream is stalled")
        return 1

    span = arrivals[-1] - arrivals[0]
    delivered = (len(arrivals) - 1) / span if span > 0 else 0.0
    gaps = [(b - a) * 1000.0 for a, b in zip(arrivals, arrivals[1:])]
    mbps = (total / span) * 8 / 1e6 if span > 0 else 0.0
    encode_fps = float(after.get("fps", 0) or 0)
    pipeline = float(after.get("latency_ms", 0) or 0)

    print()
    print(f"  pipeline latency   {pipeline:7.1f} ms   (capture -> published)")
    print(f"  encode rate        {encode_fps:7.1f} fps  (camera thread)")
    print(f"  delivered rate     {delivered:7.1f} fps  (over HTTP)")
    print(f"  frame interval     p50 {_percentile(gaps,50):.0f} ms"
          f"   p95 {_percentile(gaps,95):.0f} ms"
          f"   worst {max(gaps):.0f} ms")
    print(f"  frame size         {sum(sizes)/len(sizes)/1024:7.1f} KB avg"
          f"   -> {mbps:.1f} Mbit/s")
    if truncated:
        print(f"  TRUNCATED FRAMES   {truncated}")

    filt = after.get("filter")
    if filt:
        print(f"  filter             {filt.get('cost_ms', 0):7.2f} ms"
              f"   rows corrected {filt.get('rows_corrected', 0)}"
              f"   repaired {filt.get('rows_repaired', 0)}")
        if filt.get("degraded"):
            print("                     DEGRADED - temporal denoise auto-disabled")
        if filt.get("rows_repaired", 0) > 0:
            print("                     ^ rows being rebuilt: this is the EMI signature")

    # ---- the verdict -------------------------------------------------------
    print()
    if pipeline > 150:
        print(f"  VERDICT: the Pi is the bottleneck. {pipeline:.0f} ms is spent")
        print("           between capture and publish. Check the filter cost above;")
        print("           if it is small, the encode or a starved CPU is the cause")
        print("           (`top` while this runs). If fastNlMeans is still in")
        print("           cameras.py, that is it - this build was never deployed.")
    elif encode_fps > 0 and delivered < encode_fps * 0.7:
        print(f"  VERDICT: the link is the bottleneck. The camera publishes")
        print(f"           {encode_fps:.0f} fps but only {delivered:.0f} fps arrive,")
        print(f"           at {mbps:.1f} Mbit/s. Lower `picam.fps` or `jpeg_quality`,")
        print("           or drop `picam.width/height`, in the cameras: config block.")
    elif max(gaps) > 500:
        print(f"  VERDICT: rate is fine but delivery stalls (worst gap"
              f" {max(gaps):.0f} ms).")
        print("           That is a link dropout, not a processing cost.")
    else:
        print(f"  VERDICT: healthy. {pipeline:.0f} ms pipeline,"
              f" {delivered:.0f} fps delivered, {mbps:.1f} Mbit/s.")
        print("           Any lag you still see is downstream: browser buffering")
        print("           or Wi-Fi. Try a second viewer on a wired link to confirm.")
    return 0


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--base", default=DEFAULT_BASE, help="GCS base URL")
    ap.add_argument("--cam", type=int, action="append",
                    help="camera id (repeatable); default the Pi cam (1)")
    ap.add_argument("--seconds", type=float, default=8.0)
    args = ap.parse_args()

    # Cam 0 was the USB C270, removed 2026-09-11. The Pi camera keeps id 1.
    cams = args.cam or [1]
    worst = 0
    for cam_id in cams:
        worst = max(worst, report(args.base, cam_id, args.seconds))
    print()
    return worst


if __name__ == "__main__":
    raise SystemExit(main())
