#!/usr/bin/env python3
"""Physical left/right sanity check for the LiDAR, from the live GCS feed.

The RPLIDAR reports beams clockwise; the stack works counter-clockwise, so a
sign error anywhere in that conversion silently mirrors the scan - the radar
looks plausible and collision avoidance dodges INTO the obstacle. This is the
one thing unit tests cannot prove, so check it against a real object:

    1. leave the GCS running (it owns the serial port; this only reads /ws)
    2. stand a box ~2 m off, clearly to the RIGHT of the airframe's nose
    3. run this - the nearest return must land in a RIGHT sector

Usage:  .venv/bin/python scripts/lidar_orientation_check.py [--host pi.local:8090]
"""
from __future__ import annotations

import argparse
import asyncio
import json
import math

# (label, centre bearing) - bearings are 0 = nose, positive to the RIGHT.
SECTORS = [
    ("FRONT",       0),
    ("FRONT-RIGHT", 45),
    ("RIGHT",       90),
    ("REAR-RIGHT",  135),
    ("REAR",        180),
    ("REAR-LEFT",   225),
    ("LEFT",        270),
    ("FRONT-LEFT",  315),
]


def wrap180(a: float) -> float:
    return (a + 180.0) % 360.0 - 180.0


async def main(host: str, frames: int) -> None:
    import websockets

    nearest: dict[str, float] = {}
    fov = None
    seen = 0
    async with websockets.connect(f"ws://{host}/ws", max_size=None) as ws:
        while seen < frames:
            d = json.loads(await ws.recv())
            fov = d.get("lidar_fov", fov)
            scan = d.get("scan")
            if not scan or not scan.get("ranges"):
                continue
            seen += 1
            ranges = scan["ranges"]
            inc_deg = math.degrees(scan.get("angle_increment", math.radians(1.0)))
            for i, r in enumerate(ranges):
                if r is None or not math.isfinite(r):
                    continue
                # beam i is i*inc CCW from the nose; bearing is CW-positive
                bearing = wrap180(-i * inc_deg)
                label = min(SECTORS, key=lambda s: abs(wrap180(bearing - s[1])))[0]
                if r < nearest.get(label, math.inf):
                    nearest[label] = r

    print(f"host {host} · {seen} scans · fov {fov}\n")
    print(f"{'SECTOR':<12} {'NEAREST':>9}")
    print("-" * 22)
    for label, centre in SECTORS:
        blind = fov and fov.get("enabled") and abs(wrap180(centre)) > fov["half_deg"]
        if label in nearest:
            print(f"{label:<12} {nearest[label]:>8.2f}m")
        else:
            print(f"{label:<12} {'--- masked' if blind else '       ---':>9}")
    if nearest:
        closest = min(nearest, key=nearest.get)
        print(f"\nclosest return overall: {closest} at {nearest[closest]:.2f} m")
        print("If that is not the side the object is really on, the scan is mirrored:")
        print("  flip  lidar.clockwise  in config/real.yaml and restart the GCS.")


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--host", default="127.0.0.1:8090")
    ap.add_argument("--frames", type=int, default=5)
    a = ap.parse_args()
    asyncio.run(main(a.host, a.frames))
