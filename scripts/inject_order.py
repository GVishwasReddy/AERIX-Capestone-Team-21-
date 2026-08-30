#!/usr/bin/env python3
"""Inject a delivery order into a running stack, without Firebase.

The delivery chain (order -> expanded waypoints -> flight -> 60 s hold -> smart
RTL) should be testable before anyone downloads a service-account key, and
before anyone trusts it with a real aircraft. This script drives the same
``delivery_inject`` service the GCS "Test order" button uses, over the GCS HTTP
API, so what it exercises is the production path and not a parallel one.

Examples::

    # 40 m north of the drone's current position, hold 60 s, then come home
    scripts/inject_order.py --north 40

    # an explicit destination, accepted immediately (simulation only!)
    scripts/inject_order.py --lat 12.90211 --lon 77.65442 --auto

    # write a pending_order.json for the file-backed source instead
    scripts/inject_order.py --north 40 --to-file

Nothing here arms anything by itself: without ``--auto`` the order lands on the
GCS as PENDING and waits for someone to press ACCEPT & FLY.
"""
from __future__ import annotations

import argparse
import json
import math
import sys
import urllib.error
import urllib.request
from pathlib import Path

REPO = Path(__file__).resolve().parents[1]
DEFAULT_URL = "http://127.0.0.1:8090"


def _post(base: str, cmd: str, params: dict) -> dict:
    req = urllib.request.Request(
        f"{base.rstrip('/')}/api/command",
        data=json.dumps({"cmd": cmd, "params": params}).encode(),
        headers={"Content-Type": "application/json"},
    )
    with urllib.request.urlopen(req, timeout=10) as resp:
        return json.loads(resp.read().decode())


def _get(base: str, path: str) -> dict:
    with urllib.request.urlopen(f"{base.rstrip('/')}{path}", timeout=10) as resp:
        return json.loads(resp.read().decode())


def _offset(lat: float, lon: float, north_m: float, east_m: float) -> tuple[float, float]:
    """Shift a lat/lon by metres. Flat-earth is fine over delivery distances."""
    dlat = north_m / 111320.0
    dlon = east_m / (111320.0 * max(0.05, math.cos(math.radians(lat))))
    return lat + dlat, lon + dlon


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--url", default=DEFAULT_URL, help="GCS base URL")
    ap.add_argument("--lat", type=float, help="absolute destination latitude")
    ap.add_argument("--lon", type=float, help="absolute destination longitude")
    ap.add_argument("--north", type=float, default=0.0, help="metres north of the drone")
    ap.add_argument("--east", type=float, default=0.0, help="metres east of the drone")
    ap.add_argument("--hover", type=float, default=60.0, help="hold seconds at the drop point")
    ap.add_argument("--alt", type=float, default=3.0, help="hover altitude (clamped by the FC ceiling)")
    ap.add_argument("--order-id", default=None)
    ap.add_argument("--recipient", default="test-recipient")
    ap.add_argument("--auto", action="store_true",
                    help="accept and fly immediately instead of waiting for the operator")
    ap.add_argument("--to-file", action="store_true",
                    help="write config/pending_order.json (delivery.source: file) instead of calling the GCS")
    args = ap.parse_args(argv)

    lat, lon = args.lat, args.lon
    if lat is None or lon is None:
        if args.to_file:
            ap.error("--to-file needs explicit --lat/--lon (there is no live position to offset from)")
        try:
            live = _get(args.url, "/api/state")
        except (urllib.error.URLError, OSError) as exc:
            print(f"cannot reach the GCS at {args.url}: {exc}", file=sys.stderr)
            return 2
        pos = live.get("position") or {}
        if not pos.get("lat"):
            print("the drone has no GPS position yet - pass --lat/--lon explicitly",
                  file=sys.stderr)
            return 2
        if not (args.north or args.east):
            ap.error("give --lat/--lon, or an offset with --north/--east")
        lat, lon = _offset(pos["lat"], pos["lon"], args.north, args.east)
        print(f"drone at {pos['lat']:.7f}, {pos['lon']:.7f}")

    order_id = args.order_id or f"test-{int(abs(lat * 1e5)) % 100000}"
    print(f"order {order_id} -> {lat:.7f}, {lon:.7f} "
          f"(hover {args.hover:.0f}s at {args.alt:.1f} m)")

    if args.to_file:
        path = REPO / "config" / "pending_order.json"
        path.write_text(json.dumps({
            "orderId": order_id,
            "recipientId": args.recipient,
            "targetLat": lat,
            "targetLng": lon,
            "hoverSeconds": args.hover,
            "hoverAltM": args.alt,
            "status": "DISPATCHED",
        }, indent=2), encoding="utf-8")
        print(f"wrote {path}")
        print("the delivery node will pick it up on its next poll "
              "(delivery.source must be 'file')")
        return 0

    try:
        result = _post(args.url, "delivery_inject", {
            "lat": lat, "lon": lon,
            "hover_seconds": args.hover, "hover_alt_m": args.alt,
            "order_id": order_id, "recipient_id": args.recipient,
            "auto_accept": bool(args.auto),
        })
    except (urllib.error.URLError, OSError) as exc:
        print(f"cannot reach the GCS at {args.url}: {exc}", file=sys.stderr)
        return 2

    print(("OK: " if result.get("ok") else "FAILED: ") + str(result.get("message", "")))
    if result.get("data"):
        print(json.dumps(result["data"], indent=2))
    if result.get("ok") and not args.auto:
        print("\nnow press ACCEPT & FLY on the GCS delivery panel.")
    return 0 if result.get("ok") else 1


if __name__ == "__main__":
    raise SystemExit(main())
