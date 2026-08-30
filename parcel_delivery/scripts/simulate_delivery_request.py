#!/usr/bin/env python3
"""Write a synthetic pending delivery straight into the RTDB.

Lets you exercise firebase_client -> state_machine -> flight_controller
without the web app in the loop.

Usage:
    python scripts/simulate_delivery_request.py                    # 100m north of home
    python scripts/simulate_delivery_request.py --lat X --lon Y
    python scripts/simulate_delivery_request.py --offset-m 250     # test a geofence reject
    python scripts/simulate_delivery_request.py --watch            # follow status live
"""
from __future__ import annotations

import argparse
import datetime
import math
import os
import sys
import time

sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "pi"))

import firebase_admin  # noqa: E402
from dotenv import load_dotenv  # noqa: E402
from firebase_admin import credentials, db  # noqa: E402


def offset_north(lat: float, lon: float, meters: float) -> tuple[float, float]:
    """Return a point `meters` due north of (lat, lon)."""
    return lat + (meters / 111_320.0), lon


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--env", default=os.path.join("pi", ".env"))
    parser.add_argument("--lat", type=float, default=None)
    parser.add_argument("--lon", type=float, default=None)
    parser.add_argument("--alt", type=float, default=15.0, help="target AGL altitude (m)")
    parser.add_argument(
        "--offset-m",
        type=float,
        default=100.0,
        help="if --lat/--lon are omitted, place the target this far north of home",
    )
    parser.add_argument("--user-id", default="sim-user")
    parser.add_argument("--watch", action="store_true", help="stream status updates until terminal")
    args = parser.parse_args()

    load_dotenv(args.env)

    cred_path = os.environ["FIREBASE_CREDENTIALS_PATH"]
    db_url = os.environ["FIREBASE_DB_URL"]
    home_lat = float(os.environ.get("HOME_LAT", 0.0))
    home_lon = float(os.environ.get("HOME_LON", 0.0))

    if args.lat is not None and args.lon is not None:
        lat, lon = args.lat, args.lon
    else:
        lat, lon = offset_north(home_lat, home_lon, args.offset_m)

    if not firebase_admin._apps:
        firebase_admin.initialize_app(
            credentials.Certificate(cred_path), {"databaseURL": db_url}
        )

    ref = db.reference("/deliveries").push()
    ref.set(
        {
            "username": "sim-tester",
            "user_id": args.user_id,
            "destination": {"lat": lat, "lon": lon, "alt_agl_m": args.alt},
            "requested_at": datetime.datetime.now(datetime.timezone.utc).isoformat(),
            "status": "pending",
            "delivery_confirmed": False,
        }
    )

    distance = math.hypot((lat - home_lat) * 111_320.0, (lon - home_lon) * 111_320.0)
    print(f"Created delivery {ref.key}")
    print(f"  target:   {lat:.6f}, {lon:.6f}  (alt {args.alt}m AGL)")
    print(f"  distance: ~{distance:.0f}m from home")

    if not args.watch:
        return 0

    print("\nWatching status (Ctrl-C to stop)...")
    terminal = {"landed", "error", "aborted"}
    last = None
    try:
        while True:
            doc = db.reference(f"/deliveries/{ref.key}").get() or {}
            status = doc.get("status")
            if status != last:
                stamp = datetime.datetime.now().strftime("%H:%M:%S")
                line = f"  [{stamp}] {status}"
                if doc.get("error_message"):
                    line += f"  — {doc['error_message']}"
                print(line)
                last = status
            if status in terminal:
                break
            time.sleep(1.0)
    except KeyboardInterrupt:
        print("\nstopped watching")

    return 0


if __name__ == "__main__":
    sys.exit(main())
