#!/usr/bin/env python3
"""
GCS <-> Firestore sync utility ("Store & Forward" from the design doc).

Two directions, both run only while the Pi has real internet (warehouse
Wi-Fi) — never during the offline BLE handshake itself:

  pull-order    Before takeoff: pull the dispatched order's dynamic token +
                target GPS from Firestore, write active_order.json (which
                drone_ble_peripheral.py loads at startup), and push that same
                target as the mission's hover/drop waypoint on the GCS
                (drone_stack) so the operator-planned flight path (Mission
                Planner / GCS map clicks) always ends at the customer's real
                drop-off point, not a manually re-typed one.

  push-hover    Re-send the hover waypoint from the last-pulled
                active_order.json to the GCS, without touching Firestore.
                Use this if the GCS wasn't reachable (or the Pixhawk had no
                GPS fix yet) when pull-order ran.

  push-receipts After landing: upload any locally-queued signed delivery
                receipts (written by drone_ble_peripheral.py on a successful
                drop) to Firestore, completing the dual-audit non-repudiation
                loop with the phone's own receipt upload.

Requires:
    pip install firebase-admin

Auth: a service-account key, via either
    --cred /path/to/service-account.json
  or
    GOOGLE_APPLICATION_CREDENTIALS=/path/to/service-account.json  (env var)

Get the key from Firebase Console -> Project settings -> Service accounts ->
Generate new private key. Keep it OFF the Flutter dev machine / out of any
repo — it grants full read/write to the whole project.

Usage:
    python3 firebase_sync.py pull-order [--order-id ORD_9841] [--hover-alt 3.0] [--no-gcs-push]
    python3 firebase_sync.py push-hover
    python3 firebase_sync.py push-receipts
"""

from __future__ import annotations

import argparse
import json
import os
import sys
import urllib.error
import urllib.request
from pathlib import Path

try:
    import firebase_admin
    from firebase_admin import credentials, firestore
except ImportError:
    print("firebase-admin is not installed. Run: pip install firebase-admin")
    sys.exit(1)

_HERE = Path(__file__).resolve().parent
_ACTIVE_ORDER_PATH = _HERE / "active_order.json"
_RECEIPTS_PENDING_DIR = _HERE / "receipts" / "pending"
_RECEIPTS_SYNCED_DIR = _HERE / "receipts" / "synced"

# ── GCS (drone_stack) integration — same default port as drone_ble_peripheral.py ──
GCS_BASE_URL = os.environ.get("GCS_BASE_URL", "http://localhost:8090")
GCS_HTTP_TIMEOUT_S = 3.0
DEFAULT_HOVER_ALT_M = 5.0  # same as cruise altitude, per the mission profile


def _gcs_post(path: str, payload: dict) -> dict | None:
    url = f"{GCS_BASE_URL}{path}"
    data = json.dumps(payload).encode("utf-8")
    req = urllib.request.Request(url, data=data, headers={"Content-Type": "application/json"})
    try:
        with urllib.request.urlopen(req, timeout=GCS_HTTP_TIMEOUT_S) as response:
            return json.loads(response.read().decode())
    except (urllib.error.URLError, TimeoutError, ValueError) as exc:
        print(f"  ❌ [GCS] POST {path} failed: {exc}")
        return None


def push_hover_waypoint(order: dict, hover_alt: float | None) -> bool:
    """Push the order's target GPS to drone_stack as the mission's hover/drop
    waypoint (see NavigationNode._svc_set_hover_waypoint). Best-effort — a
    failure here does not invalidate active_order.json; retry with
    ``push-hover`` once the GCS/Pixhawk GPS is ready."""
    alt = hover_alt if hover_alt is not None else order.get("hoverAlt", DEFAULT_HOVER_ALT_M)
    params = {"lat": order["targetLat"], "lon": order["targetLng"], "alt_m": alt}
    result = _gcs_post("/api/command", {"cmd": "set_hover_waypoint", "params": params})
    ok = bool(result and result.get("ok"))
    msg = result.get("message") if result else "no response from GCS"
    print(f"  🚁 [GCS] set_hover_waypoint({params['lat']:.7f}, {params['lon']:.7f}, "
          f"alt={alt}m) -> {'accepted' if ok else 'REJECTED'}: {msg}")
    if not ok:
        print("     (GCS unreachable or no GPS fix yet? fix the drone/GCS then run: "
              "python3 firebase_sync.py push-hover)")
    return ok


def _init_app(cred_path: str | None):
    if firebase_admin._apps:
        return
    if cred_path:
        cred = credentials.Certificate(cred_path)
    else:
        # Falls back to GOOGLE_APPLICATION_CREDENTIALS env var.
        cred = credentials.ApplicationDefault()
    firebase_admin.initialize_app(cred)


def pull_order(order_id: str | None, hover_alt: float | None, gcs_push: bool) -> int:
    db = firestore.client()

    if order_id:
        doc = db.collection("orders").document(order_id).get()
        if not doc.exists:
            print(f"❌ No order found with id '{order_id}'")
            return 1
        data = doc.to_dict()
        data["orderId"] = doc.id
    else:
        query = (
            db.collection("orders")
            .where("status", "==", "DISPATCHED")
            .order_by("createdAt", direction=firestore.Query.DESCENDING)
            .limit(1)
        )
        docs = list(query.stream())
        if not docs:
            print("❌ No DISPATCHED order found in Firestore. "
                  "Place an order in the app first.")
            return 1
        data = docs[0].to_dict()
        data["orderId"] = docs[0].id

    required = ("orderId", "deliveryToken", "targetLat", "targetLng")
    missing = [f for f in required if f not in data]
    if missing:
        print(f"❌ Order document is missing fields: {missing}")
        return 1

    out = {
        "orderId": data["orderId"],
        "deliveryToken": data["deliveryToken"],
        "targetLat": data["targetLat"],
        "targetLng": data["targetLng"],
        "recipientId": data.get("recipientId", ""),
    }
    _ACTIVE_ORDER_PATH.write_text(json.dumps(out, indent=2), encoding="utf-8")
    print(f"✅ Pulled order '{out['orderId']}' -> {_ACTIVE_ORDER_PATH}")
    print(f"   Target: ({out['targetLat']}, {out['targetLng']})")

    if gcs_push:
        push_hover_waypoint(out, hover_alt)
    else:
        print("   (--no-gcs-push: mission hover waypoint NOT updated — run "
              "'push-hover' manually when ready)")
    return 0


def push_receipts() -> int:
    if not _RECEIPTS_PENDING_DIR.exists():
        print("No pending receipts directory — nothing to sync.")
        return 0

    pending = sorted(_RECEIPTS_PENDING_DIR.glob("*.json"))
    if not pending:
        print("No pending receipts to sync.")
        return 0

    db = firestore.client()
    _RECEIPTS_SYNCED_DIR.mkdir(parents=True, exist_ok=True)

    synced = 0
    failed = 0
    for path in pending:
        try:
            receipt = json.loads(path.read_text(encoding="utf-8"))
        except (OSError, json.JSONDecodeError) as exc:
            print(f"  ⚠️ skipping unreadable receipt {path.name}: {exc}")
            failed += 1
            continue

        try:
            db.collection("receipts").add(receipt)
            order_id = receipt.get("orderId")
            if order_id:
                order_ref = db.collection("orders").document(order_id)
                if order_ref.get().exists:
                    order_ref.update({"status": "DELIVERED"})
            path.rename(_RECEIPTS_SYNCED_DIR / path.name)
            print(f"  ✅ synced {path.name}")
            synced += 1
        except Exception as exc:  # noqa: BLE001 - report and keep going
            print(f"  ❌ failed to sync {path.name}: {exc}")
            failed += 1

    print(f"\nSynced {synced} receipt(s), {failed} failed (left in pending/ for retry).")
    return 1 if failed else 0


def push_hover() -> int:
    """Re-send the last-pulled order's target as the GCS hover waypoint,
    without touching Firestore. No credentials needed — reads the local
    active_order.json that a previous pull-order already wrote."""
    if not _ACTIVE_ORDER_PATH.exists():
        print(f"❌ No {_ACTIVE_ORDER_PATH.name} found — run 'pull-order' first.")
        return 1
    order = json.loads(_ACTIVE_ORDER_PATH.read_text(encoding="utf-8"))
    return 0 if push_hover_waypoint(order, None) else 1


def cancel_order() -> int:
    """Mark the active order as CANCELED in Firestore and remove the local active_order.json."""
    if not _ACTIVE_ORDER_PATH.exists():
        print(f"❌ No {_ACTIVE_ORDER_PATH.name} found — nothing to cancel locally.")
        return 1

    try:
        order_data = json.loads(_ACTIVE_ORDER_PATH.read_text(encoding="utf-8"))
        order_id = order_data.get("orderId")
    except (OSError, json.JSONDecodeError) as exc:
        print(f"❌ Failed to read {_ACTIVE_ORDER_PATH.name}: {exc}")
        return 1

    if not order_id:
        print("❌ Invalid active_order.json (missing orderId).")
        return 1

    db = firestore.client()
    try:
        order_ref = db.collection("orders").document(order_id)
        if order_ref.get().exists:
            order_ref.update({"status": "CANCELED"})
            print(f"✅ Order '{order_id}' marked as CANCELED in Firestore.")
        else:
            print(f"⚠️ Order '{order_id}' not found in Firestore.")
    except Exception as exc:
        print(f"❌ Failed to update Firestore: {exc}")
        return 1

    _ACTIVE_ORDER_PATH.unlink()
    print(f"✅ Local {_ACTIVE_ORDER_PATH.name} removed.")
    return 0


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--cred", default=None, help="path to service-account.json")
    sub = parser.add_subparsers(dest="command", required=True)

    p_pull = sub.add_parser("pull-order", help="pull the active order's token+target from Firestore")
    p_pull.add_argument("--order-id", default=None, help="specific order id (default: latest DISPATCHED order)")
    p_pull.add_argument("--hover-alt", type=float, default=None,
                         help="hover/drop altitude in metres sent to the GCS "
                              f"(default: order's hoverAlt field, else {DEFAULT_HOVER_ALT_M}m)")
    p_pull.add_argument("--no-gcs-push", dest="gcs_push", action="store_false",
                         help="write active_order.json only; don't touch the GCS mission "
                              "(use when the GCS/Pixhawk isn't up yet, then run push-hover later)")

    sub.add_parser("push-hover", help="re-send the last-pulled order's target to the GCS as the hover waypoint")
    sub.add_parser("push-receipts", help="upload queued local receipts to Firestore")
    sub.add_parser("cancel-order", help="mark the active order as CANCELED in Firestore and clear it locally")

    args = parser.parse_args(argv)

    if args.command == "push-hover":
        return push_hover()

    _init_app(args.cred)

    if args.command == "pull-order":
        return pull_order(args.order_id, args.hover_alt, args.gcs_push)
    if args.command == "push-receipts":
        return push_receipts()
    if args.command == "cancel-order":
        return cancel_order()
    parser.error(f"unknown command {args.command!r}")
    return 2


if __name__ == "__main__":
    sys.exit(main())
