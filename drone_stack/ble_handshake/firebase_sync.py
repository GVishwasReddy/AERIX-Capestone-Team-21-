#!/usr/bin/env python3
"""
GCS <-> Firestore sync utility ("Store & Forward" from the design doc).

Two directions, both run only while the Pi has real internet (warehouse
Wi-Fi) — never during the offline BLE handshake itself:

  pull-order    Before takeoff: pull the dispatched order's dynamic token +
                target GPS from Firestore and write active_order.json, which
                drone_ble_peripheral.py loads at startup.

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
    python3 firebase_sync.py pull-order [--order-id ORD_9841]
    python3 firebase_sync.py push-receipts
"""

from __future__ import annotations

import argparse
import json
import sys
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


def _init_app(cred_path: str | None):
    if firebase_admin._apps:
        return
    if cred_path:
        cred = credentials.Certificate(cred_path)
    else:
        # Falls back to GOOGLE_APPLICATION_CREDENTIALS env var.
        cred = credentials.ApplicationDefault()
    firebase_admin.initialize_app(cred)


def pull_order(order_id: str | None) -> int:
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


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--cred", default=None, help="path to service-account.json")
    sub = parser.add_subparsers(dest="command", required=True)

    p_pull = sub.add_parser("pull-order", help="pull the active order's token+target from Firestore")
    p_pull.add_argument("--order-id", default=None, help="specific order id (default: latest DISPATCHED order)")

    sub.add_parser("push-receipts", help="upload queued local receipts to Firestore")

    args = parser.parse_args(argv)
    _init_app(args.cred)

    if args.command == "pull-order":
        return pull_order(args.order_id)
    if args.command == "push-receipts":
        return push_receipts()
    parser.error(f"unknown command {args.command!r}")
    return 2


if __name__ == "__main__":
    sys.exit(main())
