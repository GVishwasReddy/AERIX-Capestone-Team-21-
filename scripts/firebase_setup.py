#!/usr/bin/env python3
"""Install and verify the Firestore service-account key for the delivery node.

Why this exists: an order placed in the phone app lands in Firestore, but the
Pi can only see it if the Admin SDK can authenticate. That needs a
service-account private key, which can only come from the Firebase console -
nothing on the drone can mint one. Until the key is in place the delivery node
reports link "no-credentials" and the GCS delivery panel stays empty no matter
how many orders the app writes.

Get the key (once)::

    Firebase console -> your project (aerix-drone-delivery)
      -> gear icon -> Project settings -> Service accounts
      -> "Generate new private key" -> Generate key

That downloads a JSON file. Then, on the Pi::

    scripts/firebase_setup.py ~/aerix-drone-delivery-firebase-adminsdk-xxxx.json

which validates it, checks the project matches, installs it at
config/firebase-service-account.json with 0600 permissions, and does a live
read of the orders collection so you know it works before you go flying.

Already installed a key and just want to check the link?

    scripts/firebase_setup.py --check

The delivery node retries its connection every poll, so the stack does not need
restarting once the key is in place - the GCS panel goes green on its own.
"""
from __future__ import annotations

import argparse
import json
import os
import shutil
import sys
from pathlib import Path

REPO = Path(__file__).resolve().parents[1]
DEST = REPO / "config" / "firebase-service-account.json"
EXPECTED_PROJECT = "aerix-drone-delivery"

# The fields Firebase puts in a service-account key. A google-services.json
# (the *app* config) has none of them, and that is the file people usually
# grab by mistake.
REQUIRED = ("type", "project_id", "private_key", "client_email")


def _fail(message: str, *hints: str) -> int:
    print(f"\n  FAILED: {message}", file=sys.stderr)
    for hint in hints:
        print(f"          {hint}", file=sys.stderr)
    return 1


def _validate(path: Path) -> tuple[dict | None, int]:
    if not path.exists():
        return None, _fail(f"{path} does not exist")
    try:
        data = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as exc:
        return None, _fail(f"{path.name} is not readable JSON: {exc}")

    if data.get("type") != "service_account":
        which = "google-services.json (the Android app config)" if "project_info" in data \
            else f"a '{data.get('type', 'unknown')}' file"
        return None, _fail(
            f"{path.name} is not a service-account key - it looks like {which}",
            "You need: Project settings -> Service accounts -> Generate new private key.",
            "The app's google-services.json cannot authenticate the Admin SDK.",
        )
    missing = [k for k in REQUIRED if not data.get(k)]
    if missing:
        return None, _fail(f"{path.name} is missing {', '.join(missing)}")

    project = data.get("project_id", "")
    if project != EXPECTED_PROJECT:
        print(f"\n  WARNING: key is for project '{project}', but the app writes "
              f"orders to '{EXPECTED_PROJECT}'.")
        print("           The drone will not see orders from the phone app.")
        if input("           Install it anyway? [y/N] ").strip().lower() != "y":
            return None, 1
    return data, 0


def _install(src: Path, data: dict) -> int:
    DEST.parent.mkdir(parents=True, exist_ok=True)
    if DEST.exists() and DEST.resolve() != src.resolve():
        backup = DEST.with_suffix(".json.bak")
        shutil.copy2(DEST, backup)
        print(f"  existing key backed up to {backup.name}")
    if src.resolve() != DEST.resolve():
        shutil.copy2(src, DEST)
    os.chmod(DEST, 0o600)               # full project access: keep it private
    print(f"  installed {DEST}  (mode 600)")
    print(f"  project   {data['project_id']}")
    print(f"  identity  {data['client_email']}")
    return 0


def _check_live() -> int:
    """Actually talk to Firestore and read the orders collection."""
    sys.path.insert(0, str(REPO))
    try:
        import firebase_admin
        from firebase_admin import credentials, firestore
    except ImportError:
        return _fail(
            "firebase-admin is not installed in this interpreter",
            f"Run: {REPO}/.venv/bin/pip install firebase-admin",
        )
    if not DEST.exists():
        return _fail(
            f"no key at {DEST}",
            "Run this script with the path to your downloaded key first.",
        )
    try:
        cred = credentials.Certificate(str(DEST))
        try:
            app = firebase_admin.get_app("aerix-setup-check")
        except ValueError:
            app = firebase_admin.initialize_app(
                cred, {"projectId": EXPECTED_PROJECT}, name="aerix-setup-check"
            )
        db = firestore.client(app)
        docs = list(db.collection("orders").limit(25).get())
    except Exception as exc:  # noqa: BLE001 - this script exists to explain failures
        return _fail(
            f"could not read the orders collection: {type(exc).__name__}: {exc}",
            "Check the Pi has internet, and that the key was not revoked.",
        )

    print(f"\n  Firestore OK - 'orders' collection reachable, {len(docs)} document(s).")
    dispatched = 0
    for doc in docs:
        d = doc.to_dict() or {}
        status = d.get("status", "?")
        if status == "DISPATCHED":
            dispatched += 1
        print(f"    {doc.id[:12]:<14} {status:<11} "
              f"{d.get('targetLat', '?')}, {d.get('targetLng', '?')}  "
              f"{d.get('createdAt', '')}")
    print(f"\n  {dispatched} order(s) currently DISPATCHED - "
          "these are what the drone picks up.")
    if not docs:
        print("  (place an order in the phone app and run --check again)")
    return 0


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("key", nargs="?", help="path to the downloaded service-account JSON")
    ap.add_argument("--check", action="store_true",
                    help="only verify the installed key and read the orders collection")
    args = ap.parse_args(argv)

    if not args.key and not args.check:
        ap.error("give the path to a downloaded key, or --check")

    if args.key:
        print(f"\nvalidating {args.key}")
        data, rc = _validate(Path(args.key).expanduser())
        if data is None:
            return rc
        rc = _install(Path(args.key).expanduser(), data)
        if rc:
            return rc

    rc = _check_live()
    if rc == 0 and args.key:
        print("\n  Done. The delivery node reconnects on its next poll "
              "(a few seconds) -\n  the GCS delivery panel will show the link "
              "as online. No restart needed.")
    return rc


if __name__ == "__main__":
    raise SystemExit(main())
