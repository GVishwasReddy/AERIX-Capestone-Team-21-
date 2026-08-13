#!/usr/bin/env python3
"""dronectl - a tiny CLI to control a running drone_stack via its dashboard.

Examples::

    scripts/dronectl.py start_mission
    scripts/dronectl.py hold
    scripts/dronectl.py rtl
    scripts/dronectl.py emergency_stop
    scripts/dronectl.py goto lat=47.3977 lon=8.5456 alt=5   # (params as key=val)
    scripts/dronectl.py --list

Talks to the dashboard's HTTP API (default http://127.0.0.1:8090). Uses only the
Python standard library so it runs anywhere.
"""
from __future__ import annotations

import argparse
import json
import sys
import urllib.error
import urllib.request


def _coerce(value: str):
    for cast in (int, float):
        try:
            return cast(value)
        except ValueError:
            continue
    if value.lower() in ("true", "false"):
        return value.lower() == "true"
    return value


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="Control a running drone_stack.")
    parser.add_argument("command", nargs="?", help="service name to call")
    parser.add_argument("params", nargs="*", help="key=value parameters")
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=8090)
    parser.add_argument("--list", action="store_true", help="list available services")
    args = parser.parse_args(argv)

    base = f"http://{args.host}:{args.port}"

    if args.list or not args.command:
        try:
            with urllib.request.urlopen(f"{base}/api/services", timeout=5) as resp:
                data = json.load(resp)
            print("\n".join(data.get("services", [])) or "(none)")
            return 0
        except urllib.error.URLError as exc:
            print(f"error: could not reach dashboard at {base}: {exc}", file=sys.stderr)
            return 1

    params = {}
    for item in args.params:
        if "=" in item:
            key, value = item.split("=", 1)
            params[key] = _coerce(value)

    payload = json.dumps({"name": args.command, "params": params}).encode()
    request = urllib.request.Request(
        f"{base}/api/service", data=payload,
        headers={"Content-Type": "application/json"}, method="POST",
    )
    try:
        with urllib.request.urlopen(request, timeout=5) as resp:
            result = json.load(resp)
    except urllib.error.URLError as exc:
        print(f"error: could not reach dashboard at {base}: {exc}", file=sys.stderr)
        return 1

    status = "OK" if result.get("success") else "FAILED"
    print(f"[{status}] {result.get('message', '')}")
    if result.get("data"):
        print(json.dumps(result["data"], indent=2))
    return 0 if result.get("success") else 2


if __name__ == "__main__":
    raise SystemExit(main())
