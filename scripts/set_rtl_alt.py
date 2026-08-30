#!/usr/bin/env python3
"""Bring RTL_ALT under the project's 3 m ceiling.

ArduPilot flies RTL itself: it climbs to RTL_ALT, returns, then descends. None
of that is commanded waypoint-by-waypoint, so the companion computer's altitude
clamp has no say in it. With RTL_ALT at its 15 m default, any return - and the
navigator falls back to plain RTL whenever SMART_RTL is refused - would climb
five times higher than the airframe is cleared for.

300 cm rather than 0: at 0 the aircraft returns at whatever altitude it happens
to be at, which could be under a metre. 3 m is the ceiling and also the cruise
altitude, so the return flies the same height as the outbound leg.

Read-modify-verify; run only while aerix-gcs is stopped.
"""
import sys
import time
from pymavlink import mavutil

PORT, BAUD = "/dev/ttyACM0", 115200
TARGET = {"RTL_ALT": 300.0}          # cm


def pid(msg):
    p = msg.param_id
    return p.decode() if isinstance(p, bytes) else p


def get_param(m, name, timeout=5.0):
    m.mav.param_request_read_send(
        m.target_system, m.target_component, name.encode(), -1
    )
    deadline = time.time() + timeout
    while time.time() < deadline:
        msg = m.recv_match(type="PARAM_VALUE", blocking=True, timeout=0.5)
        if msg and pid(msg).strip("\x00") == name:
            return msg.param_value
    return None


def set_param(m, name, value, timeout=5.0):
    m.mav.param_set_send(
        m.target_system, m.target_component, name.encode(), float(value),
        mavutil.mavlink.MAV_PARAM_TYPE_REAL32,
    )
    deadline = time.time() + timeout
    while time.time() < deadline:
        msg = m.recv_match(type="PARAM_VALUE", blocking=True, timeout=0.5)
        if msg and pid(msg).strip("\x00") == name:
            return msg.param_value
    return None


def main():
    m = mavutil.mavlink_connection(PORT, baud=BAUD)
    if not m.wait_heartbeat(timeout=15):
        print("no heartbeat - is aerix-gcs still holding the port?")
        return 1
    print(f"connected to system {m.target_system}\n")

    failed = False
    for name, want in TARGET.items():
        before = get_param(m, name)
        if before is None:
            print(f"  {name}: could not read - skipping")
            failed = True
            continue
        if abs(before - want) < 0.5:
            print(f"  {name}: already {before:g}")
            continue
        print(f"  {name}: {before:g} -> {want:g}")
        set_param(m, name, want)
        time.sleep(0.5)

        # Read back from the autopilot rather than trusting the ack: a param
        # write can be accepted and then clamped or rejected by the FC.
        after = get_param(m, name)
        if after is None or abs(after - want) >= 0.5:
            print(f"    *** VERIFY FAILED: reads back {after}")
            failed = True
        else:
            print(f"    verified: {after:g}")

    print()
    if failed:
        print("one or more parameters did not take - do NOT fly until resolved")
        return 1

    print("RTL_ALT is now within the 3 m ceiling.")
    print("Note: this is stored in the autopilot's own flash, so it survives a")
    print("Pi reflash or a drone_stack reinstall - and equally, a param reset")
    print("or firmware reflash of the FC puts the 15 m default back.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
