#!/usr/bin/env python3
"""Cap how fast a throttle stick can command a descent.

Defence in depth for the 2026-08-25 hard-landing crashes (dataflash logs 453
and 454). Their cause was the companion computer putting the aircraft into
POSHOLD for the drop-point hold - a mode that takes its ALTITUDE from the
pilot's throttle stick, which rests at RC3_MIN through an autonomous delivery.
ArduPilot read that as "descend as fast as the pilot is allowed to" and flew
the aircraft into the ground at 2.4 m/s, breaking the landing gear. The
navigator no longer enters those modes (NavigationNode._STICK_ALTITUDE_MODES),
which is the actual fix.

This is the backstop for the case the navigator cannot reach: the *pilot*
selecting POSHOLD/LOITER/ALT_HOLD mid-flight with the throttle stick down,
which commands exactly the same descent.

PILOT_SPEED_DN = 0 means "use PILOT_SPEED_UP", which is 250 cm/s here - hence
the 2.4 m/s arrival. 100 cm/s is still a brisk, usable descent for a 3 m
delivery profile and is survivable if it reaches the ground.

This does NOT touch the autonomous descents: LAND uses LAND_SPEED (30 cm/s),
RTL/SMART_RTL/GUIDED use WPNAV_SPEED_DN. Only stick-driven descent is capped.

Read-modify-verify; run only while aerix-gcs is stopped (it owns /dev/ttyACM0):
    sudo systemctl stop aerix-gcs.service
    .venv/bin/python scripts/set_pilot_descent_limit.py
    sudo systemctl start aerix-gcs.service
"""
import sys
import time

from pymavlink import mavutil

PORT, BAUD = "/dev/ttyACM0", 115200
TARGET = {"PILOT_SPEED_DN": 100.0}       # cm/s


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

        # Read back from the autopilot rather than trusting the ack: a write
        # can be accepted and then clamped or rejected by the FC.
        after = get_param(m, name)
        if after is None or abs(after - want) >= 0.5:
            print(f"    *** VERIFY FAILED: reads back {after}")
            failed = True
        else:
            print(f"    verified: {after:g}")

    print()
    if failed:
        print("a parameter did not take - do NOT fly until resolved")
        return 1
    print("Stick-commanded descent is now capped at 1.0 m/s.")
    print("Autonomous descents are unchanged (LAND_SPEED 30 cm/s,")
    print("WPNAV_SPEED_DN 150 cm/s). Stored in the autopilot's own flash, so a")
    print("param reset or firmware reflash puts the unsafe default back.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
