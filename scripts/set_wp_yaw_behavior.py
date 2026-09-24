#!/usr/bin/env python3
"""Set WP_YAW_BEHAVIOR = 1 (face next waypoint, RTL included).

It was 2 = "face next waypoint EXCEPT RTL", which meant the entire return leg
was flown without ever turning: the aircraft translates home at whatever
heading it happened to finish the delivery on, so the 250 deg LiDAR window - and
therefore everything BendyRuler is routing against - can be pointed the wrong
way for the whole flight. The rear 110 deg is streamed as 65535 = unknown, which
ArduPilot reads as CLEAR.

Run with the GCS service stopped (it owns /dev/ttyACM0). Re-runnable; reads
back what it wrote.
"""
import sys, time
from pymavlink import mavutil

WANT = {"WP_YAW_BEHAVIOR": 1.0}

m = mavutil.mavlink_connection("/dev/ttyACM0", baud=115200)
if not m.wait_heartbeat(timeout=25):
    print("no heartbeat", file=sys.stderr); raise SystemExit(1)


def read(name, timeout=8.0):
    m.mav.param_request_read_send(m.target_system, m.target_component,
                                  name.encode(), -1)
    end = time.time() + timeout
    while time.time() < end:
        msg = m.recv_match(type="PARAM_VALUE", blocking=True, timeout=2)
        if not msg:
            m.mav.param_request_read_send(m.target_system, m.target_component,
                                          name.encode(), -1)
            continue
        pid = msg.param_id.decode() if isinstance(msg.param_id, bytes) else msg.param_id
        if pid == name:
            return msg.param_value
    return None


rc = 0
for name, value in WANT.items():
    before = read(name)
    print(f"  {name}: was {before}")
    if before is not None and abs(before - value) < 1e-6:
        print(f"  {name}: already {value} - nothing to do")
        continue
    m.mav.param_set_send(m.target_system, m.target_component, name.encode(),
                         float(value), mavutil.mavlink.MAV_PARAM_TYPE_REAL32)
    time.sleep(1.0)
    after = read(name)
    ok = after is not None and abs(after - value) < 1e-6
    print(f"  {name}: now {after}  {'OK' if ok else 'FAILED'}")
    if not ok:
        rc = 1
raise SystemExit(rc)
