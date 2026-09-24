#!/usr/bin/env python3
"""Read-only dump of the params that govern the post-mission turn / RTL / land."""
import sys, time
from pymavlink import mavutil

NAMES = [
    "OA_TYPE", "OA_BR_TYPE", "OA_BR_LOOKAHEAD", "OA_MARGIN_MAX",
    "OA_DB_EXPIRE", "OA_DB_SIZE",
    "PRX1_TYPE", "PRX_FILT", "PRX1_ORIENT", "PRX1_YAW_CORR",
    "AVOID_ENABLE", "AVOID_MARGIN", "AVOID_BEHAVE", "AVOID_DIST_MAX",
    "WP_YAW_BEHAVIOR", "WPNAV_SPEED", "WPNAV_RADIUS",
    "RTL_ALT", "RTL_ALT_FINAL", "RTL_LOIT_TIME", "RTL_CLIMB_MIN", "RTL_SPEED",
    "LAND_SPEED", "LAND_SPEED_HIGH", "LAND_ALT_LOW", "PLND_ENABLED",
    "ATC_SLEW_YAW", "ATC_RATE_Y_MAX", "ATC_ACCEL_Y_MAX", "ATC_ANG_YAW_P",
    "EK3_SRC1_YAW", "COMPASS_USE", "COMPASS_USE2", "COMPASS_USE3",
    "AHRS_EKF_TYPE", "FENCE_ENABLE",
]

m = mavutil.mavlink_connection("/dev/ttyACM0", baud=115200)
m.wait_heartbeat(timeout=20)
print(f"# heartbeat from sys {m.target_system} comp {m.target_component}\n")

got = {}
for attempt in range(4):
    missing = [n for n in NAMES if n not in got]
    if not missing:
        break
    for n in missing:
        m.mav.param_request_read_send(m.target_system, m.target_component,
                                      n.encode(), -1)
        time.sleep(0.01)
    deadline = time.time() + 3.0
    while time.time() < deadline:
        msg = m.recv_match(type="PARAM_VALUE", blocking=True, timeout=0.4)
        if msg is None:
            continue
        pid = msg.param_id.strip("\x00")
        if pid in NAMES:
            got[pid] = msg.param_value

for n in NAMES:
    v = got.get(n)
    print(f"{n:18s} = " + ("MISSING" if v is None else f"{v:g}"))
