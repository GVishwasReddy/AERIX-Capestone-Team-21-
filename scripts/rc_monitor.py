#!/usr/bin/env python3
"""Live RC channel monitor. Prints ch7 & ch9 samples and tracks min/max per
channel over a fixed window, so we can read Button A (ch7) HIGH/LOW µs even if
presses land anywhere in the window. Run only while the GCS is stopped."""
import sys
import time
from pymavlink import mavutil

PORT, BAUD = "/dev/ttyACM0", 115200
WINDOW_S = float(sys.argv[1]) if len(sys.argv) > 1 else 30.0
WATCH = [7, 9]

m = mavutil.mavlink_connection(PORT, baud=BAUD)
print("waiting for heartbeat...")
m.wait_heartbeat(timeout=20)
print("heartbeat: sys=%d comp=%d" % (m.target_system, m.target_component))
# Ask the FC to stream everything (RC_CHANNELS included).
m.mav.request_data_stream_send(
    m.target_system, m.target_component,
    mavutil.mavlink.MAV_DATA_STREAM_ALL, 10, 1,
)

mins = {c: 99999 for c in range(1, 19)}
maxs = {c: 0 for c in range(1, 19)}
last_print = 0.0
t0 = time.time()
print("PRESS/RELEASE Button A (and B) repeatedly now for %.0fs..." % WINDOW_S)
while time.time() - t0 < WINDOW_S:
    msg = m.recv_match(type="RC_CHANNELS", blocking=True, timeout=2)
    if not msg:
        continue
    vals = {c: getattr(msg, "chan%d_raw" % c, 0) for c in range(1, 19)}
    for c, v in vals.items():
        if v and v != 65535:
            mins[c] = min(mins[c], v)
            maxs[c] = max(maxs[c], v)
    now = time.time()
    if now - last_print >= 0.4:
        last_print = now
        print("t=%4.1fs  " % (now - t0)
              + "  ".join("ch%d=%4d" % (c, vals[c]) for c in WATCH))

m.close()
print("\n===== min/max seen over %.0fs =====" % WINDOW_S)
for c in WATCH:
    lo = mins[c] if mins[c] != 99999 else 0
    print("ch%d: LOW=%d  HIGH=%d  (span=%d)" % (c, lo, maxs[c], maxs[c] - lo))
