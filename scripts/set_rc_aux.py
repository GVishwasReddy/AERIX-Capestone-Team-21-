#!/usr/bin/env python3
"""One-shot: set RC aux switch functions on the Pixhawk, with readback.
  RC6_OPTION=18 (Land), RC7_OPTION=17 (AutoTune), RC8_OPTION=31 (Motor E-Stop)
Run only while the GCS is stopped (it owns /dev/ttyACM0)."""
import time
from pymavlink import mavutil

PORT, BAUD = "/dev/ttyACM0", 115200
TARGETS = [("RC6_OPTION", 18), ("RC7_OPTION", 17), ("RC8_OPTION", 31)]

def pid(msg):
    p = msg.param_id
    return p.decode() if isinstance(p, bytes) else p

def get_param(m, name, timeout=6):
    m.mav.param_request_read_send(m.target_system, m.target_component, name.encode(), -1)
    t0 = time.time()
    while time.time() - t0 < timeout:
        msg = m.recv_match(type="PARAM_VALUE", blocking=True, timeout=timeout)
        if msg and pid(msg).strip("\x00") == name:
            return msg.param_value
    return None

m = mavutil.mavlink_connection(PORT, baud=BAUD)
print("waiting for heartbeat...")
m.wait_heartbeat(timeout=20)
print("heartbeat: sys=%d comp=%d" % (m.target_system, m.target_component))

all_ok = True
for name, val in TARGETS:
    before = get_param(m, name)
    m.mav.param_set_send(m.target_system, m.target_component,
                         name.encode(), float(val),
                         mavutil.mavlink.MAV_PARAM_TYPE_INT16)
    time.sleep(0.6)
    after = get_param(m, name)
    ok = after is not None and int(round(after)) == val
    all_ok = all_ok and ok
    b = "None" if before is None else str(int(round(before)))
    a = "None" if after is None else str(int(round(after)))
    status = "OK" if ok else "FAIL"
    print("%s: %s -> %s (target %d) %s" % (name, b, a, val, status))

m.close()
print("RESULT:", "ALL OK" if all_ok else "SOME FAILED")
