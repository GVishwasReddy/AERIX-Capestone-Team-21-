#!/usr/bin/env python3
"""One-shot: make the ch9 3-position switch (SwC) the flight-mode selector.

  FLTMODE_CH = 9
  FLTMODE1   = 0  (Stabilize)  <- switch LOW  (~1000us)
  FLTMODE4   = 16 (PosHold)    <- switch MID  (~1500us)
  FLTMODE6   = 21 (SmartRTL)   <- switch HIGH (~2000us)

Before/after readback on every param. FLTMODE2/3/5 are left untouched.
Run only while the GCS service is stopped (it owns /dev/ttyACM0)."""
import time
from pymavlink import mavutil

PORT, BAUD = "/dev/ttyACM0", 115200
# (name, value, mavlink param type)
I8 = mavutil.mavlink.MAV_PARAM_TYPE_INT8
TARGETS = [
    ("FLTMODE_CH", 9, I8),
    ("FLTMODE1", 0, I8),
    ("FLTMODE4", 16, I8),
    ("FLTMODE6", 21, I8),
]
MODE = {0: "Stabilize", 2: "AltHold", 5: "Loiter", 6: "RTL", 9: "Land",
        16: "PosHold", 21: "SmartRTL"}


def pid(msg):
    p = msg.param_id
    return p.decode() if isinstance(p, bytes) else p


def get_param(m, name, timeout=6):
    m.mav.param_request_read_send(
        m.target_system, m.target_component, name.encode(), -1
    )
    t0 = time.time()
    while time.time() - t0 < timeout:
        msg = m.recv_match(type="PARAM_VALUE", blocking=True, timeout=timeout)
        if msg and pid(msg).strip("\x00") == name:
            return msg.param_value
    return None


m = mavutil.mavlink_connection(PORT, baud=BAUD)
print("waiting for heartbeat...")
m.wait_heartbeat(timeout=20)
print("heartbeat: sys=%d comp=%d\n" % (m.target_system, m.target_component))

all_ok = True
for name, val, ptype in TARGETS:
    before = get_param(m, name)
    m.mav.param_set_send(
        m.target_system, m.target_component, name.encode(), float(val), ptype
    )
    time.sleep(0.6)
    after = get_param(m, name)
    ok = after is not None and int(round(after)) == val
    all_ok = all_ok and ok
    b = "None" if before is None else str(int(round(before)))
    a = "None" if after is None else str(int(round(after)))
    tag = ""
    if name.startswith("FLTMODE") and name != "FLTMODE_CH":
        tag = " (%s)" % MODE.get(val, "?")
    print("%-11s: %s -> %s (target %d%s) %s"
          % (name, b, a, val, tag, "OK" if ok else "FAIL"))

m.close()
print("\nRESULT:", "ALL OK" if all_ok else "SOME FAILED")
