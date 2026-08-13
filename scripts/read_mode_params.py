#!/usr/bin/env python3
"""Read-only: dump the flight-mode selection params so we can plan a 3-pos
switch on ch9 without clobbering an existing mode switch.
Run only while the GCS service is stopped (it owns /dev/ttyACM0)."""
import time
from pymavlink import mavutil

PORT, BAUD = "/dev/ttyACM0", 115200
NAMES = [
    "FLTMODE_CH",
    "FLTMODE1", "FLTMODE2", "FLTMODE3", "FLTMODE4", "FLTMODE5", "FLTMODE6",
    "RC9_OPTION", "RC5_OPTION",
]
MODE = {
    0: "Stabilize", 1: "Acro", 2: "AltHold", 3: "Auto", 4: "Guided",
    5: "Loiter", 6: "RTL", 7: "Circle", 9: "Land", 11: "Drift",
    13: "Sport", 14: "Flip", 15: "AutoTune", 16: "PosHold", 17: "Brake",
    18: "Throw", 20: "Guided_NoGPS", 21: "SmartRTL", 22: "FlowHold",
    23: "Follow", 24: "ZigZag", 25: "SystemID", 27: "AutoRTL",
}


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

for name in NAMES:
    v = get_param(m, name)
    if v is None:
        print("%-12s = None (not read)" % name)
        continue
    iv = int(round(v))
    if name.startswith("FLTMODE") and name != "FLTMODE_CH":
        print("%-12s = %d (%s)" % (name, iv, MODE.get(iv, "?")))
    else:
        print("%-12s = %d" % (name, iv))

m.close()
