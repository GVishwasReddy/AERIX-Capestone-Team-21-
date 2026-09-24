#!/usr/bin/env python3
"""Is anything else claiming the AUX1 pin? Read-only.

On a Pixhawk the AUX outputs are also GPIO pins: AUX1..AUX6 = pins 50..55.
If a RELAY or a servo function has claimed AUX1, the FC still computes
servo9_raw and still ACKs DO_SET_SERVO - and the pin emits no PWM at all.
That combination (perfect software, dead pin) is what we are looking at.

Run with the GCS stopped.
"""
import time
from pymavlink import mavutil

PORT, BAUD = "/dev/ttyACM0", 115200
AUX_PIN = {50: "AUX1", 51: "AUX2", 52: "AUX3",
           53: "AUX4", 54: "AUX5", 55: "AUX6"}

NAMES = (
    [f"RELAY_PIN{s}" for s in ("", "2", "3", "4", "5", "6")]
    + [f"SERVO{n}_FUNCTION" for n in range(9, 15)]
    + ["BRD_PWM_COUNT", "BRD_SAFETY_MASK", "BRD_SAFETYOPTION",
       "SERVO_RATE", "SERVO_GPIO_MASK", "SCR_ENABLE"]
)


def read(m, name, timeout=4.0):
    m.mav.param_request_read_send(m.target_system, m.target_component,
                                  name.encode(), -1)
    end = time.time() + timeout
    while time.time() < end:
        msg = m.recv_match(type="PARAM_VALUE", blocking=True, timeout=1.5)
        if not msg:
            m.mav.param_request_read_send(m.target_system, m.target_component,
                                          name.encode(), -1)
            continue
        pid = msg.param_id.decode() if isinstance(msg.param_id, bytes) else msg.param_id
        if pid == name:
            return msg.param_value
    return None


m = mavutil.mavlink_connection(PORT, baud=BAUD)
if not m.wait_heartbeat(timeout=25):
    raise SystemExit("no heartbeat")
print("heartbeat OK\n")

vals = {}
for n in NAMES:
    v = read(m, n)
    vals[n] = v
    if v is not None:
        note = ""
        if n.startswith("RELAY_PIN") and int(v) in AUX_PIN:
            note = "  <<< CLAIMS %s" % AUX_PIN[int(v)]
        if n.endswith("_FUNCTION") and v == -1:
            note = "  <<< GPIO, not PWM"
        print("  %-18s %-8g%s" % (n, v, note))
    else:
        print("  %-18s (absent)" % n)

print("\n--- verdict ---")
culprits = [n for n in NAMES
            if n.startswith("RELAY_PIN") and vals.get(n) is not None
            and int(vals[n]) == 50]
if culprits:
    print("  *** %s = 50 -> AUX1 is a RELAY pin, not a PWM output." % culprits[0])
    print("      That is why the FC ACKs DO_SET_SERVO, servo9_raw tracks, and")
    print("      the pin stays dead. FIX: set that RELAY_PIN to -1 (or move the")
    print("      servo to a free AUX), then reboot the FC.")
elif vals.get("SERVO9_FUNCTION") == -1:
    print("  *** SERVO9_FUNCTION = -1 -> AUX1 is GPIO. Set it to 0.")
else:
    print("  no relay/GPIO claim found on AUX1 from these parameters.")
gm = vals.get("SERVO_GPIO_MASK")
if gm:
    print("  note: SERVO_GPIO_MASK = %g - bit 8 set would make SERVO9 a GPIO." % gm)
    print("        bit 8 is %s." % ("SET" if int(gm) & (1 << 8) else "clear"))
