#!/usr/bin/env python3
"""Read-only audit of the autopilot parameters this project depends on.

The Pi clamps every altitude it commands to 3 m, but a handful of behaviours
are the FC's alone - RTL's climb, the FC-side fence, battery failsafes - and no
amount of companion-computer clamping constrains them. This dumps those so the
software limit and the hardware limit can be compared.

Run only while aerix-gcs is stopped: it owns /dev/ttyACM0.
"""
import sys
import time
from pymavlink import mavutil

PORT, BAUD = "/dev/ttyACM0", 115200

WANTED = [
    # --- RTL: the big one. ArduPilot climbs to RTL_ALT before returning, and
    # RTL is what the navigator falls back to when SMART_RTL is refused.
    ("RTL_ALT",        "cm", "climb to this before returning (0 = keep current alt)"),
    ("RTL_ALT_FINAL",  "cm", "alt at the end of RTL (0 = land)"),
    ("RTL_CLIMB_MIN",  "cm", "minimum climb before heading home"),
    ("RTL_LOIT_TIME",  "ms", "pause above home before descending"),
    # --- SMART_RTL
    ("SRTL_POINTS",    "",   "path buffer size (0 = SMART_RTL unavailable)"),
    ("SRTL_ACCURACY",  "m",  "path simplification accuracy"),
    # --- FC-side fence
    ("FENCE_ENABLE",   "",   "0=off 1=on"),
    ("FENCE_TYPE",     "",   "bitmask: 1=alt 2=circle 4=polygon"),
    ("FENCE_ALT_MAX",  "m",  "FC altitude fence"),
    ("FENCE_RADIUS",   "m",  "FC circular fence"),
    ("FENCE_ACTION",   "",   "0=report 1=RTL/land 2=always land 3=SmartRTL 4=brake"),
    # --- battery
    ("BATT_MONITOR",   "",   "0=disabled 3=volt only 4=volt+current"),
    ("BATT_CAPACITY",  "mAh", ""),
    ("BATT_LOW_VOLT",  "V",  "FC low-battery threshold"),
    ("BATT_CRT_VOLT",  "V",  "FC critical threshold"),
    ("BATT_FS_LOW_ACT",  "", "0=none 1=land 2=RTL 3=SmartRTL..."),
    ("BATT_FS_CRT_ACT",  "", "0=none 1=land 2=RTL 3=SmartRTL..."),
    ("BATT_ARM_VOLT",  "V",  "minimum voltage to allow arming"),
    # --- RC / pilot authority
    ("FLTMODE_CH",     "",   "transmitter channel carrying the mode switch"),
    ("FS_THR_ENABLE",  "",   "RC-loss failsafe action"),
    ("FS_GCS_ENABLE",  "",   "GCS-loss failsafe action"),
    ("FS_OPTIONS",     "",   "failsafe option bitmask"),
    ("THR_DZ",         "",   "throttle deadzone for alt-hold modes"),
    # --- speeds / limits that shape a 3 m delivery
    ("PILOT_SPEED_UP", "cm/s", "max climb rate under pilot control"),
    ("PILOT_SPEED_DN", "cm/s", "max descent rate under pilot control"),
    ("WPNAV_SPEED",    "cm/s", "auto/guided horizontal speed"),
    ("WPNAV_SPEED_UP", "cm/s", "auto/guided climb rate"),
    ("WPNAV_SPEED_DN", "cm/s", "auto/guided descent rate"),
    ("LAND_SPEED",     "cm/s", "final descent rate"),
    ("ARMING_CHECK",   "",   "bitmask; 1 = all checks enabled"),
    # --- payload
    ("SERVO9_FUNCTION", "",  "AUX1 output function"),
    ("RC9_OPTION",     "",   "transmitter ch9 assigned function"),
]


def pid(msg):
    p = msg.param_id
    return p.decode() if isinstance(p, bytes) else p


def get_param(m, name, timeout=4.0):
    m.mav.param_request_read_send(
        m.target_system, m.target_component, name.encode(), -1
    )
    deadline = time.time() + timeout
    while time.time() < deadline:
        msg = m.recv_match(type="PARAM_VALUE", blocking=True, timeout=0.5)
        if msg and pid(msg).strip("\x00") == name:
            return msg.param_value
    return None


def main():
    print(f"connecting to {PORT} @ {BAUD} ...")
    m = mavutil.mavlink_connection(PORT, baud=BAUD)
    if not m.wait_heartbeat(timeout=15):
        print("no heartbeat - is aerix-gcs still holding the port?")
        return 1
    print(f"heartbeat from system {m.target_system} component {m.target_component}\n")

    results = {}
    for name, unit, note in WANTED:
        v = get_param(m, name)
        results[name] = v
        shown = "--- not present ---" if v is None else f"{v:g} {unit}".strip()
        print(f"  {name:<17} {shown:<22} {note}")

    print("\n" + "=" * 72)
    print("FINDINGS")
    print("=" * 72)

    def val(n):
        return results.get(n)

    rtl_alt = val("RTL_ALT")
    if rtl_alt is not None:
        m_alt = rtl_alt / 100.0
        if rtl_alt == 0:
            print("  OK   RTL_ALT=0: RTL returns at the current altitude, so a")
            print("       3 m cruise stays 3 m.")
        elif m_alt > 3.0:
            print(f"  ***  RTL_ALT = {m_alt:.1f} m. ArduPilot CLIMBS TO THIS before")
            print("       returning. The Pi's 3 m clamp cannot prevent it: RTL is")
            print("       flown by the autopilot, not commanded waypoint by waypoint.")
            print("       The navigator falls back to RTL whenever SMART_RTL is")
            print(f"       refused, so a return could climb to {m_alt:.1f} m.")
            print("       FIX: set RTL_ALT to 300 (3 m) or 0 (keep current alt).")
        else:
            print(f"  OK   RTL_ALT = {m_alt:.1f} m, within the 3 m ceiling.")

    srtl = val("SRTL_POINTS")
    if srtl is not None and srtl == 0:
        print("  ***  SRTL_POINTS = 0: SMART_RTL is DISABLED on this autopilot.")
        print("       Every return will fall back to plain RTL.")

    fence_en = val("FENCE_ENABLE")
    if fence_en is not None:
        if fence_en == 0:
            print("  note FENCE_ENABLE = 0: no FC-side fence. The only altitude")
            print("       limit is the Pi's, which does not apply to pilot input.")
        else:
            fa = val("FENCE_ALT_MAX")
            print(f"  note FC fence ON, FENCE_ALT_MAX = {fa:g} m" if fa else
                  "  note FC fence ON")

    bm = val("BATT_MONITOR")
    if bm is not None and bm == 0:
        print("  ***  BATT_MONITOR = 0: the autopilot is NOT measuring battery")
        print("       voltage. The GCS battery failsafe can never fire, and the")
        print("       dashboard will read 0.0 V with a pack connected.")

    return 0


if __name__ == "__main__":
    sys.exit(main())
