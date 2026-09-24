#!/usr/bin/env python3
"""Why isn't this servo moving? Read-only except for the test pulses.

The Pi can log "DO_SET_SERVO sent" AND get "accepted" back from the flight
controller while the output pin stays dead. ArduPilot acknowledges the command
and then declines to act on it in two common cases, neither of which is visible
from the companion computer:

  1. SERVO<n>_FUNCTION != 0. DO_SET_SERVO only drives an output that has no
     flight function assigned. The GCS sets it to 0 on first use but never
     reads it back, so a failed write looks identical to a working one.
  2. The safety switch is engaged. The FC ACKs every servo command and keeps
     the PWM rails off until the switch is pressed out.

This reads the parameters that decide both, then commands a few pulses and
watches SERVO_OUTPUT_RAW.servo<n>_raw to see whether the FC actually drives the
pin. That is the measurement that splits "FC not outputting" from "FC is
outputting, so it is wiring, power or the servo".

Channel is an argument (added 2026-09-05, when a second servo went on AUX2).
AUX1 == SERVO9 (payload MG995), AUX2 == SERVO10 (MG90S). Defaults to 9, so the
invocation recorded in CLAUDE.md keeps working unchanged.

Run with the GCS stopped - it owns /dev/ttyACM0:
    sudo systemctl stop aerix-gcs.service
    .venv/bin/python scripts/servo_diagnose.py        # AUX1 / SERVO9
    .venv/bin/python scripts/servo_diagnose.py 10     # AUX2 / SERVO10
    sudo systemctl start aerix-gcs.service
"""
import sys
import time

from pymavlink import mavutil

PORT, BAUD = "/dev/ttyACM0", 115200

CH = int(sys.argv[1]) if len(sys.argv) > 1 else 9
if not 1 <= CH <= 16:
    raise SystemExit("channel must be 1-16 (AUX1 == 9, AUX2 == 10)")
# Label the pin the way the wiring is labelled, not the way the FC numbers it -
# every wiring mistake this script has ever caught was described to us as
# "AUX2", never as "SERVO10".
AUX = "AUX%d" % (CH - 8) if CH >= 9 else "MAIN%d" % CH

P_FUNCTION = "SERVO%d_FUNCTION" % CH
P_MIN = "SERVO%d_MIN" % CH
P_MAX = "SERVO%d_MAX" % CH

PARAMS = [
    (P_FUNCTION,        "must be 0 (Disabled) for DO_SET_SERVO to drive %s" % AUX),
    (P_MIN,             "ArduPilot clamps DO_SET_SERVO to this floor"),
    (P_MAX,             "...and this ceiling - caps the usable travel"),
    ("SERVO%d_TRIM" % CH,     "output at trim"),
    ("SERVO%d_REVERSED" % CH, "1 = direction inverted"),
    ("BRD_SAFETY_DEFLT","1 = safety switch starts ENGAGED (outputs dead)"),
    ("BRD_SAFETYENABLE","legacy name for the same thing on some firmware"),
    ("BRD_SAFETY_MASK", "bitmask of outputs LIVE while safety is engaged"),
    # An ANALOG servo (MG995 on AUX1, MG90S on AUX2) expects ~50 Hz. A digital
    # one tolerates 400 Hz+. SERVO_RATE is shared by all non-motor outputs, so
    # one analog servo pins the rate for both. Swapping digital -> analog on an
    # output left at ESC rate is a classic "FC says it is driving the pin,
    # servo does nothing / just buzzes".
    ("SERVO_RATE",      "PWM update rate (Hz) for non-motor outputs - ANALOG wants 50"),
    ("RC_SPEED",        "main/ESC output rate (Hz) - 400-490 is normal for motors"),
    ("BRD_PWM_COUNT",   "how many AUX pins are PWM; if %s is GPIO there is NO signal" % AUX),
]


def read(m, name, timeout=6.0):
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


def servo_raw(m, timeout=3.0):
    end = time.time() + timeout
    while time.time() < end:
        msg = m.recv_match(type="SERVO_OUTPUT_RAW", blocking=True, timeout=1)
        if msg:
            return getattr(msg, "servo%d_raw" % CH, None)
    return None


def main():
    print("diagnosing %s (FC output SERVO%d)\n" % (AUX, CH))
    m = mavutil.mavlink_connection(PORT, baud=BAUD)
    if not m.wait_heartbeat(timeout=25):
        print("no heartbeat from the flight controller", file=sys.stderr)
        return 1
    print("heartbeat OK (sys %d comp %d)\n" % (m.target_system, m.target_component))

    print("--- parameters ---")
    values = {}
    for name, why in PARAMS:
        v = read(m, name)
        values[name] = v
        shown = "NOT PRESENT" if v is None else ("%g" % v)
        print("  %-18s %-12s %s" % (name, shown, why))

    print("\n--- is the FC actually driving the pin? ---")
    before = servo_raw(m)
    print("  servo%d_raw now: %s" % (CH, before))

    seen = []
    tracked = 0
    # Stay inside SERVO<n>_MIN/MAX so a clamp cannot be mistaken for a dead pin.
    for pwm in (1200, 1400, 1600, 1800, 1500):
        m.mav.command_long_send(
            m.target_system, m.target_component,
            mavutil.mavlink.MAV_CMD_DO_SET_SERVO, 0, CH, pwm, 0, 0, 0, 0, 0)
        # Drain briefly, then take the LAST reading rather than the first:
        # SERVO_OUTPUT_RAW is streamed, so the first frame after the command
        # can still be the pre-command value. Reading the stale one is what
        # made the first run of this script report "did not change".
        time.sleep(2.0)
        got = None
        for _ in range(6):
            r = servo_raw(m, timeout=1.0)
            if r is not None:
                got = r
        seen.append(got)
        hit = "" if got is None else ("  <- tracked" if abs(got - pwm) <= 5 else "  <- MISMATCH")
        if got is not None and abs(got - pwm) <= 5:
            tracked += 1
        print("  commanded %4d -> servo%d_raw %s%s" % (pwm, CH, got, hit))

    print("\n--- verdict ---")
    fn = values.get(P_FUNCTION)
    if fn is not None and abs(fn) > 0.5:
        print("  *** %s = %g, not 0. DO_SET_SERVO is ACKed and" % (P_FUNCTION, fn))
        print("      then ignored. FIX: set %s = 0." % P_FUNCTION)
    # Compare against the BEFORE value too - the first version of this script
    # only compared the samples with each other, so a pin that moved once and
    # then held was reported as dead.
    distinct = {s for s in seen if s is not None}
    if before is not None:
        distinct.add(before)
    moved = len(distinct) > 1
    print("  tracked %d of %d commanded values" % (tracked, len(seen)))
    if moved:
        print("  OK   servo%d_raw CHANGED with the commands: the FC IS driving" % CH)
        print("       the pin. The fault is past the flight controller -")
        print("       signal wire on %s, servo supply, ground, or the servo." % AUX)
    elif all(s is None for s in seen):
        print("  ??   no SERVO_OUTPUT_RAW seen at all - cannot tell. Check the")
        print("       telemetry stream rate.")
    else:
        print("  ***  servo%d_raw did NOT change. The FC is NOT driving the pin" % CH)
        print("       despite accepting the command. Look at %s" % P_FUNCTION)
        print("       above, and at the SAFETY SWITCH: if it is blinking, press")
        print("       and hold it until solid, then re-run this.")
    rate = values.get("SERVO_RATE")
    if rate is not None and rate > 100:
        print("\n  ***  SERVO_RATE = %g Hz. An ANALOG servo expects ~50 Hz" % rate)
        print("       and will buzz, jitter or sit still at this rate. The servo")
        print("       this replaced was digital, which tolerates it.")
        print("       FIX: set SERVO_RATE = 50.")
    elif rate is not None:
        print("\n  OK   SERVO_RATE = %g Hz, fine for an analog servo." % rate)
    cnt = values.get("BRD_PWM_COUNT")
    if cnt is not None and cnt < (CH - 8 if CH >= 9 else 1):
        print("\n  ***  BRD_PWM_COUNT = %g: %s is GPIO, not PWM." % (cnt, AUX))
        print("       servo%d_raw can track perfectly and the PIN still emits" % CH)
        print("       nothing. FIX: raise BRD_PWM_COUNT so %s is a PWM output." % AUX)

    lo, hi = values.get(P_MIN), values.get(P_MAX)
    if lo is not None and hi is not None:
        print("\n  travel note: the FC clamps DO_SET_SERVO to %g-%g us." % (lo, hi))
        print("  A full 180 deg sweep (500-2500) will be cut to that unless you")
        print("  widen %s/%s too:" % (P_MIN, P_MAX))
        print("      .venv/bin/python scripts/set_servo_travel.py --ch %d" % CH)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
