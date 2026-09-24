#!/usr/bin/env python3
"""Widen SERVO<n>_MIN/MAX so a servo can reach its full travel.

ArduPilot clamps DO_SET_SERVO to SERVO<n>_MIN..SERVO<n>_MAX regardless of what
the companion computer asks for. They ship at 1100-1900, which is 800 us -
about 72 deg of a 180 deg servo. So the GCS slider can be set to 500-2500 and
the horn will still stop dead at 1100 and 1900, with nothing anywhere reporting
a clamp. This is the FC-side half of the envelope; config/*.yaml
(payload.min_us / aux2_servo.min_us ...) is the GCS-side half, and BOTH have to
be widened or the narrower one wins silently.

Channel is an argument (added 2026-09-05, when a second servo went on AUX2):
AUX1 == SERVO9 (payload MG995), AUX2 == SERVO10 (MG90S). Defaults to 9.

    before: 1100-1900  (~72 deg)
    after:   500-2500  (180 deg)

⚠️  READ THIS FIRST. Running this output over its full travel bottomed the horn
on its mechanical stop and BROKE THE MECHANISM on 2026-08-10. Only widen once
the horn is disconnected from the latch, or you have confirmed clearance. Then
sweep with the GCS slider, find the real lock/release points, and NARROW these
back to just outside them.

Run with the GCS stopped - it owns /dev/ttyACM0:
    sudo systemctl stop aerix-gcs.service
    .venv/bin/python scripts/set_servo_travel.py                 # ch9, 500-2500
    .venv/bin/python scripts/set_servo_travel.py --ch 10          # ch10, 500-2500
    .venv/bin/python scripts/set_servo_travel.py --ch 10 1100 1900  # or any pair
    sudo systemctl start aerix-gcs.service

Read-modify-verify; re-runnable; reads back what it wrote.
"""
import sys
import time

from pymavlink import mavutil

PORT, BAUD = "/dev/ttyACM0", 115200


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


def main(argv):
    argv = list(argv[1:])
    ch = 9
    if "--ch" in argv:
        i = argv.index("--ch")
        try:
            ch = int(argv[i + 1])
        except (IndexError, ValueError):
            print("--ch needs a channel number (AUX1 == 9, AUX2 == 10)",
                  file=sys.stderr)
            return 2
        del argv[i:i + 2]
    if not 1 <= ch <= 16:
        print("channel must be 1-16 (AUX1 == 9, AUX2 == 10)", file=sys.stderr)
        return 2

    lo, hi = 500.0, 2500.0
    if len(argv) == 2:
        lo, hi = float(argv[0]), float(argv[1])
    elif argv:
        print(__doc__)
        return 2
    if not (400 <= lo < hi <= 2600):
        print("refusing %g-%g: outside what an RC servo accepts" % (lo, hi),
              file=sys.stderr)
        return 2

    m = mavutil.mavlink_connection(PORT, baud=BAUD)
    if not m.wait_heartbeat(timeout=25):
        print("no heartbeat", file=sys.stderr)
        return 1

    rc = 0
    print("  target: SERVO%d (%s)" % (ch, "AUX%d" % (ch - 8) if ch >= 9 else "MAIN%d" % ch))
    for name, value in (("SERVO%d_MIN" % ch, lo), ("SERVO%d_MAX" % ch, hi)):
        before = read(m, name)
        print("  %s: was %s" % (name, before))
        if before is not None and abs(before - value) < 1e-6:
            print("  %s: already %g - nothing to do" % (name, value))
            continue
        m.mav.param_set_send(m.target_system, m.target_component, name.encode(),
                             float(value), mavutil.mavlink.MAV_PARAM_TYPE_REAL32)
        time.sleep(1.0)
        after = read(m, name)
        ok = after is not None and abs(after - value) < 1e-6
        print("  %s: now %s  %s" % (name, after, "OK" if ok else "FAILED"))
        if not ok:
            rc = 1
    if rc == 0:
        print("\ntravel is now %g-%g us. Sweep with the GCS slider, find the real"
              % (lo, hi))
        print("lock/release points, then narrow these back around them.")
    return rc


if __name__ == "__main__":
    raise SystemExit(main(sys.argv))
