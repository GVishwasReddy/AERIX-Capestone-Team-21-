#!/usr/bin/env python3
"""Restore FC params from a pid_backup_*.param file. Usage:
    python pid_restore.py logs/pid_backup_YYYYMMDD_HHMMSS.param
"""
import sys, time, glob
from pymavlink import mavutil

PORT = "/dev/ttyACM0"


def set_param(m, name, value, timeout=4.0, tries=3):
    for _ in range(tries):
        m.mav.param_set_send(m.target_system, m.target_component, name.encode(),
                             float(value), mavutil.mavlink.MAV_PARAM_TYPE_REAL32)
        t0 = time.time()
        while time.time() - t0 < timeout:
            msg = m.recv_match(type="PARAM_VALUE", blocking=True, timeout=timeout)
            if msg and msg.param_id.strip("\x00") == name:
                return msg.param_value
    return None


def main():
    path = sys.argv[1] if len(sys.argv) > 1 else sorted(glob.glob("logs/pid_backup_*.param"))[-1]
    print("restoring from", path)
    pairs = []
    for line in open(path):
        line = line.strip()
        if line and not line.startswith("#") and "," in line:
            n, v = line.split(",", 1)
            pairs.append((n, float(v)))
    m = mavutil.mavlink_connection(PORT, baud=115200)
    m.wait_heartbeat(timeout=15)
    for n, v in pairs:
        got = set_param(m, n, v)
        print(f"  {n:18s} -> {v}  {'ok' if got is not None else 'FAILED'}")
    m.close()


if __name__ == "__main__":
    main()
