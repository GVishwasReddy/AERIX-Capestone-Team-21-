#!/usr/bin/env python3
"""Apply a calmer PID / feel tune to the Pixhawk (ArduCopter) for a
Tarot-650, ~4 kg, 5010 360 KV, 40 A ESC, 6S 4000 mAh build.

Backs up every parameter it touches to logs/pid_backup_<ts>.param BEFORE
changing anything, so the whole tune is one-command reversible.
"""
import sys, time
from pymavlink import mavutil

PORT = "/dev/ttyACM0"

# --- calmer tune -------------------------------------------------------------
# Feel / smoothness (the main "calm" levers), gentler angle gains, conservative
# rate gains for a big/heavy frame with slow props, calmer nav speeds, and
# correct 6S battery + thrust-voltage scaling.
PARAMS = {
    # smoothness / stick feel
    "ATC_INPUT_TC": 0.20,          # softer stick response (def 0.15)
    "ANGLE_MAX": 2500,             # 25 deg max lean (def 3000)
    "ATC_ACCEL_R_MAX": 90000.0,    # calmer roll accel (def 110000)
    "ATC_ACCEL_P_MAX": 90000.0,
    "ATC_ACCEL_Y_MAX": 15000.0,    # calmer yaw accel (def 27000)
    "ATC_RATE_R_MAX": 120.0,       # cap roll rate deg/s (def 0=unlimited)
    "ATC_RATE_P_MAX": 120.0,
    "ATC_RATE_Y_MAX": 45.0,
    # angle (stabilize) P - slightly softer
    "ATC_ANG_RLL_P": 4.0,          # def 4.5
    "ATC_ANG_PIT_P": 4.0,
    "ATC_ANG_YAW_P": 4.0,
    # rate PIDs - conservative starting values for a 650-class frame
    "ATC_RAT_RLL_P": 0.09, "ATC_RAT_RLL_I": 0.09, "ATC_RAT_RLL_D": 0.005,
    "ATC_RAT_PIT_P": 0.09, "ATC_RAT_PIT_I": 0.09, "ATC_RAT_PIT_D": 0.005,
    "ATC_RAT_YAW_P": 0.18, "ATC_RAT_YAW_I": 0.018, "ATC_RAT_YAW_D": 0.0,
    "ATC_RAT_RLL_FLTD": 15.0, "ATC_RAT_RLL_FLTT": 15.0,
    "ATC_RAT_PIT_FLTD": 15.0, "ATC_RAT_PIT_FLTT": 15.0,
    # calmer autonomous speeds
    "WPNAV_SPEED": 500.0, "WPNAV_SPEED_UP": 150.0, "WPNAV_SPEED_DN": 100.0,
    "WPNAV_ACCEL": 100.0,
    "LOIT_SPEED": 500.0, "PILOT_SPEED_UP": 200.0, "PILOT_ACCEL_Z": 150.0,
    # motor / 6S battery
    "MOT_THST_EXPO": 0.70,         # big props (def 0.65)
    "MOT_SPIN_ARM": 0.10, "MOT_SPIN_MIN": 0.15,
    "MOT_BAT_VOLT_MAX": 25.2,      # 6S full
    "MOT_BAT_VOLT_MIN": 19.8,      # 6S min (~3.3 V/cell)
    "BATT_LOW_VOLT": 21.6, "BATT_CRT_VOLT": 20.4, "BATT_CAPACITY": 4000.0,
}


def get_param(m, name, timeout=4.0):
    m.mav.param_request_read_send(m.target_system, m.target_component,
                                  name.encode(), -1)
    t0 = time.time()
    while time.time() - t0 < timeout:
        msg = m.recv_match(type="PARAM_VALUE", blocking=True, timeout=timeout)
        if msg and msg.param_id.strip("\x00") == name:
            return msg.param_value
    return None


def set_param(m, name, value, timeout=4.0, tries=3):
    for _ in range(tries):
        m.mav.param_set_send(m.target_system, m.target_component,
                             name.encode(), float(value),
                             mavutil.mavlink.MAV_PARAM_TYPE_REAL32)
        t0 = time.time()
        while time.time() - t0 < timeout:
            msg = m.recv_match(type="PARAM_VALUE", blocking=True, timeout=timeout)
            if msg and msg.param_id.strip("\x00") == name:
                if abs(msg.param_value - float(value)) <= max(1e-4, abs(float(value)) * 1e-3):
                    return msg.param_value
    return None


def main():
    m = mavutil.mavlink_connection(PORT, baud=115200)
    print("waiting for heartbeat...")
    m.wait_heartbeat(timeout=15)
    print("connected sys=%d comp=%d" % (m.target_system, m.target_component))

    ts = time.strftime("%Y%m%d_%H%M%S")
    backup = f"logs/pid_backup_{ts}.param"
    ok = 0
    fail = []
    with open(backup, "w") as bf:
        bf.write(f"# AERIX PID backup {ts} - restore with pid_restore.py\n")
        for name, val in PARAMS.items():
            old = get_param(m, name)
            if old is None:
                print(f"  {name:18s} NOT FOUND (skip)")
                fail.append(name)
                continue
            bf.write(f"{name},{old}\n")
            got = set_param(m, name, val)
            if got is None:
                print(f"  {name:18s} {old} -> {val}  FAILED")
                fail.append(name)
            else:
                print(f"  {name:18s} {old} -> {got}")
                ok += 1
    print(f"\nbackup saved: {backup}")
    print(f"applied {ok}/{len(PARAMS)} params" + (f", FAILED: {fail}" if fail else ""))
    m.close()
    return 0 if not fail else 1


if __name__ == "__main__":
    sys.exit(main())
