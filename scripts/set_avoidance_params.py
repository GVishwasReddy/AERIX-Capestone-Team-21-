#!/usr/bin/env python3
"""Configure the autopilot for obstacle avoidance in AUTONOMOUS flight.

Manual flight was already covered: AVOID_ENABLE drives ArduPilot's *simple*
avoidance, which limits the pilot's own stick demand. That system works in
**AltHold and Loiter only** and contributes nothing to a delivery flight.

Autonomous modes use an entirely separate system - path planning, OA_TYPE -
which covers **AUTO, GUIDED and RTL**. It is fed by the same OBSTACLE_DISTANCE
stream ProximityNode already sends, but it has its own margin parameter and is
off by default. Without this script a GUIDED delivery flies with no FC-side
avoidance whatsoever.

    https://ardupilot.org/copter/docs/common-simple-object-avoidance.html
    https://ardupilot.org/copter/docs/common-oa-bendyruler.html

Note SMART_RTL is supported by neither system, which is why
delivery.return_mode was changed to RTL.

Run only while aerix-gcs is stopped: it owns /dev/ttyACM0.

    sudo systemctl stop aerix-gcs
    .venv/bin/python scripts/set_avoidance_params.py
    sudo systemctl start aerix-gcs

Then POWER-CYCLE THE FLIGHT CONTROLLER. ArduPilot instantiates the avoidance
backends at boot; OA_TYPE and PRX1_TYPE do not take effect until it restarts.
"""
from __future__ import annotations

import sys
import time

from pymavlink import mavutil

PORT, BAUD = "/dev/ttyACM0", 115200

#: (name, value, why). Ordered so the readback reads like an argument.
WANTED: list[tuple[str, float, str]] = [
    # --- path planning: the whole point of this script -----------------------
    ("OA_TYPE",         1, "1=BendyRuler. Path planning for AUTO/GUIDED/RTL. "
                           "0 here means autonomous flight has no avoidance."),
    ("OA_BR_TYPE",      1, "1=Horizontal. Vertical search is useless to us: the "
                           "LiDAR is 2D and the ceiling is 3 m, so there is "
                           "neither data nor room to climb over anything."),
    ("OA_BR_LOOKAHEAD", 10, "metres probed ahead for a clear path. The C1 is "
                           "rated to 12 m and proximity.max_distance_m already "
                           "streams the full range, so this uses it. A LONGER "
                           "lookahead bends the path earlier and therefore more "
                           "gently - a late correction is always a sharp one. "
                           "Kept below 12 so the planner is not reasoning about "
                           "space at the very edge of what the sensor returns."),
    ("OA_MARGIN_MAX", 3.0, "clearance the PLANNED ROUTE keeps in Guided/RTL. "
                           "NOT the brake distance, and deliberately well "
                           "outside it: at 1.7 the planner shaved its path to "
                           "exactly the line the aircraft brakes at, so any "
                           "drift fired the brake. NavigationNode pushes this "
                           "from navigation.avoidance_route_margin_m and clamps "
                           "it to at least the brake distance + 0.5."),
    ("OA_DB_EXPIRE",    3, "seconds an obstacle lingers in the FC's database. "
                           "The 10 s default is built for static geometry: a "
                           "person who walked through nine seconds ago still "
                           "blocks the path, so the aircraft routes around "
                           "where somebody used to be. 3 s is still 30 LiDAR "
                           "revolutions - ample for anything that is genuinely "
                           "standing still."),
    # --- responsiveness ------------------------------------------------------
    ("PRX_FILT",      2.0, "low-pass cutoff on each proximity face, Hz. The "
                           "0.25 Hz default is a ~640 ms time constant - by far "
                           "the largest lag in the chain, bigger than the LiDAR "
                           "revolution, the median filter and the nav loop "
                           "combined. Spike rejection already happens on the Pi "
                           "(min_points + median), so this does not need to."),
    # --- speed ---------------------------------------------------------------
    ("WPNAV_SPEED",   100, "cm/s. GUIDED goto speed is the autopilot's, not the "
                           "Pi's, so navigation.cruise_speed_ms alone would not "
                           "slow the aircraft down. 1.0 m/s = ~10 cm of travel "
                           "per LiDAR revolution."),
]

#: Read back but never written here - these are the manual-flight settings and
#: the obstacle-database tuning, reported so a mismatch is visible.
AUDIT = [
    ("AVOID_ENABLE",  "bitmask of avoidance sources (manual flight)"),
    ("AVOID_MARGIN",  "stand-off for simple avoidance (AltHold/Loiter)"),
    ("AVOID_BEHAVE",  "0=slide 1=stop"),
    ("PRX1_TYPE",     "2=MAVLink proximity. Needs a reboot to take effect."),
    ("OA_DB_EXPIRE",  "seconds an obstacle lingers in the FC database. Matters "
                      "for MOVING objects: too long and the aircraft avoids "
                      "somewhere a person used to be."),
    ("OA_DB_SIZE",    "obstacle database capacity"),
    ("RTL_ALT",       "cm climbed before returning - RTL is now the return mode"),
]


def main() -> int:
    print(f"connecting to {PORT} @ {BAUD} ...")
    master = mavutil.mavlink_connection(PORT, baud=BAUD)
    if master.wait_heartbeat(timeout=15) is None:
        print("no heartbeat - is aerix-gcs still holding the port?")
        return 1
    print(f"heartbeat from sys={master.target_system} comp={master.target_component}")

    def fetch(name: str, timeout: float = 3.0):
        master.mav.param_request_read_send(
            master.target_system, master.target_component, name.encode(), -1
        )
        deadline = time.time() + timeout
        while time.time() < deadline:
            msg = master.recv_match(type="PARAM_VALUE", blocking=True, timeout=1)
            if msg and msg.param_id.strip("\x00") == name:
                return msg.param_value
        return None

    print("\n--- writing ---")
    failed = []
    for name, value, why in WANTED:
        before = fetch(name)
        master.mav.param_set_send(
            master.target_system, master.target_component,
            name.encode(), float(value), mavutil.mavlink.MAV_PARAM_TYPE_REAL32,
        )
        time.sleep(0.35)
        after = fetch(name)
        ok = after is not None and abs(after - float(value)) < 1e-3
        if not ok:
            failed.append(name)
        mark = "ok " if ok else "FAIL"
        shown = "?" if before is None else f"{before:g}"
        print(f"  [{mark}] {name:<16} {shown:>8} -> {value:<6g}  {why}")

    print("\n--- audit (not written here) ---")
    for name, why in AUDIT:
        value = fetch(name)
        shown = "?" if value is None else f"{value:g}"
        print(f"        {name:<16} {shown:>8}   {why}")

    if failed:
        print(f"\nFAILED to set: {', '.join(failed)}")
        return 1
    print("\nAll parameters set.")
    print("POWER-CYCLE THE FLIGHT CONTROLLER before flying - OA_TYPE and")
    print("PRX1_TYPE are only read when ArduPilot boots.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
