#!/usr/bin/env python3
"""Find a servo's real, reachable working positions - interactively and safely.

WHY THIS EXISTS
---------------
`aux2_servo.down_deg/up_deg` (165/90) and `deg_span: 180` were never measured -
they are datasheet nominals. The AUX6 MG90S reaches DOWN quietly but BUZZES at
UP, which is an analog servo's only way of saying "I am being told to go
somewhere I cannot reach or cannot hold". Buzzing is not cosmetic: the servo is
drawing stall current continuously and cooking its own gears and driver.

This walks the horn from a KNOWN-GOOD position toward the suspect one, in small
steps, and asks the operator what they see at each step. The us where motion
stops but noise starts is the real mechanical limit.

WHY IT TALKS HTTP, NOT MAVLINK
------------------------------
`POST /api/command` reaches the identical `GcsHub.command()` dispatch the web UI
uses, so this needs NO `systemctl stop aerix-gcs.service` and never fights the
GCS for /dev/ttyACM0 - the single most common way to break a bench session.
It also means every step is clamped by the real per-channel envelope and lands
in the GCS console, so the operator sees exactly what the UI would have sent.

  Read-only until you answer a prompt. Ctrl-C is safe at any point: the horn is
  always returned to the position it started from.

SAFETY
------
Full travel bottomed a horn on its stop and BROKE that mechanism on 2026-08-10.
So this tool NEVER sweeps blind:
  * it refuses to leave [--floor, --ceil], which default to the two positions
    the mechanism is already known to survive;
  * it moves one --step at a time, waiting for a human between steps;
  * it stops the moment you report binding or buzzing.

USAGE
    .venv/bin/python scripts/servo_calibrate.py --ch 14 --start 2333 --toward 1500

    At each step:  m = moved, still quiet      (keep going)
                   b = BUZZING / straining     (stop: limit found)
                   n = did not move, but quiet (stop: limit found)
                   back = undo one step        q = quit and restore
"""
import argparse
import json
import sys
import time
import urllib.error
import urllib.request

GCS = "http://127.0.0.1:8090"


def send(channel: int, pwm: int, timeout: float = 5.0) -> dict:
    """Command one servo position through the GCS, exactly as the UI does."""
    body = json.dumps(
        {"cmd": "set_servo", "params": {"channel": channel, "pwm": int(pwm)}}
    ).encode()
    req = urllib.request.Request(
        f"{GCS}/api/command", data=body,
        headers={"Content-Type": "application/json"}, method="POST",
    )
    with urllib.request.urlopen(req, timeout=timeout) as r:
        return json.loads(r.read().decode())


def envelope(channel: int) -> dict:
    """The hub's configured envelope for this channel - the clamp we live under."""
    with urllib.request.urlopen(f"{GCS}/api/state", timeout=5) as r:
        state = json.loads(r.read().decode())
    for block in ("aux2_servo", "payload"):
        cfg = state.get(block) or {}
        if int(cfg.get("channel", -1)) == channel:
            return cfg
    return {}


def us_to_deg(us: float, cfg: dict) -> float:
    lo, hi = float(cfg.get("min_us", 500)), float(cfg.get("max_us", 2500))
    span = float(cfg.get("deg_span", 180.0))
    return (us - lo) * span / (hi - lo)


def ask(prompt: str) -> str:
    try:
        return input(prompt).strip().lower()
    except EOFError:
        return "q"


def recommend(observations, cfg, direction):
    """Turn the observation log into the endpoint to write into config.

    `observations` is a list of (pwm_us, verdict) in the order they were tried,
    verdict being "ok" (moved freely and stayed quiet), "buzz" (straining) or
    "stuck" (refused to move further but stayed quiet).

    `direction` is -1 when stepping DOWN in us (toward a smaller pulse) and +1
    when stepping up, so "just short of the limit" means opposite `direction`.

    Returns (safe_us, reason_string) - or (None, why_not) if nothing conclusive.
    """
    if not observations:
        return None, "no observations recorded"

    bad_i = next((i for i, (_, v) in enumerate(observations) if v != "ok"), None)

    # Two inconclusive outcomes, and BOTH are real results worth reporting
    # honestly rather than dressing up as a number.
    if bad_i is None:
        return None, (
            "swept the entire range and the horn never complained - so the "
            "buzz is NOT positional. A servo that is quiet at every angle on "
            "the way in but buzzes once parked is fighting a holding LOAD or a "
            "sagging servo rail, not a mechanical stop. Changing up_deg will "
            "not fix that; measure Vservo under load next"
        )
    if bad_i == 0:
        return None, (
            "the very first step already complained, so the limit lies outside "
            "the range swept - nothing was measured. Restart from a position "
            "known to be quiet, with a smaller --step"
        )

    last_ok = observations[bad_i - 1][0]
    first_bad = observations[bad_i][0]
    step = abs(first_bad - last_ok)

    lo, hi = float(cfg.get("min_us", 500)), float(cfg.get("max_us", 2500))
    us_per_deg = (hi - lo) / float(cfg.get("deg_span", 180.0))

    # The travel the horn actually proved it can do quietly: from where the
    # sweep began (one step before the first reading) to the last quiet one.
    start_us = observations[0][0] - direction * step
    working_range = abs(last_ok - start_us)

    # The margin has to cover two unrelated things, so take whichever is
    # bigger rather than picking one and hoping:
    #
    #   - MEASUREMENT UNCERTAINTY. The true limit is somewhere in the `step`
    #     between last_ok and first_bad. We do not know where, so half a step
    #     is spent on not knowing before any safety margin at all.
    #   - WEAR AND SAG. 10% of the range the mechanism actually uses. This is
    #     the term that scales: on a 75 deg range it yields a comfortable
    #     ~7.5 deg, on a 10 deg range it correctly shrinks to ~1 deg instead
    #     of a fixed 50 us eating a third of the travel.
    #
    # Backing off from last_ok (not from first_bad) is deliberate: last_ok is
    # the furthest point with EVIDENCE behind it. first_bad is already broken.
    margin = max(0.5 * step, 0.10 * working_range)
    safe = int(round(last_ok - direction * margin))
    safe = max(int(lo), min(int(hi), safe))

    backoff = abs(safe - last_ok)
    reason = (
        f"limit is between {last_ok} and {first_bad} us; backed off "
        f"{backoff:.0f} us ({backoff / us_per_deg:.1f} deg) from the last quiet "
        f"reading = max(half a {step} us step, 10% of the {working_range:.0f} us "
        f"proven range)"
    )
    return safe, reason


def main(argv) -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--ch", type=int, default=14, help="FC servo channel (AUXn == SERVO n+8)")
    ap.add_argument("--start", type=int, required=True, help="known-good position to begin from")
    ap.add_argument("--toward", type=int, required=True, help="suspect position to walk toward")
    ap.add_argument("--step", type=int, default=50, help="us per step (default 50 ~= 4.5 deg)")
    ap.add_argument("--settle", type=float, default=0.8, help="seconds to wait after each step")
    args = ap.parse_args(argv[1:])

    cfg = envelope(args.ch)
    if not cfg:
        print(f"!! channel {args.ch} is not in any configured envelope - refusing to drive it")
        return 2
    lo, hi = int(cfg["min_us"]), int(cfg["max_us"])
    direction = 1 if args.toward > args.start else -1

    print(__doc__.split("USAGE")[0])
    print(f"channel {args.ch} (AUX{args.ch - 8})   envelope {lo}-{hi} us"
          f"   deg_span {cfg.get('deg_span')}")
    print(f"walking {args.start} -> {args.toward} us in {args.step} us steps"
          f"  ({args.step / ((hi - lo) / float(cfg.get('deg_span', 180))):.1f} deg per step)\n")
    if ask("horn is CLEAR to move and you are watching it? [yes/no] ") not in ("y", "yes"):
        print("aborted - nothing commanded")
        return 1

    observations = []
    pwm = args.start
    send(args.ch, pwm)
    print(f"  -> {pwm} us  (start, known good)")
    time.sleep(args.settle)

    try:
        while True:
            nxt = pwm + direction * args.step
            if direction * (nxt - args.toward) > 0:
                nxt = args.toward
            if nxt == pwm:
                print("\nreached the target without finding a limit.")
                break
            send(args.ch, nxt)
            time.sleep(args.settle)
            pwm = nxt
            print(f"  -> {pwm} us  ({us_to_deg(pwm, cfg):.1f} deg)")
            v = ask("     [m]oved+quiet  [b]uzzing  [n]o movement  back  [q]uit: ")
            if v in ("q", "quit"):
                break
            if v == "back":
                observations.pop() if observations else None
                pwm -= direction * args.step
                send(args.ch, pwm)
                time.sleep(args.settle)
                continue
            verdict = {"m": "ok", "b": "buzz", "n": "stuck"}.get(v[:1], "ok")
            observations.append((pwm, verdict))
            if verdict in ("buzz", "stuck"):
                print(f"\nlimit found at {pwm} us - stopping.")
                break
    finally:
        print(f"\nrestoring to {args.start} us")
        try:
            send(args.ch, args.start)
        except urllib.error.URLError as exc:
            print(f"!! COULD NOT RESTORE: {exc} - move it from the GCS before leaving")

    print("\n--- observations ---")
    for p, v in observations:
        print(f"  {p:5d} us  {us_to_deg(p, cfg):6.1f} deg   {v}")

    safe_us, why = recommend(observations, cfg, direction)
    if safe_us is None:
        print(f"\nno recommendation: {why}")
        return 0
    print(f"\nrecommended working endpoint: {safe_us} us"
          f"  ({us_to_deg(safe_us, cfg):.1f} deg)   [{why}]")
    print("\nput it in config/default.yaml under aux2_servo - not in app.js:")
    print(f"  up_deg: {us_to_deg(safe_us, cfg):.1f}")
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv))
