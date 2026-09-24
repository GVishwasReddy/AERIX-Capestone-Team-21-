# 31 Aug 2026 — camera lag, avoidance deadlock, post-delivery RTL

Status doc. Everything below is **deployed and running on the Pi** unless it
says otherwise. Continues `30auglidarintegration.md`.

---

## TL;DR

| # | Problem | Cause | Status |
|---|---|---|---|
| 1 | Camera lag / "froze once it went flying" | `fastNlMeansDenoisingColored` on every 720p frame | **FIXED** — 1 s/frame → 13 ms |
| 2 | Still laggy after that | Wi-Fi delivers 2.2 Mbit/s; stream produced 23.2 | **FIXED** — stream refitted to 1.3 Mbit/s |
| 3 | Horizontal line glitches | ESC interference on the CSI ribbon (row noise) | **FIXED** — destripe running, 128–230 rows/frame |
| 4 | "Avoidance isn't working properly" | Brake was a **one-way trip** — deadlock in BRAKE | **FIXED** |
| 5 | "Handshake done, stays hovering, no RTL" | Handler promised an RTL it could not issue | **FIXED** |
| 6 | Altitude hardlock braking mid-flight | Ceiling is 2.0 m, by decision | **CLOSED — 2 m confirmed, keep it** |
| 7 | Rear 110° unscanned, read as CLEAR by the FC | mask + `WP_YAW_BEHAVIOR=2` | **FIXED** — yaw gate + `WP_YAW_BEHAVIOR=1` |

350 tests pass on the Pi (`--ignore=tests/novelty`).

---

## 1–3. Camera

### What was wrong

`PiCamera._denoise` ran `cv2.fastNlMeansDenoisingColored` on **every** frame at
720p. Measured: **199.7 ms** on an M-series Mac against a 3 ms JPEG encode — 66×
the cost of the encode it fed, and several times worse on a Pi 5. Its docstring
claimed "tuned for 720p on a Pi 5 at ~30 fps"; it was off by ~30×.

Worse, non-local means **cannot remove the stripes anyway**. It works by finding
repeated structure and preserving it, and a horizontal stripe is nothing but
repeated structure. It was paying a second a frame to keep the artefact.

The lines themselves are row-correlated interference: the CSI ribbon runs
centimetres from four ESCs switching tens of amps, so the coupling lands on the
sensor's row readout. Two tells: horizontal (the readout axis), and only once
the motors spin.

### What replaced it

`drone_stack/gcs/frame_filter.py` — numpy only, three stages:

1. **Destripe** — each row's mean against a running *median* of its neighbours',
   removed by soft-thresholding. The median is load-bearing: it tracks a real
   horizontal edge (a horizon) without overshoot, so real content leaves ~zero
   residual. A mean would smear the horizon and then erase it as "stripe".
2. **Row repair** — rows destroyed outright are redrawn from the nearest
   surviving rows. Guarded so it does **not** fire next to a real edge (that bug
   halved an 89 DN horizon before the guard went in).
3. **Temporal** — motion-gated IIR. Gate reads block means with a deadband, or
   it mistakes the noise it is removing for motion and switches itself off.

Plus two free ISP wins that were never set: `NoiseReductionMode=Fast` and
`AeExposureMode=Short` (shorter exposure → less motion blur *and* less
rolling-shutter skew).

### The second bottleneck — the link

Fixing the Pi side exposed the real remaining problem. Measured on the aircraft:

```
raw Wi-Fi throughput  Pi → laptop :  2.2 Mbit/s     (20 MB took 72.7 s)
camera stream produced             : 23.2 Mbit/s
therefore delivered                :  1.9 fps of a 30 fps stream
```

The radio is *fine* — **-51 dBm, 72 Mbit/s PHY**. The problem is the network it
is on: SSID `HATHWAY GIRISH_2.4G_EXT`, a **2.4 GHz range extender on channel 4**.
A repeater halves throughput at best, and 2.4 GHz ch 4 in a residential area is
congested. That is where 30× of the capacity goes.

So the stream was refitted to the link that actually exists:

| | before | after |
|---|---|---|
| resolution | 1280×720 | 640×360 |
| fps | 30 | 10 |
| JPEG quality | 60 | 38 |
| frame size | 93.8 KB | 20.5 KB |
| produced | 23.2 Mbit/s | 1.7 Mbit/s |
| **delivered over Wi-Fi** | **1.5 Mbit/s ≈ 1.9 fps** | **1.28 Mbit/s ≈ 10 fps** |
| pipeline latency | ~1 s | **10.7–15 ms** |

**The single highest-value fix left is not software**: put the Pi on 5 GHz, or
on the main router instead of the `_EXT` repeater. Then put the resolution and
frame rate straight back up — the config comments say so and name the numbers.

Watching **both** cameras at once does not fit in 2.2 Mbit/s at any quality.
Open one at a time.

### Measuring it, not guessing

* `latency_ms` on each camera tile — stamped at capture, read at publish
  (grab → filter → overlay → encode). **Watch this, not the fps**: a camera can
  hold 30 fps while every frame it serves is a second old.
* `scripts/camera_latency.py` — run on the Pi against the live service. It
  separates *pipeline latency*, *encode rate* and *delivered rate*; the gap
  between the last two says whether the Pi or the link is the bottleneck.
  Validated against three simulated faults.
* The filter self-limits: over `budget_ms` it drops the temporal stage and logs
  it. It did exactly that at 720p on the Pi (35.6 ms > 22 ms) and keeps it at
  640×360.

---

## 4. The avoidance deadlock — the real cause of "avoidance isn't working"

### What the log showed (11:12:57 – 11:13:15 flight)

```
11:13:03  takeoff altitude 2.0 m reached - settling 2s
11:13:05  command 'brake' sent            <- avoidance braked
11:13:06  FC refused GUIDED and stayed in BRAKE     x4 over 10 s
11:13:06  command 'velocity' sent          x ~100, at 10 Hz
11:13:15  pilot override: transmitter moved - standing down
11:13:15  delivery phase -> ABORTED
```

### Why

`_do_navigate`'s SLOW branch streamed a `velocity` setpoint at 10 Hz.
`"velocity"` is in `_MODE_COMMANDS` (implying GUIDED) but **not** in
`_MODE_ONLY_COMMANDS` — so it recorded GUIDED as the commanded mode and **never
emitted a SET_MODE frame**. The same is true of `goto`.

So once avoidance braked, *nothing in the navigate path ever asked for GUIDED
again*. The FC sat in BRAKE ignoring every setpoint, `_note_mode_refusal`
reported "FC refused GUIDED" once a second, and it stayed that way until a human
took the aircraft.

**Every avoidance brake was a one-way trip.** Since the brake fires at 1.7 m and
there is always something within 1.7 m on a rooftop, it braked seconds after
takeoff and never recovered. That is the whole of "it isn't doing avoidance
properly" — the sensing was fine all along (FC params verified correct: `OA_TYPE 1`,
`OA_BR_LOOKAHEAD 10`, `OA_MARGIN_MAX 3`, `AVOID_MARGIN 1.7`, `PRX1_TYPE 2`,
`PRX_FILT 2`, `WPNAV_SPEED 100`).

### The fix

* **`_ensure_guided()`** — before commanding motion, if the FC is in BRAKE, ask
  for GUIDED and wait. Only BRAKE, because that is the mode this node put the
  aircraft in itself; a pilot-selected mode latches `_pilot_override` long
  before this is reached. `_set_mode` is self-pacing, so calling it every tick
  does **not** re-command the mode at loop rate (the 2026-08-22 rule stands —
  verified by a test that allows ≤2 SET_MODE frames in 20 ticks).
* **The SLOW branch no longer sends velocity at all.** The Pi does not steer:
  BendyRuler routes, `WPNAV_SPEED` sets the speed, and this node's only output
  is the brake. Verified: **0 velocity commands since restart.**
* **Release hysteresis** (`avoidance_release_m: 0.4`) — the gap must open to
  `stop + 0.4 m` before the hold is handed back, so scan jitter around the stop
  distance cannot chatter BRAKE against GUIDED at 10 Hz. CLEAR is exempt.

> A first attempt gated on `mode == "GUIDED"`, which silently stopped the
> mission whenever the mode was merely *unknown* (no heartbeat yet).
> `tests/test_delivery.py` caught it immediately. The gate is now narrowed to
> BRAKE only.

---

## 5. "Handshake done — it just hovers"

`_on_ble_delivery_result` set `_ble_delivered_at` and logged
**"RTL in 3s"** unconditionally. But that field is only ever read by
`_do_hover`. On the 11:14:07 handshake the mission had already aborted, so it
was not hovering — the promise was empty and the aircraft sat there.

Now the handler branches on what it can actually do:

| situation | behaviour |
|---|---|
| in the delivery hold | unchanged — `_do_hover` runs the countdown then RTLs |
| **airborne, not holding** | **commands RTL immediately** ← the reported bug |
| pilot override latched | commands **nothing**, says so loudly. Transmitter authority outranks the delivery. |
| on the ground | nothing, logged as such |

---

## 6. OPEN — the altitude ceiling. Needs your call.

The aircraft is being braked mid-flight by its own safety ceiling:

```
11:44–11:45  ALTITUDE HARDLOCK: 2.94 m above home exceeds ceiling 2.0 m
             + margin 0.7 m - braking. Take manual control.     (x187)
```

Config history on the Pi:

```
real.yaml.pre-vfh-20260830_152659   takeoff=3.0  ceiling=3.0
real.yaml.bak-20260830-153907       takeoff=2.0  ceiling=2.0   <- changed here
```

Someone dropped `safety.max_altitude_m`, `takeoff_altitude_m` and
`cruise_altitude_m` from 3.0 to 2.0 on 30 Aug ~15:39. It was flagged then and
never answered. The agreed standard profile is **3 m**.

**Decided 31 Aug: the lock stays at 2.0 m.** Nothing was changed — it was
already 2.0. The consequence to keep in mind: the hardlock trips above
**2.7 m** (2.0 ceiling + 0.7 margin), so *manual* flight above that will keep
braking the aircraft and spamming the console. That is the lock doing its job,
not a fault. Autonomous missions cruise at 2.0 m, leaving 0.7 m of headroom for
altitude-hold error and baro drift.

---

## 7. The masked rear — "look before you go"

The LiDAR window is 250 deg front-referenced, so the rear **110 deg is never
scanned**. Those sectors are streamed to the FC as `65535 = unknown`, and
**ArduPilot's proximity database treats unknown as CLEAR** — so BendyRuler
would route into ground the sensor has never seen, with complete confidence.
The aircraft also cannot reverse out of the rear (that is what the mask is for)
and at a 2 m ceiling it cannot climb over.

Two halves, because two different things fly the aircraft.

### The FC half — `WP_YAW_BEHAVIOR 2 → 1`

It was **2 = "face next waypoint EXCEPT RTL"**. Outbound that is fine: the nose
already follows the route, so the window covers the path. But **the entire
return leg was flown without ever turning** — the aircraft translates home at
whatever heading it finished the delivery on, so the scan window, and therefore
everything BendyRuler routes against, can point the wrong way for the whole
flight. That quietly undid the reason `return_mode` was changed
SMART_RTL → RTL in the first place.

Now **1 = face next waypoint, RTL included**. Set and read back with
`scripts/set_wp_yaw_behavior.py` (re-runnable).

### The Pi half — a yaw gate before translating

`WP_YAW_BEHAVIOR` makes the FC turn *while* flying. It does not stop the
aircraft moving before the turn finishes — at 1 m/s and 60 deg/s slew, a 180 deg
turn covers ~3 m of partially-unseen ground.

So `_yaw_gate_ok()` now gates every goto this node issues: a waypoint whose
bearing lands beyond ±125 deg is **not flown at**. The aircraft holds station,
yaws until its nose is on the path, and only then translates.

* **Holding costs no command.** Simply not issuing the goto leaves the aircraft
  on its previous one, which it has already reached — so it station-keeps in
  GUIDED while it turns. No mode change, no thrash.
* **Wide hysteresis**: engages beyond 125 deg, releases inside **60 deg**. It
  does not let go at the very edge of the window (the sparsest part of the
  scan) and cannot oscillate on its own threshold.
* **25 deg/s**, deliberately slow: a 180 deg turn takes ~7 s ≈ 7 full LiDAR
  revolutions, so the scan resolves what is back there instead of smearing
  through it.
* **The turn is commanded once, not per tick.** `CONDITION_YAW` is a discrete
  command, not a setpoint — re-sending it at 10 Hz restarts the turn forever.
* **On timeout it HOLDS and shouts.** It does not give up and fly blind:
  defeating a safety gate on a timer defeats the gate. But it logs an ERROR and
  puts `HELD:` on the status line, because the failure that actually hurts is a
  *silent* hold (see §4). `avoidance_yaw_before_move: false` disables it.
* **A braking obstacle still outranks it** — STOP is decided before the gate is
  consulted.

21 tests in `tests/test_yaw_gate.py`. **371 pass** overall.

---

## What is left

1. ~~Decide the altitude ceiling~~ — **done: staying at 2.0 m.**
2. **Move the Pi off the 2.4 GHz `_EXT` repeater.** Biggest remaining win for
   video, by a wide margin, and it is not a software problem.
3. **Fly it.** The avoidance recovery path, the yaw gate and the pre-RTL turn
   (§ 10) have been unit-tested but never flown. Expect the aircraft to brake at 1.7 m and then **resume by itself** —
   that is the fix working. Watch `rows_repaired` on the camera tile the moment
   the motors spin: climbing from zero confirms the EMI diagnosis.
4. ~~Pre-existing, untouched: `parcel_delivery/pi/lidar_bridge.py` has no FOV
   mask.~~ — **done, see § 8.**

---

## 8. The other lidar path — `parcel_delivery` got the FOV mask

`parcel_delivery/` is the standalone Firebase delivery stack. It is **not what
`aerix-gcs.service` runs** — nothing here touches the live service — but it
drives the same RPLIDAR C1 and the same Pixhawk, and it was streaming all
**72 sectors** of the scan while drone_stack's `ProximityNode` masks 22 of them.

So the two stacks disagreed about what the aircraft can see. Whichever one was
running last set the FC's picture, and one of the two was feeding it the
airframe's own tail — the exact thing `lidar.fov_deg` exists to stop, and the
reason an unmasked scan brakes seconds after a rooftop takeoff (§ 4).

`lidar_bridge.sector_keep_mask()` now applies the same rule as
`proximity_node.sector_keep_mask()`, judged on sector centres: **50 sectors kept
(0-24, 47-71), the rear 22 left `65535`**, symmetric about the nose. New env
vars `LIDAR_FOV_ENABLED` / `LIDAR_FOV_DEG` (default `true` / `250`), threaded
through `config.py` → `main.py` → `LidarBridge`. `LIDAR_FOV_ENABLED` is parsed
strictly — `bool("false")` is `True`, and silently disabling a safety mask on a
typo is not acceptable — so a bad value raises at load.

Deliberately masked to **unknown, not to a large distance**: a fake "nothing for
12 m" would be a claim about ground the sensor never scanned. Which carries the
same caveat as § 7 — ArduPilot reads unknown as *clear*, so this stops the FC
being fed the airframe; it does **not** stop BendyRuler routing into the unseen
rear. That is `WP_YAW_BEHAVIOR = 1` plus the yaw gate's job.

27 new tests (`parcel_delivery/tests/test_lidar_bridge.py`) covering the mask
geometry, nose/tail symmetry, the 122.5-vs-127.5 deg boundary sector, that a
rear beam cannot reach the FC, that the mask is applied *after* the mounting
offset, and that the bridge builds it by default. **100 pass** in
`parcel_delivery` (73 before). drone_stack's own runtime is untouched.

End-to-end check through the real send path, not just the bucketer: a synthetic
360 deg wall produces a 72-element array, all values inside uint16, **50 sectors
reporting 400 cm and the rear 22 at 65535**, `increment_f = 5.0`,
`frame = MAV_FRAME_BODY_FRD`.

One gotcha found and documented rather than changed: this bridge **adds**
`angle_offset_deg` to the raw clockwise angle, while drone_stack negates the raw
angle first and then adds its offset. The two rotate opposite ways, so the value
is not portable between them. Both are `0.0` on this airframe, so nothing is
wrong today — but a remount that sets one must not copy the number into the
other.

---

## 9. Three novelty tests that were asserting the dev machine

Found while checking nothing else broke. `tests/novelty` is excluded from the
usual run, which is how these stayed hidden: **on the Pi they failed.**

`test_registry_models_degrade_gracefully_without_hailo` and two siblings used
the shared `valid_config_dir` fixture, whose `hef_path` values are repo-relative
`models/*.hef`. On a Mac checkout that resolves to nothing, the adapter degrades,
and `ok is False` holds. **The Pi ships the real weights** (`terrain.hef`,
`yolov8n.hef`, 7-10 MB each) and has HailoRT, so the model loaded fine, `ok` was
`True`, and the assertion failed.

The test was asserting "this is a dev machine", not "the adapter degrades" — and
it passed everywhere except the aircraft, which is the wrong way round. Same
failure mode as the `test_adapters.py` fix on 14 Aug.

Fixed by pinning the environment instead of assuming it: a new
`missing_hef_config_dir` fixture points `hef_path` at a directory inside
`tmp_path` that is never created, so "no weights on disk" is true on every
machine and the degradation path is actually exercised. No runtime code changed.

`test_registry_versions_have_expected_shape` already handled both cases
correctly, and was left alone.

**490 pass on the Pi with nothing ignored** (373 + 117 novelty), plus 100 in
`parcel_delivery`. `aerix-gcs.service` and `aerix-ble.service` were untouched and
stayed up throughout.

---

## 10. Turn onto home before RTL — closing § 7's other half

§ 7 fixed "look before you go" for the outbound legs, but left the return leg
depending on `WP_YAW_BEHAVIOR` — an FC parameter this node **cannot read**. If it
is ever not 1, the aircraft flies home at whatever heading the delivery ended on,
with the unscanned rear 110 deg potentially leading, and nothing on the Pi knows.

So the Pi now turns the nose onto home *before* handing over. The return leg's
scan coverage stops depending on a setting nobody can see from here.

### Where it lives

No new `MissionPhase`. `_enter_rtl(turn_first=True)` sets phase `RTL` and starts
the turn without commanding the mode; `_do_rtl()` — which already runs every tick
in that phase — carries the turn as its first stage and calls the new
`_commit_rtl()` when it is done. Phase reads `RTL` throughout, which is honest:
it *is* returning. No enum change, no GCS ripple, and no gotos can leak out
because the RTL phase does not dispatch `_do_navigate`.

### It usually does nothing

Engages on the same criterion as `_yaw_gate_ok` — home beyond `fov_half_deg`
(125 deg), i.e. genuinely in the masked rear — and releases inside
`yaw_release_deg` (60 deg), reusing that gate's tuned constants because it is the
same physical question about a different leg. **A delivery that ends roughly
pointed homeward returns exactly as it did before, with no added delay.**

### Only the calm callers

`_enter_rtl` has seven callers and they are not the same kind of event. Only the
three "job finished" ones turn: BLE handshake outside the hold, waypoints
exhausted, hover complete.

The **failsafe** RTL does not. It fires on a low battery or a geofence breach,
and delaying that to turn is wrong in exactly the situation where delay costs
most. Nor do the three operator-commanded returns (`_svc_rtl`,
`_svc_abort_delivery`, the "rtl" intent) — a human pressing RTL is usually
reacting to something and means *now*. `_enter_rtl()` with no argument keeps its
old behaviour exactly, so a caller added later cannot silently inherit the delay.

### It never traps the aircraft

Skipped outright if home is unknown, the fix is invalid, or the bearing is
unresolvable. On timeout it **returns anyway** and logs an ERROR naming
`WP_YAW_BEHAVIOR` as the thing to check — deliberately unlike `_yaw_gate_ok`,
which holds. That gate decides whether to fly at a waypoint, where holding is the
safe answer; refusing to *start* an RTL keeps the aircraft airborne burning
battery until a human notices.

`avoidance_rtl_yaw_timeout_s` is **8 s**, not the gate's 20. A 180 deg turn at
25 deg/s is ~7 s, so 8 s means "should have finished by now". It also bounds a
real side effect: the phase is already RTL while turning, and `_check_failsafe`
deliberately does not fight an in-progress RTL — so the GCS-side failsafes are
suppressed for exactly that long. Survivable at 8 s, not at 20. The FC's own
battery failsafe is unaffected. `avoidance_yaw_before_rtl: false` disables it.

### The trap this could easily have shipped with

`_do_rtl`'s SMART_RTL fallback measures from `_rtl_requested_at` — "if the FC is
not in SMART_RTL 3 s after we asked, fall back to plain RTL". Stamping that at
the *start* of the turn would fire the fallback mid-turn, against a mode nobody
had asked for yet. It is stamped in `_commit_rtl`, when the mode is actually
commanded, and there is a test pinning that. `config/real.yaml` is
`return_mode: RTL` so the path is dormant on this airframe, but `default.yaml`
is SMART_RTL and every sim run would have hit it.

### Evidence

24 tests in `tests/test_rtl_yaw.py`: engages only when home is behind, including
a 120-vs-125 deg boundary case; **no delay and no yaw command when home is
already ahead**; timeout returns rather than holds; `CONDITION_YAW` sent once, not
per tick; zero mode changes during the turn; the SMART_RTL clock does not start
during it; and all four non-calm callers verified unaffected. Two of them drive
the real `_do_hover` path end-to-end rather than calling `_enter_rtl` directly.

**514 pass on the Pi** with nothing ignored. A 12 s sim bringup
(`config/sim.yaml`) boots all 8 nodes, runs and shuts down cleanly with the change
in. Not flown — add it to § "What is left" item 3.

## Backing any of it out

Every file touched has a timestamped backup beside it on the Pi
(`.predeploy_*`, `.bak_*`, `.bw_*`). `scripts/apply_camera_filter.py --check`
reports what is applied without changing anything.
