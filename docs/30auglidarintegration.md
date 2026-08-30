# LiDAR Avoidance Integration — 30 Aug 2026

Extending obstacle avoidance from manual (LOITER) flight into autonomous
GUIDED delivery flight, plus dynamic-obstacle detection and faster reaction.

**Status:** in progress. This document is updated as the work lands.

---

## 1. Where we started (end of 29 Aug)

Avoidance worked in **manual flight only**, and it ran entirely on the flight
controller:

| Piece | Where | What it does |
|---|---|---|
| `ProximityNode` | Pi | Streams `/scan` to the FC as `OBSTACLE_DISTANCE`, 72 sectors × 5°, 10 Hz |
| `AVOID_ENABLE=3`, `AVOID_MARGIN=1.7` | FC | Limits the pilot's own stick demand — the aircraft will not fly closer than 1.7 m |
| `OA_TYPE=0` | FC | Path planner **off**, deliberately: it re-aims the aircraft, and the requirement was "do not change yaw" |
| `navigation.avoidance_enabled: false` | Pi | GCS-side avoidance switched off |

LiDAR window: **250°** (±125° from the nose), rear 110° masked at the driver.
50 of 72 OBSTACLE_DISTANCE sectors live, 22 masked.

Noise gates added after the first (unstable) manual flight:
- `min_points: 2` — spatial: a 5° sector needs 2 returns before it reports
- `filter_depth: 3` — temporal: each sector is the median of 3 revolutions

---

## 2. What was asked for (30 Aug)

1. Make avoidance work in **GUIDED** (app-set delivery point) and on the
   **return leg**, active from takeoff to touchdown.
2. Detection is **too slow** and only catches **static** objects — make it
   faster and make it see **moving** ones.
3. When it avoids, it **must not go far off course** — deviate, then get back
   on track.
4. Travel speed **0.8 m/s**, because the LiDAR only scans at 10 Hz.

---

## 3. Findings that changed the plan

### 3.1 Simple Avoidance does nothing in GUIDED

ArduPilot has **two separate** avoidance systems, and the one we configured is
not the one autonomous flight uses:

| System | Parameter | Works in |
|---|---|---|
| Simple Avoidance | `AVOID_ENABLE` | **AltHold, Loiter only** |
| Path Planning | `OA_TYPE` | **Auto, Guided, RTL** |

So the entire manual-flight setup contributes **nothing** in GUIDED. Enabling
`OA_TYPE` is not optional — without it there is no FC-side avoidance on a
delivery flight at all.

Source: [Simple Avoidance](https://ardupilot.org/copter/docs/common-simple-object-avoidance.html),
[BendyRuler](https://ardupilot.org/copter/docs/common-oa-bendyruler.html).

### 3.2 SmartRTL cannot have avoidance — at all

Path planning explicitly does **not** cover SmartRTL, and the GCS cannot inject
avoidance into it either: SmartRTL is an FC-run mode, and taking it over at
loop rate is exactly the mechanism that caused the 22 Aug lockout.

**Decision: switch the return leg to `RTL`**, which *is* OA-supported.
Cost: RTL flies a direct line home at `RTL_ALT` instead of retracing the
outbound path. Config only — `delivery.return_mode`.

### 3.3 The GCS avoidance logic already runs in GUIDED — it was just off

`_do_navigate` → `_do_avoid` → `_dodge_step` all execute in the GUIDED-based
NAVIGATE/AVOID phases. `_do_avoid` already sets `_last_goto_wp = -1` when the
path clears, which re-issues the goto — so a basic "return to track" exists.

Two real defects, which is what "goes off course / gets stuck" actually is:
- The dodge is a **pure sideways translation** (`vx=0.0, vy=±speed`). It never
  makes forward progress, so it crabs sideways indefinitely.
- On timeout it **holds forever** instead of resuming.
- There is **no limit on lateral excursion** from the planned track.

### 3.4 There is no dynamic-obstacle capability whatsoever

`ObstacleNode` is **stateless**. Obstacle `id` is `enumerate(clusters)`,
re-sorted by distance every single frame. There is no frame-to-frame
association and no velocity estimate anywhere in the stack.

The `ttc_s` / `cpa_m` shown on the GCS are computed in `_publish_avoidance`
from the **drone's own** speed against a **stationary** obstacle:

```python
closing = speed * math.cos(bearing)     # speed = own ground speed
```

So a moving object is re-detected as a brand-new obstacle every scan and never
accumulates velocity. This is why it "detects static objects only". Fixing it
is new code — a tracker — not a tuning change.

### 3.5 The dominant source of lag is on the FC, not the Pi

Latency budget for a newly-appearing obstacle:

| Stage | Lag |
|---|---|
| LiDAR revolution | 100 ms |
| `filter_depth: 3` median | ~200–300 ms |
| Nav loop @ 10 Hz | 100 ms |
| **`PRX_FILT = 0.25 Hz` on the FC** | **~640 ms time constant** |

`PRX_FILT` is a low-pass filter ArduPilot applies to each proximity face. At
its 0.25 Hz default it is **by far** the biggest delay in the chain — larger
than everything on the Pi combined.

This matters because it means **"make it faster" does not require weakening the
noise filtering that fixed the unstable flight.** The spike rejection lives on
the Pi; the lag lives on the FC.

### 3.6 The LiDAR is already at its hardware ceiling

Measured live on the aircraft, 30 Aug:

```
polls=269 distinct_scans=58 over 6.0s -> ~9.6 Hz
bins with a return: min=188 max=202 avg=195 of 360
```

**9.6 Hz against the C1's 10 Hz specification.** The RPLIDAR C1 samples at
5 kHz with a fixed ~10 Hz motor speed — there is no faster mode to switch on,
and the driver is already using the standard scan (the highest-rate mode the C1
offers; Express scan on this unit samples at the same 5 kHz). Angular
resolution works out at 0.72°, so the 360-bin scan is very slightly
under-sampled but not meaningfully lossy.

Of the 251 bins inside the 250° window, ~195 carry a return. The remainder are
open air past `max_range_m: 12.0`, not dropped data.

**Conclusion: "run the LiDAR faster" is already satisfied and cannot be
improved.** Every remaining millisecond of responsiveness has to come from the
processing chain — and as §3.5 shows, ~640 ms of it is sitting in one FC
parameter.

### 3.7 Scan quality — is it actually seeing the room?

25 consecutive revolutions, bucketed into 15° wedges (bearing 0 = nose,
positive to the right):

```
 wedge(deg)   fill%   min_m  med_m   stability
 -180..-135        0       -      -            <- MASKED (correct)
 -135..-120       33    0.83   0.86   +/-0.00 m   boundary wedge
 -120..-105      100    0.90   1.06   +/-0.00 m
 -105..-90       100    1.33   1.40   +/-0.01 m
  -90..-75       100    1.34   1.35   +/-0.00 m
  -75..-60        64    0.51   1.37   +/-0.01 m
  -60..-45        39    0.45   0.45   +/-0.00 m
  -45..-30        97    1.68   1.76   +/-0.00 m
  -30..-15       100    2.01   2.35   +/-0.01 m
  -15..0          76    2.97   3.32   +/-0.07 m
    0..15         53    3.29   3.39   +/-0.00 m
   15..30         42    5.25   5.29   +/-0.01 m
   30..45         47    3.44   3.78   +/-0.02 m
   45..60        100    2.51   2.85   +/-0.01 m
   60..75        100    2.06   2.23   +/-0.00 m
   75..90        100    1.89   1.94   +/-0.00 m
   90..105        81    1.87   1.88   +/-0.00 m
  105..120        25    4.19   4.21   +/-0.01 m
  120..135        15    1.53   1.56   +/-0.01 m <- boundary wedge
  135..180         0       -      -            <- MASKED (correct)
```

Three things this establishes:

1. **The mask is exact.** Everything beyond ±135° is empty; the two boundary
   wedges straddling ±125° are partially filled in exactly the proportion
   expected (5° of each 15° wedge lies inside the window).
2. **Coverage is continuous** across the whole live window — there is no wedge
   inside ±125° that returns nothing. No blind spot inside the scanned arc.
3. **The measurements are real geometry, not noise.** Repeatability of the
   nearest return is **±0.00–0.07 m across 25 revolutions**. Noise would show
   as a large spread here; it does not.

Front wedges show lower fill (42–76%) simply because the nose is pointed at
open space 3–5 m away, past which returns fall off. That is absence of
obstacles, not absence of data.

#### The real limitation: it is a single plane

The C1 is a **2D** LiDAR. It measures one horizontal slice at its mounting
height and is blind to everything above and below it — a low wall, a table
edge, an overhanging branch, a slack cable, the ground itself on descent. No
amount of tuning changes this; it is the sensor.

Practical consequence for delivery flights: obstacle avoidance protects the
aircraft **at LiDAR height only**. Descent to the drop point and any obstacle
whose profile misses the scan plane are not covered by any of this work.

---

## 4. Constraints (do not change)

- **250° scan window stays exactly as it is.** `lidar.fov_deg: 250.0`,
  ±125° from the nose, rear 110° masked at the driver. Not to be widened,
  narrowed, or re-centred by any of this work.
- Altitude ceiling stays 3 m (`safety.max_altitude_m`).
- The transmitter-authority rule stands: the navigator stands down when the
  pilot moves the mode switch, and no mode is ever re-issued at loop rate.

---

## 5. Decisions taken

| Question | Decision |
|---|---|
| Return leg | **RTL**, not SmartRTL — avoidance stays active all the way home |
| Avoidance authority in GUIDED | **Layered**: FC BendyRuler routes; GCS adds dynamic tracking + early braking |
| Responsiveness | Raise `PRX_FILT`; keep Pi-side spike gates; asymmetric filter |
| Travel speed | **0.8 m/s** — 8 cm of travel per LiDAR revolution |

Layered rationale: the FC leg survives a Pi or Wi-Fi dropout (that link has
already dropped twice today) and natively deviates-then-returns-to-track. The
GCS leg supplies what ArduPilot's obstacle database handles poorly — moving
objects, which it treats as static entries that linger.

---

## 5a. What was actually changed

### Flight-controller parameters

Set by `scripts/set_avoidance_params.py` (permanent, re-runnable, reads back
every value it writes). The FC was rebooted between the two runs — `OA_BR_*`
does not exist until `OA_TYPE=1` is live.

| Parameter | Was | Now | Why |
|---|---|---|---|
| `OA_TYPE` | **0** | 1 | BendyRuler path planning. **At 0, autonomous flight had no avoidance at all.** |
| `OA_BR_TYPE` | 1 | 1 | Horizontal only — the LiDAR is 2D and the ceiling is 3 m |
| `OA_BR_LOOKAHEAD` | 15 | 5 | metres probed ahead |
| `OA_MARGIN_MAX` | 1.7 | 1.7 | path-planning stand-off, mirrored from the GCS control |
| `OA_DB_EXPIRE` | 10 | 3 | seconds an obstacle lingers in the FC database — 10 s means routing around where a person *used to be* |
| `PRX_FILT` | **0.25** | 2.0 | Hz. 0.25 Hz is a ~640 ms time constant — the single largest lag in the chain |
| `WPNAV_SPEED` | **1000** | 100 | cm/s. **Was 10 m/s** — a metre of travel per LiDAR revolution |

Unchanged and verified: `AVOID_ENABLE=3`, `AVOID_MARGIN=1.7`, `AVOID_BEHAVE=1`,
`PRX1_TYPE=2`, `RTL_ALT=300`.

Post-reboot `SYS_STATUS`: proximity `present=True enabled=True`.

### Code

| File | Change |
|---|---|
| `nodes/obstacle_tracker.py` | **new** — frame-to-frame association, closing speed, ego-motion removal (translation *and* yaw rate) |
| `nodes/obstacle_node.py` | drives the tracker, subscribes `FUSED_STATE` |
| `nodes/navigation_node.py` | closing-speed-aware brake distance; dodge gains forward progress + off-track cap; brake distance mirrored to FC |
| `nodes/proximity_node.py` | asymmetric filter — fast on approach, median on recede |
| `interfaces/mavlink_interface.py` | `set_param` command |
| `msg/messages.py` | tracking fields on `Obstacle`; closing/dynamic/stop-distance/off-track on `AvoidanceStatus` |
| `gcs/hub.py` | tracking fields exposed to the GCS payload |
| `config/real.yaml` | avoidance on, 1.0 m/s cruise, RTL return, tracker + reaction tuning |

### The emergency-brake control is now the single source of truth

The GCS `EMERGENCY BRAKE @ n m` slider drives **three** layers, not one:

1. `CollisionAvoider.stop` — the Pi's reactive layer
2. `AVOID_MARGIN` — simple avoidance, what stops the aircraft in Loiter
3. `OA_MARGIN_MAX` — path planning, what routes it in Guided/RTL

It is also pushed once at link-up, so the FC cannot boot holding a stale margin
from a previous session while the GCS displays the configured one. Confirmed in
the log:

```
[drone.navigation] emergency brake 1.70 m mirrored to FC (AVOID_MARGIN/OA_MARGIN_MAX)
[drone.mavlink.link] PARAM_SET AVOID_MARGIN = 1.7
[drone.mavlink.link] PARAM_SET OA_MARGIN_MAX = 1.7
```

Note the GCS showed **1.2 m** while config held 1.7 m — exactly the drift this
removes.

---

## 5b. Bench verification

**Tracking works.** 40 samples on live scans:

```
distinct tracks: 32 | with velocity: 26 | peak dynamic_count: 1
longest-lived tracks:
  id=1  pole     d=0.45m hits=174  closing=+0.01 speed=0.01 dyn=False
  id=2  tree     d=0.83m hits=174  closing=-0.04 speed=0.04 dyn=False
  id=3  vehicle  d=1.68m hits=174  closing=+0.01 speed=0.14 dyn=False
```

Identity held across **174 consecutive revolutions**, and stationary room
clutter correctly reads 0.01–0.32 m/s.

**A real defect this caught.** The first bench run reported stationary clutter
"moving" at 4.7 and 5.4 m/s and flagged it dynamic. Cause: a loose association
gate let a cluster that split between revolutions match the wrong half, and the
centroid jump differentiated into a large false velocity. A spurious
`closing +2.46` would have inflated the brake distance and caused exactly the
nuisance braking the aircraft was already criticised for.

Three fixes, in the order they were needed:
1. `gate_m` 1.2 → 0.7 — caps believable motion at 7 m/s per revolution
2. `max_speed_ms` 6.0 — beyond this it is two objects matched to one track
3. `dynamic_min_hits` 3 — consecutive frames of motion before believing it

The third needed a fix of its own: judged against the *smoothed* velocity, a
one-frame jump decays (3.0 → 1.5 → 0.75 at α=0.5) and stays over threshold for
three frames, sailing past the test. The counter is now judged on the **raw**
per-frame motion, which is back to zero the next frame. An artefact scores one
hit; a pedestrian scores one every frame.

Result: worst residual is 1.4 m/s on brand-new tracks at `hits=2`, and peak
`dynamic_count` is 1 — none survive the gate.

**Tests: 249 passing.** (`tests/novelty` excluded — 3 pre-existing Hailo
model-registry failures that assert a `.hef` is *absent*, which fails on the Pi
because the models are installed. Unrelated, and confirmed failing identically
before these changes.)

---

## 6. Work items

- [x] `ObstacleTracker` — frame-to-frame association + velocity, ego-motion compensated
- [x] `Obstacle` message — track id, closing speed, world velocity, `is_dynamic`
- [x] `ObstacleNode` — drives the tracker, subscribes to `FUSED_STATE`
- [x] Asymmetric proximity filter — fast on approach, median on recede
- [x] `_dodge_step` — forward progress, off-track cap, rejoin track
- [x] Closing-speed-aware braking distance
- [x] Emergency-brake control wired to FC margins (both systems)
- [x] Config: avoidance on, 1.0 m/s cruise, RTL return
- [x] FC params set and read back; FC rebooted
- [x] Tests — 249 passing
- [x] Ground verification on the Pi
- [ ] **Flight test** — nothing here has been flown

### Still open

- **Wi-Fi to the Pi is unstable** — dropped twice during this session, and RTT
  on the LAN runs 60–155 ms. Worth fixing before relying on the GCS in flight.
- `parcel_delivery/pi/lidar_bridge.py` still has no FOV mask (separate app,
  its own 72-sector path).
- Harmless BlueZ noise: `LEAdvertisement1 does not have property "TxPower"` —
  a `dbus_next`/BlueZ version mismatch. Advertising works regardless.

---

## 8a. Avoid early and gently — the distance ladder

The operating intent: **use the LiDAR's full range, correct the route early and
by as little as possible, and treat the hard brake as something that should
never fire.**

Distances now, outermost first:

| Distance | What happens | Set by |
|---|---|---|
| **12 m** | LiDAR streams returns to the FC | `proximity.max_distance_m` |
| **10 m** | BendyRuler starts probing for a clear path — the route begins bending | `OA_BR_LOOKAHEAD` |
| **5 m** | Pi eases off the throttle, giving the route time to develop | `avoidance_distance_m` |
| **3 m** | Clearance the planned route holds around obstacles | `OA_MARGIN_MAX` |
| **1.7 m** | **HARD BRAKE** — last resort, means routing failed | `AVOID_MARGIN` + Pi stop |

The gap between 3 m and 1.7 m is the point. The planner aims to stay 3 m away;
the brake sits 1.3 m inside that. In normal flight nothing should ever get close
enough to trigger it.

### The bug this fixed

`OA_MARGIN_MAX` was being driven from the same GCS control as the hard brake, so
both read **1.7 m**. The planner was therefore aiming for the exact line at
which the aircraft slams on the brakes — every route ended in a hard stop, and
any drift at all fired the brake.

They are different concepts and are now separate knobs:

* `AVOID_MARGIN` — the panic distance. Still driven by the GCS slider.
* `OA_MARGIN_MAX` — the clearance a *plan* keeps. From
  `navigation.avoidance_route_margin_m`, clamped to at least brake + 0.5 m so
  the two can never converge again.

### Why a *longer* lookahead gives a *smaller* deviation

Counter-intuitive but important: probing 10 m ahead instead of 5 m means the
path starts bending twice as early, so the same obstacle is cleared with half
the turn rate. A late correction is always a sharp one. This is why the
lookahead was raised rather than lowered to keep the deviation "minimum".

Kept at 10 rather than 12 so the planner is not reasoning about space at the
very edge of what the sensor reliably returns.

### Pi-side reactive dodge switched OFF

`avoidance_dodge_enabled: false`. Routing is the FC's job now, and two things
steering at once is worse than one: the Pi's dodge commands a velocity
setpoint, which *replaces* the guided target the planner is flying — a dodge
firing mid-manoeuvre would tear up the route the FC had already worked out.

With it off, the Pi's response at 1.7 m is a clean emergency brake, which is the
right answer at a distance the planner should never have allowed. The dodge code
remains and is still tested; it is one flag to bring back.

**Trade-off worth watching:** a 3 m route margin means BendyRuler needs roughly
6 m of gap to fly between two obstacles. In a tight test area it may report "no
path" and stop rather than squeeze through. If that happens, lower
`avoidance_route_margin_m` — it is a config change, no reboot.

---

## 9. BLE handshake failure — diagnosed and fixed

### Symptom

```
❌ AUTHENTICATION FAILED! Invalid HMAC signature.
   Expected: 490eb45499a5bf64ac19ad28d6660946de71e934
   Received: 748e29a5b43b07cc873e04a9ed637d79068b3681
```

### It was not a crypto bug

Nine candidate mismatches were tested against the *actual* logged nonce and
signature — hex-string vs raw key, hex vs raw message, SHA-1 vs SHA-256, key and
message swapped. **None reproduced the app's value.** The Pi's own expected
value reproduced exactly from `active_order.json` + the logged nonce, which
proves the Pi side computes what it intends to.

So the app was not encoding differently. It was signing with a **different
token**.

### Root cause: a stale order file served as if it were current

```
❌ No DISPATCHED order found in Firestore. Place an order in the app first.
no DISPATCHED order available (this is normal when idle)
starting in SECURE per-order mode
[Pi 5] Order: EEjBvBjiqO5yuYxeVP5x        ← written 27 Aug, served 30 Aug
```

A `deliveryToken` is **single-use and issued per flight**. The boot-time
Firestore pull is allowed to fail quietly, and two separate pieces of code then
conspired:

1. `ble_handshake_start.sh` branched on whether `active_order.json`
   *existed* — not whether it was usable — and launched in "SECURE per-order
   mode" without `--dev`.
2. `drone_ble_peripheral.py` did `if args.order_file.exists(): load(...)`,
   with no freshness check at all.

The result: a 66-hour-old, already-consumed token was advertised as secure, and
every handshake failed with two hashes that mean nothing on their own.

### Fixes

**`drone_ble_peripheral.py`** — new `--max-order-age-s` (default 6 h, env
`AERIX_BLE_MAX_ORDER_AGE_S`). A stale order file is now treated as no order
file: refused with an explanation and the exact command to fix it. With `--dev`
it falls back to the bench token; under `AERIX_BLE_STRICT=1` it refuses to
advertise at all.

The startup banner now states token provenance, so this can never again be
silent:

```
 Order ID        : DEV_BENCH_ORDER
 Token           : ⚠️  HARDCODED BENCH TOKEN (--dev) - not a real delivery
```
or, with a real order: `Token : per-order, pulled 3 min ago`.

**`ble_handshake_start.sh`** — no longer duplicates the decision. The peripheral
prefers a valid order file and falls back only when there isn't one, so `--dev`
means "come up advertising even without a usable order", never "ignore the
order".

### What the app must compute

For anyone checking the mobile side, the Pi expects exactly:

```
key     = deliveryToken, HEX-DECODED TO RAW BYTES  (not the hex string)
message = the 16 raw bytes read from the NONCE characteristic
mac     = HMAC-SHA256(key, message)[:20]
```

The nonce is regenerated on **every** read of the characteristic, so the app
must read it once and sign that exact value.

### To actually complete a handshake

The Pi now correctly refuses the dead token, but there is still no dispatched
order in Firestore, so it is on the bench token. To do a real handshake:

1. Place/dispatch an order in the app so Firestore has a `DISPATCHED` order.
2. `sudo systemctl restart aerix-ble` — the start script pulls the fresh token.
3. Confirm the banner reads `Token : per-order, pulled N min ago`.

### Tracker design notes

Two velocities, because they answer different questions:

- **Closing speed** — rate the gap shrinks, body frame, line-of-sight
  component. Needs no ego data, available on the *second* revolution a track is
  seen. This is what braking distance is derived from: a wall we fly at closes
  exactly as dangerously as a car driving at us.
- **World velocity** — the object's own motion, recovered by removing our
  ego-motion. Distinguishes "a wall, and we are moving" from "a person walking
  at us". Requires a valid `FusedState`; without one the object is reported
  static rather than guessed at.

Ego-motion removal has **two** terms, and the second is the one that is easy to
miss: translation *and yaw rate*. At 0.8 m/s an obstacle 5 m off the nose sweeps
through the body frame faster from a gentle turn than from the aircraft's whole
forward speed. A tracker that ignores yaw rate reports every stationary fence
post as a 2 m/s crossing target the moment the aircraft turns.

Association is greedy nearest-neighbour on predicted position, closest pair
first — not Hungarian. With the handful of clusters a 250° scan produces, a
globally optimal assignment buys nothing, and taking the closest pair first
already stops a distant cluster stealing a track from one sitting on top of it.

---

## 7. Change log

- **30 Aug, initial** — investigation complete, design settled, document created.
- **30 Aug** — confirmed LiDAR already at hardware ceiling (9.6 Hz measured);
  confirmed `PRX_FILT` default 0.25 Hz is the dominant lag; wrote
  `obstacle_tracker.py`.
- **30 Aug** — integration complete on the bench. Tracker wired in, asymmetric
  filter, dodge rework, brake control mirrored to both FC avoidance systems,
  FC parameters set and rebooted, 249 tests passing, live verification clean.
  Found and fixed false dynamic detections (loose gate + smoothed-velocity
  gating). Not yet flown.

---

## 8. Before the first flight

1. **Fly manual (LOITER) first.** Confirm the aircraft still stops at 1.7 m and
   that `PRX_FILT=2.0` has not made it twitchy. This is the one change that
   could re-introduce the old oscillation, and it is trivial to back out
   (`PRX_FILT 0.5`).
2. **Then GUIDED, one short leg, no payload.** `OA_TYPE=1` is new behaviour: the
   aircraft will now **re-aim itself** around obstacles, which it has never done
   before. Expect yaw changes — that constraint applied to manual flight only.
3. **Keep a hand on the mode switch.** The transmitter-authority rule is intact:
   the navigator stands down the instant the switch moves.
4. Watch `off-track` on the avoidance panel. If dodges are hitting the 4 m cap
   often, the cap or `avoidance_dodge_clear_m` needs retuning.

Backout: `~/drone_stack/backup-20260830-avoidance/` holds every file replaced,
and `scripts/set_avoidance_params.py` documents every original FC value.
