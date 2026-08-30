# CLAUDE.md — drone_stack / AERIX GROUND CONTROL

Project reference for Claude Code. Read this first before working in this repo.

> **Host:** Raspberry Pi (`pi@pi.local`, aarch64, Debian bookworm, kernel 6.12).
> **Repo root:** `~/drone_stack` — pure-Python (no ROS) drone GCS + autonomy stack.
> **Golden rule:** the stack defaults to **SIM** mode. Real hardware only runs
> when explicitly launched with `real`. Keep it that way — sim is the safety net.

---

## 1. What this is

`drone_stack` is a self-contained ground-control + obstacle-avoidance stack that
runs entirely on the Pi. It exposes a single-page web GCS called
**AERIX GROUND CONTROL** (FastAPI + WebSocket + Leaflet map) and drives a real
Pixhawk (ArduPilot) over MAVLink plus an RPLIDAR for obstacle sensing. The same
code runs against mock hardware (sim) so nothing physical is required to develop.

---

## 2. Hardware (verified present on the Pi)

| Device | Port / bus | Notes |
|---|---|---|
| Pixhawk 2.4.8 (ArduPilot, `sys=1`) | `/dev/ttyACM0` (`by-id: usb-ArduPilot_Pixhawk1_*`) | USB. `115200` baud in config, USB CDC ignores it. |
| Slamtec RPLIDAR C1 | `/dev/ttyUSB0` (`CP2102N` bridge) | `460800` baud. Read the device descriptor first. |
| USB webcam (C270) + Pi Cam v3 (imx708) | USB / CSI | Streamed as MJPEG tiles over the map. |
| Hailo-8 AI HAT+ (26 TOPS) | PCIe | Installed; not wired into the GCS yet. |
| **MG995R payload servo** | **Pixhawk AUX1 = SERVO output ch 9** | **Separate power supply; common GND shared with Pixhawk. Signal → AUX1.** Added 2026-08-07. See §6. |

---

## 3. Architecture

```
config/*.yaml ─▶ Config ─▶ build_supervisor() ─▶ Supervisor(bus)
                                                     ├─ MavlinkNode   (telemetry ⇄ commands)
                                                     ├─ LidarNode     (RPLIDAR scans)
                                                     ├─ FusionNode    (complementary filter)
                                                     ├─ ObstacleNode  (obstacle extraction)
                                                     ├─ NavigationNode(mission + avoidance)
                                                     └─ DiagnosticsNode
MessageBus (pub/sub, topic snapshots) is the spine. ServiceRegistry holds
callable commands (arm, disarm, rtl, land, scan_*, avoid_*, start_mission, ...).
```

- **Interfaces** (`drone_stack/interfaces/`) abstract hardware:
  `mavlink_interface.py` (`RealMavlink` via pymavlink / mock), `lidar_interface.py`.
- **Sim** (`drone_stack/sim/`): `mock_pixhawk.py` + `world.py` (kinematic drone),
  `mock_lidar.py`. `world.command()` ignores unknown NavCommands (sim-safe).
- **GCS** (`drone_stack/gcs/`): `server.py` (FastAPI, `include_web=False` engine),
  `hub.py` (`GcsHub` — owns the supervisor, builds the unified JSON payload at
  15 Hz over `/ws`, dispatches UI commands), `cameras.py`, `static/` (front-end).

### Command flow (UI → autopilot)
```
button/JS  ──send(cmd,params)──▶  /ws  ──▶  GcsHub.command()/_dispatch()
   └─ direct service (arm/rtl/…) OR publish NavCommand on Topics.MAVLINK_CMD
        └─ MavlinkNode._flush_commands ──▶ MavlinkInterface.send_command/_dispatch
             └─ pymavlink command_long_send / set_position_target_* to Pixhawk
```

---

## 4. Running it

```bash
# from ~/drone_stack (uses .venv, pymavlink 2.4.49 installed there)
scripts/start_gcs.sh sim     # detached, sim mode   → http://pi.local:8090
scripts/start_gcs.sh real    # detached, REAL Pixhawk + RPLIDAR
scripts/run_gcs.sh config/sim.yaml   # foreground (port 8000 default)
```

- **Web UI port: 8090** (`web.port` in `real.yaml`; `GCS_PORT` env overrides).
- Logs → `~/drone_stack/logs/gcs.out`. `start_gcs.sh` kills the previous
  instance and waits for the port to free before relaunching.

### Auto-start on boot (systemd)  (added 2026-08-08)
The GCS runs on boot via **`/etc/systemd/system/aerix-gcs.service`** (enabled),
`User=pi`, `WorkingDirectory=/home/pi/drone_stack`, **real mode**
(`config/real.yaml`), `Environment=GCS_PORT=8090`, `Restart=on-failure`. An
`ExecStartPre=/bin/sleep 10` gives the USB devices (Pixhawk `/dev/ttyACM0`,
RPLIDAR `/dev/ttyUSB0`) time to enumerate before the server opens the port.
```bash
sudo systemctl {status|restart|stop|start} aerix-gcs.service
journalctl -u aerix-gcs.service -f          # live logs (includes FC STATUSTEXT)
```
- **Only one instance may hold `/dev/ttyACM0` + port 8090.** Do NOT also run
  `start_gcs.sh` while the service is up — stop the service first
  (`sudo systemctl stop aerix-gcs.service`) or you get a bind conflict + the two
  processes fight over the Pixhawk/LiDAR/cameras.
- Setting FC params (e.g. `scripts/set_rc_aux.py`) also needs the serial port:
  `systemctl stop` → run the script → `systemctl start`.
- Static files (`index.html`, `app.js`, `prop3d.js`, `style.css`, `models/`) are
  served from disk per request — **front-end edits need no server restart**;
  Python edits (`hub.py`, interfaces, nodes) **do** require a restart.

### Config
- `config/default.yaml` — every tunable with defaults.
- `config/sim.yaml` / `config/real.yaml` — deep-merged on top. `real.yaml` pins
  `mavlink.connection: /dev/ttyACM0`, `lidar.port: /dev/ttyUSB0`, port 8090.
- `real.yaml` currently has `safety.failsafes_enabled: false` (bench testing —
  re-enable before real flights) and 6S LiPo voltage thresholds.

---

## 5. Live 3D drone widget  (added 2026-08-07)

A WebGL viewport, top-right, floating over the map, that **mirrors the physical
drone** in real time. Model: `gcs/static/models/motorpropsjoinednew.glb`
(Shapr3D export, ~36 MB). Node names: `front_left/right`, `back_left/right`
(the 4 rotors) + one body node.

**Files:** `gcs/static/prop3d.js` (three.js, ES module), plus the
`#prop3d-wrap / #prop3d-glass / #prop3d-canvas-host` block in `index.html` and
`style.css`. three.js `0.160.0` loads from the unpkg CDN via the `<importmap>`
in `index.html` (same online dependency the Leaflet map already has).

What it does:
- **Prop spin** scales with telemetry: `0` when disarmed, `IDLE_SPEED` when
  armed, `+SPEED_GAIN × ground_speed` (clamped). Adjacent rotors spin opposite.
- **Attitude**: `roll / pitch / heading` from telemetry are applied to a pivot
  group (order `YXZ`) and damped-lerped each frame, so the on-screen drone banks
  and yaws like the real one. Bridged from `app.js` via
  `window.setPropTelemetry(telemetryObject)` (called in `updateHeader`).

### ⚠️ The "black box" problem — do not reintroduce it
The GLB's materials are near-black carbon/dark metal with **no textures**
(baseColor ≈ 0.02–0.24). Naive three.js lighting renders it as a flat **black
blob / black box**. It is kept visible by:
1. **Transparent framebuffer** (`alpha:true`, clear alpha 0) — the canvas never
   paints an opaque rectangle over the map; a load failure shows *nothing*, not
   a black box.
2. **Image-based lighting** — a bright vertical-gradient environment map
   (`makeGradientEnv`) + `material.envMapIntensity = 2.4` so dark metal has
   something to reflect.
3. A cool **rim/back light** for edge highlights that separate the silhouette
   from the blurred map behind it, plus key/fill/hemi/ambient.

The frosted look behind the model is **pure CSS** (`#prop3d-glass`):
`backdrop-filter: blur(8px)` blurs only the slice of map under the widget, with
a radial `mask` feathering the edges so it reads as a HUD panel, not a hard box.
If you ever swap the model, keep these four things or the black box returns.

Perf note: 36 MB is large (CAD tessellation). It renders on the Pi's hardware
GL browser but is too heavy for headless/software-GL screenshotting. Consider
decimating the mesh / Draco compression if the widget feels sluggish.

---

## 6. Payload servo — Pixhawk AUX1 / MG995R  (added 2026-08-07)

**Wiring:** MG995R signal → **AUX1** (ArduPilot output **SERVO9**); servo powered
from its **own supply**; that supply's **GND is common with the Pixhawk**. Pi ↔
Pixhawk over USB (`/dev/ttyACM0`).

**Command path:** UI buttons (sidebar “PAYLOAD · AUX1 SERVO”) →
`send("set_servo",{channel:9,pwm:…})` → `GcsHub._dispatch` publishes
`NavCommand("set_servo")` → `MavlinkNode` → `RealMavlink._dispatch`:
- On first use per channel it sets **`SERVO9_FUNCTION = 0` (Disabled)** so
  ArduPilot lets `MAV_CMD_DO_SET_SERVO` drive that pin (persists on the FC).
- Then sends `MAV_CMD_DO_SET_SERVO` with `param1=channel(9)`, `param2=pwm`
  (clamped 800–2200 µs).

**PWM mapping (UI):** Lock = `1170`, Center = `1500`, **Release** = `1830`
(Release is behind a confirm dialog). **Travel limited to ~80° total** (±40° about centre, ~8.3µs/°) as of 2026-08-10 so the MG995R horn never bottoms out on its mechanical stop (earlier full 1000/2000 travel caused a mechanical failure). Widen only if the mechanism has clearance.

**Verified on hardware 2026-08-07:** heartbeat OK, `SERVO9_FUNCTION=0` set &
read back, `DO_SET_SERVO 9` → 1500/2000/1000 all `ACCEPTED`, `SERVO_OUTPUT_RAW`
`servo9_raw` tracked the commanded PWM. Servo left at 1000 µs (locked).
Works in sim too (mock ignores it, no crash).

---

## 6a. RC auxiliary switches — Pixhawk / transmitter  (added 2026-08-08)

Aux switch functions set on the FC via `RCx_OPTION` params (persist on the FC).
Set + read-back verified with **`scripts/set_rc_aux.py`** (one-shot pymavlink;
run only while the GCS service is stopped so the port is free):

| TX channel | `RCx_OPTION` | Value | Function |
|---|---|---|---|
| 6 | `RC6_OPTION` | **18** | **Land** (flight mode; works on the ground) |
| 7 (3-pos) | `RC7_OPTION` | **17** | **AutoTune** |
| 8 | `RC8_OPTION` | **31** | **Motor Emergency Stop** (motor cutoff) |

- **AutoTune only initializes when ARMED and flying in AltHold/Loiter.** On the
  bench (disarmed) flipping ch7 gives `FC: Mode change to AUTOTUNE failed: init
  failed` — expected, not a bug. In the air the mode field reads `AUTOTUNE`.
- **Land** and **Motor E-Stop** are seen immediately: Land switches mode on the
  ground; E-Stop emits `FC: RC8: MotorEStop HIGH/LOW` STATUSTEXT (motors cut,
  not a mode change).
- All FC STATUSTEXT (incl. these aux events) is forwarded to the GCS console and
  now to the on-screen announcement (see §5a).

---

## 5a. On-screen mode / aux announcement (translucent)  (added 2026-08-08)

A frosted, centred banner (`#mode-toast`) fades in/out on **any flight-mode
change** (`MODE · LOITER`, green) and on **aux-switch / mode STATUSTEXT from the
FC** (`RC8: MotorEStop HIGH`, `Mode change to AUTOTUNE failed: init failed` —
colour-coded by severity), so transmitter actions are trackable even when they
aren't a flight-mode change. Pure front-end (no restart needed):
- `gcs/static/index.html` — the `#mode-toast` div.
- `gcs/static/style.css` — `#mode-toast{.show,.good,.warn,.bad}` (translucent,
  `backdrop-filter: blur`, opacity/transform transition).
- `gcs/static/app.js` — `showToast()`; mode-change detect in `updatePanels`
  (`lastMode`), aux-STATUSTEXT detect in `updateConsole` (guarded by
  `seededConsole` so the console backlog doesn't flood on load).

---

## 7. Safety notes (carry-over — verify before flight)

- Pixhawk `ARMING_CHECK` was set to `0` in the past to get it to arm; failsafes
  have been disabled for bench work (`FS_GCS_ENABLE`, `real.yaml` failsafes).
  **Re-enable failsafes and arming checks before any real flight.** A Pi/link
  death mid-flight with GCS failsafe off is unrecoverable.
- Never trigger `arm` / `Release Payload` casually — both are behind confirm
  dialogs in the UI for this reason.

---

## 8. Backups & conventions

- Every edit session backs up to `~/drone_stack/.claude_backup_<timestamp>/`
  (static/, hub.py, mavlink_interface.py). Latest: `.claude_backup_20260808_153556`.
- Access: `ssh pi@pi.local` (password `123456789`). Use the `.venv` for Python.
- Keep this file updated as the project evolves (append to the relevant section).

## 9. Changelog
- **2026-08-07** — Live telemetry-driven 3D drone widget (attitude + prop spin,
  black-box-proof lighting, frosted map blur). Payload servo control on AUX1
  (SERVO9) end-to-end, verified on the physical Pixhawk. This CLAUDE.md created.
- **2026-08-07 (later)** — Two fixes: (1) **Arming** — `FRAME_CLASS` was `0`
  (undefined) on the Pixhawk, causing `PreArm/Arm: Motors: Check frame class and
  type` and `arm/disarm failed`. Se                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                       
## 10. Hailo NPU camera overlays  (added 2026-08-10)

Both camera feeds now carry **Hailo-8 (26 TOPS) NPU** inference overlays. All
neural inference runs on the accelerator — the Pi CPU only does capture, the
overlay draw, and JPEG encode. Verified: whole GCS (2 models + MAVLink + LiDAR +
WS) uses ~0.85 of one core, temp ~60 °C, LiDAR still 10 Hz, ~6.4 GB RAM free.

**Files:** `gcs/hailo_infer.py` (new — inference), `gcs/cameras.py` (wiring).

- **Shared device:** one process-global `VDevice` with the round-robin
  **scheduler** (`hailo_infer._get_vdevice`) is shared by both models, so the two
  camera threads run concurrently and HailoRT time-slices the Hailo-8.
- **USB webcam (C270, cam 0) → `terrain.hef`** segmentation. Input 384×640, one
  logit channel; thresholded (logit ≥ 0 ⇔ p ≥ 0.5, no `exp`) → translucent green
  tint + bright contour + `TERRAIN` label (`Segmenter`).
- **Pi camera (imx708, cam 1) → `yolov8n.hef`** person detection. On-chip NMS →
  flat FLOAT32 buffer `[count, (ymin,xmin,ymax,xmax,score)*100]`, de-letterboxed
  to frame pixels, drawn as boxes + `PERSON nn%` (`Detector`).

### Model choice & the weak-model caveat
`models/` holds `terrain.hef`, `yolov8n.hef`, `person.hef`. We use **yolov8n**
for the Pi cam (its baked-in NMS = clean, robust, minimal decode) over
`person.hef` (raw YOLOv8 head → needs manual DFL+NMS decode, and it was **not**
meaningfully more accurate in testing). **Both person models are weakly
trained/quantized**: on clear test people they peak at only ~0.27 (yolov8n) /
~0.37 (person.hef) confidence. Decode is verified correct (box lands on the
person), so this is a *model quality* limit, not a pipeline bug. Consequences:
- `Detector` score threshold is **0.25** (default `person.hef` NMS thr is 0.20);
  higher thresholds hide every real detection.
- For reliable person detection, **retrain / replace** the `.hef` (ideally on the
  drone's own aerial viewpoint) and drop it into `models/` — no code change
  needed; bump `score_thr` once the model is stronger.

### Camera capture changes (perf)
- **USB is decoupled**: a grabber thread keeps only the freshest frame while the
  main loop annotates+encodes in parallel, so the ~30 ms overlay no longer
  throttles capture. The C270 is **hardware-capped ~15 fps @ 640×480** on this
  USB bus (raw `read()` is 15 fps even at minimum exposure); 640×480 is kept so
  it doesn't starve the RPLIDAR. Result: **~15–20 fps**.
- **USB manual exposure**: auto-exposure is pinned to manual
  (`v4l2-ctl auto_exposure=1`, `exposure_time_absolute=USB_EXPOSURE` def 200,
  `gain=USB_GAIN` def 200), set **after** the device opens (opening resets UVC
  controls). Aperture-priority auto-exposure otherwise stretches exposure in low
  light and collapsed the frame rate to ~7 fps. Tune via `USB_EXPOSURE`/`USB_GAIN`.
- **Pi camera** switched from the hardware `JpegEncoder` to `capture_array` +
  `cv2.imencode` (needed to draw overlays). On CSI (no USB/LiDAR contention),
  runs **every frame at 30 fps** (`skip=1, fps=30`) — detection is only ~9 ms.
- Graceful degradation: if HailoRT / a `.hef` is missing, `_build_hailo_processors`
  returns `None` and the cameras serve raw video (sim-safe, no error).



## 11. Firebase delivery (order -> waypoints -> hover -> smart RTL)

One coordinate from the phone app becomes a flight. `FirebaseDeliveryNode`
(`nodes/firebase_delivery_node.py`) polls the order source, offers the order to
the operator, and hands the accepted target to `NavigationNode`, which expands
it into waypoints and flies the profile.

### The flight profile
Ground start -> home latched at power-on -> climb to **3 m** -> fly to the drop
point -> **hold in GUIDED for `hover_seconds`** -> **SMART_RTL** home -> land
and disarm.

### ⚠️ Never hover in POSHOLD/LOITER — it broke the airframe twice (2026-08-27)

`delivery.hover_mode` was `POSHOLD`. **POSHOLD, LOITER and ALT_HOLD take their
ALTITUDE from the pilot's throttle stick.** They hold altitude for a pilot
holding throttle at centre — and during an autonomous delivery nobody is
touching the transmitter, so the stick rests at `RC3_MIN` (999 µs here) and
ArduPilot reads it as *"descend at the full pilot rate."*

Proven from the FC's own dataflash logs (453 and 454, both flights identical):

| t (s) | what |
|---|---|
| —     | holding 3.4–3.7 m in GUIDED, climb rate ~0, `ThO`≈0.20 (= `MOT_THST_HOVER`) |
| 622.35 | `MODE POSHOLD (GCS_COMMAND)` — the navigator's `_enter_hover` |
| 622.70 | descent begins; **`DAlt` (desired alt) is driven down** — commanded, not a failure |
| 623.9  | −233 cm/s |
| 624.7  | passes Alt 0 still doing **−251 cm/s ⇒ ~2.5 m/s impact** — this is what broke the landing gear |
| 624.8  | ch6 → 2000: the **pilot reacting**, 1.3 s *after* the aircraft was already down |
| 627.2  | `RC8: MotorEStop HIGH` — pilot cutting motors post-crash |

The throttle channel read 999 throughout. `PILOT_SPEED_DN` was `0`, which means
"use `PILOT_SPEED_UP`" = 250 cm/s — exactly the observed rate. The E-stop and
the LAND switch in the logs are the **pilot's reaction, not the cause**; do not
re-diagnose this as pilot error.

The fix, in three layers:
1. `delivery.hover_mode: GUIDED` in `default.yaml` + `real.yaml`. The hold is
   entered by re-asserting the position setpoint (a `goto`), **not** a mode
   change, and issues zero commands for its duration.
2. `NavigationNode._STICK_ALTITUDE_MODES` — `_safe_hover_mode` **rejects** any
   stick-driven hover mode at construction and substitutes GUIDED, so config
   cannot re-arm the crash. `_guard_hover_altitude` is the backstop: sag more
   than `hover_alt_tolerance_m` (1 m) below the hold altitude and it re-asserts
   GUIDED + the target, once per breach.
3. FC param `PILOT_SPEED_DN` `0 -> 100` (`scripts/set_pilot_descent_limit.py`)
   so a *pilot* selecting POSHOLD with the throttle down gets 1 m/s, not 2.5.
   Autonomous descents are untouched (`LAND_SPEED` 30 cm/s, `WPNAV_SPEED_DN` 150).

**Why sim never caught it:** `SimWorld.set_mode` parks the aircraft in
POSHOLD/LOITER — it has no throttle stick to model. A stick-driven hover passes
in sim and destroys the real aircraft. Never take a green sim run as evidence
that a hold mode is safe.

**Still outstanding (transmitter-side, needs the operator):** `FLTMODE_CH=6`
with `FLTMODE1/2/3/5 = STABILIZE (0)`. STABILIZE maps the throttle stick
*directly to motor output* — grabbing the mode switch mid-delivery and landing
on one of those detents with the throttle down is an immediate free-fall, and
`PILOT_SPEED_DN` does not help there. Set those positions to LOITER/RTL/LAND,
and keep the throttle stick at mid during autonomous flight.

- **The 3 m ceiling is absolute.** Every commanded altitude passes through
  `NavigationNode._clamp_alt`, at both the plan level and in `_send`, so no
  service call, mission, NL command or order can exceed it. The `max_altitude`
  failsafe (ceiling + `altitude_margin_m`) is a backstop for an *uncommanded*
  climb only.
- **Home latches once**, on the ground, at the first 3D fix while disarmed. It
  never moves afterwards, so a mid-flight GPS jump cannot relocate "home".
- **SMART_RTL falls back to RTL.** ArduPilot refuses SMART_RTL silently when its
  path buffer is empty; `_do_rtl` watches for the mode not taking within 3 s and
  switches to plain RTL rather than loitering forever.
- **The transmitter always wins.** Nothing here disables RC LAND or motor
  cutoff. Instead the navigator *arbitrates*: an uncommanded mode change is
  detected (after `pilot_override_grace_s`, so command lag is not mistaken for a
  takeover), the navigator stands down into MANUAL and stops issuing commands.
  Only an operator `resume` / `start_mission` hands control back.

### Order sources (`interfaces/firebase_interface.py`)
`firestore` (real), `file` (sim / `scripts/inject_order.py`), `none`.
Field names follow the Flutter `OrderModel`: `targetLat`/`targetLng`,
`status: DISPATCHED`, `createdAt`. Write-back uses separate `drone*` keys with
`merge=True` so the app's `status == 'DISPATCHED'` query keeps working and the
customer's order does not vanish from their screen at takeoff.

**The service-account key is required and is not in the repo.**
Firebase console -> Project settings -> Service accounts -> Generate new private
key, then:

    scripts/firebase_setup.py <downloaded-key.json>

It validates the file (catching the common `google-services.json` mistake),
installs it at `config/firebase-service-account.json` mode 600, and does a live
read of the `orders` collection. The node retries every poll, so no restart is
needed. Without it the delivery panel shows link `no-credentials` and an amber
setup banner. `firebase-admin` must be in the venv (it is in `requirements.txt`).

### Safety gates
`auto_accept: false` in `real.yaml` — a human presses ACCEPT & FLY. Orders that
were already `DISPATCHED` when the node started are *never* auto-accepted (a
stale row must not launch an aircraft on boot); they are flagged
"pre-existing - confirm before flying". Targets are refused beyond
`max_delivery_radius_m`, at null island, or without a GPS fix.

### The GCS panel
The dashboard renders only what the node publishes — no delivery state lives in
the browser, so a reload or a second operator sees the same thing. It shows the
order inbox (**every** order the app has written; unflyable ones stay visible
with their reason), a six-step phase timeline, live ETA and remaining distance,
and the full order book in the bottom panel. Services: `delivery_accept`,
`delivery_reject`, `delivery_abort`, `delivery_select`, `delivery_refresh`,
`delivery_set_auto`, `delivery_status`, `delivery_inject`.

### Testing it without a phone or a key
    scripts/inject_order.py --north 40           # 40 m north, waits for ACCEPT
    scripts/inject_order.py --lat .. --lon .. --auto   # simulation only
The GCS "Test order" button does the same at the map cursor.

### Gotchas
- `ServiceRegistry.call(name, /, **data)` — `name` is **positional-only**. A
  payload key called `name` used to collide with it; pass `mission_name`.
- The FC mission upload is queued on `Topics.MISSION_UPLOAD` so the blocking
  MAVLink handshake runs on MavlinkNode's own thread (no reader race); the real
  outcome comes back as `MissionUploadResult`.
- `SimWorld.set_mode` genuinely changes behaviour (POSHOLD parks, RTL descends
  and disarms). Without that a simulated hover or RTL "passes" while the
  aircraft keeps flying to its last goto.

### Changelog
- **2026-08-27** — **Hard-landing root cause found and fixed.** The drop-point
  hover was flown in POSHOLD, whose altitude comes from the pilot's throttle
  stick; with the transmitter untouched that commanded a 2.4 m/s descent into
  the ground on every delivery, breaking the landing gear (dataflash 453/454).
  Hover is now GUIDED, stick-driven hover modes are rejected in code, a hover
  altitude-sag backstop was added, and `PILOT_SPEED_DN` was capped at 1 m/s.
  See the ⚠️ section above. 273 tests pass (3 pre-existing `test_model_registry`
  failures are unrelated — they assert a missing `.hef`, which now exists).
- **2026-08-20** — Firebase delivery chain end to end: order -> expanded
  waypoints -> 3 m cruise -> 60 s POSHOLD -> SMART_RTL -> land/disarm, with the
  altitude ceiling and pilot-override arbitration described in §11. New
  `FirebaseDeliveryNode`, `firebase_interface`, GCS delivery panel with an order
  inbox, `scripts/firebase_setup.py`, `scripts/inject_order.py`. Verified end to
  end in sim; live Firestore link confirmed against the phone app.
- **2026-08-10 (later)** — Hailo NPU overlays on both camera feeds (terrain seg
  on USB, yolov8n person detect on Pi cam); decoupled USB capture + manual
  exposure; Pi cam 30 fps. See §10. Backups in `.claude_backup_20260810_214900`.

---

## 11. AERIX Novelty Layer — autonomous markerless delivery (added 2026-08-14)

A new, **optional, additive** package `drone_stack/novelty/` sitting ABOVE the
flight stack in this file's §1-§10 — it never touches PX4 params, the MAVLink
interface, avoidance, or RTL logic directly. Off by default
(`config/default.yaml`'s `novelty.enabled: false`); `launch/bringup.py` only
constructs its one node (`DeliveryNode`) when that flag is true. Built and
tested on the Mac mirror first (per standing instruction), pushed and
committed here 2026-08-14 (`git log` — commit `297b1f7`).

**What it adds** (patent-oriented, one doc per module in `docs/novelty/*.md`,
one YAML per module in `config/novelty/*.yaml`, every threshold `# GUESSED`
pending real flight data):
- `landing_zone.py` (§2.1) — markerless zone scoring from terrain
  segmentation (surface suitability, slope, clutter, contiguous safe area).
- `recipient_auth.py` (§2.2/§2.5) — dual-factor BLE+vision release gating
  (six branches) and multi-person disambiguation (α/β/γ weighted score +
  margin rule). BLE's own position estimate is fused: phone GPS preferred,
  RSSI log-distance range as a coarser fallback.
- `motion_monitor.py` (§2.4) — descent-abort on recipient velocity (AND
  low-altitude) or a third party entering the landing zone.
- `mission_fsm.py` — a declarative state/transition table (patent Figure
  material) plus a small generic engine (`MissionFSM.advance`/
  `check_timeout`) that walks it.
- `delivery_node.py` — the ONE bus-facing node. Ground-projects raw
  `PersonDetection`s, sequences the modules above through the FSM, and
  issues `NavCommand`s exactly like `NavigationNode` already does
  (`hold`/`rtl` on `Topics.MISSION_CMD`, `goto`/`set_servo` on
  `Topics.MAVLINK_CMD`) — it pulls `NavigationNode` out of its own mission
  with `hold` before commanding descent directly, and hands off to
  `NavigationNode`'s own `rtl` service on `RTL`/`ABORT_RTL` rather than
  duplicating that lifecycle.
- **BLE re-gated**: `ble_handshake/drone_ble_peripheral.py` no longer drives
  the payload servo itself — a confirmed drop (HMAC-verified, as before) now
  POSTs to this same process's existing `/api/command` HTTP API
  (`cmd: ble_auth_event`), which `gcs/hub.py`'s new `_on_ble_auth_event`
  bridges onto the novelty bus for `DualFactorAuthenticator` to read
  alongside the INDEPENDENT vision channel — a spoofed/wrong-person BLE
  handshake can no longer release anything on its own. Also added a
  GPS-write BLE characteristic (16 bytes, 2× float64 LE) and fixed a
  staleness gap (publishes `authenticated=false` when the anti-replay nonce
  refreshes, so a disconnected phone's old "authenticated" state doesn't sit
  latched on the bus forever).
- **Camera tap**: `gcs/cameras.py`'s `_build_novelty_processors` replaces the
  old `_build_hailo_processors` — inference still runs exactly once per kept
  frame, but now via the novelty `ModelRegistry` (not raw `Detector`/
  `Segmenter` construction), publishing structured results
  (`PersonDetection` list / `SegmentationFrame`) on `NoveltyTopics` before
  drawing the identical overlay from that already-computed result.
  `CameraManager` now takes an optional `bus` param; `GcsHub` passes its own.

**Model**: `models/fabseg.hef` (already on this Pi, `models/` dir, sha256
`1993cde...`) is wired in as the real multi-class terrain model, replacing
the old binary `terrain.hef` for the novelty-layer overlay/scoring path
(`terrain.hef` itself is untouched, still on disk, no longer referenced by
`config/novelty/models.yaml`). **`yolov8n.hef` retrain still pending** —
person detection keeps using the current weak/placeholder weights already
documented in §10's "Model choice & the weak-model caveat".

### ⚠️ fabseg.hef's real class taxonomy (corrected 2026-08-14, same day)

Initially wired assuming a 7-class output matching `TerrainClass` exactly.
**Wrong** — verified on THIS Pi's real Hailo-8 after the service restart: the
model's real output is **8 channels**
(`MultiClassSegmenter(terrain): output shape (640, 640, 8)`), matching the
source landing_seg capstone's own 8-class training taxonomy (background,
safe_ground, paved, water, vegetation, structure, vehicle, person — see that
project's `labelmap.py`), not `TerrainClass`'s 7 values. Fixed in
`config/novelty/models.yaml`'s `class_map` (8 entries now, channel-commented,
`structure`/`vehicle`/`person` all collapse onto `TerrainClass.OBSTACLE` —
argmax runs over all 8 real channels first, so no discriminating information
is lost by three channels sharing one target class). **This is the kind of
bug that ONLY shows up against real hardware** — 190/190 tests were green on
both the Mac and this Pi before this was caught, because no test fed a real
8-channel Hailo output through the adapter; they all used synthetic
7-channel data matching the (wrong) assumption. Lesson for next time: get
real model output shape from actual hardware inference BEFORE writing the
class_map, not just from the model provider's stated class count.

### Known limitations (see docs/novelty/*.md for the full list per module)
- All `# GUESSED` thresholds across the five `config/novelty/*.yaml` files
  need real flight/bench data.
- `RELEASING`'s servo-release confirmation is a fixed 1.0s timer, not a real
  `SERVO_OUTPUT_RAW` readback (not on any bus topic yet).
- BLE peripheral changes are code-reviewed but were never integration-tested
  against real BlueZ/D-Bus (not installable on the Mac dev machine) until
  this process actually runs on hardware with a real phone.
- `fabseg.hef`'s NHWC-vs-NCHW output-layout defensive check in
  `MultiClassSegmenter.infer_class_map` is still unverified either way in
  the sense that only ONE layout has actually been exercised on real
  hardware so far (whichever this Pi's HailoRT export used) — the OTHER
  branch remains defensive-but-unexercised code.

### Changelog
- **2026-08-14** — AERIX novelty layer (Steps 2-6 of the implementation
  plan) built and tested on the Mac mirror, pushed here via `scp`, committed
  (`297b1f7`), 190/190 tests green on this Pi's real hardware (including the
  real `HAILO_OUT_OF_PHYSICAL_DEVICES` degradation path, since
  `aerix-gcs.service` already holds the one physical Hailo-8 — a bug in
  `tests/novelty/test_adapters.py`'s own test fixtures only surfaced here,
  fixed same day). Service restarted to load it (`novelty.enabled` stayed
  `false`, so no behavioral change from the restart itself). Real-hardware
  verification then caught `fabseg.hef`'s true 8-channel output (see the
  ⚠️ note above) — `config/novelty/models.yaml` corrected same day.
