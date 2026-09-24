# AERIX — Firebase-to-Drone Parcel Delivery

A user requests a delivery to their current GPS location from a web app. The
request lands in Firebase. A Raspberry Pi onboard the drone picks it up,
validates it, flies a Pixhawk/ArduCopter airframe there via MAVLink, hovers,
releases the payload, and returns home — pushing live status and telemetry back
to the browser the whole time.

```
[webapp/ : browser Geolocation + Firebase SDK] --write--> [Firebase RTDB: /deliveries/{id}]
                              |
                        (listener)
                              v
                    [Pi: firebase_client.py]
                              |
                    [Pi: mission_validator.py]   validates + geofences
                              |
                    [Pi: state_machine.py] -> [flight_controller.py] --MAVSDK--> [Pixhawk]
                              |
                    pushes status + telemetry back up
                              v
                    [Firebase RTDB: /deliveries/{id}/status]

    [RPLIDAR C1] --> [Pi: lidar_bridge.py] --pymavlink--> [Pixhawk]
                     (streams OBSTACLE_DISTANCE;  OA_TYPE does the rerouting
                      continuously, independent   onboard — no path planning
                      of the delivery mission)    on the Pi)
```

> ⚠️ **This flies a real aircraft.** Everything below assumes you test in SITL
> first. Do not point this at a real Pixhawk until the full state machine —
> including every failsafe path — has run end-to-end in simulation.

## Layout

| Path | What it is |
|---|---|
| `pi/` | Everything that runs on the Raspberry Pi |
| `webapp/` | Vite + vanilla JS client, deploys to Vercel |
| `firebase/` | RTDB schema + security rules |
| `scripts/` | `simulate_delivery_request.py` for testing without the web app |
| `tests/` | pytest suite (no hardware required) |

## 1. Firebase setup

1. Create a Firebase project and enable **Realtime Database**.
2. Enable **Anonymous** sign-in under Authentication → Sign-in method.
3. Deploy the rules in `firebase/database.rules.json`:
   ```bash
   firebase deploy --only database
   ```
4. Generate a service-account key (Project settings → Service accounts →
   Generate new private key). Save it on the Pi, **outside the repo**, and point
   `FIREBASE_CREDENTIALS_PATH` at it. It grants full project read/write — never
   commit it.

Schema and who-writes-what: [`firebase/schema.md`](firebase/schema.md).

## 2. Pi-side stack

```bash
cd pi
python3 -m venv .venv && source .venv/bin/activate
pip install -r requirements.txt
cp .env.example .env      # then edit it
```

`HOME_LAT` / `HOME_LON` must match your actual launch point — the geofence is
measured from there, so a wrong home makes every request either reject or, worse,
allow a flight you did not intend.

Run it:

```bash
python main.py                # normal
python main.py --no-lidar     # SITL with no lidar attached
```

### Testing against SITL

Start ArduPilot SITL:

```bash
sim_vehicle.py -v ArduCopter --console --map --out=udp:127.0.0.1:14540
```

Point `MAVLINK_CONNECTION=udp://:14540` in `pi/.env`, then in three terminals:

```bash
python pi/main.py --no-lidar
```
```bash
python scripts/simulate_delivery_request.py --watch
```

You should see the status walk through
`mission_received → takeoff → enroute → arrived_hover → delivering →
delivery_confirmed → rtl → landed`, mirrored in `pi/logs/delivery.jsonl`.

Failsafe paths worth exercising before any real flight:

| Failsafe | How to trigger in SITL |
|---|---|
| Geofence reject (pre-flight) | `simulate_delivery_request.py --offset-m 5000` |
| Altitude reject | `simulate_delivery_request.py --alt 500` |
| Battery abort | `param set SIM_BATT_VOLTAGE 10.5` mid-flight |
| Geofence breach abort | Set `GEOFENCE_RADIUS_M` very low, then request a farther target |
| Hover timeout | Set `HOVER_TIMEOUT_S=10` and stub the payload release to never confirm |

### Required ArduPilot parameters (obstacle avoidance)

`lidar_bridge.py` only *streams* what the lidar sees. ArduPilot does the actual
rerouting, and only if these are set on the flight controller:

```
PRX1_TYPE  = 2      # MAVLink proximity sensor (PRX_TYPE on older firmware)
OA_TYPE    = 1      # 1 = BendyRuler, 2 = Dijkstra
AVOID_ENABLE = 7    # all avoidance sources
```

Set them in Mission Planner / QGroundControl or via MAVProxy `param set`. Verify
with `PRX_` status in the proximity view — if the Pi is streaming correctly you
will see returns on the radar display.

### Field of view — the rear wedge is masked

The C1 spins a full circle, but only **250 deg of it is streamed** to the flight
controller: 125 deg left + 125 deg right of the nose. The remaining 110 deg
directly behind is reported as `65535` (unknown) in every revolution.

That rear wedge is the aircraft's own tail and legs, plus whatever it is
standing next to. Those returns never move, so streaming them makes ArduPilot
brake for obstacles that are effectively bolted to the vehicle — which is what
a rooftop takeoff looks like to an unmasked scan.

```
LIDAR_FOV_ENABLED=true
LIDAR_FOV_DEG=250          # must match lidar.fov_deg in drone_stack/config/real.yaml
```

Two things this does **not** do, both worth knowing before you trust it:

- **Unknown is not "blocked".** ArduPilot's proximity database reads an unknown
  sector as *clear*, so BendyRuler will still happily route into ground the
  sensor has never scanned. Not turning your back on unscanned ground is a
  separate mechanism: `WP_YAW_BEHAVIOR = 1` plus drone_stack's yaw gate. See
  `docs/31aug_status.md` § 7.
- **It is not a reverse escape.** With the rear unmeasured there is no safe way
  out backwards; front and both sides blocked means brake and hold.

Set `LIDAR_FOV_ENABLED=false` only to bench-test a lidar off the airframe.

### Lidar driver notes (RPLIDAR C1)

The C1 is driven over **raw serial** by `lidar_bridge.RPLidarC1`, not through the
`rplidar` PyPI package. That package does not support the C1: its handshake fails
with `Descriptor length mismatch`, because the C1 free-runs its scan on power-up
and floods the buffer before the library ever gets to talk. The sequence that
does work is: `STOP` → flush → DTR low (motor on) → `SCAN` → 5s spin-up → parse
5-byte legacy measurement nodes.

Two hardware gotchas, both verified on this airframe:

- **Use the by-id device path.** The CP2102N bridge re-enumerates after any USB
  reset (`ttyUSB0` → `ttyUSB1` → `ttyUSB2`), so a hardcoded `/dev/ttyUSB0` starts
  failing with `No such file or directory`. `ls -l /dev/serial/by-id/` gives the
  stable path; put that in `LIDAR_PORT`.
- **Watch the USB current budget.** On a Pi 5 without a detected 5A USB-PD
  supply, firmware caps *total* USB current at 600 mA. The C1's motor draws its
  peak during the 5s spin-up, and exceeding the cap browns out the whole bus —
  every device re-enumerates at once and the scan dies mid-stream. Check with:

  ```bash
  sudo dmesg | grep -i over-current
  ```

  A clean run adds no new `over-current change` lines. If it does, fix the supply
  (official 27W PD PSU, `usb_max_current_enable=1` in `/boot/firmware/config.txt`
  if the drone's BEC genuinely sources 5A, or a separate 5V rail for the lidar)
  before trusting obstacle avoidance in flight.

Verify a real scan with a 90-second soak: expect ~10 Hz revolutions, ~25 kB/s,
and no new over-current lines.

### Link-loss behaviour

The Pixhawk's own RC/GCS failsafe brings the aircraft home independently of the
Pi. Nothing in `pi/` is required for the drone to get itself back. Verify this in
SITL by killing `main.py` mid-flight and confirming ArduCopter still triggers its
own failsafe RTL. Confirm `FS_GCS_ENABLE` and `FS_THR_ENABLE` are configured
before flying for real.

## 3. Web app

```bash
cd webapp
npm install
cp .env.example .env.local   # fill in your Firebase client config
npm run dev
```

Open `http://localhost:5173`. Browser Geolocation requires a **secure context** —
`localhost` counts, so local dev works over plain HTTP, but any other host needs
HTTPS. On Vercel's production URL this is automatic.

Firebase client config (`apiKey`, `projectId`, …) is public by design. The
protection is the security rules, not hiding those values.

### Deploy to Vercel

```bash
cd webapp
vercel            # first run links the project
vercel --prod
```

Set the `VITE_FIREBASE_*` variables from `.env.example` as Environment Variables
in the Vercel project (Settings → Environment Variables) before deploying —
Vite inlines them at build time, so a build without them produces a broken app.

## 4. Tests

```bash
pip install pytest
python -m pytest tests/ -q
```

100 tests covering coordinate/geofence/altitude validation, every state-machine
transition and failure path, the C1 measurement-node parser, the
`OBSTACLE_DISTANCE` message construction and the field-of-view mask (with
synthetic scans — no lidar needed).

## Known limitations

- **Altitude is relative-to-home**, not true AGL. The offboard setpoint uses
  `REL_HOME`, which assumes roughly flat terrain between launch and the delivery
  site. A downward rangefinder for real AGL hover is a later pass.
- **Payload release is a stub.** `payload_controller.py` is an interface only;
  on this airframe the real implementation should drive the Pixhawk AUX servo
  output rather than a Pi GPIO pin.
- **Precision landing / marker detection is a stub** (`precision_locate()`).
- **One mission at a time.** A second request arriving mid-flight is rejected.
- **This is a standalone stack.** It does not integrate with the existing
  `drone_stack` GCS on the Pi — see below.

## Relationship to `drone_stack` on the Pi

The Pi already runs a separate, mature stack (`~/drone_stack`, "AERIX GROUND
CONTROL") with its own MAVLink and lidar nodes, started at boot by
`aerix-gcs.service`.

**Only one process can hold `/dev/ttyACM0` (Pixhawk) and the lidar's `ttyUSB*`
at a time.** This stack is deliberately standalone for now, so before running it
against real hardware you must stop the other one:

```bash
sudo systemctl stop aerix-gcs.service
```

Running both simultaneously will make the two processes fight over the serial
ports. The planned follow-up is to fold this Firebase/RTDB path into
`drone_stack` as an additional node once it is proven separately, at which point
the conflict disappears.
