# Configuration guide

All behaviour is controlled by YAML files in `config/`. There is **no hardcoded
port anywhere in the code.**

## How configuration is resolved

1. `config/default.yaml` is loaded first (every parameter, documented).
2. The profile you pass (`--config config/sim.yaml` or `config/real.yaml`) is
   **deep-merged on top**, overriding only the keys it lists.
3. Optional environment overrides: any variable named `DRONE_<SECTION>_<KEY>`
   (upper-case, dots → underscores) overrides the merged value. Example:
   `DRONE_MAVLINK_CONNECTION=/dev/ttyACM1` or `DRONE_WEB_PORT=9000`.

This means a profile like `real.yaml` only needs the few values that differ from
the defaults.

## Going from simulation to real hardware

Edit `config/real.yaml`:

```yaml
mode: real
mavlink:
  connection: "/dev/ttyACM0"   # your Pixhawk port (or /dev/serial/by-id/...)
  baud: 115200
lidar:
  port: "/dev/ttyUSB0"         # your RPLIDAR C1 port
  baud: 460800
```

That is the **only** change required. Run `scripts/run_real.sh`.

## Key parameters

### `mode`
`sim` uses the mock Pixhawk and mock LiDAR (no hardware). `real` uses pymavlink
and the RPLIDAR driver.

### `mavlink.connection`
A pymavlink connection string:
- Serial: `/dev/ttyACM0`, `/dev/serial0`, `/dev/serial/by-id/...`
- UDP (SITL / MAVProxy): `udp:127.0.0.1:14550`
- TCP: `tcp:127.0.0.1:5760`

### `lidar.port` / `lidar.baud`
Serial device and baudrate for the RPLIDAR C1 (default baud `460800`).

### `safety.*`
Battery voltages/percentages, GCS link timeout, minimum satellites and GPS fix,
geofence radius and max altitude that drive the failsafe logic.

### `navigation.*`
Waypoint capture radius, cruise altitude/speed, and the obstacle distances at
which collision avoidance slows down (`avoidance_distance_m`) and hard-stops
(`avoidance_stop_m`).

### `web.port`
Dashboard port (default `8090`). Browse to `http://<pi-ip>:<port>`.

See `config/default.yaml` for the fully-commented list of every parameter.
