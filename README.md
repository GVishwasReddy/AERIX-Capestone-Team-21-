# drone_stack

A **complete, ROS-free autonomous drone software stack** in pure Python for the
Raspberry Pi 5 (Raspberry Pi OS / Debian 12 "bookworm", Python 3.11+).

It talks to a **Pixhawk 2.4.8 (ArduPilot)** over MAVLink and a **Slamtec
RPLIDAR C1** over USB, and it runs **fully in simulation with no hardware
attached**. Switching to real hardware only requires editing serial ports in a
YAML file.

> **Why not ROS 2?** ROS 2 Jazzy ships official binaries only for Ubuntu 24.04,
> not Raspberry Pi OS (bookworm). Instead of forcing Docker/conda, this project
> reproduces the parts of ROS we need — a **named-topic publish/subscribe bus**,
> **typed messages**, **parameters**, **launch files**, and a **visualiser** —
> in dependency-light pure Python that runs natively on the Pi.

---

## Features

| Phase | Capability |
|------:|------------|
| 1 | Clean package, YAML config, launch orchestration, logging, tests |
| 2 | MAVLink telemetry: heartbeat, GPS, attitude, IMU, battery, altitude, velocity, mode, armed, sys-status, RC, link quality — auto-connect + auto-reconnect |
| 3 | RPLIDAR C1: `LaserScan` + `PointCloud` + diagnostics, auto-reconnect |
| 4 | Sensor fusion (LiDAR + IMU + GPS + attitude + altitude), EKF-ready interface |
| 5 | Obstacle detection & classification (wall / tree / building / pole / person / vehicle) with distance + bearing |
| 6 | Navigation: waypoints, collision avoidance, emergency stop, failsafe, RTL, mission state machine |
| 7 | Simulation: mock Pixhawk / LiDAR / GPS / battery / IMU, one-switch sim↔real |
| 8 | Web dashboard (drone, laser scan, point cloud, obstacles, path, waypoints) |
| 9 | Diagnostics: CPU, RAM, temperature, node health, sensor health, link status |
| 10 | Unit, integration and mock tests (pytest) |

Every node runs in its own supervised thread and **restarts automatically** on
failure. Nothing is hardcoded — all ports and tunables come from `config/`.

---

## Architecture (ROS concepts → this project)

```
        +-------------------- MessageBus (in-process pub/sub) --------------------+
        |                                                                         |
  MavlinkNode  ->  /telemetry/*        FusionNode   ->  /state/fused              |
  LidarNode    ->  /scan /cloud        ObstacleNode ->  /obstacles                |
  NavigationNode <- /obstacles /state  ->  /mission/state  (+ MAVLink commands)   |
  DiagnosticsNode -> /diagnostics      WebDashboard <- (subscribes to everything) |
        |                                                                         |
        +-------------------------------------------------------------------------+

  ROS topic      -> MessageBus topic (string name)
  ROS message    -> @dataclass in drone_stack/msg
  ROS service    -> request/response in drone_stack/srv
  roslaunch      -> drone_stack/launch/bringup.py (YAML-driven)
  RViz           -> drone_stack/web dashboard (http://<pi>:8090)
  rosparam/YAML  -> config/*.yaml
```

See [docs/ARCHITECTURE.md](docs/ARCHITECTURE.md) for details.

---

## Quick start (simulation, no hardware)

```bash
git clone <this-repo> drone_stack && cd drone_stack
python3 -m venv .venv && source .venv/bin/activate
pip install -r requirements-dev.txt
pip install -e .

# run the whole stack in simulation
scripts/run_sim.sh
# or: python -m drone_stack.launch.bringup --config config/sim.yaml
```

Then open the dashboard at **http://localhost:8090** (or
`http://<pi-ip>:8090`). Run the tests with `pytest`.

## Running on real hardware

1. `pip install -r requirements-hardware.txt`
2. Plug in the Pixhawk and RPLIDAR.
3. Edit `config/real.yaml` → set `mavlink.connection` and `lidar.port`
   (see [docs/CONFIGURATION.md](docs/CONFIGURATION.md)).
4. `scripts/run_real.sh`

Full setup: [docs/INSTALL.md](docs/INSTALL.md).

---

## Project layout

```
drone_stack/
├── config/            # YAML parameters (default/sim/real)
├── docs/              # install / configuration / architecture guides
├── scripts/           # run_sim.sh, run_real.sh, dronectl CLI
├── tests/             # unit / integration / mock tests
└── drone_stack/       # the importable package
    ├── bus/           # MessageBus + topic name constants
    ├── msg/           # typed dataclass messages   (ROS msg/)
    ├── srv/           # request/response services  (ROS srv/)
    ├── utils/         # config, logging, geometry, NodeBase + Supervisor
    ├── interfaces/    # MAVLink + LiDAR hardware interfaces (real + abstract)
    ├── nodes/         # mavlink / lidar / fusion / obstacle / navigation / diagnostics
    ├── sim/           # mock Pixhawk / LiDAR + simulated world
    ├── launch/        # launch orchestrator + bringup entrypoint
    └── web/           # Flask dashboard (visualisation)
```

## License

MIT.
