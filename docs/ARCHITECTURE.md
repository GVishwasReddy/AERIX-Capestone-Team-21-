# Architecture

## Overview

`drone_stack` is a single Python process that hosts many **nodes**. Nodes never
call each other directly; they communicate only through a **MessageBus** by
publishing and subscribing to **named topics** — exactly the ROS model, but
in-process and dependency-free.

```
                         MessageBus  (thread-safe, latched, pub/sub)
   publishers ─────────────► topic strings ─────────────► subscribers
```

## Nodes

| Node | Subscribes | Publishes |
|------|------------|-----------|
| `MavlinkNode` | `/cmd/mavlink` | `/telemetry/*` (heartbeat, gps, attitude, imu, battery, altitude, velocity, mode, armed, sys_status, rc, link) |
| `LidarNode` | — | `/scan`, `/cloud`, `/diagnostics/lidar` |
| `FusionNode` | `/telemetry/attitude`, `/telemetry/imu`, `/telemetry/gps`, `/telemetry/altitude`, `/telemetry/velocity`, `/scan` | `/state/fused` |
| `ObstacleNode` | `/scan`, `/state/fused` | `/obstacles` |
| `NavigationNode` | `/obstacles`, `/state/fused`, `/telemetry/*`, `/mission/cmd` | `/mission/state`, `/cmd/mavlink` |
| `DiagnosticsNode` | `/diagnostics/*`, node health | `/diagnostics` |
| `WebDashboard` | everything (read-only) | — (serves HTTP/SSE) |

## Message flow

```
Pixhawk ─(MAVLink)─► MavlinkNode ─► /telemetry/* ─┬─► FusionNode ─► /state/fused ─┐
RPLIDAR ─(serial)──► LidarNode  ─► /scan ─────────┴─► ObstacleNode ─► /obstacles ─┤
                                                                                  ▼
                                              NavigationNode (mission state machine)
                                                        │  /cmd/mavlink
                                                        ▼
                                              MavlinkNode ─(MAVLink)─► Pixhawk
```

## Threading & recovery model

- Each node subclasses `NodeBase` (a `threading.Thread`).
- `NodeBase.run()` wraps `setup() → loop() → teardown()` in a resilient loop:
  any exception is logged, the node backs off, and it restarts itself
  automatically. A crash in one node never takes down the others.
- The `Supervisor` starts/stops all nodes, monitors liveness, and restarts any
  thread that dies outright.
- Bus dispatch isolates subscriber exceptions so a bad subscriber cannot break a
  publisher.

## Hardware abstraction (sim ↔ real)

Both hardware families are hidden behind an interface with two implementations:

```
interfaces/mavlink_interface.py : MavlinkInterface (abstract)
    ├── RealMavlink   (pymavlink, real Pixhawk)          [interfaces/]
    └── MockMavlink   (simulated vehicle physics)         [sim/mock_pixhawk.py]

interfaces/lidar_interface.py   : LidarInterface (abstract)
    ├── RealLidar     (rplidar driver, real RPLIDAR C1)   [interfaces/]
    └── MockLidar     (ray-casts the simulated world)     [sim/mock_lidar.py]
```

`mode: sim|real` in the config selects the implementation. Nodes are identical in
both cases, so simulation exercises the same publish/subscribe, fusion,
obstacle, navigation and safety code paths as real flight.

## EKF-readiness (Phase 4)

`FusionNode` uses a `StateEstimator` interface. The shipped
`ComplementaryEstimator` fuses attitude/IMU/GPS/altitude into a `FusedState`
with populated covariance fields. Swapping in an EKF later means implementing the
same interface — no changes to publishers or subscribers.

## Deployment

Run under systemd on the Pi (example unit):

```ini
[Unit]
Description=drone_stack
After=network.target

[Service]
User=pi
WorkingDirectory=/home/pi/drone_stack
ExecStart=/home/pi/drone_stack/.venv/bin/python -m drone_stack.launch.bringup --config config/real.yaml
Restart=on-failure
RestartSec=3

[Install]
WantedBy=multi-user.target
```
