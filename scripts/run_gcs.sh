#!/usr/bin/env bash
# Launch the Drone GCS web application (FastAPI + WebSocket).
# Simulation by default; edit config/real.yaml (or set MAVLINK_PORT/LIDAR_PORT) for hardware.
set -euo pipefail
cd "$(dirname "$0")/.."
exec python -m drone_stack.gcs.server --config "${1:-config/sim.yaml}" --port "${GCS_PORT:-8000}"
