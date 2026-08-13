#!/usr/bin/env bash
# Run the full stack in SIMULATION (no hardware required).
set -euo pipefail
cd "$(dirname "$0")/.."
exec python -m drone_stack.launch.bringup --config config/sim.yaml "$@"
