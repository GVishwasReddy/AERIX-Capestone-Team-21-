#!/usr/bin/env bash
# Run the full stack against REAL HARDWARE.
# Edit config/real.yaml first to set mavlink.connection and lidar.port.
set -euo pipefail
cd "$(dirname "$0")/.."
exec python -m drone_stack.launch.bringup --config config/real.yaml "$@"
