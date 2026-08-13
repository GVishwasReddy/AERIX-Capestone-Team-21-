#!/usr/bin/env bash
# Launch the AERIX Ground Control server detached. Usage: start_gcs.sh [sim|real]
cd /home/pi/drone_stack || exit 1
MODE="${1:-sim}"
export GCS_PORT="${GCS_PORT:-8090}"

# stop any previous instance and wait for the port + camera to be released
pkill -9 -f gcs.server 2>/dev/null
for i in $(seq 1 20); do
  ss -ltn | grep -q ":${GCS_PORT}\b" || break
  sleep 0.5
done
sleep 1

setsid .venv/bin/python -m drone_stack.gcs.server --config "config/${MODE}.yaml" \
  </dev/null >logs/gcs.out 2>&1 &
disown
echo "AERIX GCS starting in ${MODE} mode on port ${GCS_PORT} (pid $!)"
exit 0
