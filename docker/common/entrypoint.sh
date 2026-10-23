#!/bin/bash
# Source the image's ROS environment, then run the command (tini is PID 1 via compose `init: true`).
#
# With HEARTBEAT_FILE set, also a watchdog: Docker only *reports* an unhealthy container, it never restarts it,
# and roslaunch keeps running when a non-required node dies (a killed detector left the camera running and the
# container "unhealthy" for good, 2026-10-23). After HEARTBEAT_GRACE s (start-up), a heartbeat older than
# HEARTBEAT_MAX_AGE s stops roslaunch, the container exits non-zero and `restart: unless-stopped` starts it fresh.
# Covers hangs as well as crashes. Stopping the robot is not the watchdog's job (safety FAULTs within 0.3 s).
set -e
source /opt/scout/setup.bash
[ -z "${HEARTBEAT_FILE:-}" ] && exec "$@"

"$@" &
child=$!
trap 'kill -INT "$child" 2>/dev/null; wait "$child"; exit $?' INT TERM
start=$(date +%s)
while kill -0 "$child" 2>/dev/null; do
  sleep 2 & wait $!
  [ $(( $(date +%s) - start )) -lt "${HEARTBEAT_GRACE:-120}" ] && continue
  if ! healthcheck.sh "$HEARTBEAT_FILE" "${HEARTBEAT_MAX_AGE:-10}"; then
    echo "watchdog: $HEARTBEAT_FILE older than ${HEARTBEAT_MAX_AGE:-10} s, stopping roslaunch so the container restarts" >&2
    kill -INT "$child" 2>/dev/null
    for i in $(seq 1 20); do kill -0 "$child" 2>/dev/null || break; sleep 1; done
    kill -KILL "$child" 2>/dev/null || true
    exit 1
  fi
done
set +e
wait "$child"
exit $?
