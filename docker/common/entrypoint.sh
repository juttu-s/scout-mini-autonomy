#!/bin/bash
# Source the image's ROS environment, then run the command (tini is PID 1 via compose `init: true`).
#
# Exit code = what `restart: on-failure` should do:
#   0  a deliberate stop (SIGINT/SIGTERM from docker stop, robot_down.sh, shutdown): not restarted, and Docker
#      doesn't start the container again at boot (unless-stopped did, and looped without a master, 2026-10-23)
#   1  roslaunch exited on its own (a required node died; roslaunch itself exits 0 then) or the watchdog fired:
#      restarted
# Watchdog (HEARTBEAT_FILE set): Docker only *reports* an unhealthy container, it never restarts it, and roslaunch
# keeps running when a non-required node dies (a killed detector left the camera running and the container
# unhealthy for good, 2026-10-23). After HEARTBEAT_GRACE s, a heartbeat older than HEARTBEAT_MAX_AGE s stops
# roslaunch and exits 1. Covers hangs as well as crashes. Stopping the robot is not its job (safety FAULTs in 0.3 s).
set -e
source /opt/scout/setup.bash
# Never start by itself after a reboot: ~/robot_up.sh stamps the boot it ran in. A container that Docker restarts at
# boot (e.g. after a power cut, when its last exit was non-zero) sees a different boot id and stays down.
if [ -n "${SCOUT_BOOT_ID:-}" ] && [ "$SCOUT_BOOT_ID" != "$(cat /proc/sys/kernel/random/boot_id)" ]; then
  echo "entrypoint: started by Docker after a reboot, not by ~/robot_up.sh: not starting" >&2
  exit 0
fi

"$@" &
child=$!
stopping=0
trap 'stopping=1; kill -INT "$child" 2>/dev/null' INT TERM
start=$(date +%s)
while kill -0 "$child" 2>/dev/null; do
  sleep 2 & wait $! || true
  [ "$stopping" = 1 ] && break
  [ -z "${HEARTBEAT_FILE:-}" ] && continue
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
wait "$child"; code=$?
if [ "$stopping" = 1 ]; then
  exit 0
fi
echo "entrypoint: '$1' exited on its own (code $code): exiting 1 so the container is restarted" >&2
exit 1
