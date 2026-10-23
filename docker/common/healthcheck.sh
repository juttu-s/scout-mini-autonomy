#!/bin/bash
# Container health: a heartbeat file touched by the running node must be younger than $2 s (default 3).
# No ROS client is started, so the check costs nothing next to the robot's own load.
f=${1:?heartbeat file}; max=${2:-3}
[ -f "$f" ] && [ $(( $(date +%s) - $(stat -c %Y "$f") )) -lt "$max" ]
