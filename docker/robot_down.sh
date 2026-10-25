#!/bin/bash
# Stop the containerized Scout stack. A running recording (record_nav.sh / record_session.sh in the base
# container) is stopped first and its bag closed, so nothing is truncated; then the containers stop in reverse
# start order (perception, base, safety, master last).
set -u
# ~/robot_down.sh is a symlink to docker/robot_down.sh in ~/ros_utils.
C="docker compose -f $(dirname "$(readlink -f "$0")")/compose.yaml"
if [ -n "$($C ps -q base 2>/dev/null)" ]; then
  if $C exec -T base pgrep -f '[r]ecord_(nav|session)\.sh' >/dev/null 2>&1; then
    echo "stopping the recording ..."
    $C exec -T base pkill -INT -f '[r]ecord_(nav|session)\.sh'
    for _ in $(seq 1 120); do
      $C exec -T base pgrep -f '[r]ecord_(nav|session)\.sh' >/dev/null 2>&1 || break
      sleep 1
    done
    left=$(find ~/bags -name '*.bag.active' -newermt '-1 day' 2>/dev/null)
    [ -n "$left" ] && echo "WARNING: bag(s) not closed: $left  (rosbag reindex, then rename)" >&2
  fi
fi
$C down
