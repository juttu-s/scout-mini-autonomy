#!/bin/bash
# Run scenario_test.py against a fresh roscore + safety_node (config/safety.yaml, or $SAFETY_CONFIG) and record a bag.
#   test/run_scenario.sh <scenario id>|--all <out dir>
# One scenario: <out dir>/{result.json, stdout.log, safety_node.log, <id>.bag, ros/log}; --all: one such dir per id.
# Needs its own master: refuses if one already answers at $ROS_MASTER_URI (the robot's node publishes /cmd_vel too).
# In CI and on the robot it runs inside the safety image, which has its own network namespace:
#   docker run --rm --entrypoint bash -v $PWD/detection_ws/src/scout_safety/test:/test -v /tmp/out:/out \
#       scout-safety:latest /test/run_scenario.sh --all /out
set -o pipefail
[ $# -eq 2 ] || { echo "usage: $0 <scenario id>|--all <out dir>" >&2; exit 2; }
here=$(cd "$(dirname "$0")" && pwd)
# shellcheck disable=SC1090
source "${SCOUT_SETUP:-/opt/ros/noetic/setup.bash}"
set -u   # after sourcing: the ROS setup scripts read unset variables
export ROS_MASTER_URI=${ROS_MASTER_URI:-http://localhost:11311}
if timeout 3 rosparam list >/dev/null 2>&1; then
  echo "a ROS master already runs at $ROS_MASTER_URI; run this in its own container or network namespace" >&2
  exit 2
fi
config=${SAFETY_CONFIG:-$(rospack find scout_safety)/config/safety.yaml}

run_one() {  # <id> <dir>
  local id=$1 d=$2 rc p alive pids=()
  mkdir -p "$d" && touch "$d/.writable" || { echo "cannot write to $d" >&2; return 2; }
  rm -f "$d/.writable"
  export ROS_HOME=$d/ros
  roscore >"$d/roscore.log" 2>&1 & pids+=($!)
  for _ in $(seq 50); do rosparam list >/dev/null 2>&1 && break; sleep 0.2; done
  rosparam load "$config" /scout_safety
  rosrun tf2_ros static_transform_publisher 0.3 0 0.2 -1.5708 0 -1.5708 base_link camera_color_optical_frame \
    __name:=camera_tf >/dev/null 2>&1 & pids+=($!)
  rosrun scout_safety safety_node.py __name:=scout_safety >"$d/safety_node.log" 2>&1 & pids+=($!)
  rosbag record -q -O "$d/$id.bag" -e '/(cmd_vel|teleop/cmd_vel|nav/cmd_vel|scout_safety/state|rs_detector/.*)' \
    __name:=scenario_bag >/dev/null 2>&1 & pids+=($!)
  python3 "$here/scenario_test.py" --scenario "$id" --json "$d/result.json" 2>&1 | tee "$d/stdout.log"
  rc=${PIPESTATUS[0]}
  rosnode kill /scenario_bag >/dev/null 2>&1   # closes the bag cleanly
  sleep 1
  kill -INT "${pids[@]}" 2>/dev/null
  for _ in $(seq 50); do
    alive=0; for p in "${pids[@]}"; do kill -0 "$p" 2>/dev/null && alive=1; done
    [ $alive = 0 ] && break; sleep 0.1
  done
  kill -9 "${pids[@]}" 2>/dev/null   # whatever ignored SIGINT for 5 s
  wait "${pids[@]}" 2>/dev/null
  return "$rc"
}

if [ "$1" = --all ]; then
  failed=()
  for id in $(python3 "$here/scenario_test.py" --list | tr -d '[]",'); do
    run_one "$id" "$2/$id" || failed+=("$id")
  done
  echo "failed: ${failed[*]:-none}"
  [ ${#failed[@]} -eq 0 ]
else
  run_one "$1" "$2"
fi
