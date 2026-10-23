#!/bin/bash
# Record a navigation run on a saved map (for replay / analysis): starts navigation unless it is already running.
#
#   rosrun scout_nav record_nav.sh <map> [init_x init_y init_yaw]
#
# Output: $BAG_ROOT/<map>/nav_runs/<date_time>/nav.bag (2 GB chunks), launch.log, run.txt
#   BAG_ROOT defaults to ~/maps; the containers set it to ~/bags (a bind mount, so bags never land in an image layer).
# In a container (SCOUT_CONTAINER=1) navigation must already be running (~/robot_up.sh --nav <map>): this only records.
# Records goals, plans, costmaps, commands, safety, localization, odometry, TF, lidar + IMU, detections/tracks,
# the annotated camera image (JPEG) and the camera depth cloud. No raw camera images (~5 MB/s instead of ~90).
# Stop with Ctrl+C (or SIGTERM). Navigation is stopped too if this script started it.
set -u
set -m   # job control: background jobs keep SIGINT
source "${SCOUT_SETUP:-$HOME/ros_utils/setup.bash}"
MAP=${1:?usage: record_nav.sh <map> [init_x init_y init_yaw]}
INIT=""
[ $# -ge 4 ] && INIT="init_x:=$2 init_y:=$3 init_yaw:=$4"
[ -f ~/maps/$MAP/grid.yaml ] || { echo "~/maps/$MAP has no grid.yaml" >&2; exit 1; }
OUT=${BAG_ROOT:-$HOME/maps}/$MAP/nav_runs/$(date +%Y%m%d_%H%M%S)
mkdir -p "$OUT"

LAUNCH_PID=""
if rostopic list 2>/dev/null | grep -qx /move_base/status; then
  echo "Navigation is already running: recording it."
elif [ "${SCOUT_CONTAINER:-0}" = 1 ]; then
  echo "Navigation is not running. Start it with  ~/robot_up.sh --nav $MAP  then run this again." >&2
  exit 1
elif rostopic list >/dev/null 2>&1; then
  # A robot stack without navigation (e.g. robot.launch from another terminal): starting navigation.launch would
  # launch a second copy of every node; ROS kills the duplicates and the required scout_base_node takes both down.
  echo "ROS is running without navigation (robot.launch from another terminal?). Stop it first (Ctrl+C there)," >&2
  echo "then run this again: it starts the robot stack and navigation together." >&2
  exit 1
else
  if ! ip link show can0 2>/dev/null | grep -q "state UP"; then
    echo "can0 is not up: run ~/can_up.sh first." >&2; exit 1
  fi
  echo "Starting navigation on $MAP ${INIT:-(at the map origin)} (log: $OUT/launch.log) ..."
  roslaunch scout_nav navigation.launch map:=$MAP $INIT > "$OUT/launch.log" 2>&1 < /dev/null &
  LAUNCH_PID=$!
  for i in $(seq 1 120); do rostopic list 2>/dev/null | grep -qx /move_base/status && break; kill -0 $LAUNCH_PID 2>/dev/null || break; sleep 1; done
  if ! kill -0 $LAUNCH_PID 2>/dev/null; then
    echo "Navigation did not start. Last lines of $OUT/launch.log:" >&2; grep -vE "^\s*$|^ \* /" "$OUT/launch.log" | tail -15 >&2; exit 1
  fi
  sleep 10  # let localization converge
  if [ -z "$INIT" ]; then
    echo "No start pose given: searching the whole map for the robot (/relocalize) ..."
    timeout 120 rosservice call /relocalize "{}" >/dev/null 2>&1 && echo "  relocalized" || echo "  /relocalize failed" >&2
    sleep 5
  fi
fi
echo "Localization: $(timeout 5 rostopic echo -n 1 /localization/status 2>/dev/null | grep -E 'inlier_fraction' | xargs)  (should be > 0.95; else set a pose estimate on /initialpose)"

TOPICS=(
  /move_base_simple/goal /move_base/goal /move_base/current_goal /move_base/status /move_base/result /move_base/feedback
  /move_base/GlobalPlanner/plan /move_base/DWAPlannerROS/local_plan /move_base/DWAPlannerROS/global_plan
  /move_base/global_costmap/costmap /move_base/global_costmap/costmap_updates
  /move_base/local_costmap/costmap /move_base/local_costmap/costmap_updates /move_base/local_costmap/footprint
  /map /initialpose
  /nav/cmd_vel /teleop/cmd_vel /cmd_vel /scout_safety/state /scout_safety/markers /scout_status
  /localization/status /localization/odom /dlio/odom_node/odom /odom /tf /tf_static
  /livox/lidar /livox/imu /rs_detector/depth_points
  /rs_detector/detections /rs_detector/detections_2d /rs_detector/tracks /rs_detector/image_annotated/compressed
  /camera/camera/color/camera_info /rosout_agg
)
{ echo "map: $MAP"; echo "date: $(date -Iseconds)"; echo "init: ${INIT:-map origin / already running}"; echo "topics:"; printf '  %s\n' "${TOPICS[@]}"; } > "$OUT/run.txt"
echo "Recording to $OUT  (Ctrl+C to stop)"
rosbag record -O "$OUT/nav" --split --size=2048 -b 1024 "${TOPICS[@]}" &
BAG_PID=$!

stop() {
  trap '' INT TERM
  echo; echo "Stopping the recording ..."
  kill -INT $BAG_PID 2>/dev/null
  for sig in INT INT INT INT INT INT TERM TERM KILL; do
    kill -0 $BAG_PID 2>/dev/null || break
    sleep 2; kill -0 $BAG_PID 2>/dev/null && [ $sig != INT ] && kill -$sig -- -$BAG_PID 2>/dev/null
  done
  wait $BAG_PID 2>/dev/null
  { echo; echo "rosbag info:"; rosbag info "$OUT"/*.bag 2>&1; } >> "$OUT/run.txt"
  if [ -n "$LAUNCH_PID" ]; then echo "Stopping navigation ..."; kill -INT $LAUNCH_PID 2>/dev/null; wait $LAUNCH_PID 2>/dev/null; fi
  du -sh "$OUT"; ls "$OUT"; exit 0
}
trap stop INT TERM
while kill -0 $BAG_PID 2>/dev/null; do sleep 10; printf "\r  recorded: %s   " "$(du -sh "$OUT" 2>/dev/null | cut -f1)"; done
stop
