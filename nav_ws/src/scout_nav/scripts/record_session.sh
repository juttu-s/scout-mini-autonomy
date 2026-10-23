#!/bin/bash
# Record a mapping session: DLIO 3D map + navigation grid, and a rosbag with camera, both IMUs, lidar, odometry and TF
# (enough to run RTAB-Map, or anything else, offline later).
#
#   rosrun scout_nav record_session.sh <name> [--no-raw-depth] [--camera-imu]
#
# --camera-imu also records the D455 IMU. OFF by default: with the ROS librealsense 2.50 (RSUSB backend) and the
# Jetson's kernel HID sensor drivers loaded, enabling the D455 IMU stalls the whole camera (0 Hz, "hwmon ... HW not
# ready"; tested 2026-10-18). The MID360 IMU (/livox/imu) is always recorded.
#
# Drive with teleop in another terminal (rosrun scout_safety teleop.sh). Press Ctrl+C here when done: the bag is
# closed, then the DLIO map and grid are saved (BEFORE the stack stops, or DLIO's map is lost), then the stack stops
# (only if this script started it).
#
# Output: ~/maps/<name>/
#   dlio_map.pcd, grid.pgm/.yaml, costmap_heights.yaml, grid_info.yaml   (scout_nav navigation map)
#   recording/<name>_N.bag (4 GB chunks), launch.log, session.txt (settings, topic list, rosbag info)
#   With BAG_ROOT set (the containers set ~/bags), the bags go to $BAG_ROOT/<name>/recording and
#   ~/maps/<name>/recording is a symlink to it, so bag_to_dense_map.py etc. still find them.
# In a container (SCOUT_CONTAINER=1) the robot stack must already be running (~/robot_up.sh): this only records.
#
# Data rate is ~90 MB/s (~5.5 GB per minute; ~3.5 GB/min with --no-raw-depth).
set -u
set -m   # job control: background jobs keep SIGINT (a script's '&' jobs otherwise ignore it, and rosbag could not be stopped)
source "${SCOUT_SETUP:-$HOME/ros_utils/setup.bash}"

NAME=${1:?usage: record_session.sh <name> [--no-raw-depth] [--camera-imu]}
RAW_DEPTH=1
CAMERA_IMU=false
for arg in "${@:2}"; do
  case $arg in
    --no-raw-depth) RAW_DEPTH=0 ;;
    --camera-imu) CAMERA_IMU=true ;;
    *) echo "unknown option $arg" >&2; exit 1 ;;
  esac
done
LIDAR_HEIGHT=${LIDAR_HEIGHT:-0.76}   # MID360 centre above the floor (measured 2026-10-17)
DIR=~/maps/$NAME
REC=${BAG_ROOT:-$HOME/maps}/$NAME/recording

if [ -e "$DIR" ]; then
  echo "~/maps/$NAME already exists; pick another name (nothing is overwritten)." >&2
  exit 1
fi

# CAN must be up: scout_base_node is required in the launch and aborts without it, taking everything down.
# (Bringing it up needs sudo, so the script can only check.)
if ! rostopic list >/dev/null 2>&1 && ! ip link show can0 2>/dev/null | grep -q "state UP"; then
  echo "can0 is not up. Run these first, then start this script again:" >&2
  echo "  sudo modprobe gs_usb && rosrun scout_bringup bringup_can2usb.bash" >&2
  exit 1
fi
mkdir -p "$REC"
if [ "$REC" != "$DIR/recording" ]; then mkdir -p "$DIR" && ln -s "$REC" "$DIR/recording"; fi

# Start the robot stack (Scout, MID360, DLIO, D455 + its IMU, detector, safety) unless it is already running
LAUNCH_PID=""
if rostopic list >/dev/null 2>&1; then
  echo "ROS is already running: using the existing stack."
  if [ $CAMERA_IMU = true ] && ! rostopic list | grep -qx /camera/camera/imu; then
    echo "WARNING: /camera/camera/imu is not published, so the camera IMU will NOT be recorded."
    echo "         For it, stop the stack and let this script start it (camera_imu:=true)."
    read -r -p "Continue without the camera IMU? [y/N] " a
    [ "$a" = "y" ] || exit 1
  fi
elif [ "${SCOUT_CONTAINER:-0}" = 1 ]; then
  echo "The robot stack is not running. Start it with  ~/robot_up.sh  then run this again." >&2
  exit 1
else
  echo "Starting the robot stack (log: $REC/launch.log) ..."
  # no setsid: with job control (set -m) the job already has its own process group, so the terminal's Ctrl+C
  # doesn't reach it; and setsid would fork, leaving $! pointing at a process that already exited
  roslaunch scout_safety robot.launch foxglove:=true camera_imu:=$CAMERA_IMU > "$REC/launch.log" 2>&1 < /dev/null &
  LAUNCH_PID=$!
fi

wait_topic() {  # wait until a topic delivers a message (up to $2 s)
  timeout "$2" rostopic echo -n 1 "$1" >/dev/null 2>&1
}
launch_alive() { [ -z "$LAUNCH_PID" ] || kill -0 "$LAUNCH_PID" 2>/dev/null; }
echo "Waiting for the ROS master ..."
for i in $(seq 1 60); do rostopic list >/dev/null 2>&1 && break; launch_alive || break; sleep 1; done
if ! rostopic list >/dev/null 2>&1 || ! launch_alive; then
  echo "The robot stack did not start. Last lines of $REC/launch.log:" >&2
  grep -vE "^\s*$|^ \* /" "$REC/launch.log" | tail -15 >&2
  exit 1
fi
sleep 5  # let the nodes advertise
echo "Waiting for sensors ..."
SENSORS=(/livox/lidar /livox/imu /camera/camera/color/image_raw /camera/camera/depth/image_rect_raw /dlio/odom_node/odom /odom)
[ $CAMERA_IMU = true ] && SENSORS+=(/camera/camera/imu)
for t in "${SENSORS[@]}"; do
  if wait_topic "$t" 60; then echo "  ok      $t"; else echo "  MISSING $t (recording anyway)"; fi
  if ! launch_alive; then
    echo "The robot stack stopped (a required node died). Last lines of $REC/launch.log:" >&2
    grep -vE "^\s*$|^ \* /" "$REC/launch.log" | tail -15 >&2
    exit 1
  fi
done
# aligned depth is only published by the detector while someone subscribes; check it with a subscriber
if wait_topic /camera/camera/aligned_depth_to_color/image_raw 30; then
  echo "  ok      /camera/camera/aligned_depth_to_color/image_raw"
else
  echo "  MISSING /camera/camera/aligned_depth_to_color/image_raw (detector running?)"
fi

TOPICS=(
  # camera (D455, 848x480 @ 30): color, depth registered to color (by the detector, GPU), intrinsics, IMU
  /camera/camera/color/image_raw /camera/camera/color/camera_info
  /camera/camera/aligned_depth_to_color/image_raw /camera/camera/aligned_depth_to_color/camera_info
  /camera/camera/depth/camera_info /camera/camera/extrinsics/depth_to_color
  # lidar (Livox MID360) and its IMU (acceleration in g, not m/s^2)
  /livox/lidar /livox/imu
  # odometry
  /dlio/odom_node/odom /odom /scout_status
  # transforms (tf_static: camera frames, base_link->camera_link, ...)
  /tf /tf_static
  # what the robot did and saw (small)
  /cmd_vel /teleop/cmd_vel /nav/cmd_vel /scout_safety/state /rs_detector/tracks /rs_detector/detections
)
[ $RAW_DEPTH = 1 ] && TOPICS+=(/camera/camera/depth/image_rect_raw)
[ $CAMERA_IMU = true ] && TOPICS+=(/camera/camera/imu)   # D455 IMU: gyro 200 Hz + accel 100 Hz, interpolated

{
  echo "session: $NAME"
  echo "date: $(date -Iseconds)"
  echo "lidar_height_m: $LIDAR_HEIGHT   # MID360 centre above the floor"
  echo "robot_launch: scout_safety robot.launch foxglove:=true camera_imu:=$CAMERA_IMU (defaults incl. camera mount)"
  echo "camera_mount (base_link = MID360 -> camera_link):"
  grep -E 'name="cam_(x|y|z|yaw|pitch|roll)"' "$(rospack find scout_safety)/launch/robot.launch" | sed 's/^/  /'
  echo "raw_depth: $RAW_DEPTH"
  echo "topics:"; printf '  %s\n' "${TOPICS[@]}"
} > "$REC/session.txt"

echo
echo "Recording to $REC  (Ctrl+C to stop, then the map is saved)"
rosbag record -O "$REC/$NAME" --split --size=4096 -b 4096 --tcpnodelay "${TOPICS[@]}" &
BAG_PID=$!

stop() {
  trap '' INT TERM
  echo; echo "Stopping the recording ..."
  # rosbag closes the bag on SIGINT; escalate if it doesn't exit (e.g. stuck retrying a dead master)
  kill -INT $BAG_PID 2>/dev/null
  for sig in INT INT INT INT INT INT TERM TERM KILL; do
    kill -0 $BAG_PID 2>/dev/null || break
    sleep 2; kill -0 $BAG_PID 2>/dev/null && [ $sig != INT ] && kill -$sig -- -$BAG_PID 2>/dev/null
  done
  wait $BAG_PID 2>/dev/null
  for f in "$REC"/*.bag.active; do  # a bag that wasn't closed cleanly
    [ -e "$f" ] && echo "WARNING: $f was not closed; repair with: rosbag reindex $f" >&2
  done
  echo "Saving the DLIO map and navigation grid ..."
  LIDAR_HEIGHT=$LIDAR_HEIGHT rosrun scout_nav save_map.sh "$NAME" || echo "MAP NOT SAVED: is DLIO running?" >&2
  { echo; echo "rosbag info:"; rosbag info "$REC"/*.bag 2>&1; } >> "$REC/session.txt"
  if [ -n "$LAUNCH_PID" ]; then
    echo "Stopping the robot stack ..."
    kill -INT $LAUNCH_PID 2>/dev/null; wait $LAUNCH_PID 2>/dev/null
  fi
  echo; du -sh "$DIR"; ls "$DIR" "$REC"
  exit 0
}
trap stop INT TERM

# progress line every 10 s
while kill -0 $BAG_PID 2>/dev/null; do
  sleep 10
  printf "\r  recorded: %s   " "$(du -sh "$REC" 2>/dev/null | cut -f1)"
done
stop
