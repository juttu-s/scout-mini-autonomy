#!/bin/bash
# Start the containerized Scout stack (master, safety, base, perception). Run ~/can_up.sh first.
#   ~/robot_up.sh                         robot stack (as robot.launch)
#   ~/robot_up.sh --nav <map> [x y yaw]   plus navigation on ~/maps/<map> (as navigation.launch)
#   TAG=<tag> ~/robot_up.sh ...           run an older image build (docker images | grep scout-)
#   PERCEPTION_TAG=<tag>                  perception image tag when it differs from TAG (CI releases: see deploy.sh)
#   SCOUT_DWA_DEBUG=true ~/robot_up.sh --nav ...   DWA publishes its scored trajectories (diagnosis)
#   ROBOT_UP_TIMEOUT=<s>                  how long to wait for all four healthy (default 240); exit 1 if they aren't
# Record:  ~/robot_exec.sh base rosrun scout_nav record_nav.sh <map>     (bags -> ~/bags)
# ~/robot_up.sh is a symlink to docker/robot_up.sh in ~/ros_utils.
set -u
C="docker compose -f $(dirname "$(readlink -f "$0")")/compose.yaml"
export SCOUT_NAV=false SCOUT_MAP="" SCOUT_INIT_X=0.0 SCOUT_INIT_Y=0.0 SCOUT_INIT_YAW=0.0 SCOUT_DWA_DEBUG=${SCOUT_DWA_DEBUG:-false}
if [ "${1:-}" = "--nav" ]; then
  SCOUT_NAV=true SCOUT_MAP=${2:?usage: robot_up.sh --nav <map> [x y yaw]}
  [ $# -ge 5 ] && SCOUT_INIT_X=$3 SCOUT_INIT_Y=$4 SCOUT_INIT_YAW=$5
  [ -f ~/maps/"$SCOUT_MAP"/grid.yaml ] || { echo "$HOME/maps/$SCOUT_MAP has no grid.yaml" >&2; exit 1; }
fi
docker info >/dev/null 2>&1 || { echo "No access to Docker: run ~/docker_setup.sh once (then log out and in)." >&2; exit 1; }
ip link show can0 2>/dev/null | grep -q "state UP" || { echo "can0 is not up: run ~/can_up.sh first." >&2; exit 1; }
if pgrep -x rosmaster >/dev/null && [ -z "$($C ps -q master 2>/dev/null)" ]; then
  echo "A native ROS master is running (roslaunch robot.launch / navigation.launch?). Stop it first." >&2; exit 1
fi
mkdir -p ~/bags ~/.ros/docker_logs/{master,safety,base,perception}
SCOUT_BOOT_ID=$(cat /proc/sys/kernel/random/boot_id)   # containers only run in the boot that started them
export SCOUT_BOOT_ID
$C up -d || exit 1
timeout=${ROBOT_UP_TIMEOUT:-240}
echo "waiting up to $timeout s for the containers to report healthy ..."
ok=0
for _ in $(seq 1 $((timeout / 2))); do
  st=$($C ps --format '{{.Service}} {{.Health}}' 2>/dev/null)
  bad=$(echo "$st" | grep -vc " healthy$")
  [ "$bad" -eq 0 ] && [ "$(echo "$st" | wc -l)" -eq 4 ] && { ok=1; break; }
  sleep 2
done
$C ps --format 'table {{.Service}}\t{{.Image}}\t{{.Status}}'
echo "logs: $C logs -f <service>    Foxglove: ws://$(hostname -I | awk '{print $1}'):8765"
[ "$ok" = 1 ] || { echo "not all four containers are healthy after $timeout s" >&2; exit 1; }
