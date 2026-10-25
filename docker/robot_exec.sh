#!/bin/bash
# Run a command inside one of the Scout containers with its ROS environment loaded
# (plain `docker compose exec` skips the entrypoint, so rosrun/roslaunch aren't on the PATH).
#   ~/robot_exec.sh base rosrun scout_nav record_nav.sh lab2
#   ~/robot_exec.sh safety rosrun scout_safety teleop.sh
#   ~/robot_exec.sh base rosrun scout_nav goal_monitor.py
svc=${1:?usage: robot_exec.sh <master|safety|base|perception> <command ...>}; shift
tty=-T; [ -t 0 ] && tty=
exec docker compose -f "$(dirname "$(readlink -f "$0")")/compose.yaml" exec $tty "$svc" bash -c 'source /opt/scout/setup.bash && exec "$@"' -- "$@"
