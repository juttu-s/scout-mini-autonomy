#!/bin/bash
# Source the image's ROS environment, then run the command (tini is PID 1 via compose `init: true`).
set -e
source /opt/scout/setup.bash
exec "$@"
