#!/bin/bash
# Save DLIO's 3D map and build the 2D navigation grid from it.
#   rosrun scout_nav save_map.sh <name> [voxel_m]      -> ~/maps/<name>/{dlio_map.pcd, grid.pgm/.yaml, ...}
# Run while the robot stack (DLIO) is still up: its map is lost when the launch stops.
source ~/ros_utils/setup.bash
NAME=${1:?give a map name, e.g. lab}
LEAF=${2:-0.05}
DIR=~/maps/$NAME
mkdir -p "$DIR"
rosservice call /dlio_map/save_pcd "leaf_size: $LEAF
save_path: '$DIR'" || exit 1
# MID360 centre is 0.76 m above the floor (measured 2026-10-17), so the robot incl. lidar is ~0.85 m tall;
# override with LIDAR_HEIGHT=... ROBOT_HEIGHT=...
rosrun scout_nav pcd_to_grid.py "$DIR" --lidar-height ${LIDAR_HEIGHT:-0.76} --robot-height ${ROBOT_HEIGHT:-0.85}
ls -la "$DIR"
