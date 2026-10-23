#!/bin/bash

# 1. Source base ROS
source /opt/ros/noetic/setup.bash

# 2. Source workspaces using the --extend flag
# This prevents them from overwriting the previous ones
source ~/ros_utils/scout_ws/devel/setup.bash --extend
source ~/ros_utils/ws_livox/devel/setup.bash --extend
source ~/ros_utils/realsense_ws/devel/setup.bash --extend
source ~/ros_utils/dlio_ws/devel/setup.bash --extend
source ~/ros_utils/detection_ws/devel/setup.bash --extend
source ~/ros_utils/nav_ws/devel/setup.bash --extend

# 3. Source your final workspace (where dm3_pkg is)
# Replace 'catkin_ws' with the actual workspace name if different
source ~/ros_utils/dm3_ws/devel/setup.bash --extend