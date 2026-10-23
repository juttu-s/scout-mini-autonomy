# syntax=docker/dockerfile:1
# scout-base: Scout base (CAN), MID360 driver, DLIO, Foxglove, and (nav:=true) localization + move_base.
# CPU only. Build context: ~/ros_utils  (docker compose -f docker/compose.yaml build base)
FROM ros:noetic-ros-base-focal

RUN apt-get update && apt-get install -y --no-install-recommends \
      git python3-yaml python3-numpy cmake build-essential libpcl-dev libeigen3-dev libomp-dev \
      ros-noetic-foxglove-bridge ros-noetic-move-base ros-noetic-global-planner ros-noetic-dwa-local-planner \
      ros-noetic-map-server ros-noetic-pcl-ros ros-noetic-nodelet ros-noetic-tf2-ros ros-noetic-rosbag \
      can-utils iproute2 \
 && rm -rf /var/lib/apt/lists/*

COPY docker/third_party.repos /opt/scout/docker/
COPY docker/patches /opt/scout/docker/patches
COPY docker/common/fetch_third_party.py /usr/local/bin/

# Livox SDK2 (shared library used by livox_ros_driver2)
RUN fetch_third_party.py /opt/scout/docker/third_party.repos /opt/src Livox-SDK2 \
 && cmake -S /opt/src/Livox-SDK2 -B /tmp/livox-build -DCMAKE_BUILD_TYPE=Release \
 && make -C /tmp/livox-build -j"$(nproc)" install && ldconfig && rm -rf /tmp/livox-build

WORKDIR /opt/scout/ws
RUN fetch_third_party.py /opt/scout/docker/third_party.repos src \
      scout_ros ugv_sdk livox_ros_driver2 direct_lidar_inertial_odometry \
      ndt_omp fast_gicp hdl_localization hdl_global_localization \
 && cp src/livox_ros_driver2/package_ROS1.xml src/livox_ros_driver2/package.xml
RUN apt-get update && rosdep update --rosdistro noetic \
 && rosdep install --from-paths src --ignore-src -r -y --rosdistro noetic \
 && rm -rf /var/lib/apt/lists/*
# third-party first, own code last: editing scout_nav only rebuilds the last layers
RUN . /opt/ros/noetic/setup.sh && catkin_make -DCMAKE_BUILD_TYPE=Release -DROS_EDITION=ROS1 -j"$(nproc)"
COPY nav_ws/src/scout_nav src/scout_nav
RUN . /opt/ros/noetic/setup.sh && catkin_make -DCMAKE_BUILD_TYPE=Release -DROS_EDITION=ROS1 -j"$(nproc)" && rm -rf build

COPY docker/common/entrypoint.sh docker/common/healthcheck.sh /usr/local/bin/
COPY docker/launch/base.launch /opt/scout/launch/
RUN printf 'source /opt/ros/noetic/setup.bash\nsource /opt/scout/ws/devel/setup.bash\n' > /opt/scout/setup.bash \
 && useradd -u 1000 -m -d /home/agile agile
ENV SCOUT_SETUP=/opt/scout/setup.bash SCOUT_CONTAINER=1 BAG_ROOT=/home/agile/bags
USER agile
ENTRYPOINT ["entrypoint.sh"]
CMD ["roslaunch", "--wait", "/opt/scout/launch/base.launch"]
