# syntax=docker/dockerfile:1
# scout-safety: the safety monitor (and the ROS master). System Python only, no GPU, no torch.
# Build context: ~/ros_utils  (docker compose -f docker/compose.yaml build safety)

FROM ros:noetic-ros-base-focal AS build
RUN apt-get update && apt-get install -y --no-install-recommends git python3-yaml && rm -rf /var/lib/apt/lists/*
WORKDIR /opt/scout/ws
COPY docker/common/fetch_third_party.py /usr/local/bin/
COPY docker/pins/vision_msgs.* docker/pins/scout_ros.* /opt/scout/pins/
RUN fetch_third_party.py /opt/scout/pins src vision_msgs scout_ros \
 && find src/scout_ros -mindepth 1 -maxdepth 1 -type d ! -name scout_msgs -exec rm -rf {} +   # only the messages
COPY detection_ws/src/rs_object_detection/package.xml detection_ws/src/rs_object_detection/CMakeLists.txt src/rs_object_detection/
COPY detection_ws/src/rs_object_detection/msg src/rs_object_detection/msg
COPY detection_ws/src/scout_safety src/scout_safety
# rs_object_detection is built for its messages only (the detector itself lives in scout-perception)
RUN mkdir -p src/rs_object_detection/scripts src/rs_object_detection/launch src/rs_object_detection/config src/rs_object_detection/rviz \
 && touch src/rs_object_detection/scripts/detector_node.py \
 && . /opt/ros/noetic/setup.sh && catkin_make -DCMAKE_BUILD_TYPE=Release && rm -rf build

FROM ros:noetic-ros-core-focal
RUN apt-get update && apt-get install -y --no-install-recommends \
      python3-numpy ros-noetic-tf ros-noetic-tf2-ros ros-noetic-nav-msgs ros-noetic-visualization-msgs \
      ros-noetic-teleop-twist-keyboard \
 && rm -rf /var/lib/apt/lists/*
COPY --from=build /opt/scout/ws /opt/scout/ws
COPY docker/common/entrypoint.sh docker/common/healthcheck.sh /usr/local/bin/
COPY docker/launch/safety.launch /opt/scout/launch/
RUN printf 'source /opt/ros/noetic/setup.bash\nsource /opt/scout/ws/devel/setup.bash\n' > /opt/scout/setup.bash \
 && useradd -u 1000 -m -d /home/agile agile
ENV SCOUT_SETUP=/opt/scout/setup.bash SCOUT_CONTAINER=1
USER agile
ENTRYPOINT ["entrypoint.sh"]
CMD ["roslaunch", "--wait", "/opt/scout/launch/safety.launch"]
