cmake_minimum_required(VERSION 3.0.2)
project(rs_object_detection)

find_package(catkin REQUIRED COMPONENTS message_generation std_msgs geometry_msgs vision_msgs)

add_message_files(FILES TrackedObject.msg TrackedObjectArray.msg)
generate_messages(DEPENDENCIES std_msgs geometry_msgs vision_msgs)

catkin_package(CATKIN_DEPENDS message_runtime std_msgs geometry_msgs vision_msgs)

catkin_install_python(PROGRAMS scripts/detector_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch config rviz
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
