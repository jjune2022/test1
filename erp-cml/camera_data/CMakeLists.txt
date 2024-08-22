cmake_minimum_required(VERSION 3.0.2)
project(camera_data)

find_package(catkin REQUIRED COMPONENTS
  rospy
  sensor_msgs
  message_filters
  cv_bridge
  std_msgs
  message_generation
  geometry_msgs
)

add_message_files(
  FILES
  ReferencePoses.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

catkin_package(
  CATKIN_DEPENDS rospy sensor_msgs message_filters cv_bridge message_runtime std_msgs geometry_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

catkin_install_python(PROGRAMS
  src/turn_on_camera.py
  src/main_ROS_prs.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
