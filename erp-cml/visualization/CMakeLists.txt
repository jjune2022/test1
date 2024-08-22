cmake_minimum_required(VERSION 3.0.2)
project(visualization)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  geometry_msgs
  visualization_msgs
)

## Generate added messages and services
# generate_messages(
#   DEPENDENCIES
#   std_msgs
#   geometry_msgs
# )

## Catkin specific configuration
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES your_package_name
  CATKIN_DEPENDS rospy std_msgs geometry_msgs visualization_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

#############
## Install ##
#############

## Mark executable scripts (Python etc.) for installation
catkin_install_python(PROGRAMS
  # src/visualize_values.py
  src/broadcaster.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_your_package_name.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${catkin_LIBRARIES})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
