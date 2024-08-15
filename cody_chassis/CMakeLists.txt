cmake_minimum_required(VERSION 3.0.2)
project(cody_chassis)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

FILE(GLOB SRCS src/*)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
  nav_msgs
  cody_msgs
  tf
)

find_package(ttzn_sdk REQUIRED)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES cody_com
#  CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}_node ${SRCS})
add_dependencies(${PROJECT_NAME}_node cody_msgs_gencpp)

## Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
  ttzn_sdk
)