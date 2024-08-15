cmake_minimum_required(VERSION 3.0.2)
project(cody_startup)

find_package(catkin REQUIRED)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES cody_startup
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)

##### INSTALL launch file #####
file(GLOB LAUNCH_FILES launch/*.launch)
## Mark other files for installation (e.g. launch and bag files, etc.)
install(FILES
  ${LAUNCH_FILES}
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)