cmake_minimum_required(VERSION 3.0.2)
project(chassis_driver)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
  nav_msgs
)
find_package(yaml-cpp)
find_package(CSerialPort QUIET)
find_package(cody_msgs REQUIRED)


add_compile_definitions(WORKSPACE_PATH="${PROJECT_SOURCE_DIR}")#用于代码中获取工程路径


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES chassis_driver
#  CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
)


message(${catkin_INCLUDE_DIRS})
include_directories(
  include
  ${CATKIN_DEVEL_PREFIX}/include
  ${catkin_INCLUDE_DIRS}
  ${CSerialPort_INCLUDE_DIR}
)


add_executable(Chassis_driver src/chassis_driver_node.cpp src/chassis_driver)


target_link_libraries(Chassis_driver
  "/usr/local/lib/libyaml-cpp.a"
  ${catkin_LIBRARIES}
  ${CSerialPort_LIBRARY}
)

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# catkin_install_python(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_executables.html
# install(TARGETS ${PROJECT_NAME}_node
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark libraries for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_libraries.html
# install(TARGETS ${PROJECT_NAME}
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_chassis_driver.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
