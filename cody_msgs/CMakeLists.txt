cmake_minimum_required(VERSION 3.8)
project(cody_msgs)
# 1.In find_package add message_generation components
find_package(catkin REQUIRED COMPONENTS
  message_generation
  std_msgs
)
 
# 2.add_message_files
add_message_files(
        #DIRECTORY msg
        FILES
        SysStatus.msg
        MoveCtrl.msg
        MotorInfoFb.msg
        WarnFb.msg
        ModeCtrl.msg
        LightCtrl.msg
        OdomFb.msg
        BMSFb.msg
)

add_service_files(
        FILES
        # ModeCtrl.srv
)
 
# 3.generate dependency std_msgs
generate_messages(
        DEPENDENCIES
        std_msgs
)
 
# 4.runtime dependency
catkin_package(
  CATKIN_DEPENDS std_msgs message_runtime
)