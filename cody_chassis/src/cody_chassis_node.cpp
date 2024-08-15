/*
 * Created on Tue Aug 13 2024
 *
 * Author: Joonchen Liau
 * Email: liaojuncheng123@foxmail.com
 *
 * Copyright (c) 2024 Tiante Intelligent Technology
 */

#include "cody_chassis/cody_chassis.hpp"

#include <ros/ros.h>


int main(int argc, char** argv) {
    ros::init(argc, argv, "cody_chassis_node");
    CODYDriver cody_driver("/dev/ttyUSB0", 50, DevType::USB_TTL_CAN);
    cody_driver.run();

    return 0;
}