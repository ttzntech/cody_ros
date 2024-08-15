/*
 * Created on Thu Aug 15 2024
 *
 * Author: Joonchen Liau
 * Email: liaojuncheng123@foxmail.com
 *
 * Copyright (c) 2024 Tiante Intelligent Technology
 */

#include <cmath>

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

#include "cody_msgs/MoveCtrl.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "cycle_test");
    ros::NodeHandle nh;
    ros::Publisher cmd_pub = nh.advertise<cody_msgs::MoveCtrl>("/cmd_vel", 10);
    ros::Rate loop_rate(50);  // 50 Hz

    cody_msgs::MoveCtrl msg;

    ros::Time time;
    double period = 20;

    double speed, corner;
	
    while (ros::ok())
    {   
        speed = 1.5 * sin(time.toSec() * 2 * M_PI / period);
        corner = 30 * sin(time.toSec() * 2 * M_PI / period);
        msg.speed = speed;
        msg.corner = corner;

        cmd_pub.publish(msg);

        time += loop_rate.expectedCycleTime();

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}