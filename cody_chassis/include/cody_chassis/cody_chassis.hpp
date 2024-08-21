/*
 * Created on Tue Aug 13 2024
 *
 * Author: Joonchen Liau
 * Email: liaojuncheng123@foxmail.com
 *
 * Copyright (c) 2024 Tiante Intelligent Technology
 */

#ifndef CODY_CHASSIS_H
#define CODY_CHASSIS_H

#include <ros/ros.h>
#include <string>

#include <tf/tf.h>
#include <tf2_ros/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>

#include "ttzn_sdk/cody/tran.hpp"
#include "cody_chassis/robot.hpp"
#include "cody_msgs/SysStatus.h"
#include "cody_msgs/MoveCtrl.h"
#include "cody_msgs/MotorInfoFb.h"
#include "cody_msgs/WarnFb.h"
#include "cody_msgs/ModeCtrl.h"
#include "cody_msgs/LightCtrl.h"
#include "cody_msgs/OdomFb.h"
#include "cody_msgs/BMSFb.h"


class CODYDriver {
    /* Robot Model */
    Robot                           robot;
    
    /* CAN tran and recv */
    cody::CANTran                   canTran;

    /* ROS related */
    ros::NodeHandle                 nh;
    ros::Rate                       loopRate;
    tf2_ros::TransformBroadcaster   tfBroadcaster;
    ros::Time                       currentTime;
    

    /* Subscriber */
    ros::Subscriber                 moveCtrlSub;
    ros::Subscriber                 modeCtrlSub;
    ros::Subscriber                 lightCtrlSub;
    
    /* Publisher for feedback */
    ros::Publisher                  sysStatusPub;
    ros::Publisher                  moveCtrlFbPub;
    ros::Publisher                  motorInfoFbPub;
    /* odomFbPub data is from STM32 left and right wheel's odom */
    ros::Publisher                  odomFbPub; 

    /* odomPub data is calculate through math kinematic model */
    ros::Publisher                  odomPub; 

    /* messages for feedback */
    cody_msgs::SysStatus            sysStatusMsg;
    cody_msgs::MoveCtrl             moveCtrlFbMsg;
    cody_msgs::MotorInfoFb          motroInfoFbMsg;
    cody_msgs::OdomFb               odomFbMsg;

    nav_msgs::Odometry              odomMsg;

    /* Parameters */
    bool                            pub_tf;
    /* frame name */
    std::string                     base_frame;
    std::string                     odom_frame;
    /* CAN device */
    std::string                     dev_path;
    std::string                     dev_type;

public:
    CODYDriver(int rate=50);

    void run();

    ~CODYDriver();

private:
    void moveCtrlCallback(const cody_msgs::MoveCtrl& msg);

    void modeCtrlCallback(const cody_msgs::ModeCtrl& msg);

    void lightCtrlCallback(const cody_msgs::LightCtrl& msg);

    void publishOdom(double speed, double corner, double dt);
};

#endif /* END CODY_CHASSIS_H */