/*
 * Created on Tue Aug 13 2024
 *
 * Author: Joonchen Liau
 * Email: liaojuncheng123@foxmail.com
 *
 * Copyright (c) 2024 Tiante Intelligent Technology
 */

#include "cody_chassis/cody_chassis.hpp"

using namespace cody;


CODYDriver::CODYDriver(std::string tty_path, int rate, DevType dev_type): 
robot(0.68),
/* ROS related */
canTran(std::move(tty_path), dev_type),
nh(), 
loopRate(rate),
tfBroadcaster(),
currentTime(ros::Time::now()),
/* Subscriber Init */
moveCtrlSub(nh.subscribe("/cmd_vel", 10, &CODYDriver::moveCtrlCallback, this)), 
modeCtrlSub(nh.subscribe("mode_ctrl", 10, &CODYDriver::modeCtrlCallback, this)),
lightCtrlSub(nh.subscribe("light_ctrl", 10, &CODYDriver::lightCtrlCallback, this)),
/* Publisher Init */
sysStatusPub(nh.advertise<cody_msgs::SysStatus>("sys_status_fb", 10)),
moveCtrlFbPub(nh.advertise<cody_msgs::MoveCtrl>("move_ctrl_fb", 10)),
motorInfoFbPub(nh.advertise<cody_msgs::MotorInfoFb>("motor_info_fb", 10)),
odomFbPub(nh.advertise<cody_msgs::OdomFb>("odom_fb", 10)),
odomPub(nh.advertise<nav_msgs::Odometry>("/odom", 10)),
/* Parameter */
pub_tf(nh.param<bool>("pub_tf", false)),
/* frame name*/
base_frame(nh.param<std::string>("base_frame", "map")),
odom_frame(nh.param<std::string>("odom_frame", "odom"))
{
    /* config mode */
    canTran.data.i421ModeCtrl.mode = cody::E421Mode::SPEED;
    canTran.send(ID_ModeCtrl);

    /* odomMsg covariance matrix setup */
    odomMsg.pose.covariance = {
        0.001,      0.0,        0.0,        0.0,        0.0,        0.0,
        0.0,        0.001,      0.0,        0.0,        0.0,        0.0,
        0.0,        0.0,        1000000.0,  0.0,        0.0,        0.0,
        0.0,        0.0,        0.0,        1000000.0,  0.0,        0.0,
        0.0,        0.0,        0.0,        0.0,        1000000.0,  0.0,
        0.0,        0.0,        0.0,        0.0,        0.0,        1000.0};
    odomMsg.twist.covariance = {
        0.001,      0.0,        0.0,        0.0,        0.0,        0.0,
        0.0,        0.001,      0.0,        0.0,        0.0,        0.0,
        0.0,        0.0,        1000000.0,  0.0,        0.0,        0.0,
        0.0,        0.0,        0.0,        1000000.0,  0.0,        0.0,
        0.0,        0.0,        0.0,        0.0,        1000000.0,  0.0,
        0.0,        0.0,        0.0,        0.0,        0.0,        1000.0};
}

void CODYDriver::run() {
    double dt;
    ros::Time currentTime_;
    while (ros::ok()) {
        currentTime_ = ros::Time::now();
        dt = (currentTime_ - currentTime).toSec();
        currentTime = currentTime_;

        /* sys status publisher */
        canTran.recv(ID_SysStatus);
        sysStatusMsg.cur_status = canTran.data.i211SysStatus.cur_status;
        sysStatusMsg.ctrl_mode = canTran.data.i211SysStatus.ctrl_mode;
        sysStatusMsg.bat_vol = canTran.data.i211SysStatus.bat_vol;
        sysStatusMsg.error_info = canTran.data.i211SysStatus.error_info;
        sysStatusMsg.parity = canTran.data.i211SysStatus.parity;
        sysStatusPub.publish(sysStatusMsg);

        /* move ctrl feedback publisher */
        canTran.recv(ID_MoveCtrlFb);
        moveCtrlFbMsg.speed = canTran.data.i221MoveCtrlFb.speed;
        moveCtrlFbMsg.corner = canTran.data.i221MoveCtrlFb.corner;
        moveCtrlFbPub.publish(moveCtrlFbMsg);

        /* motor info feedback publisher */
        canTran.recv(ID_Motor1InfoFb);
        motroInfoFbMsg.motor1_rpm = canTran.data.i251Motor1InfoFb.rpm;
        motroInfoFbMsg.motor1_pos = canTran.data.i251Motor1InfoFb.pos;
        canTran.recv(ID_Motor2InfoFb);
        motroInfoFbMsg.motor2_rpm = canTran.data.i252Motor2InfoFb.rpm;
        motroInfoFbMsg.motor2_pos = canTran.data.i252Motor2InfoFb.pos;
        motorInfoFbPub.publish(motroInfoFbMsg);

        /* odom feedback publisher */
        canTran.recv(ID_OdomFb);
        odomFbMsg.left = canTran.data.i311OdomFb.left;
        odomFbMsg.right = canTran.data.i311OdomFb.right;
        odomFbPub.publish(odomFbMsg);

        /* calculate odom and publish */
        publishOdom(moveCtrlFbMsg.speed, moveCtrlFbMsg.corner, dt);

        ros::spinOnce();
        loopRate.sleep();
    }
}

CODYDriver::~CODYDriver() {
    /* de-config mode */
    canTran.data.i421ModeCtrl.mode = cody::E421Mode::SPEED;
    canTran.send(ID_ModeCtrl);
}

void CODYDriver::moveCtrlCallback(const cody_msgs::MoveCtrl& msg) {
    canTran.data.i111MoveCtrl.speed = msg.speed;
    canTran.data.i111MoveCtrl.corner = msg.corner;
    canTran.send(ID_MoveCtrl);
}

void CODYDriver::modeCtrlCallback(const cody_msgs::ModeCtrl& msg) {
    canTran.data.i421ModeCtrl.mode = static_cast<cody::E421Mode>(msg.mode);
    canTran.send(ID_ModeCtrl);
}

void CODYDriver::lightCtrlCallback(const cody_msgs::LightCtrl& msg) {
    canTran.data.i121LightCtrl.front = static_cast<cody::E121Light>(msg.front);
    canTran.data.i121LightCtrl.rear = static_cast<cody::E121Light>(msg.rear);
    canTran.data.i121LightCtrl.parity = msg.parity;
    canTran.send(ID_LightCtrl);
}

void CODYDriver::publishOdom(double speed, double corner, double dt) {
    /* step model */
    robot.step(speed, corner, dt);

    geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(robot.theta);

    /* publish tf transformation */
    if (pub_tf) {
        geometry_msgs::TransformStamped tf_msg;
        tf_msg.header.stamp = currentTime;
        tf_msg.header.frame_id = base_frame;
        tf_msg.child_frame_id = odom_frame;

        tf_msg.transform.translation.x = robot.x;
        tf_msg.transform.translation.y = robot.y;
        tf_msg.transform.translation.z = 0.0;
        tf_msg.transform.rotation = odom_quat;

        tfBroadcaster.sendTransform(tf_msg);
    }

    /* publish odometry message */
    odomMsg.header.stamp = currentTime;
    odomMsg.header.frame_id = odom_frame;
    odomMsg.child_frame_id = base_frame;

    odomMsg.pose.pose.position.x = robot.x;
    odomMsg.pose.pose.position.y = robot.y;
    odomMsg.pose.pose.position.z = 0.0;
    odomMsg.pose.pose.orientation = odom_quat;

    odomMsg.twist.twist.linear.x = robot.speed;
    odomMsg.twist.twist.linear.y = 0.0;
    odomMsg.twist.twist.angular.z = robot.omega;

    //   std::cerr << "linear: " << linear_speed_ << " , angular: " << steering_angle_
    //             << " , pose: (" << position_x_ << "," << position_y_ << ","
    //             << theta_ << ")" << std::endl;

    odomPub.publish(odomMsg);
}