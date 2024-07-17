#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <cmath>
#include "cody_msgs/CtrlCmd.h"
int main(int argc, char** argv)
{
    ros::init(argc, argv, "draw_sine");
    ros::NodeHandle nh;
    ros::Publisher velocity_publisher = nh.advertise<cody_msgs::CtrlCmd>("/cmd_vel", 10);
    ros::Rate loop_rate(100);  // 10 Hz

    cody_msgs::CtrlCmd vel_msg;

    double time = 0.0;
    double amplitude = 1.0;
    double frequency = 0.5;
    double angle_=0;
    double true_speed=0;
	
    while (ros::ok())
    {   
        angle_ = 500 * sin(time * 2 * 3.1415926 / 20);
        true_speed = 1500* sin(time * 2 * 3.1415926 / 20 + 0.5*3.1415926);
        vel_msg.ctrl_cmd_velocity = true_speed;
        vel_msg.ctrl_cmd_steering = angle_;
      

        velocity_publisher.publish(vel_msg);

        time += loop_rate.expectedCycleTime().toSec();
        ros::spinOnce;
        loop_rate.sleep();
    }

    return 0;
}

