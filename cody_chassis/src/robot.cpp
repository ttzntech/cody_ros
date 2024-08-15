/*
 * Created on Thu Aug 15 2024
 *
 * Author: Joonchen Liau
 * Email: liaojuncheng123@foxmail.com
 *
 * Copyright (c) 2024 Tiante Intelligent Technology
 */

#include "cody_chassis/robot.hpp"

Robot::Robot (double wheelbase)
        : x(0.0), y(0.0), theta(0.0), speed(0.0), wheelbase_(wheelbase) {}

Robot::Robot (double x, double y,  double theta, double speed, double wheelbase)
        : x(x), y(y), theta(theta), speed(speed), wheelbase_(wheelbase) {}

void Robot::step(double speed, double corner, double dt) {
    /* Update speed */
    this->speed = speed;

    /* Calculate the angular velocity (turning rate) */
    omega = speed * std::tan(corner) / wheelbase_;

    /* Update heading angle (theta) */
    theta += omega * dt;

    /* Update position (x, y) */
    x += speed * std::cos(theta) * dt;
    y += speed * std::sin(theta) * dt;
}