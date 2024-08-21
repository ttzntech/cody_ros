/*
 * Created on Thu Aug 15 2024
 *
 * Author: Joonchen Liau
 * Email: liaojuncheng123@foxmail.com
 *
 * Copyright (c) 2024 Tiante Intelligent Technology
 */

#ifndef ROBOT_H
#define ROBOT_H

#include <cmath>


class Robot {
    double wheelbase_;  /* Distance between front and rear axis (meters) */
public:
    Robot (double wheelbase);

    Robot (double x, double y, double theta, double speed, double wheelbase);

    void step(double speed, double corner, double dt);

    double x;           /* Current x position */
    double y;           /* Current y position */
    double theta;       /* Current heading angle (radians) */
    double speed;       /* Current speed (m/s) */
    double omega;       /* Current omega (rad/s) */
};

#endif /* END ROBOT_H */