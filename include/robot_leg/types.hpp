#ifndef ROBOT_LEG_TYPES_HPP
#define ROBOT_LEG_TYPES_HPP

#include <Eigen/Dense>
#include <cmath>

// Physical constants for the robot leg
const double FEMUR_LENGTH = 0.175; // meters
const double TIBIA_LENGTH = 0.175; // meters
const double LEG_MASS = 2.0;       // kg

// Transformation from motor angles [theta_f, theta_t] to joint angles [q_f, q_t]
// As specified: [q_joint] = M * [theta_motor]
const Eigen::Matrix2d MOTOR_TO_JOINT_MATRIX =
    (Eigen::Matrix2d() << 4.0, 0.0,
                          1.4, 5.4)
    .finished();

// Pre-defined joint angle configurations (in radians)
const Eigen::Vector2d JOINT_HOME_POSITION(0.0, 0.0);
const Eigen::Vector2d JOINT_REST_POSITION(30.0 * M_PI / 180.0, 90.0 * M_PI / 180.0);

#endif // ROBOT_LEG_TYPES_HPP
