#ifndef ROBOT_LEG_VIRTUAL_LEG_CONTROLLER_H
#define ROBOT_LEG_VIRTUAL_LEG_CONTROLLER_H

#include "robot_leg/types.hpp"

// Represents the leg in a simplified polar coordinate system.
struct VirtualLeg {
    double length; // l: distance from hip to foot
    double angle;  // alpha: angle of the hip-to-foot line
};

/**
 * @brief Calculates the virtual leg properties from the foot's Cartesian position.
 * @param foot_position The (x, y) coordinates of the foot.
 * @return A VirtualLeg struct containing the calculated length and angle.
 */
VirtualLeg calculate_virtual_leg(const Eigen::Vector2d& foot_position);

/**
 * @brief Converts forces from the virtual leg model into the required motor torques.
 * @param virtual_actuator_forces A 2D vector [F_leg, tau_alpha], where F_leg is the
 * force along the virtual leg axis and tau_alpha is the torque at the hip.
 * @param joint_angles The current joint angles [q1, q2] of the robot leg.
 * @return A 2D vector of the required torques for the femur and tibia motors.
 */
Eigen::Vector2d convert_virtual_to_motor_torques(const Eigen::Vector2d& virtual_actuator_forces, const Eigen::Vector2d& joint_angles);

#endif // ROBOT_LEG_VIRTUAL_LEG_CONTROLLER_H
