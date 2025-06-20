#ifndef ROBOT_LEG_KINEMATICS_H
#define ROBOT_LEG_KINEMATICS_H

#include "robot_leg/types.hpp"
#include <optional>

/**
 * @brief Calculates the forward kinematics for the 2-DOF leg.
 * @param joint_angles A 2D vector [femur_angle, tibia_angle] in radians.
 * @return The (x, y) position of the foot relative to the hip.
 */
Eigen::Vector2d calculate_forward_kinematics(const Eigen::Vector2d& joint_angles);

/**
 * @brief Calculates the geometric Jacobian of the leg for a given configuration.
 * @param joint_angles A 2D vector [femur_angle, tibia_angle] in radians.
 * @return A 2x2 matrix representing the Jacobian.
 */
Eigen::Matrix2d calculate_geometric_jacobian(const Eigen::Vector2d& joint_angles);

/**
 * @brief Calculates the inverse kinematics for the 2-DOF leg.
 * @param foot_position The desired (x, y) position of the foot.
 * @param elbow_down Chooses the "elbow down" solution, which is typical for legs.
 * @return An optional containing the [femur_angle, tibia_angle] solution if reachable,
 * otherwise std::nullopt.
 */
std::optional<Eigen::Vector2d> calculate_inverse_kinematics(
    const Eigen::Vector2d& foot_position, bool elbow_down = true);

#endif // ROBOT_LEG_KINEMATICS_H