#include "robot_leg/kinematics.h"
#include <cmath>

// Note: The DH parameters for a 2R planar manipulator are:
// i | alpha_{i-1} | a_{i-1} | d_i | theta_i
// --|-------------|---------|-----|---------
// 1 |      0      |    L1   |  0  |   q1
// 2 |      0      |    L2   |  0  |   q2
// The forward kinematics and Jacobian formulas below are the direct result
// of applying the DH convention.

Eigen::Vector2d calculate_forward_kinematics(const Eigen::Vector2d& joint_angles) {
    double q1 = joint_angles(0); // Femur angle
    double q2 = joint_angles(1); // Tibia angle

    double x = FEMUR_LENGTH * cos(q1) + TIBIA_LENGTH * cos(q1 + q2);
    double y = FEMUR_LENGTH * sin(q1) + TIBIA_LENGTH * sin(q1 + q2);

    return Eigen::Vector2d(x, y);
}

Eigen::Matrix2d calculate_geometric_jacobian(const Eigen::Vector2d& joint_angles) {
    double q1 = joint_angles(0);
    double q2 = joint_angles(1);

    double s1 = sin(q1);
    double c1 = cos(q1);
    double s12 = sin(q1 + q2);
    double c12 = cos(q1 + q2);

    Eigen::Matrix2d J;
    J(0, 0) = -FEMUR_LENGTH * s1 - TIBIA_LENGTH * s12;
    J(0, 1) = -TIBIA_LENGTH * s12;
    J(1, 0) =  FEMUR_LENGTH * c1 + TIBIA_LENGTH * c12;
    J(1, 1) =  TIBIA_LENGTH * c12;

    return J;
}

std::optional<Eigen::Vector2d> calculate_inverse_kinematics(
    const Eigen::Vector2d& foot_position, bool elbow_down) {
    double x = foot_position.x();
    double y = foot_position.y();
    double l1 = FEMUR_LENGTH;
    double l2 = TIBIA_LENGTH;

    double c2_num = x * x + y * y - l1 * l1 - l2 * l2;
    double c2_den = 2 * l1 * l2;
    
    // Check if the position is reachable
    double c2_val = c2_num / c2_den;
    if (c2_val < -1.0 || c2_val > 1.0) {
        return std::nullopt; // Position is out of reach
    }

    double c2 = c2_val;
    double s2 = elbow_down ? -sqrt(1 - c2 * c2) : sqrt(1 - c2 * c2);
    double q2 = atan2(s2, c2);

    double k1 = l1 + l2 * c2;
    double k2 = l2 * s2;
    double q1 = atan2(y, x) - atan2(k2, k1);
    
    return Eigen::Vector2d(q1, q2);
}