#include "robot_leg/virtual_leg_controller.h"
#include "robot_leg/kinematics.h" // Needed for Jacobian and FK

VirtualLeg calculate_virtual_leg(const Eigen::Vector2d& foot_position) {
    VirtualLeg vl;
    vl.length = foot_position.norm();
    vl.angle = atan2(foot_position.y(), foot_position.x());
    return vl;
}

Eigen::Vector2d convert_virtual_to_motor_torques(const Eigen::Vector2d& virtual_actuator_forces, const Eigen::Vector2d& joint_angles) {
    // Unpack virtual forces
    double f_leg = virtual_actuator_forces(0);
    double tau_alpha = virtual_actuator_forces(1);

    // Step 1: Get current leg geometry
    Eigen::Vector2d foot_pos = calculate_forward_kinematics(joint_angles);
    VirtualLeg v_leg = calculate_virtual_leg(foot_pos);
    double l = v_leg.length;
    double alpha = v_leg.angle;

    // Step 2: Convert virtual forces (polar) to Cartesian forces at the foot
    // F_cartesian = R(alpha) * [F_radial, F_tangential]
    // where F_radial = f_leg and F_tangential = tau_alpha / l
    if (l < 1e-6) { // Avoid division by zero if leg is at the origin
        return Eigen::Vector2d::Zero();
    }
    double f_x = f_leg * cos(alpha) - (tau_alpha / l) * sin(alpha);
    double f_y = f_leg * sin(alpha) + (tau_alpha / l) * cos(alpha);
    Eigen::Vector2d cartesian_force(f_x, f_y);

    // Step 3: Map Cartesian foot forces to joint torques using the Jacobian
    // tau_joint = J^T * F_cartesian
    Eigen::Matrix2d J = calculate_geometric_jacobian(joint_angles);
    Eigen::Vector2d joint_torques = J.transpose() * cartesian_force;

    // Step 4: Map joint torques to motor torques using the motor transformation matrix
    // tau_motor = M^T * tau_joint
    Eigen::Vector2d motor_torques = MOTOR_TO_JOINT_MATRIX.transpose() * joint_torques;

    return motor_torques;
}
