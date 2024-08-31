#include <eigen3/Eigen/Dense>
#include "moteus.h"
#include <memory>
#include <vector>
#include <optional>
#include <string>


#ifndef HAL_H
#define HAL_H

class HAL
{   
public:
    HAL(int argc, char* argv[]); 
    ~HAL();

    struct RobotState {
        Eigen::VectorXd position; // Position of COM
        Eigen::VectorXd velocity; // Velocity of COM
        Eigen::MatrixXd frames;
        enum class State {
            SQUAT,
            STAND,
            WALK,
            RUN,
            JUMP,
            FLIP,
        } state;

        RobotState() : position(9), velocity(9), frames(9, 3), state(State::SQUAT)  {}
    };

    struct RobotGoal {
        Eigen::VectorXd target_position;
        Eigen::VectorXd target_velocity;

        RobotGoal() : target_position(9), target_velocity(9) {}
    };

    struct MotorState {
        Eigen::VectorXd motor_position;
        Eigen::VectorXd motor_velocity;
        Eigen::VectorXd motor_torque;

        MotorState() : motor_position(9), motor_velocity(9), motor_torque(9) {}
    };

    struct MotorGoal {
        Eigen::VectorXd target_motor_position;
        Eigen::VectorXd target_motor_velocity;
        Eigen::VectorXd target_kp;
        Eigen::VectorXd target_kd;

        MotorGoal() : target_motor_position(9), target_motor_velocity(9), target_kd(9), target_kp(9) {}
    };

    void robot_ik(); // Inverse kinematics
    void robot_fk();  // Forward kinematics

    bool robot_stand();
    bool robot_jump();
    
    void send_motor_command(HAL::MotorGoal goal);
    bool receive_motor_state(int id);

    bool check_joint_limits();

    std::shared_ptr<mjbots::moteus::Transport> transport;
    Eigen::VectorXd get_wrist_goal_pos();
    std::optional<mjbots::moteus::Query::Result> FindServo(const std::vector<mjbots::moteus::CanFdFrame>& frames, int id); 

private:
    // HAl values
    RobotState robot_state;
    RobotGoal robot_goal;

    // Moteus Values
    std::vector<std::shared_ptr<mjbots::moteus::Controller>> controllers;
    mjbots::moteus::PositionMode::Command cmd;

    MotorState motor_state;
    MotorGoal motor_goal;

    // communication 
    int missed_replies;
    std::vector<mjbots::moteus::CanFdFrame> send_frames;
    std::vector<mjbots::moteus::CanFdFrame> receive_frames;

protected:
    Eigen::Vector3f HAL_MAX;
    Eigen::Vector3f HAL_MIN; 
};

#endif