#include <iostream>
#include <string>
#include <vector>
#include <memory>
#include <chrono>
#include <thread>
#include <cmath>

#include "moteus.h"

#include "robot_leg/types.hpp"
#include "robot_leg/kinematics.h"
#include "robot_leg/virtual_leg_controller.h"

// Enum to manage the leg's state
enum class State {
    kStance,
    kCrouch,
    kThrust,
    kFlight
};

// Helper function to find a servo's response from a vector of CAN frames.
const moteus::QueryResult* FindServo(const std::vector<moteus::CanFdFrame>& frames, uint8_t id) {
    for (const auto& frame : frames) {
        // The first byte of a moteus reply contains the ID.
        if (frame.size > 0 && (frame.data[0] & 0x1f) == id) {
            return moteus::Query::Parse(frame.data, frame.size);
        }
    }
    return nullptr;
}

int main(int argc, char** argv) {
    // --- Controller Setup ---
    auto transport = moteus::Controller::MakeSingletonTransport({});
    moteus::Controller::Options options_common;

    // We need position and velocity feedback from the controllers.
    auto& pf = options_common.position_format;
    pf.position = moteus::kFloat;
    pf.velocity = moteus::kFloat;
    pf.feedforward_torque = moteus::kFloat;

    // Initialize controllers for servo 1 (femur) and 2 (tibia)
    std::vector<std::shared_ptr<moteus::Controller>> controllers = {
        std::make_shared<moteus::Controller>([&]() {
            auto options = options_common;
            options.id = 1;
            return options;
        }()),
        std::make_shared<moteus::Controller>([&]() {
            auto options = options_common;
            options.id = 2;
            return options;
        }()),
    };

    // --- Control Parameters ---
    State current_state = State::kStance;
    const double stance_height = -0.25; // m
    const double crouch_height = -0.30; // m
    const double flight_height = -0.20; // m (tucked in)
    const double thrust_force = 150.0;  // Newtons - This is the "jump" force
    const double stance_kp = 500.0; // Proportional gain for position control
    const double stance_kd = 10.0;  // Derivative gain for damping

    auto state_start_time = std::chrono::steady_clock::now();
    
    // Stop any existing motion
    for (auto& c : controllers) { c->SetStop(); }

    moteus::PositionMode::Command cmd;
    std::vector<moteus::CanFdFrame> send_frames;
    std::vector<moteus::CanFdFrame> receive_frames;
    int missed_replies = 0;

    printf("Starting hopping controller...\n");

    // --- Main Control Loop ---
    while (true) {
        auto loop_start = std::chrono::steady_clock::now();

        // --- Prepare CAN frames for sending ---
        send_frames.clear();
        for (size_t i = 0; i < controllers.size(); i++) {
            // We will calculate and set the torque below.
            cmd.feedforward_torque = 0.0; 
            send_frames.push_back(controllers[i]->MakePosition(cmd));
        }

        // --- Send and Receive Data ---
        receive_frames.clear();
        transport->BlockingCycle(&send_frames[0], send_frames.size(), &receive_frames);
        
        // --- Parse Responses ---
        auto maybe_servo1 = FindServo(receive_frames, 1);
        auto maybe_servo2 = FindServo(receive_frames, 2);

        if (!maybe_servo1 || !maybe_servo2) {
            if (++missed_replies > 5) {
                printf("\n\nServos not responding. Exiting.\n");
                break;
            }
            continue;
        }
        missed_replies = 0;

        // --- Get Current State ---
        // Convert motor revolutions to motor angle in radians.
        Eigen::Vector2d motor_angles(
            maybe_servo1->position * 2.0 * M_PI,
            maybe_servo2->position * 2.0 * M_PI
        );
        // Convert motor angles to joint angles using our transformation matrix.
        Eigen::Vector2d joint_angles = MOTOR_TO_JOINT_MATRIX * motor_angles;
        
        // Calculate current foot position and velocity.
        Eigen::Vector2d current_pos = calculate_forward_kinematics(joint_angles);
        Eigen::Matrix2d J = calculate_geometric_jacobian(joint_angles);
        Eigen::Vector2d joint_velocities = MOTOR_TO_JOINT_MATRIX * Eigen::Vector2d(maybe_servo1->velocity * 2.0 * M_PI, maybe_servo2->velocity * 2.0 * M_PI);
        Eigen::Vector2d current_vel = J * joint_velocities;


        // --- State Machine Logic ---
        auto now = std::chrono::steady_clock::now();
        double state_duration_ms = std::chrono::duration_cast<std::chrono::milliseconds>(now - state_start_time).count();
        
        Eigen::Vector2d target_pos(0.0, stance_height);
        Eigen::Vector2d virtual_force(0,0);
        bool in_air = current_pos.y() > -0.02; // Roughly ground level

        switch (current_state) {
            case State::kStance:
                target_pos.y() = stance_height;
                if (state_duration_ms > 1000) { // After 1s in stance, start a jump
                    current_state = State::kCrouch;
                    state_start_time = now;
                    printf("State: CROUCH\n");
                }
                break;
            case State::kCrouch:
                target_pos.y() = crouch_height;
                if (state_duration_ms > 200) { // After 200ms crouching, thrust
                    current_state = State::kThrust;
                    state_start_time = now;
                    printf("State: THRUST\n");
                }
                break;
            case State::kThrust:
                target_pos.y() = stance_height; // Aim upwards
                virtual_force.y() = thrust_force; // Apply large upward force
                if (in_air || state_duration_ms > 150) { // If we left the ground or timeout
                    current_state = State::kFlight;
                    state_start_time = now;
                    printf("State: FLIGHT\n");
                }
                break;
            case State::kFlight:
                target_pos.y() = flight_height; // Tuck the leg
                if (!in_air && state_duration_ms > 100) { // Once we've landed
                    current_state = State::kStance;
                    state_start_time = now;
                    printf("State: STANCE\n");
                }
                break;
        }

        // --- Calculate Required Torques ---
        // Use a PD controller on position error to generate desired Cartesian forces.
        Eigen::Vector2d pos_error = target_pos - current_pos;
        Eigen::Vector2d desired_force = stance_kp * pos_error - stance_kd * current_vel;
        
        // Add the state-based virtual forces (only non-zero during thrust)
        desired_force += virtual_force;
        
        // Convert the Cartesian force into a virtual leg force/torque representation.
        double alpha = atan2(current_pos.y(), current_pos.x());
        double f_leg = desired_force.x() * cos(alpha) + desired_force.y() * sin(alpha);
        double tau_alpha = -current_pos.norm() * (desired_force.x() * sin(alpha) - desired_force.y() * cos(alpha));

        Eigen::Vector2d virtual_actuator_forces(f_leg, tau_alpha);
        Eigen::Vector2d motor_torques = convert_virtual_to_motor_torques(virtual_actuator_forces, joint_angles);
        
        // --- Send Commands ---
        send_frames.clear();
        for(size_t i = 0; i < controllers.size(); i++) {
            cmd.feedforward_torque = motor_torques(i);
            send_frames.push_back(controllers[i]->MakePosition(cmd));
        }
        // This second blocking cycle sends the torques we just calculated.
        transport->BlockingCycle(&send_frames[0], send_frames.size());

        // Maintain a 1kHz control loop
        std::this_thread::sleep_until(loop_start + std::chrono::milliseconds(1));
    }

    // Stop motors on exit
    for (auto& c : controllers) { c->SetStop(); }

    return 0;
}
