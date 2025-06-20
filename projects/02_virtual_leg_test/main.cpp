#include <iostream>
#include <vector>
#include <memory>
#include <chrono>
#include <thread>
#include <iomanip>

#include "moteus.h"
#include "robot_leg/types.hpp"
#include "robot_leg/kinematics.h"
#include "robot_leg/virtual_leg_controller.h"

// Helper function to find a servo's response from a vector of CAN frames.
const moteus::QueryResult* FindServo(const std::vector<moteus::CanFdFrame>& frames, uint8_t id) {
    for (const auto& frame : frames) {
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
    auto& pf = options_common.position_format;
    pf.position = moteus::kFloat;
    pf.velocity = moteus::kIgnore;
    pf.feedforward_torque = moteus::kFloat;

    std::vector<std::shared_ptr<moteus::Controller>> controllers = {
        std::make_shared<moteus::Controller>({1, &options_common}),
        std::make_shared<moteus::Controller>({2, &options_common}),
    };
    
    // --- Control Parameters ---
    // Apply a constant virtual force: 50N along the leg axis (pushing out)
    // and 0Nm of rotational torque.
    const Eigen::Vector2d virtual_force_command(50.0, 0.0);

    for (auto& c : controllers) { c->SetStop(); }

    moteus::PositionMode::Command cmd;
    std::vector<moteus::CanFdFrame> send_frames;
    std::vector<moteus::CanFdFrame> receive_frames;

    printf("Starting virtual leg test. Applying a constant 50N virtual force.\n");
    printf("Press Ctrl+C to exit.\n\n");

    // --- Main Control Loop ---
    while (true) {
        // --- Prepare CAN frames to poll the controllers ---
        send_frames.clear();
        for (size_t i = 0; i < controllers.size(); i++) {
            cmd.feedforward_torque = 0.0; // We will overwrite this later
            send_frames.push_back(controllers[i]->MakePosition(cmd));
        }

        // --- Send and Receive Data ---
        receive_frames.clear();
        transport->BlockingCycle(&send_frames[0], send_frames.size(), &receive_frames);
        
        auto maybe_servo1 = FindServo(receive_frames, 1);
        auto maybe_servo2 = FindServo(receive_frames, 2);

        if (!maybe_servo1 || !maybe_servo2) {
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
            continue;
        }

        // --- Calculate Torques ---
        Eigen::Vector2d motor_angles(maybe_servo1->position * 2.0 * M_PI, maybe_servo2->position * 2.0 * M_PI);
        Eigen::Vector2d joint_angles = MOTOR_TO_JOINT_MATRIX * motor_angles;
        
        // Convert the constant virtual force into motor torques for the current configuration.
        Eigen::Vector2d motor_torques = convert_virtual_to_motor_torques(virtual_force_command, joint_angles);
        
        // --- Send Torque Commands ---
        send_frames.clear();
        for(size_t i = 0; i < controllers.size(); i++) {
            cmd.feedforward_torque = motor_torques(i);
            send_frames.push_back(controllers[i]->MakePosition(cmd));
        }
        transport->BlockingCycle(&send_frames[0], send_frames.size());

        std::cout << std::fixed << std::setprecision(3)
                  << "Virtual Force Cmd: [" << std::setw(6) << virtual_force_command.x() << ", " << std::setw(6) << virtual_force_command.y() << "] | "
                  << "Motor Torque Cmd (Nm): [" << std::setw(6) << motor_torques.x() << ", " << std::setw(6) << motor_torques.y() << "]\r"
                  << std::flush;

        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }

    // Stop motors on exit
    for (auto& c : controllers) { c->SetStop(); }

    return 0;
}
