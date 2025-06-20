#include <iostream>
#include <vector>
#include <memory>
#include <chrono>
#include <thread>
#include <iomanip>

#include "moteus.h"
#include "robot_leg/types.hpp"
#include "robot_leg/kinematics.h"

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
    pf.position = moteus::kFloat; // We only need position feedback for this test
    pf.velocity = moteus::kIgnore;
    pf.feedforward_torque = moteus::kIgnore;

    std::vector<std::shared_ptr<moteus::Controller>> controllers = {
        std::make_shared<moteus::Controller>({1, &options_common}),
        std::make_shared<moteus::Controller>({2, &options_common}),
    };

    // Send a stop command to ensure the controllers are in a safe state,
    // only reporting data.
    for (auto& c : controllers) { c->SetStop(); }

    std::vector<moteus::CanFdFrame> send_frames;
    std::vector<moteus::CanFdFrame> receive_frames;

    printf("Starting passive test. Move the leg by hand to see values change.\n");
    printf("Press Ctrl+C to exit.\n\n");

    // --- Main Loop ---
    while (true) {
        // --- Prepare CAN frames ---
        // We send a "stop" command in a loop. This acts as a poll,
        // requesting a reply from the controllers without commanding motion.
        send_frames.clear();
        for (auto& c : controllers) {
            send_frames.push_back(c->MakeStop());
        }

        // --- Send and Receive Data ---
        receive_frames.clear();
        transport->BlockingCycle(&send_frames[0], send_frames.size(), &receive_frames);
        
        // --- Parse Responses ---
        auto maybe_servo1 = FindServo(receive_frames, 1);
        auto maybe_servo2 = FindServo(receive_frames, 2);

        if (!maybe_servo1 || !maybe_servo2) {
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
            continue;
        }

        // --- Calculate and Display State ---
        // 1. Get motor positions from the controllers
        Eigen::Vector2d motor_revolutions(maybe_servo1->position, maybe_servo2->position);
        
        // 2. Convert motor revolutions to motor angle in radians.
        Eigen::Vector2d motor_angles = motor_revolutions * 2.0 * M_PI;

        // 3. Convert motor angles to joint angles using the transformation matrix.
        Eigen::Vector2d joint_angles = MOTOR_TO_JOINT_MATRIX * motor_angles;
        
        // 4. Calculate the foot's Cartesian position using forward kinematics.
        Eigen::Vector2d foot_pos = calculate_forward_kinematics(joint_angles);

        // Display all values, using carriage return to update a single line.
        std::cout << std::fixed << std::setprecision(3)
                  << "Motor Angles (rad): [" << std::setw(6) << motor_angles.x() << ", " << std::setw(6) << motor_angles.y() << "] | "
                  << "Joint Angles (rad): [" << std::setw(6) << joint_angles.x() << ", " << std::setw(6) << joint_angles.y() << "] | "
                  << "Foot Pose (m):    [" << std::setw(6) << foot_pos.x() << ", " << std::setw(6) << foot_pos.y() << "]\r"
                  << std::flush;

        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }

    return 0;
}