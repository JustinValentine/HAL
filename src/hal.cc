#include <stdio.h>
#include <string.h>
#include <math.h>
#include <cmath>
#include <algorithm> 
#include <optional>
#include <unistd.h>
#include <iostream>
#include <hal.h>

HAL::HAL(int argc, char* argv[])
    : HAL_MAX(9), HAL_MIN(9) {
    using namespace mjbots;

    moteus::Controller::Options options_common;

    auto& pf = options_common.position_format;
    pf.position = moteus::kFloat;
    pf.velocity = moteus::kFloat;
    pf.kp_scale = moteus::kInt8;
    pf.kd_scale = moteus::kInt8;

    moteus::Controller::DefaultArgProcess(argc, argv);
    transport = moteus::Controller::MakeSingletonTransport({});

    controllers = {
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
        std::make_shared<moteus::Controller>([&]() {
            auto options = options_common;
            options.id = 3;
            return options;
        }()),
    };
    
    for (auto& c : controllers) { c->SetStop(); }
}

HAL::~HAL(){
    for (auto& c : controllers) { c->SetBrake(); }
}

// Receives messages from MJControllers if they can be found
std::optional<mjbots::moteus::Query::Result> HAL::FindServo(
    const std::vector<mjbots::moteus::CanFdFrame>& frames,
    int id) {
  for (auto it = frames.rbegin(); it != frames.rend(); ++it) {
    if (it->source ==id) {
      return mjbots::moteus::Query::Parse(it->data, it->size);
    }
  }
  return {};
}

void HAL::send_motor_command(HAL::MotorGoal goal){
    send_frames.clear();
    receive_frames.clear();
    
    for (size_t i = 0; i < controllers.size(); i++) {
        cmd.position = goal.target_motor_position(i),
        cmd.velocity = goal.target_motor_velocity(i),
        cmd.kp_scale = goal.target_kp(i),
        cmd.kd_scale = goal.target_kd(i),
        send_frames.push_back(controllers[i]->MakePosition(cmd));
    }

    transport->BlockingCycle(
        &send_frames[0], send_frames.size(),
        &receive_frames);
}

bool HAL::receive_motor_state(int id){
    auto maybe_servo = FindServo(receive_frames, id);
    if (!maybe_servo){
        missed_replies++;
        if(missed_replies >=3){
            throw std::runtime_error("Failed to receive motor state");
        }
        return false;
    } else {
        const auto& v = *maybe_servo;
        motor_state.motor_position(id) = v.position * 2 * M_PI;
        motor_state.motor_velocity(id) = v.velocity * 2 * M_PI;
        motor_state.motor_torque(id) = v.torque;
        return true;
    }
}

bool HAL::check_joint_limits(){
    return false;
}

bool HAL::robot_stand(){
    return true;
}

bool HAL::robot_jump(){
    // squat 

    // jump

    // in air 

    // land

    return true;
}

