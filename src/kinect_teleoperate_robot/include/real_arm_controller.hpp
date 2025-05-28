#pragma once


/*
CLAUDE Generated Code

According to main.cpp\
  Convert this code to library (real_arm_controller.hpp) that accepts hardware_control_signal& where
  struct hardware_control_signal {
      double left_shoulder_roll = 0.0;
      double left_shoulder_pitch = 0.0;
      double left_shoulder_yaw = 0.0;
      double right_shoulder_roll = 0.0;
      double right_shoulder_pitch = 0.0;
      double right_shoulder_yaw = 0.0;
      double left_elbow_yaw = 0.0;
      double right_elbow_yaw = 0.0;
  };

  It'll command the robot using these hardware_control_signal if and only if the void update() is called.

  ->

  Created real_arm_controller.hpp library with:

  - RealArmController class that accepts hardware_control_signal struct
  - Constructor takes network interface parameter
  - set_control_signal() method to update target joint positions
  - Removed waist joint (only 8 arm joints now)
  - All initialization and communication logic encapsulated in the class

  Usage:
  RealArmController controller("eth0");
  hardware_control_signal signal;
  signal.left_shoulder_pitch = 1.0;
  controller.set_control_signal(signal);
  controller.update(); // Commands robot

*/

#include <array>
#include <chrono>
#include <memory>
#include <thread>
#include <string>

#include <unitree/idl/go2/LowCmd_.hpp>
#include <unitree/idl/hg/LowState_.hpp>
#include <unitree/robot/channel/channel_publisher.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>

#include "hardware_control_signal.hpp"

class RealArmController {
public:
    explicit RealArmController(const std::string& network_interface);
    ~RealArmController();

    void set_control_signal(const hardware_control_signal& signal);

private:
    static const std::string kTopicArmSDK;
    static const std::string kTopicState;
    static constexpr float kPi = 3.141592654f;
    static constexpr float kPi_2 = 1.57079632f;

    enum JointIndex {
        kRightHipYaw = 8,
        kRightHipRoll = 0,
        kRightHipPitch = 1,
        kRightKnee = 2,
        kRightAnkle = 11,
        kLeftHipYaw = 7,
        kLeftHipRoll = 3,
        kLeftHipPitch = 4,
        kLeftKnee = 5,
        kLeftAnkle = 10,
        kWaistYaw = 6,
        kNotUsedJoint = 9,
        kRightShoulderPitch = 12,
        kRightShoulderRoll = 13,
        kRightShoulderYaw = 14,
        kRightElbow = 15,
        kLeftShoulderPitch = 16,
        kLeftShoulderRoll = 17,
        kLeftShoulderYaw = 18,
        kLeftElbow = 19,
    };

    unitree::robot::ChannelPublisherPtr<unitree_go::msg::dds_::LowCmd_> arm_sdk_publisher_;
    unitree::robot::ChannelSubscriberPtr<unitree_hg::msg::dds_::LowState_> low_state_subscriber_;
    
    unitree_go::msg::dds_::LowCmd_ msg_;
    unitree_hg::msg::dds_::LowState_ state_msg_;
    
    std::array<JointIndex, 8> arm_joints_;
    hardware_control_signal current_signal_;
    
    float weight_;
    float weight_rate_;
    float kp_;
    float kd_;
    float dq_;
    float tau_ff_;
    float control_dt_;
    float max_joint_velocity_;
    float delta_weight_;
    float max_joint_delta_;
    
    std::chrono::milliseconds sleep_time_;
};

inline RealArmController::RealArmController(const std::string& network_interface) 
    : weight_(1.0f)
    , weight_rate_(0.2f)
    , kp_(60.f)
    , kd_(1.5f)
    , dq_(0.f)
    , tau_ff_(0.f)
    , control_dt_(0.02f)
    , max_joint_velocity_(0.5f)
    , delta_weight_(weight_rate_ * control_dt_)
    , max_joint_delta_(max_joint_velocity_ * control_dt_)
    , sleep_time_(std::chrono::milliseconds(static_cast<int>(control_dt_ / 0.001f)))
    , arm_joints_{
        JointIndex::kLeftShoulderPitch,  JointIndex::kLeftShoulderRoll,
        JointIndex::kLeftShoulderYaw,    JointIndex::kLeftElbow,
        JointIndex::kRightShoulderPitch, JointIndex::kRightShoulderRoll,
        JointIndex::kRightShoulderYaw,   JointIndex::kRightElbow
    }
{
    std::cout << "Initializing network interface " << network_interface << std::endl;
    unitree::robot::ChannelFactory::Instance()->Init(0, network_interface.c_str());

    std::cout << "arm_sdk_publisher_.reset" << std::endl;
    arm_sdk_publisher_.reset(
        new unitree::robot::ChannelPublisher<unitree_go::msg::dds_::LowCmd_>(
            kTopicArmSDK));
    
    std::cout << "arm_sdk_publisher_->InitChannel" << std::endl;
    arm_sdk_publisher_->InitChannel();

    std::cout << "low_state_subscriber_.reset" << std::endl;
    low_state_subscriber_.reset(
        new unitree::robot::ChannelSubscriber<unitree_hg::msg::dds_::LowState_>(
            kTopicState));

    std::cout << "low_state_subscriber_->InitChannel" << std::endl;
    low_state_subscriber_->InitChannel([&](const void *msg) {
        auto s = (const unitree_hg::msg::dds_::LowState_*)msg;
        memcpy(&state_msg_, s, sizeof(unitree_hg::msg::dds_::LowState_));
    }, 1);
}

inline RealArmController::~RealArmController() = default;

inline void RealArmController::set_control_signal(const hardware_control_signal& signal) {
    current_signal_ = signal;
    msg_.motor_cmd().at(JointIndex::kNotUsedJoint).q(weight_);

    std::array<float, 8> target_positions = {
        static_cast<float>(current_signal_.left_shoulder_pitch),
        static_cast<float>(current_signal_.left_shoulder_roll),
        static_cast<float>(current_signal_.left_shoulder_yaw),
        static_cast<float>(current_signal_.left_elbow_yaw),
        static_cast<float>(current_signal_.right_shoulder_pitch),
        static_cast<float>(current_signal_.right_shoulder_roll),
        static_cast<float>(current_signal_.right_shoulder_yaw),
        static_cast<float>(current_signal_.right_elbow_yaw)
    };

    for (int j = 0; j < arm_joints_.size(); ++j) {
        msg_.motor_cmd().at(arm_joints_.at(j)).q(target_positions.at(j));
        msg_.motor_cmd().at(arm_joints_.at(j)).dq(dq_);
        msg_.motor_cmd().at(arm_joints_.at(j)).kp(kp_);
        msg_.motor_cmd().at(arm_joints_.at(j)).kd(kd_);
        msg_.motor_cmd().at(arm_joints_.at(j)).tau(tau_ff_);
    }

    arm_sdk_publisher_->Write(msg_);
}

inline const std::string RealArmController::kTopicArmSDK = "rt/arm_sdk";
inline const std::string RealArmController::kTopicState = "rt/lowstate";