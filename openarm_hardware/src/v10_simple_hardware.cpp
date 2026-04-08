// Copyright 2025 Enactic, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "openarm_hardware/v10_simple_hardware.hpp"

#include <Eigen/Core>
#include <algorithm>
#include <cctype>
#include <chrono>
#include <cmath>
#include <sstream>
#include <thread>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/logging.hpp"
#include "rclcpp/rclcpp.hpp"

namespace openarm_hardware {

OpenArm_v10HW::OpenArm_v10HW() = default;

bool OpenArm_v10HW::parse_config(const hardware_interface::HardwareInfo& info) {
  // Parse CAN interface (default: can0)
  auto it = info.hardware_parameters.find("can_interface");
  can_interface_ = (it != info.hardware_parameters.end()) ? it->second : "can0";

  // Parse arm prefix (default: empty for single arm, "left_" or "right_" for
  // bimanual)
  it = info.hardware_parameters.find("arm_prefix");
  arm_prefix_ = (it != info.hardware_parameters.end()) ? it->second : "";

  // Parse gripper enable (default: true for V10)
  it = info.hardware_parameters.find("hand");
  if (it == info.hardware_parameters.end()) {
    hand_ = true;  // Default to true for V10
  } else {
    // Handle both "true"/"True" and "false"/"False"
    std::string value = it->second;
    std::transform(value.begin(), value.end(), value.begin(), ::tolower);
    hand_ = (value == "true");
  }

  // Parse CAN-FD enable (default: true for V10)
  it = info.hardware_parameters.find("can_fd");
  if (it == info.hardware_parameters.end()) {
    can_fd_ = true;  // Default to true for V10
  } else {
    // Handle both "true"/"True" and "false"/"False"
    std::string value = it->second;
    std::transform(value.begin(), value.end(), value.begin(), ::tolower);
    can_fd_ = (value == "true");
  }

  // Parse gravity compensation enable (default: false)
  it = info.hardware_parameters.find("use_gravity_compensation");
  if (it == info.hardware_parameters.end()) {
    use_gravity_compensation_ = false;  // Default to false
  } else {
    // Handle both "true"/"True" and "false"/"False"
    std::string value = it->second;
    std::transform(value.begin(), value.end(), value.begin(), ::tolower);
    use_gravity_compensation_ = (value == "true");
  }

  // Parse robot_description for URDF (optional, will fallback to parameter
  // server)
  it = info.hardware_parameters.find("robot_description");
  if (it != info.hardware_parameters.end()) {
    robot_description_ = it->second;
  }

  // Parse control gains
  for (size_t i = 1; i <= ARM_DOF; ++i) {
    it = info.hardware_parameters.find("kp" + std::to_string(i));
    if (it != info.hardware_parameters.end()) {
      kp_[i - 1] = std::stod(it->second);
    }
    it = info.hardware_parameters.find("kd" + std::to_string(i));
    if (it != info.hardware_parameters.end()) {
      kd_[i - 1] = std::stod(it->second);
    }
  }

  RCLCPP_INFO(rclcpp::get_logger("OpenArm_v10HW"),
              "Configuration: CAN=%s, arm_prefix=%s, hand=%s, can_fd=%s, "
              "grav_comp=%s",
              can_interface_.c_str(), arm_prefix_.c_str(),
              hand_ ? "enabled" : "disabled", can_fd_ ? "enabled" : "disabled",
              use_gravity_compensation_ ? "enabled" : "disabled");
  return true;
}

void OpenArm_v10HW::generate_joint_names() {
  joint_names_.clear();

  // If we have a Pinocchio model loaded from URDF, extract joint names from it
  // Filter for movable joints (exclude fixed joints and the root joint)
  if (model_.nq > 0) {
    RCLCPP_INFO(rclcpp::get_logger("OpenArm_v10HW"),
                "Extracting joint names from URDF model...");
    for (const auto& joint_name : model_.names) {
      // Skip "universe" (root joint) and any empty names
      if (joint_name.empty() || joint_name == "universe") {
        continue;
      }
      // Check if joint is in the arm prefix (or use all if no prefix)
      if (arm_prefix_.empty() || joint_name.find(arm_prefix_) != std::string::npos) {
        joint_names_.push_back(joint_name);
      }
    }
    RCLCPP_INFO(rclcpp::get_logger("OpenArm_v10HW"),
                "Generated %zu joint names from URDF model", joint_names_.size());
    return;
  }

  // Fallback: generate arm joint names using hardcoded pattern
  // openarm_{arm_prefix}joint{N}
  for (size_t i = 1; i <= ARM_DOF; ++i) {
    std::string joint_name =
        "openarm_" + arm_prefix_ + "joint" + std::to_string(i);
    joint_names_.push_back(joint_name);
  }

  // Generate gripper joint name if enabled
  if (hand_) {
    std::string gripper_joint_name = "openarm_" + arm_prefix_ + "finger_joint1";
    joint_names_.push_back(gripper_joint_name);
    RCLCPP_INFO(rclcpp::get_logger("OpenArm_v10HW"), "Added gripper joint: %s",
                gripper_joint_name.c_str());
  } else {
    RCLCPP_INFO(rclcpp::get_logger("OpenArm_v10HW"),
                "Gripper joint NOT added because hand_=false");
  }

  RCLCPP_INFO(rclcpp::get_logger("OpenArm_v10HW"),
              "Generated %zu joint names for arm prefix '%s' using hardcoded pattern",
              joint_names_.size(), arm_prefix_.c_str());
}

hardware_interface::CallbackReturn OpenArm_v10HW::on_init(
    const hardware_interface::HardwareInfo& info) {
  if (hardware_interface::SystemInterface::on_init(info) !=
      CallbackReturn::SUCCESS) {
    return CallbackReturn::ERROR;
  }
  // Parse configuration
  if (!parse_config(info)) {
    return CallbackReturn::ERROR;
  }

  // Load URDF early if robot_description is provided in hardware parameters
  // This allows generate_joint_names to use URDF joint names
  if (!robot_description_.empty() && use_gravity_compensation_) {
    RCLCPP_INFO(rclcpp::get_logger("OpenArm_v10HW"),
                "URDF provided in hardware parameters, loading early...");
    if (!load_urdf_and_initialize_pinocchio()) {
      RCLCPP_WARN(rclcpp::get_logger("OpenArm_v10HW"),
                  "Failed to load URDF early. Will fallback to hardcoded joint names.");
      // Don't return ERROR here - gravity comp will be disabled in on_configure
      use_gravity_compensation_ = false;
    }
  }

  // Generate joint names based on arm prefix (or URDF if loaded)
  generate_joint_names();
  if (!parse_config(info)) {
    return CallbackReturn::ERROR;
  }

  // Generate joint names based on arm prefix
  generate_joint_names();

  // Validate joint count (7 arm joints + optional gripper)
  size_t expected_joints = ARM_DOF + (hand_ ? 1 : 0);
  if (joint_names_.size() != expected_joints) {
    RCLCPP_ERROR(rclcpp::get_logger("OpenArm_v10HW"),
                 "Generated %zu joint names, expected %zu", joint_names_.size(),
                 expected_joints);
    return CallbackReturn::ERROR;
  }

  // Initialize OpenArm with configurable CAN-FD setting
  RCLCPP_INFO(rclcpp::get_logger("OpenArm_v10HW"),
              "Initializing OpenArm on %s with CAN-FD %s...",
              can_interface_.c_str(), can_fd_ ? "enabled" : "disabled");
  openarm_ =
      std::make_unique<openarm::can::socket::OpenArm>(can_interface_, can_fd_);

  // Initialize arm motors with V10 defaults
  openarm_->init_arm_motors(DEFAULT_MOTOR_TYPES, DEFAULT_SEND_CAN_IDS,
                            DEFAULT_RECV_CAN_IDS);

  // Initialize gripper if enabled
  if (hand_) {
    RCLCPP_INFO(rclcpp::get_logger("OpenArm_v10HW"), "Initializing gripper...");
    openarm_->init_gripper_motor(DEFAULT_GRIPPER_MOTOR_TYPE,
                                 DEFAULT_GRIPPER_SEND_CAN_ID,
                                 DEFAULT_GRIPPER_RECV_CAN_ID);
  }

  // Initialize state and command vectors based on generated joint count
  const size_t total_joints = joint_names_.size();
  pos_commands_.resize(total_joints, 0.0);
  vel_commands_.resize(total_joints, 0.0);
  tau_commands_.resize(total_joints, 0.0);
  pos_states_.resize(total_joints, 0.0);
  vel_states_.resize(total_joints, 0.0);
  tau_states_.resize(total_joints, 0.0);

  RCLCPP_INFO(rclcpp::get_logger("OpenArm_v10HW"),
              "OpenArm V10 Simple HW initialized successfully");

  return CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn OpenArm_v10HW::on_configure(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  // Set callback mode to ignore during configuration
  openarm_->refresh_all();
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  openarm_->recv_all();

  // Initialize Pinocchio if gravity compensation is enabled
  // Skip if already loaded in on_init()
  if (use_gravity_compensation_ && model_.nq == 0) {
    RCLCPP_INFO(rclcpp::get_logger("OpenArm_v10HW"),
                "Gravity compensation enabled, loading URDF and "
                "initializing Pinocchio...");
    if (!load_urdf_and_initialize_pinocchio()) {
      RCLCPP_ERROR(rclcpp::get_logger("OpenArm_v10HW"),
                   "Failed to load URDF and initialize Pinocchio. "
                   "Gravity compensation will be disabled.");
      use_gravity_compensation_ = false;
      return CallbackReturn::ERROR;
    }
    RCLCPP_INFO(rclcpp::get_logger("OpenArm_v10HW"),
                "Pinocchio initialized successfully for gravity compensation");
  } else if (use_gravity_compensation_) {
    RCLCPP_INFO(rclcpp::get_logger("OpenArm_v10HW"),
                "Pinocchio already initialized, skipping URDF loading");
  } else {
    RCLCPP_INFO(rclcpp::get_logger("OpenArm_v10HW"),
                "Gravity compensation disabled, skipping URDF loading");
  }

  return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface>
OpenArm_v10HW::export_state_interfaces() {
  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (size_t i = 0; i < joint_names_.size(); ++i) {
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        joint_names_[i], hardware_interface::HW_IF_POSITION, &pos_states_[i]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        joint_names_[i], hardware_interface::HW_IF_VELOCITY, &vel_states_[i]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        joint_names_[i], hardware_interface::HW_IF_EFFORT, &tau_states_[i]));
  }

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface>
OpenArm_v10HW::export_command_interfaces() {
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  // TODO: consider exposing only needed interfaces to avoid undefined behavior.
  for (size_t i = 0; i < joint_names_.size(); ++i) {
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
        joint_names_[i], hardware_interface::HW_IF_POSITION,
        &pos_commands_[i]));
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
        joint_names_[i], hardware_interface::HW_IF_VELOCITY,
        &vel_commands_[i]));
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
        joint_names_[i], hardware_interface::HW_IF_EFFORT, &tau_commands_[i]));
  }

  return command_interfaces;
}

hardware_interface::CallbackReturn OpenArm_v10HW::on_activate(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  RCLCPP_INFO(rclcpp::get_logger("OpenArm_v10HW"), "Activating OpenArm V10...");
  openarm_->set_callback_mode_all(openarm::damiao_motor::CallbackMode::STATE);
  openarm_->enable_all();
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  openarm_->recv_all();

  // Return to zero position
  return_to_zero();

  RCLCPP_INFO(rclcpp::get_logger("OpenArm_v10HW"), "OpenArm V10 activated");
  return CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn OpenArm_v10HW::on_deactivate(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  RCLCPP_INFO(rclcpp::get_logger("OpenArm_v10HW"),
              "Deactivating OpenArm V10...");

  // Disable all motors (like full_arm.cpp exit)
  openarm_->disable_all();
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  openarm_->recv_all();

  RCLCPP_INFO(rclcpp::get_logger("OpenArm_v10HW"), "OpenArm V10 deactivated");
  return CallbackReturn::SUCCESS;
}

hardware_interface::return_type OpenArm_v10HW::read(
    const rclcpp::Time& /*time*/, const rclcpp::Duration& /*period*/) {
  // Receive all motor states
  openarm_->refresh_all();
  openarm_->recv_all();

  // Read arm joint states
  const auto& arm_motors = openarm_->get_arm().get_motors();
  for (size_t i = 0; i < ARM_DOF && i < arm_motors.size(); ++i) {
    pos_states_[i] = arm_motors[i].get_position();
    vel_states_[i] = arm_motors[i].get_velocity();
    tau_states_[i] = arm_motors[i].get_torque();
  }

  // Read gripper state if enabled
  if (hand_ && joint_names_.size() > ARM_DOF) {
    const auto& gripper_motors = openarm_->get_gripper().get_motors();
    if (!gripper_motors.empty()) {
      // TODO the mappings are approximates
      // Convert motor position (radians) to joint value (0-0.044m)
      double motor_pos = gripper_motors[0].get_position();
      pos_states_[ARM_DOF] = motor_radians_to_joint(motor_pos);

      // Unimplemented: Velocity and torque mapping
      vel_states_[ARM_DOF] = 0;  // gripper_motors[0].get_velocity();
      tau_states_[ARM_DOF] = 0;  // gripper_motors[0].get_torque();
    }
  }

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type OpenArm_v10HW::write(
    const rclcpp::Time& /*time*/, const rclcpp::Duration& /*period*/) {
  // Compute gravity-compensated torques if enabled
  std::vector<double> tau_feedforward(joint_names_.size(), 0.0);

  if (use_gravity_compensation_) {
    // Build joint position and velocity vectors for all controlled joints
    Eigen::VectorXd q(model_.nq);
    Eigen::VectorXd v(model_.nv);
    q.setZero();
    v.setZero();

    // Map hardware joint positions and velocities to Pinocchio model
    // configuration
    for (size_t i = 0; i < joint_names_.size(); ++i) {
      const auto& joint_name = joint_names_[i];
      if (model_.existJointName(joint_name)) {
        auto joint_id = model_.getJointId(joint_name);
        auto q_idx = model_.idx_qs[joint_id];
        auto v_idx = model_.idx_vs[joint_id];
        q[q_idx] = pos_states_[i];
        v[v_idx] = vel_states_[i];
      }
    }

    // Compute gravity and velocity-dependent torques via RNEA (includes
    // Coriolis/centrifugal)
    Eigen::VectorXd tau_gravity = compute_gravity_torques(q, v);

    // Map gravity torques back to hardware joint order
    for (size_t i = 0; i < joint_names_.size(); ++i) {
      const auto& joint_name = joint_names_[i];
      if (model_.existJointName(joint_name)) {
        auto joint_id = model_.getJointId(joint_name);
        auto joint_idx = model_.idx_vs[joint_id];
        tau_feedforward[i] = tau_gravity[joint_idx];
      }
    }
  }

  // Control arm motors with MIT control
  std::vector<openarm::damiao_motor::MITParam> arm_params;
  for (size_t i = 0; i < ARM_DOF; ++i) {
    double tau_pd = tau_commands_[i];  // Use commanded torque from controller
    double tau_cmd = tau_feedforward[i] + tau_pd;

    arm_params.push_back(
        {kp_[i], kd_[i], pos_commands_[i], vel_commands_[i], tau_cmd});
  }
  openarm_->get_arm().mit_control_all(arm_params);

  // Control gripper if enabled
  if (hand_ && joint_names_.size() > ARM_DOF) {
    // TODO the true mappings are unimplemented.
    double motor_command = joint_to_motor_radians(pos_commands_[ARM_DOF]);
    openarm_->get_gripper().mit_control_all(
        {{GRIPPER_KP, GRIPPER_KD, motor_command, 0, 0}});
  }

  openarm_->recv_all(1000);
  return hardware_interface::return_type::OK;
}

void OpenArm_v10HW::return_to_zero() {
  RCLCPP_INFO(rclcpp::get_logger("OpenArm_v10HW"),
              "Returning to zero position...");

  // Return arm to zero with MIT control
  std::vector<openarm::damiao_motor::MITParam> arm_params;
  for (size_t i = 0; i < ARM_DOF; ++i) {
    arm_params.push_back({kp_[i], kd_[i], 0.0, 0.0, 0.0});
  }
  openarm_->get_arm().mit_control_all(arm_params);

  // Return gripper to zero if enabled
  if (hand_) {
    openarm_->get_gripper().mit_control_all(
        {{GRIPPER_KP, GRIPPER_KD, GRIPPER_JOINT_0_POSITION, 0.0, 0.0}});
  }
  std::this_thread::sleep_for(std::chrono::microseconds(1000));
  openarm_->recv_all();
}

// Gripper mapping helper functions
double OpenArm_v10HW::joint_to_motor_radians(double joint_value) {
  // Joint 0=closed -> motor 0 rad, Joint 0.044=open -> motor -1.0472 rad
  return (joint_value / GRIPPER_JOINT_0_POSITION) *
         GRIPPER_MOTOR_1_RADIANS;  // Scale from 0-0.044 to 0 to -1.0472
}

double OpenArm_v10HW::motor_radians_to_joint(double motor_radians) {
  // Motor 0 rad=closed -> joint 0, Motor -1.0472 rad=open -> joint 0.044
  return GRIPPER_JOINT_0_POSITION *
         (motor_radians /
          GRIPPER_MOTOR_1_RADIANS);  // Scale from 0 to -1.0472 to 0-0.044
}

// URDF loading and Pinocchio initialization
bool OpenArm_v10HW::load_urdf_and_initialize_pinocchio() {
  // If robot_description is not in hardware parameters, try to get it from
  // parameter server
  if (robot_description_.empty()) {
    RCLCPP_WARN(rclcpp::get_logger("OpenArm_v10HW"),
                "robot_description not provided in hardware parameters, "
                "attempting to read from /robot_description parameter...");

    // Create a temporary node to read the parameter
    auto node = rclcpp::Node::make_shared("openarm_hw_urdf_loader");
    if (!node->has_parameter("robot_description")) {
      node->declare_parameter<std::string>("robot_description", "");
    }

    try {
      robot_description_ = node->get_parameter("robot_description").as_string();
    } catch (const std::exception& e) {
      RCLCPP_ERROR(rclcpp::get_logger("OpenArm_v10HW"),
                   "Failed to read /robot_description parameter: %s", e.what());
      return false;
    }
  }

  if (robot_description_.empty()) {
    RCLCPP_ERROR(rclcpp::get_logger("OpenArm_v10HW"),
                 "URDF is empty! Please ensure /robot_description parameter is "
                 "published or provide it in hardware parameters.");
    return false;
  }

  RCLCPP_INFO(rclcpp::get_logger("OpenArm_v10HW"),
              "URDF loaded successfully (%zu bytes)",
              robot_description_.size());

  // Build Pinocchio model from URDF XML string
  try {
    pinocchio::urdf::buildModelFromXML(robot_description_, model_);
    data_ = pinocchio::Data(model_);

    // Set gravity vector: [0, 0, -9.81] m/s^2
    model_.gravity.linear(Eigen::Vector3d(0.0, 0.0, -9.81));

    RCLCPP_INFO(rclcpp::get_logger("OpenArm_v10HW"),
                "Pinocchio model initialized with %d DOF, %zu joints",
                model_.nv, model_.joints.size());
  } catch (const std::exception& e) {
    RCLCPP_ERROR(rclcpp::get_logger("OpenArm_v10HW"),
                 "Failed to build Pinocchio model from URDF: %s", e.what());
    return false;
  }

  // Validate that all hardware joints exist in the URDF model
  for (const auto& joint_name : joint_names_) {
    if (!model_.existJointName(joint_name)) {
      RCLCPP_ERROR(
          rclcpp::get_logger("OpenArm_v10HW"),
          "Joint '%s' from hardware configuration not found in URDF model",
          joint_name.c_str());
      return false;
    }
  }

  RCLCPP_INFO(rclcpp::get_logger("OpenArm_v10HW"),
              "All %zu hardware joints validated in URDF model",
              joint_names_.size());
  return true;
}

// Compute gravity torques using Pinocchio RNEA
Eigen::VectorXd OpenArm_v10HW::compute_gravity_torques(
    const Eigen::VectorXd& q, const Eigen::VectorXd& v) {
  // RNEA with actual velocities and zero acceleration gives:
  // tau = C(q, q_dot)q_dot + g(q)
  // This includes Coriolis/centrifugal forces (velocity-dependent) and gravity
  Eigen::VectorXd a = Eigen::VectorXd::Zero(model_.nv);

  return pinocchio::rnea(model_, data_, q, v, a);
}

}  // namespace openarm_hardware

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(openarm_hardware::OpenArm_v10HW,
                       hardware_interface::SystemInterface)
