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

#include <algorithm>
#include <cctype>
#include <chrono>
#include <limits>
#include <thread>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/logging.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/bool.hpp"

namespace openarm_hardware {

OpenArm_v10HW::OpenArm_v10HW() = default;

static constexpr double GRIPPER_METERS_PER_RAD = 0.0810; 

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

  // Parse torque limiting parameters (default: no limits if not specified)
  max_torque_.resize(ARM_DOF, std::numeric_limits<double>::infinity());
  max_pos_error_.resize(ARM_DOF, std::numeric_limits<double>::infinity());
  max_vel_error_.resize(ARM_DOF, std::numeric_limits<double>::infinity());

  // Parse max_torque (can be single value or comma-separated per joint)
  it = info.hardware_parameters.find("max_torque");
  if (it != info.hardware_parameters.end()) {
    std::string value = it->second;
    // Try to parse as comma-separated list
    size_t pos = 0;
    size_t idx = 0;
    while (pos < value.length() && idx < ARM_DOF) {
      size_t next_pos = value.find(',', pos);
      std::string token = (next_pos == std::string::npos) 
                          ? value.substr(pos) 
                          : value.substr(pos, next_pos - pos);
      max_torque_[idx] = std::stod(token);
      pos = (next_pos == std::string::npos) ? value.length() : next_pos + 1;
      idx++;
    }
    // If only one value provided, use it for all joints
    if (idx == 1) {
      std::fill(max_torque_.begin(), max_torque_.end(), max_torque_[0]);
    }
  }

  // Parse max_pos_error (can be single value or comma-separated per joint)
  it = info.hardware_parameters.find("max_pos_error");
  if (it != info.hardware_parameters.end()) {
    std::string value = it->second;
    size_t pos = 0;
    size_t idx = 0;
    while (pos < value.length() && idx < ARM_DOF) {
      size_t next_pos = value.find(',', pos);
      std::string token = (next_pos == std::string::npos) 
                          ? value.substr(pos) 
                          : value.substr(pos, next_pos - pos);
      max_pos_error_[idx] = std::stod(token);
      pos = (next_pos == std::string::npos) ? value.length() : next_pos + 1;
      idx++;
    }
    if (idx == 1) {
      std::fill(max_pos_error_.begin(), max_pos_error_.end(), max_pos_error_[0]);
    }
  }

  // Parse max_vel_error (can be single value or comma-separated per joint)
  it = info.hardware_parameters.find("max_vel_error");
  if (it != info.hardware_parameters.end()) {
    std::string value = it->second;
    size_t pos = 0;
    size_t idx = 0;
    while (pos < value.length() && idx < ARM_DOF) {
      size_t next_pos = value.find(',', pos);
      std::string token = (next_pos == std::string::npos) 
                          ? value.substr(pos) 
                          : value.substr(pos, next_pos - pos);
      max_vel_error_[idx] = std::stod(token);
      pos = (next_pos == std::string::npos) ? value.length() : next_pos + 1;
      idx++;
    }
    if (idx == 1) {
      std::fill(max_vel_error_.begin(), max_vel_error_.end(), max_vel_error_[0]);
    }
  }

  // Parse high pass filter cutoff frequency (default: 0.1 Hz)
  it = info.hardware_parameters.find("hpf_cutoff_freq");
  hpf_cutoff_freq_ = (it != info.hardware_parameters.end()) 
      ? std::stod(it->second) 
      : 0.5;  // Default 0.1 Hz

  // Parse external torque threshold (default: 0.5 Nm)
  it = info.hardware_parameters.find("external_torque_threshold");
  external_torque_threshold_ = (it != info.hardware_parameters.end()) 
      ? std::stod(it->second) 
      : 0.5;  // Default 0.5 Nm

  RCLCPP_INFO(rclcpp::get_logger("OpenArm_v10HW"),
              "Configuration: CAN=%s, arm_prefix=%s, hand=%s, can_fd=%s",
              can_interface_.c_str(), arm_prefix_.c_str(),
              hand_ ? "enabled" : "disabled", can_fd_ ? "enabled" : "disabled");
  
  // Log torque limiting configuration
  bool has_limits = false;
  for (size_t i = 0; i < ARM_DOF; ++i) {
    if (max_torque_[i] != std::numeric_limits<double>::infinity() ||
        max_pos_error_[i] != std::numeric_limits<double>::infinity() ||
        max_vel_error_[i] != std::numeric_limits<double>::infinity()) {
      has_limits = true;
      break;
    }
  }
  if (has_limits) {
    RCLCPP_INFO(rclcpp::get_logger("OpenArm_v10HW"),
                "Torque limits enabled: max_torque=%.2f Nm, max_pos_error=%.3f rad, max_vel_error=%.2f rad/s",
                max_torque_[0], max_pos_error_[0], max_vel_error_[0]);
  } else {
    RCLCPP_INFO(rclcpp::get_logger("OpenArm_v10HW"), "Torque limits: disabled (no limits)");
  }
  
  return true;
}

void OpenArm_v10HW::generate_joint_names() {
  joint_names_.clear();
  // TODO: read from urdf properly and sort in the future.
  // Currently, the joint names are hardcoded for order consistency to align
  // with hardware. Generate arm joint names: openarm_{arm_prefix}joint{N}
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
              "Generated %zu joint names for arm prefix '%s'",
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

  // Initialize high pass filter state vectors
  tau_filtered_.resize(total_joints, 0.0);
  tau_prev_.resize(total_joints, 0.0);
  hpf_alpha_ = 0.0;  // Will be calculated in read() based on period

  // Initialize ROS2 node for publishing
  std::string node_name = "openarm_hardware";
  if (!arm_prefix_.empty()) {
    node_name += "_" + arm_prefix_;
  }
  node_ = rclcpp::Node::make_shared(node_name);

  // Create publishers for filtered torque and external torque detection
  std::string filtered_torque_topic = arm_prefix_.empty() 
      ? "filtered_joint_torques" 
      : arm_prefix_ + "filtered_joint_torques";
  std::string external_torque_topic = arm_prefix_.empty() 
      ? "external_torque_detected" 
      : arm_prefix_ + "external_torque_detected";

  filtered_torque_pub_ = node_->create_publisher<sensor_msgs::msg::JointState>(
      filtered_torque_topic, 10);
  external_torque_pub_ = node_->create_publisher<std_msgs::msg::Bool>(
      external_torque_topic, 10);

  RCLCPP_INFO(rclcpp::get_logger("OpenArm_v10HW"),
              "OpenArm V10 Simple HW initialized successfully");
  RCLCPP_INFO(rclcpp::get_logger("OpenArm_v10HW"),
              "High pass filter: cutoff_freq=%.3f Hz, external_torque_threshold=%.3f Nm",
              hpf_cutoff_freq_, external_torque_threshold_);
  RCLCPP_INFO(rclcpp::get_logger("OpenArm_v10HW"),
              "Publishing filtered torque to: %s", filtered_torque_topic.c_str());
  RCLCPP_INFO(rclcpp::get_logger("OpenArm_v10HW"),
              "Publishing external torque detection to: %s", external_torque_topic.c_str());

  return CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn OpenArm_v10HW::on_configure(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  // Set callback mode to ignore during configuration
  openarm_->refresh_all();
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  openarm_->recv_all();

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

  set_current_pose();

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
    const rclcpp::Time& time, const rclcpp::Duration& period) {
  // Receive all motor states
  openarm_->refresh_all();
  openarm_->recv_all();

  // Calculate high pass filter coefficient based on actual period
  const double dt = period.seconds();
  if (dt > 0.0 && hpf_cutoff_freq_ > 0.0) {
    // First-order high pass filter: alpha = dt * cutoff / (1 + dt * cutoff)
    hpf_alpha_ = (dt * hpf_cutoff_freq_) / (1.0 + dt * hpf_cutoff_freq_);
  } else {
    hpf_alpha_ = 0.0;  // No filtering if invalid period
  }

  // Track maximum filtered torque for collision detection
  double max_filtered_torque = 0.0;

  // Read arm joint states
  const auto& arm_motors = openarm_->get_arm().get_motors();
  for (size_t i = 0; i < ARM_DOF && i < arm_motors.size(); ++i) {
    pos_states_[i] = arm_motors[i].get_position();
    vel_states_[i] = arm_motors[i].get_velocity();
    
    // Read raw torque
    double tau_raw = arm_motors[i].get_torque();
    
    // Update state interface with raw torque (for /joint_states topic)
    tau_states_[i] = tau_raw;
    
    // Apply high pass filter: y[n] = alpha * (x[n] - x[n-1]) + (1-alpha) * y[n-1]
    // This removes DC/low-frequency components (gravity, static loads)
    if (hpf_alpha_ > 0.0) {
      tau_filtered_[i] = hpf_alpha_ * (tau_raw - tau_prev_[i]) + 
                         (1.0 - hpf_alpha_) * tau_filtered_[i];
      tau_prev_[i] = tau_raw;
    } else {
      // No filtering on first call or if period is invalid
      tau_filtered_[i] = tau_raw;
      tau_prev_[i] = tau_raw;
    }
    
    // Track maximum absolute filtered torque for collision detection
    double abs_filtered = std::abs(tau_filtered_[i]);
    if (abs_filtered > max_filtered_torque) {
      max_filtered_torque = abs_filtered;
    }
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
      
      // Read raw gripper torque if available
      double tau_raw = 0;  // gripper_motors[0].get_torque(); // Uncomment when available
      
      // Update state interface with raw torque (for /joint_states topic)
      tau_states_[ARM_DOF] = tau_raw;
      
      // Apply filter to gripper torque if available
      if (hpf_alpha_ > 0.0) {
        tau_filtered_[ARM_DOF] = hpf_alpha_ * (tau_raw - tau_prev_[ARM_DOF]) + 
                                 (1.0 - hpf_alpha_) * tau_filtered_[ARM_DOF];
        tau_prev_[ARM_DOF] = tau_raw;
      } else {
        tau_filtered_[ARM_DOF] = tau_raw;
        tau_prev_[ARM_DOF] = tau_raw;
      }
      
      // Check gripper torque for collision
      double abs_filtered = std::abs(tau_filtered_[ARM_DOF]);
      if (abs_filtered > max_filtered_torque) {
        max_filtered_torque = abs_filtered;
      }
    }
  }

  // Detect external torque (collision) if filtered torque exceeds threshold
  bool external_torque_detected = (max_filtered_torque > external_torque_threshold_);

  // Publish filtered torque as JointState message
  if (filtered_torque_pub_ && node_) {
    sensor_msgs::msg::JointState msg;
    msg.header.stamp = time;
    msg.header.frame_id = arm_prefix_.empty() ? "base_link" : arm_prefix_ + "base_link";
    msg.name = joint_names_;
    msg.effort.resize(joint_names_.size());
    
    // Copy filtered torques
    for (size_t i = 0; i < joint_names_.size() && i < tau_filtered_.size(); ++i) {
      msg.effort[i] = tau_filtered_[i];
    }
    
    filtered_torque_pub_->publish(msg);
    }

  // Publish external torque detection (collision detection)
  if (external_torque_pub_ && node_) {
    std_msgs::msg::Bool msg;
    msg.data = external_torque_detected;
    external_torque_pub_->publish(msg);
  }

  // Process any ROS2 callbacks (if needed)
  rclcpp::spin_some(node_);

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type OpenArm_v10HW::write(
    const rclcpp::Time& /*time*/, const rclcpp::Duration& /*period*/) {
  // Control arm motors with MIT control
  std::vector<openarm::damiao_motor::MITParam> arm_params;
  for (size_t i = 0; i < ARM_DOF; ++i) {
    // Limit position error (distance from current position)
    double pos_error = pos_commands_[i] - pos_states_[i];
    double clamped_pos_error = std::clamp(pos_error, -max_pos_error_[i], max_pos_error_[i]);
    double clamped_pos_cmd = pos_states_[i] + clamped_pos_error;

    // Limit velocity error (velocity derivative)
    double vel_error = vel_commands_[i] - vel_states_[i];
    double clamped_vel_error = std::clamp(vel_error, -max_vel_error_[i], max_vel_error_[i]);
    double clamped_vel_cmd = vel_states_[i] + clamped_vel_error;

    // Limit feedforward torque
    double clamped_tau = std::clamp(tau_commands_[i], -max_torque_[i], max_torque_[i]);

    arm_params.push_back({DEFAULT_KP[i], DEFAULT_KD[i], clamped_pos_cmd,
                          clamped_vel_cmd, clamped_tau});
  }
  openarm_->get_arm().mit_control_all(arm_params);
  // Control gripper if enabled
  if (hand_ && joint_names_.size() > ARM_DOF) {
    // TODO the true mappings are unimplemented.
    double motor_command = joint_to_motor_radians(pos_commands_[ARM_DOF]);
    openarm_->get_gripper().mit_control_all(
        {{GRIPPER_DEFAULT_KP, GRIPPER_DEFAULT_KD, motor_command, 0, 0}});
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
    arm_params.push_back({DEFAULT_KP[i], DEFAULT_KD[i], 0.0, 0.0, 0.0});
  }
  openarm_->get_arm().mit_control_all(arm_params);

  // Return gripper to zero if enabled
  if (hand_) {
    openarm_->get_gripper().mit_control_all(
        {{GRIPPER_DEFAULT_KP, GRIPPER_DEFAULT_KD, GRIPPER_JOINT_0_POSITION, 0.0,
          0.0}});
  }
  std::this_thread::sleep_for(std::chrono::microseconds(1000));
  openarm_->recv_all();
}

void OpenArm_v10HW::set_current_pose() {
  RCLCPP_INFO(rclcpp::get_logger("OpenArm_v10HW"),
            "Seeting current position...");
  openarm_->refresh_all();
  openarm_->recv_all();

  const auto& arm_motors = openarm_->get_arm().get_motors();
  const size_t n = std::min<std::size_t>(ARM_DOF, arm_motors.size());

  // Set the current positions and zero the velocities
  for (size_t i = 0; i < n; ++i) {
    const double q   = arm_motors[i].get_position();
    const double dq  = arm_motors[i].get_velocity();
    const double tau = arm_motors[i].get_torque();

    pos_states_[i] = q;
    vel_states_[i] = dq;
    tau_states_[i] = tau;

    pos_commands_[i] = q;
    vel_commands_[i] = 0.0;
    tau_commands_[i] = 0.0;
  }

  // check if gripper exists and set current positions 
  if (hand_ && joint_names_.size() > ARM_DOF) {
    const auto& gripper_motors = openarm_->get_gripper().get_motors();
    if (!gripper_motors.empty()) {
      const double motor_pos = gripper_motors[0].get_position();
      const double joint_pos = motor_radians_to_joint(motor_pos);

      pos_states_[ARM_DOF] = joint_pos;
      vel_states_[ARM_DOF] = 0.0;
      tau_states_[ARM_DOF] = 0.0;

      pos_commands_[ARM_DOF] = joint_pos;
      vel_commands_[ARM_DOF] = 0.0;
      tau_commands_[ARM_DOF] = 0.0;
    }
  }
}

// Gripper mapping helper functions
double OpenArm_v10HW::joint_to_motor_radians(double joint_value_m) {
  // convert meters -> motor radians
  return joint_value_m / GRIPPER_METERS_PER_RAD;
}

double OpenArm_v10HW::motor_radians_to_joint(double motor_radians) {
  // convert motor radians -> meters for ROS
  return motor_radians * GRIPPER_METERS_PER_RAD;
}

}  // namespace openarm_hardware

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(openarm_hardware::OpenArm_v10HW,
                       hardware_interface::SystemInterface)
