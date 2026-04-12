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

#include "openarm_hardware/v10_hardware.hpp"

#include <algorithm>
#include <cerrno>
#include <cctype>
#include <chrono>
#include <cstdio>
#include <cstring>
#include <thread>
#include <vector>

#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/logging.hpp"
#include "rclcpp/rclcpp.hpp"

namespace openarm_hardware {

OpenArm_v10KDLHW::OpenArm_v10KDLHW() = default;

static constexpr double GRIPPER_METERS_PER_RAD = 0.0810; 

void OpenArm_v10KDLHW::configure_can_ids() {
  arm_send_can_ids_ = DEFAULT_SEND_CAN_IDS;
  arm_recv_can_ids_ = DEFAULT_RECV_CAN_IDS;
  gripper_send_can_id_ = DEFAULT_GRIPPER_SEND_CAN_ID;
  gripper_recv_can_id_ = DEFAULT_GRIPPER_RECV_CAN_ID;
  startup_motor_ids_ = {0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08};

  if (arm_prefix_ == "right_") {
    arm_send_can_ids_ = {0x09, 0x0A, 0x0B, 0x0C, 0x0D, 0x0E, 0x0F};
    arm_recv_can_ids_ = {0x19, 0x1A, 0x1B, 0x1C, 0x1D, 0x1E, 0x1F};
    gripper_send_can_id_ = 0x10;
    gripper_recv_can_id_ = 0x20;
    startup_motor_ids_ = {0x09, 0x0A, 0x0B, 0x0C, 0x0D, 0x0E, 0x0F, 0x10};
  }
}

bool OpenArm_v10KDLHW::parse_config(const hardware_interface::HardwareInfo& info) {
  // Parse CAN interface (default: can0)
  auto it = info.hardware_parameters.find("can_interface");
  can_interface_ = (it != info.hardware_parameters.end()) ? it->second : "can0";

  // Parse arm prefix (default: empty for single arm, "left_" or "right_" for bimanual)
  it = info.hardware_parameters.find("arm_prefix");
  arm_prefix_ = (it != info.hardware_parameters.end()) ? it->second : "";

  if (arm_prefix_ == "left_") {
    gravity_vector_ = KDL::Vector(0.0, 9.81, 0.0);
  } else if (arm_prefix_ == "right_") {
    gravity_vector_ = KDL::Vector(0.0, -9.81, 0.0);
  } else {
    // single-arm / default case
    gravity_vector_ = KDL::Vector(0.0, 0.0, -9.81);
  }

  // Parse gripper enable (default: true for V10)
  it = info.hardware_parameters.find("hand");
  if (it == info.hardware_parameters.end()) {
    hand_ = true;  // Default to true for V10
  } else {
    std::string value = it->second;
    std::transform(value.begin(), value.end(), value.begin(), ::tolower);
    hand_ = (value == "true");
  }

  // Parse CAN-FD enable (default: true for V10)
  it = info.hardware_parameters.find("can_fd");
  if (it == info.hardware_parameters.end()) {
    can_fd_ = true;  // Default to true for V10
  } else {
    std::string value = it->second;
    std::transform(value.begin(), value.end(), value.begin(), ::tolower);
    can_fd_ = (value == "true");
  }

  configure_can_ids();

  // Gravity compensation toggle (default: true)
  it = info.hardware_parameters.find("gravity_compensation");
  if (it == info.hardware_parameters.end()) {
    gravity_compensation_enabled_ = true;
  } else {
    std::string value = it->second;
    std::transform(value.begin(), value.end(), value.begin(), ::tolower);
    gravity_compensation_enabled_ = (value == "true");
  }

  // Root / tip links for the KDL chain (optional overrides)
  it = info.hardware_parameters.find("kdl_root_link");
  if (it != info.hardware_parameters.end()) {
    root_link_name_ = it->second;
  } else {
    // Reasonable default: link0 of this arm, or you can choose a base link here
    root_link_name_ = "openarm_" + arm_prefix_ + "link0";
  }

  it = info.hardware_parameters.find("kdl_tip_link");
  if (it != info.hardware_parameters.end()) {
    tip_link_name_ = it->second;
  } else {
    tip_link_name_ =
        "openarm_" + arm_prefix_ + (hand_ ? "hand" : "link7");
  }

  RCLCPP_INFO(
      rclcpp::get_logger("OpenArm_v10KDLHW"),
      "Configuration: CAN=%s, arm_prefix=%s, hand=%s, can_fd=%s, "
      "gravity_comp=%s, kdl_root=%s, kdl_tip=%s, "
      "arm_send_ids=[0x%02X..0x%02X], arm_recv_ids=[0x%02X..0x%02X], "
      "gripper_send_id=0x%02X, gripper_recv_id=0x%02X",
      can_interface_.c_str(),
      arm_prefix_.c_str(),
      hand_ ? "enabled" : "disabled",
      can_fd_ ? "enabled" : "disabled",
      gravity_compensation_enabled_ ? "enabled" : "disabled",
      root_link_name_.c_str(),
      tip_link_name_.c_str(),
      arm_send_can_ids_.front(),
      arm_send_can_ids_.back(),
      arm_recv_can_ids_.front(),
      arm_recv_can_ids_.back(),
      gripper_send_can_id_,
      gripper_recv_can_id_);

  return true;
}

void OpenArm_v10KDLHW::update_kdl_state_from_joint_states() {
  if (!kdl_initialized_) {
    return;
  }

  const unsigned int nj = kdl_chain_.getNrOfJoints();
  const size_t n = std::min<std::size_t>(ARM_DOF, nj);

  // We assume the first ARM_DOF entries of pos_states_/vel_states_ correspond
  // to the KDL chain joints in order (joint1..joint7).
  // If that ever changes, we'd need a name-based mapping.
  for (size_t i = 0; i < n; ++i) {
    kdl_q_(i)    = pos_states_[i];
    kdl_qdot_(i) = vel_states_[i];
  }

  // Zero any remaining joints in the chain if nj > ARM_DOF
  for (unsigned int i = static_cast<unsigned int>(n); i < nj; ++i) {
    kdl_q_(i)    = 0.0;
    kdl_qdot_(i) = 0.0;
  }
}

bool OpenArm_v10KDLHW::compute_gravity_torques(std::vector<double>& out_gravity) {
  if (!kdl_initialized_ || !kdl_dyn_) {
    return false;
  }

  const unsigned int nj = kdl_chain_.getNrOfJoints();
  if (nj == 0) {
    return false;
  }

  // Make sure our output vector is sized to at least ARM_DOF
  out_gravity.assign(ARM_DOF, 0.0);

  // Compute gravity torques in KDL joint space: G(q)
  int ret = kdl_dyn_->JntToGravity(kdl_q_, kdl_G_);
  if (ret != 0) {
    auto logger = rclcpp::get_logger("OpenArm_v10KDLHW");
    RCLCPP_WARN(logger,
                "compute_gravity_torques: JntToGravity returned %d (non-zero).",
                ret);
    return false;
  }

  // Copy the first ARM_DOF entries into out_gravity.
  // Again, we assume the KDL chain joint ordering matches joint_names_ / hardware.
  const size_t n = std::min<std::size_t>(ARM_DOF, nj);
  for (size_t i = 0; i < n; ++i) {
    out_gravity[i] = kdl_G_(i);
  }

  return true;
}

void OpenArm_v10KDLHW::generate_joint_names() {
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
    RCLCPP_INFO(rclcpp::get_logger("OpenArm_v10KDLHW"), "Added gripper joint: %s",
                gripper_joint_name.c_str());
  } else {
    RCLCPP_INFO(rclcpp::get_logger("OpenArm_v10KDLHW"),
                "Gripper joint NOT added because hand_=false");
  }

  RCLCPP_INFO(rclcpp::get_logger("OpenArm_v10KDLHW"),
              "Generated %zu joint names for arm prefix '%s'",
              joint_names_.size(), arm_prefix_.c_str());
}

hardware_interface::CallbackReturn OpenArm_v10KDLHW::on_init(
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
    RCLCPP_ERROR(rclcpp::get_logger("OpenArm_v10KDLHW"),
                 "Generated %zu joint names, expected %zu",
                 joint_names_.size(), expected_joints);
    return CallbackReturn::ERROR;
  }

  // Initialize OpenArm with configurable CAN-FD setting
  RCLCPP_INFO(rclcpp::get_logger("OpenArm_v10KDLHW"),
              "Initializing OpenArm on %s with CAN-FD %s...",
              can_interface_.c_str(),
              can_fd_ ? "enabled" : "disabled");
  openarm_ =
      std::make_unique<openarm::can::socket::OpenArm>(can_interface_, can_fd_);

  // Initialize arm motors with the CAN IDs assigned to this arm instance.
  openarm_->init_arm_motors(
      DEFAULT_MOTOR_TYPES, arm_send_can_ids_, arm_recv_can_ids_);

  // Initialize gripper if enabled
  if (hand_) {
    RCLCPP_INFO(rclcpp::get_logger("OpenArm_v10KDLHW"),
                "Initializing gripper...");
    openarm_->init_gripper_motor(
        DEFAULT_GRIPPER_MOTOR_TYPE,
        gripper_send_can_id_,
        gripper_recv_can_id_);
  }

  // Initialize state and command vectors based on generated joint count
  const size_t total_joints = joint_names_.size();
  pos_commands_.assign(total_joints, 0.0);
  vel_commands_.assign(total_joints, 0.0);
  tau_commands_.assign(total_joints, 0.0);
  pos_states_.assign(total_joints, 0.0);
  vel_states_.assign(total_joints, 0.0);
  tau_states_.assign(total_joints, 0.0);

  // Initialize KDL structures from URDF (robot_description in info.original_xml)
  if (!init_kdl_from_urdf(info)) {
    RCLCPP_ERROR(rclcpp::get_logger("OpenArm_v10KDLHW"),
                 "Failed to initialize KDL chain for OpenArm V10");
    // You can either treat this as fatal or just disable gravity_compensation
    gravity_compensation_enabled_ = false;
  }

  RCLCPP_INFO(rclcpp::get_logger("OpenArm_v10KDLHW"),
              "OpenArm V10 HW initialized successfully");
  return CallbackReturn::SUCCESS;
}

bool OpenArm_v10KDLHW::init_kdl_from_urdf(const hardware_interface::HardwareInfo& info) {
  auto logger = rclcpp::get_logger("OpenArm_v10KDLHW");

  // 1) Get URDF XML string
  std::string urdf_xml;

  // Preferred: ros2_control passes the original URDF in HardwareInfo
  if (!info.original_xml.empty()) {
    urdf_xml = info.original_xml;
  } else {
    // Fallback: look for a robot_description parameter in hardware_parameters
    auto it = info.hardware_parameters.find("robot_description");
    if (it != info.hardware_parameters.end()) {
      urdf_xml = it->second;
    }
  }

  if (urdf_xml.empty()) {
    RCLCPP_ERROR(logger,
                 "init_kdl_from_urdf: no URDF available (original_xml empty "
                 "and no robot_description parameter).");
    kdl_initialized_ = false;
    return false;
  }

  // 2) Build KDL tree from URDF
  kdl_tree_ = KDL::Tree();
  if (!kdl_parser::treeFromString(urdf_xml, kdl_tree_)) {
    RCLCPP_ERROR(logger, "init_kdl_from_urdf: failed to parse URDF into KDL tree.");
    kdl_initialized_ = false;
    return false;
  }

  RCLCPP_INFO(logger, "init_kdl_from_urdf: KDL tree constructed successfully.");

  // 3) Extract KDL chain for this arm from root_link_name_ to tip_link_name_
  kdl_chain_ = KDL::Chain();
  if (!kdl_tree_.getChain(root_link_name_, tip_link_name_, kdl_chain_)) {
    RCLCPP_ERROR(logger,
                 "init_kdl_from_urdf: failed to extract KDL chain from '%s' "
                 "to '%s'.",
                 root_link_name_.c_str(), tip_link_name_.c_str());
    kdl_initialized_ = false;
    return false;
  }

  const unsigned int nj = kdl_chain_.getNrOfJoints();
  if (nj != ARM_DOF) {
    RCLCPP_WARN(logger,
                "init_kdl_from_urdf: chain joint count (%u) != ARM_DOF (%zu). "
                "Continuing but this may indicate a mismatch.",
                nj, ARM_DOF);
  } else {
    RCLCPP_INFO(logger,
                "init_kdl_from_urdf: KDL chain has %u joints (expected %zu).",
                nj, ARM_DOF);
  }

  // 4) Initialize dynamic parameter object and joint-space arrays
  // gravity_vector_ is already initialized to (0, 0, -9.81) in the class
  kdl_dyn_ = std::make_unique<KDL::ChainDynParam>(kdl_chain_, gravity_vector_);

  kdl_q_     = KDL::JntArray(nj);
  kdl_qdot_  = KDL::JntArray(nj);
  kdl_G_     = KDL::JntArray(nj);

  // Start zeroed
  for (unsigned int i = 0; i < nj; ++i) {
    kdl_q_(i)    = 0.0;
    kdl_qdot_(i) = 0.0;
    kdl_G_(i)    = 0.0;
  }

  kdl_initialized_ = true;

  RCLCPP_INFO(logger,
              "init_kdl_from_urdf: KDL dynamics initialized (root='%s', tip='%s', "
              "gravity=[0, 0, -9.81]).",
              root_link_name_.c_str(), tip_link_name_.c_str());

  return true;
}

hardware_interface::CallbackReturn OpenArm_v10KDLHW::on_configure(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  // Set callback mode to ignore during configuration
  openarm_->refresh_all();
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  openarm_->recv_all();

  return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface>
OpenArm_v10KDLHW::export_state_interfaces() {
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
OpenArm_v10KDLHW::export_command_interfaces() {
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

hardware_interface::CallbackReturn OpenArm_v10KDLHW::on_activate(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  RCLCPP_INFO(rclcpp::get_logger("OpenArm_v10KDLHW"), "Activating OpenArm V10...");
  if (!send_startup_can_sequence()) {
    RCLCPP_ERROR(rclcpp::get_logger("OpenArm_v10KDLHW"),
                 "Failed to send OpenArm startup CAN sequence");
    return CallbackReturn::ERROR;
  }
  openarm_->set_callback_mode_all(openarm::damiao_motor::CallbackMode::STATE);
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  openarm_->recv_all();

  set_current_pose();

  RCLCPP_INFO(rclcpp::get_logger("OpenArm_v10KDLHW"), "OpenArm V10 activated");
  return CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn OpenArm_v10KDLHW::on_deactivate(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  RCLCPP_INFO(rclcpp::get_logger("OpenArm_v10KDLHW"),
              "Deactivating OpenArm V10...");

  // Disable all motors (like full_arm.cpp exit)
  openarm_->disable_all();
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  openarm_->recv_all();

  RCLCPP_INFO(rclcpp::get_logger("OpenArm_v10KDLHW"), "OpenArm V10 deactivated");
  return CallbackReturn::SUCCESS;
}

hardware_interface::return_type OpenArm_v10KDLHW::read(
    const rclcpp::Time&, const rclcpp::Duration&) 
{
  // Update hardware
  openarm_->refresh_all();
  openarm_->recv_all();

  // Read arm joint states
  const auto& arm_motors = openarm_->get_arm().get_motors();
  for (size_t i = 0; i < ARM_DOF && i < arm_motors.size(); ++i) {
    pos_states_[i] = arm_motors[i].get_position();
    vel_states_[i] = arm_motors[i].get_velocity();
    tau_states_[i] = arm_motors[i].get_torque();
  }

  // Read gripper if present
  if (hand_ && joint_names_.size() > ARM_DOF) {
    const auto& gm = openarm_->get_gripper().get_motors();
    if (!gm.empty()) {
      pos_states_[ARM_DOF] = motor_radians_to_joint(gm[0].get_position());
      vel_states_[ARM_DOF] = 0.0;
      tau_states_[ARM_DOF] = 0.0;
    }
  }

  // ----- Gravity compensation logging only -----
  if (gravity_compensation_enabled_ && kdl_initialized_) {
    update_kdl_state_from_joint_states();

    std::vector<double> g;
    if (compute_gravity_torques(g)) {
      std::ostringstream oss;
      oss << "[ ";
      for (double v : g) oss << v << " ";
      oss << "]";
      // RCLCPP_INFO(
      //     rclcpp::get_logger("OpenArm_v10KDLHW"),
      //     "Gravity torques: %s", oss.str().c_str());
    } else {
      RCLCPP_ERROR(
          rclcpp::get_logger("OpenArm_v10KDLHW"),
          "compute_gravity_torques() FAILED!");
    }
  }

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type OpenArm_v10KDLHW::write(
    const rclcpp::Time& /*time*/, const rclcpp::Duration& /*period*/) 
{
  // Optional gravity compensation (compute joint-space gravity torques)
  std::vector<double> gravity;
  if (gravity_compensation_enabled_ && kdl_initialized_) {
    // Update KDL joint state from current pos_states_
    update_kdl_state_from_joint_states();

    if (!compute_gravity_torques(gravity)) {
      RCLCPP_WARN(
          rclcpp::get_logger("OpenArm_v10KDLHW"),
          "write(): compute_gravity_torques() failed, disabling gravity feedforward for this cycle.");
      gravity.clear();
    }
  }

  std::vector<openarm::damiao_motor::MITParam> arm_params;
  arm_params.reserve(ARM_DOF);

  for (size_t i = 0; i < ARM_DOF; ++i) {
    double tau_ff = tau_commands_[i];

    if (!gravity.empty() && i < gravity.size()) {
      // RCLCPP_INFO(rclcpp::get_logger("OpenArm_v10KDLHW"),
      //         "Taking in tau_ff");
      tau_ff += gravity[i]*1;
    }

    arm_params.push_back(
        {DEFAULT_KP[i], DEFAULT_KD[i],
         pos_commands_[i],
         vel_commands_[i],
         tau_ff});
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

bool OpenArm_v10KDLHW::send_startup_can_sequence() {
  auto logger = rclcpp::get_logger("OpenArm_v10KDLHW");
  RCLCPP_INFO(logger, "Sending OpenArm startup sequence on %s...",
              can_interface_.c_str());

  for (uint32_t can_id : startup_motor_ids_) {
    if (!send_can_frame(can_id, {0xFF, 0xFF, 0xFF, 0xFF,
                                 0xFF, 0xFF, 0xFF, 0xFC})) {
      RCLCPP_ERROR(logger, "Failed enabling motor at CAN ID 0x%03X", can_id);
      return false;
    }
  }

  for (uint32_t motor_id : startup_motor_ids_) {
    if (!send_can_frame(0x7FF, {motor_id, 0x00, 0x55, 0x0A,
                                0x01, 0x00, 0x00, 0x00})) {
      RCLCPP_ERROR(logger,
                   "Failed switching motor 0x%02X to MIT control mode",
                   static_cast<unsigned int>(motor_id));
      return false;
    }
  }

  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  return true;
}

bool OpenArm_v10KDLHW::send_can_frame(
    uint32_t can_id, const std::initializer_list<uint8_t>& data) {
  if (data.size() > CAN_MAX_DLEN) {
    RCLCPP_ERROR(rclcpp::get_logger("OpenArm_v10KDLHW"),
                 "Refusing to send CAN frame with %zu bytes", data.size());
    return false;
  }

  int socket_fd = ::socket(PF_CAN, SOCK_RAW, CAN_RAW);
  if (socket_fd < 0) {
    RCLCPP_ERROR(rclcpp::get_logger("OpenArm_v10KDLHW"),
                 "socket(PF_CAN) failed on %s: %s",
                 can_interface_.c_str(), std::strerror(errno));
    return false;
  }

  ifreq ifr {};
  std::snprintf(ifr.ifr_name, IFNAMSIZ, "%s", can_interface_.c_str());
  if (::ioctl(socket_fd, SIOCGIFINDEX, &ifr) < 0) {
    RCLCPP_ERROR(rclcpp::get_logger("OpenArm_v10KDLHW"),
                 "ioctl(SIOCGIFINDEX) failed on %s: %s",
                 can_interface_.c_str(), std::strerror(errno));
    ::close(socket_fd);
    return false;
  }

  sockaddr_can addr {};
  addr.can_family = AF_CAN;
  addr.can_ifindex = ifr.ifr_ifindex;
  if (::bind(socket_fd, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) < 0) {
    RCLCPP_ERROR(rclcpp::get_logger("OpenArm_v10KDLHW"),
                 "bind() failed on %s: %s",
                 can_interface_.c_str(), std::strerror(errno));
    ::close(socket_fd);
    return false;
  }

  can_frame frame {};
  frame.can_id = can_id;
  frame.len = static_cast<__u8>(data.size());
  std::copy(data.begin(), data.end(), frame.data);

  const ssize_t bytes_written = ::write(socket_fd, &frame, sizeof(frame));
  ::close(socket_fd);

  if (bytes_written != static_cast<ssize_t>(sizeof(frame))) {
    RCLCPP_ERROR(rclcpp::get_logger("OpenArm_v10KDLHW"),
                 "write() failed for CAN ID 0x%03X on %s: %s",
                 can_id, can_interface_.c_str(), std::strerror(errno));
    return false;
  }

  return true;
}

void OpenArm_v10KDLHW::return_to_zero() {
  RCLCPP_INFO(rclcpp::get_logger("OpenArm_v10KDLHW"),
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

void OpenArm_v10KDLHW::set_current_pose() {
  RCLCPP_INFO(rclcpp::get_logger("OpenArm_v10KDLHW"),
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
double OpenArm_v10KDLHW::joint_to_motor_radians(double joint_value_m) {
  // convert meters -> motor radians
  return joint_value_m / GRIPPER_METERS_PER_RAD;
}

double OpenArm_v10KDLHW::motor_radians_to_joint(double motor_radians) {
  // convert motor radians -> meters for ROS
  return motor_radians * GRIPPER_METERS_PER_RAD;
}

}  // namespace openarm_hardware

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(openarm_hardware::OpenArm_v10KDLHW,
                       hardware_interface::SystemInterface)
