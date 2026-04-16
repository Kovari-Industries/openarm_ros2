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

#pragma once

#include <chrono>
#include <memory>
#include <string>
#include <vector>

#include <openarm/can/socket/openarm.hpp>
#include <openarm/damiao_motor/dm_motor_constants.hpp>

#include <kdl/tree.hpp>
#include <kdl/chain.hpp>
#include <kdl/jntarray.hpp>
#include <kdl/jntspaceinertiamatrix.hpp>
#include <kdl/chaindynparam.hpp>
#include <kdl_parser/kdl_parser.hpp>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "openarm_hardware/visibility_control.h"
#include "rclcpp/macros.hpp"
#include "rclcpp_lifecycle/state.hpp"

namespace openarm_hardware {

/**
 * @brief OpenArm V10 Hardware Interface with internal KDL dynamics
 *
 * - Talks to the motors via OpenArm CAN API.
 * - Exposes ROS2 control state/command interfaces.
 * - Owns a KDL chain for the 7-DOF arm and can compute gravity torques
 *   in the hardware layer (gravity compensation).
 */
class OpenArm_v10KDLHW : public hardware_interface::SystemInterface {
 public:
  OpenArm_v10KDLHW();

  TEMPLATES__ROS2_CONTROL__VISIBILITY_PUBLIC
  hardware_interface::CallbackReturn on_init(
      const hardware_interface::HardwareInfo& info) override;

  TEMPLATES__ROS2_CONTROL__VISIBILITY_PUBLIC
  hardware_interface::CallbackReturn on_configure(
      const rclcpp_lifecycle::State& previous_state) override;

  TEMPLATES__ROS2_CONTROL__VISIBILITY_PUBLIC
  std::vector<hardware_interface::StateInterface> export_state_interfaces()
      override;

  TEMPLATES__ROS2_CONTROL__VISIBILITY_PUBLIC
  std::vector<hardware_interface::CommandInterface> export_command_interfaces()
      override;

  TEMPLATES__ROS2_CONTROL__VISIBILITY_PUBLIC
  hardware_interface::CallbackReturn on_activate(
      const rclcpp_lifecycle::State& previous_state) override;

  TEMPLATES__ROS2_CONTROL__VISIBILITY_PUBLIC
  hardware_interface::CallbackReturn on_deactivate(
      const rclcpp_lifecycle::State& previous_state) override;

  TEMPLATES__ROS2_CONTROL__VISIBILITY_PUBLIC
  hardware_interface::return_type read(const rclcpp::Time& time,
                                       const rclcpp::Duration& period) override;

  TEMPLATES__ROS2_CONTROL__VISIBILITY_PUBLIC
  hardware_interface::return_type write(
      const rclcpp::Time& time, const rclcpp::Duration& period) override;

 private:
  static constexpr size_t ARM_DOF = 7;
  static constexpr bool ENABLE_GRIPPER = true;

  const std::vector<openarm::damiao_motor::MotorType> DEFAULT_MOTOR_TYPES = {
      openarm::damiao_motor::MotorType::DM8009,  // Joint 1
      openarm::damiao_motor::MotorType::DM8009,  // Joint 2
      openarm::damiao_motor::MotorType::DM4340,  // Joint 3
      openarm::damiao_motor::MotorType::DM4340,  // Joint 4
      openarm::damiao_motor::MotorType::DM4310,  // Joint 5
      openarm::damiao_motor::MotorType::DM4310,  // Joint 6
      openarm::damiao_motor::MotorType::DM4310   // Joint 7
  };

  const std::vector<uint32_t> DEFAULT_SEND_CAN_IDS = {0x01, 0x02, 0x03, 0x04,
                                                      0x05, 0x06, 0x07};
  const std::vector<uint32_t> DEFAULT_RECV_CAN_IDS = {0x11, 0x12, 0x13, 0x14,
                                                      0x15, 0x16, 0x17};

  const openarm::damiao_motor::MotorType DEFAULT_GRIPPER_MOTOR_TYPE =
      openarm::damiao_motor::MotorType::DM4310;
  const uint32_t DEFAULT_GRIPPER_SEND_CAN_ID = 0x08;
  const uint32_t DEFAULT_GRIPPER_RECV_CAN_ID = 0x18;

  const std::vector<double> DEFAULT_KP = {45.0, 55.0, 25.0, 25.0,
                                          5.0,  15.0, 15.0, 5.0};
  const std::vector<double> DEFAULT_KD = {5.5,  7.5,  1.5,  2.25,
                                          0.9,  1.0,  0.8,  0.1};                                      

  const double GRIPPER_JOINT_0_POSITION = 0.044;
  const double GRIPPER_JOINT_1_POSITION = 0.0;
  const double GRIPPER_MOTOR_0_RADIANS = 0.0;
  const double GRIPPER_MOTOR_1_RADIANS = -1.0472;
  const double GRIPPER_DEFAULT_KP = 5.0;
  const double GRIPPER_DEFAULT_KD = 0.1;

  std::string can_interface_;
  std::string arm_prefix_;
  bool hand_;
  bool can_fd_;
  std::vector<uint32_t> arm_send_can_ids_;
  std::vector<uint32_t> arm_recv_can_ids_;
  uint32_t gripper_send_can_id_;
  uint32_t gripper_recv_can_id_;
  std::vector<uint32_t> startup_motor_ids_;

  bool gravity_compensation_enabled_{true};
  std::string root_link_name_;
  std::string tip_link_name_;

  std::unique_ptr<openarm::can::socket::OpenArm> openarm_;

  std::vector<std::string> joint_names_;

  std::vector<double> pos_commands_;
  std::vector<double> vel_commands_;
  std::vector<double> tau_commands_;
  std::vector<double> pos_states_;
  std::vector<double> vel_states_;
  std::vector<double> tau_states_;

  bool kdl_initialized_{false};

  KDL::Tree kdl_tree_;
  KDL::Chain kdl_chain_;

  KDL::Vector gravity_vector_{0.0, 0.0, -9.81};

  std::unique_ptr<KDL::ChainDynParam> kdl_dyn_;

  KDL::JntArray kdl_q_;
  KDL::JntArray kdl_qdot_;
  KDL::JntArray kdl_G_;

  void return_to_zero();
  bool parse_config(const hardware_interface::HardwareInfo& info);
  void configure_can_ids();
  void generate_joint_names();
  void set_current_pose();

  bool init_kdl_from_urdf(const hardware_interface::HardwareInfo& info);

  void update_kdl_state_from_joint_states();

  bool compute_gravity_torques(std::vector<double>& out_gravity);
  bool send_startup_can_sequence();
  bool send_can_frame(uint32_t can_id,
                      const std::initializer_list<uint8_t>& data);

  double joint_to_motor_radians(double joint_value);
  double motor_radians_to_joint(double motor_radians);
};

}
