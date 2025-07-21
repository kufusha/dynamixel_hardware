// Copyright 2020 Yutaka Kondo <yutaka.kondo@youtalk.jp>
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

#ifndef DYNAMIXEL_HARDWARE__DYNAMIXEL_HARDWARE_HPP_
#define DYNAMIXEL_HARDWARE__DYNAMIXEL_HARDWARE_HPP_

#include <dynamixel_workbench_toolbox/dynamixel_workbench.h>

#include <map>
#include <vector>

#include <hardware_interface/handle.hpp>
#include <hardware_interface/hardware_info.hpp>
#include <hardware_interface/system_interface.hpp>
#include <rclcpp_lifecycle/state.hpp>
#include <rclcpp/rclcpp.hpp>
// #include <std_srvs/srv/set_bool.hpp>

#include "dynamixel_hardware/visiblity_control.h"
#include "rclcpp/macros.hpp"

using hardware_interface::CallbackReturn;
using hardware_interface::return_type;

namespace dynamixel_hardware
{
struct JointValue
{
  double position{0.0};
  double velocity{0.0};
  double effort{0.0};
};

struct CalibrationData
{
  std::string external_type{"none"};
  
  // Simple 2-point calibration for potentiometers
  double angle_min{-3.14};    // Minimum angle [rad]
  double adc_min{1024.0};     // ADC value at minimum angle
  double angle_max{3.14};     // Maximum angle [rad] 
  double adc_max{3072.0};     // ADC value at maximum angle
  
  // Dual limit sensor settings (for ID:7)
  double high_limit{3.14};         // 正側リミット角度
  double low_limit{-3.14};         // 負側リミット角度
  int external_io_high{1};         // High limit用外部I/O番号
  int external_io_low{2};          // Low limit用外部I/O番号
  double safety_margin{0.1};
};

struct Joint
{
  JointValue state{};
  JointValue command{};
  JointValue prev_command{};
};

enum class ControlMode
{
  Position,
  Velocity,
};

class DynamixelHardware : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(DynamixelHardware)

  DYNAMIXEL_HARDWARE_PUBLIC
  CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override;

  DYNAMIXEL_HARDWARE_PUBLIC
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  DYNAMIXEL_HARDWARE_PUBLIC
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  DYNAMIXEL_HARDWARE_PUBLIC
  CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;

  DYNAMIXEL_HARDWARE_PUBLIC
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

  DYNAMIXEL_HARDWARE_PUBLIC
  return_type read(const rclcpp::Time & time, const rclcpp::Duration & period) override;

  DYNAMIXEL_HARDWARE_PUBLIC
  return_type write(const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  return_type enable_torque(const bool enabled);

  return_type set_control_mode(const ControlMode & mode, const bool force_set = false);

  void set_operating_modes();

  return_type reset_command();

  CallbackReturn set_joint_positions();
  CallbackReturn set_joint_velocities();
  CallbackReturn set_joint_params();
  
  // Calibration functions
  double convert_external_sensor_to_angle(int joint_index, int32_t raw_value);
  bool check_proximity_limit(int joint_index, double target_angle);
  bool read_external_io(int io_number);
  
  // Multiturn restoration
  void restore_multiturn_from_potential();
  int calculate_turn_offset(double true_angle, double single_turn_angle);
  
  // Service callbacks
  // void torque_enable_service_callback(
  //   const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
  //   std::shared_ptr<std_srvs::srv::SetBool::Response> response);

  DynamixelWorkbench dynamixel_workbench_;
  std::map<const char * const, const ControlItem *> control_items_;
  std::vector<Joint> joints_;
  std::vector<uint8_t> joint_ids_;
  std::vector<double> mechanical_reductions_;
  std::vector<std::string> external_types_;
  std::vector<CalibrationData> calibration_data_;
  bool torque_enabled_{false};
  ControlMode control_mode_{ControlMode::Position};
  bool mode_changed_{false};
  bool use_dummy_{false};
  bool is_external_pos_{false};
  bool multiturn_restored_{false};
  
  // ROS2 services
  // rclcpp::Node::SharedPtr node_;
  // rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr torque_service_;
};
}  // namespace dynamixel_hardware

#endif  // DYNAMIXEL_HARDWARE__DYNAMIXEL_HARDWARE_HPP_
