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
  
  DynamixelWorkbench dynamixel_workbench_;
  std::map<const char * const, const ControlItem *> control_items_;
  std::vector<Joint> joints_;
  std::vector<uint8_t> joint_ids_;
  std::vector<double> mechanical_reductions_;
  std::vector<std::string> external_types_;
  bool torque_enabled_{false};
  ControlMode control_mode_{ControlMode::Position};
  bool mode_changed_{false};
  bool use_dummy_{false};
  bool is_external_pos_{false};
  bool multiturn_restored_{false};
  std::vector<size_t> id_sorted_indices_;  // Mapping from ID-sorted order to original order
  
  // Backlash compensation
  std::vector<double> prev_command_positions_;  // Previous command positions for dead band filtering
  static constexpr double BACKLASH_DEAD_BAND = 0.02;  // ~1.1 degrees in radians
  
  std::vector<double> present_currents_A_;
  std::vector<double> current_limits_A_;
  std::vector<uint8_t> hw_error_bits_;
  std::vector<double>  hw_error_code_;
  std::vector<double> external_port1_;
  std::vector<double> external_port2_;
  
  std::vector<double> external_scale_rad_per_count_;
  std::vector<double> external_offset_rad_; 

  // Simulation_values when usb_dummy is ture
  double dummy_current_limit_A_ = 3.0;
  double dummy_current_slope_A_per_rad_s_ = 0.6;

  // Syncread handler
  int sr_idx_p_cur_ = -1, sr_idx_x_cur_ = -1;
  int sr_idx_p_err_ = -1, sr_idx_x_err_ = -1;

  uint16_t addr_p_cur_ = 0, len_p_cur_ = 0;
  uint16_t addr_x_cur_ = 0, len_x_cur_ = 0;
  uint16_t addr_p_err_ = 0, len_p_err_ = 0;
  uint16_t addr_x_err_ = 0, len_x_err_ = 0;

  uint16_t addr_p_pos_{0}, len_p_pos_{0};
  uint16_t addr_x_pos_{0}, len_x_pos_{0};
  uint16_t addr_p_vel_{0}, len_p_vel_{0};
  uint16_t addr_x_vel_{0}, len_x_vel_{0};

  

  // Exponential Moving Average filter for ADC noise reduction
  struct EMAFilter {
    double alpha;
    double filtered_value;
    bool initialized;
    
    EMAFilter(double alpha_val = 0.2) : alpha(alpha_val), filtered_value(0.0), initialized(false) {}
    
    double update(double new_value) {
      if (!initialized) {
        filtered_value = new_value;
        initialized = true;
      } else {
        filtered_value = alpha * new_value + (1.0 - alpha) * filtered_value;
      }
      return filtered_value;
    }
  };
  
  std::vector<EMAFilter> adc_filters_;
  
  // ROS2 services
  // rclcpp::Node::SharedPtr node_;
  // rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr torque_service_;
};
}  // namespace dynamixel_hardware

#endif  // DYNAMIXEL_HARDWARE__DYNAMIXEL_HARDWARE_HPP_
