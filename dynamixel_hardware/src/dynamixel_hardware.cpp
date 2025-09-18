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

#include "dynamixel_hardware/dynamixel_hardware.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <limits>
#include <string>
#include <vector>
#include <cmath>
#include <thread>

#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"



// X-series (XM540)
static constexpr uint16_t X_ADDR_PRESENT_POSITION = 132;
static constexpr uint16_t X_LEN_PRESENT_POSITION  = 4;
static constexpr uint16_t X_ADDR_PRESENT_VELOCITY = 128;
static constexpr uint16_t X_LEN_PRESENT_VELOCITY  = 4;
static constexpr uint16_t X_ADDR_PRESENT_CURRENT = 126;
static constexpr uint16_t X_LEN_PRESENT_CURRENT  = 2;
static constexpr uint16_t X_ADDR_CURRENT_LIMIT = 38;
static constexpr uint16_t X_LEN_CURRENT_LIMIT  = 2;
static constexpr uint16_t X_ADDR_HARDWARE_ERROR_STATUS = 70;
static constexpr uint16_t X_LEN_HARDWARE_ERROR_STATUS  = 1;
static constexpr uint16_t X_ADDR_EXTERNAL_PORT_DATA_1 = 152;
static constexpr uint16_t X_LEN_EXTERNAL_PORT_DATA_1  = 2;
static constexpr uint16_t X_ADDR_EXTERNAL_PORT_DATA_2 = 154;
static constexpr uint16_t X_LEN_EXTERNAL_PORT_DATA_2  = 2;

// P-series (PH42/PM54)
static constexpr uint16_t P_ADDR_PRESENT_POSITION = 580;
static constexpr uint16_t P_LEN_PRESENT_POSITION  = 4;
static constexpr uint16_t P_ADDR_PRESENT_VELOCITY = 576;
static constexpr uint16_t P_LEN_PRESENT_VELOCITY  = 4;
static constexpr uint16_t P_ADDR_PRESENT_CURRENT = 574;
static constexpr uint16_t P_LEN_PRESENT_CURRENT  = 2;
static constexpr uint16_t P_ADDR_CURRENT_LIMIT = 38;
static constexpr uint16_t P_LEN_CURRENT_LIMIT  = 2;
static constexpr uint16_t P_ADDR_HARDWARE_ERROR_STATUS = 518;
static constexpr uint16_t P_LEN_HARDWARE_ERROR_STATUS  = 1;
static constexpr uint16_t P_ADDR_EXTERNAL_PORT_DATA_1 = 600;
static constexpr uint16_t P_LEN_EXTERNAL_PORT_DATA_1  = 2;
static constexpr uint16_t P_ADDR_EXTERNAL_PORT_DATA_2 = 602;
static constexpr uint16_t P_LEN_EXTERNAL_PORT_DATA_2  = 2;


static constexpr uint16_t P_SR_START = P_ADDR_HARDWARE_ERROR_STATUS; // 518
static constexpr uint16_t P_SR_END   = P_ADDR_EXTERNAL_PORT_DATA_2 + P_LEN_EXTERNAL_PORT_DATA_2 - 1;
static constexpr uint16_t P_SR_LEN   = (P_SR_END - P_SR_START + 1);

static constexpr uint16_t X_SR_START = X_ADDR_HARDWARE_ERROR_STATUS; // 70
static constexpr uint16_t X_SR_END   = X_ADDR_EXTERNAL_PORT_DATA_2 + X_LEN_EXTERNAL_PORT_DATA_2 - 1;
static constexpr uint16_t X_SR_LEN   = (X_SR_END - X_SR_START + 1);



namespace dynamixel_hardware
{
constexpr const char * kDynamixelHardware = "DynamixelHardware";
constexpr uint8_t kGoalPositionIndex = 0;
constexpr uint8_t kGoalVelocityIndex = 1;
constexpr uint8_t kPresentPositionCurrentIndex = 0;
constexpr uint8_t kExternalPortIndex = 1;
constexpr const char * kGoalPositionItem = "Goal_Position";
constexpr const char * kGoalVelocityItem = "Goal_Velocity";
constexpr const char * kGoalCurrentItem  = "Goal_Current";
constexpr const char * kPresentPositionItem = "Present_Position";
constexpr const char * kPresentVelocityItem = "Present_Velocity";
constexpr const char * kPresentCurrentItem = "Present_Current";
constexpr const char * kPresentLoadItem = "Present_Load";
constexpr const char * kExternalPortItem_1 = "External_Port_Data_1";
constexpr const char * kExternalPortItem_2 = "External_Port_Data_2";
constexpr const char * kCurrentLimitItem = "Current_Limit";
constexpr const char * kHardwareErrorStatusItem = "Hardware_Error_Status";
constexpr const char * const kExtraJointParameters[] = {
  "Profile_Velocity",
  "Profile_Acceleration",
  "Position_P_Gain",
  "Position_I_Gain",
  "Position_D_Gain",
  "Velocity_P_Gain",
  "Velocity_I_Gain",
};

constexpr const char * IF_PRESENT_CURRENT = "present_current";
constexpr const char * IF_CURRENT_LIMIT   = "current_limit";
constexpr const char * IF_HW_ERROR        = "hardware_error";

constexpr const char * IF_EXTERNAL_TYPE = "external_type";
constexpr const char * IF_EXT_PORT1 = "external_port_data_1";
constexpr const char * IF_EXT_PORT2 = "external_port_data_2";

CallbackReturn DynamixelHardware::on_init(const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS) {
    return CallbackReturn::ERROR;
  }


  // Init Variables
  joints_.resize(info_.joints.size(), Joint());
  joint_ids_.resize(info_.joints.size(), 0);
  mechanical_reductions_.resize(info_.joints.size(), 1.0);
  external_types_.resize(info_.joints.size(), "none");
  enable_auto_reboot_.resize(info_.joints.size(), false);
  reboot_requested_.resize(info_.joints.size(), false);
  adc_filters_.resize(info_.joints.size(), EMAFilter(0.2));  // α=0.2 for moderate filtering
  prev_command_positions_.resize(info_.joints.size(), 0.0);  // Initialize previous command positions
  present_currents_A_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  current_limits_A_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_error_bits_.resize(info_.joints.size(), 0);
  hw_error_code_.resize(info_.joints.size(), 0.0);
  external_port1_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  external_port2_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  external_scale_rad_per_count_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  external_offset_rad_.resize(info_.joints.size(), 0.0);
  operating_modes_.resize(info_.joints.size(), 1);
  effort_to_current_scale_.resize(info_.joints.size(), 1.0);
  dummy_error_countdown_.resize(info_.joints.size(), 0);
  dummy_error_active_.resize(info_.joints.size(), false);
  post_reboot_grace_.resize(info_.joints.size(), 0);
  error_detection_suspend_.resize(info_.joints.size(), 0);


  // Create ID-sorted index mapping to ensure joints are processed in ID order
  std::vector<std::pair<int, size_t>> id_index_pairs;
  for (size_t i = 0; i < info_.joints.size(); i++) {
    int joint_id = std::stoi(info_.joints[i].parameters.at("id"));
    id_index_pairs.push_back({joint_id, i});
  }
  std::sort(id_index_pairs.begin(), id_index_pairs.end());
  
  // Create sorted index mapping
  id_sorted_indices_.resize(info_.joints.size());
  for (size_t i = 0; i < id_index_pairs.size(); i++) {
    id_sorted_indices_[i] = id_index_pairs[i].second;
  }
  
  // Debug: Print original and sorted joint order

  for (uint i = 0; i < info_.joints.size(); i++) {
    size_t orig_idx = id_sorted_indices_[i];
    
    joint_ids_[i] = std::stoi(info_.joints[orig_idx].parameters.at("id"));
    
    if (info_.joints[orig_idx].parameters.count("mechanical_reduction") > 0) {
      mechanical_reductions_[i] = std::stof(info_.joints[orig_idx].parameters.at("mechanical_reduction"));
    }

    if (info_.joints[orig_idx].parameters.count("external_type") > 0) {
      external_types_[i] = info_.joints[orig_idx].parameters.at("external_type");
    }

    if (info_.joints[orig_idx].parameters.count("enable_auto_reboot") > 0) {
      enable_auto_reboot_[i] = (info_.joints[orig_idx].parameters.at("enable_auto_reboot") == "true");
    }

    joints_[i].state.position = std::numeric_limits<double>::quiet_NaN();
    joints_[i].state.effort = std::numeric_limits<double>::quiet_NaN();
    joints_[i].command.position = std::numeric_limits<double>::quiet_NaN();
    joints_[i].command.effort = std::numeric_limits<double>::quiet_NaN();
    joints_[i].prev_command.position = joints_[i].command.position;
    joints_[i].prev_command.effort = joints_[i].command.effort;

    // External calc
    if (info_.joints[orig_idx].parameters.count("angle_min") > 0 &&
      info_.joints[orig_idx].parameters.count("angle_max") > 0 &&
      info_.joints[orig_idx].parameters.count("adc_min") > 0 &&
      info_.joints[orig_idx].parameters.count("adc_max") > 0)
    {
      const double a_min = std::stod(info_.joints[orig_idx].parameters.at("angle_min"));
      const double a_max = std::stod(info_.joints[orig_idx].parameters.at("angle_max"));
      const double u_min = std::stod(info_.joints[orig_idx].parameters.at("adc_min"));
      const double u_max = std::stod(info_.joints[orig_idx].parameters.at("adc_max"));
      const double du = (u_max - u_min);

      if (std::abs(du) > 1e-9) {
        external_scale_rad_per_count_[i] = (a_max - a_min) / du;
        external_offset_rad_[i] = a_min - external_scale_rad_per_count_[i] * u_min;
      } else {
        RCLCPP_WARN(rclcpp::get_logger(kDynamixelHardware),
          "Invalid adc range for joint %s", info_.joints[orig_idx].name.c_str());
      }
    }

    if (info_.joints[orig_idx].parameters.count("effort_scale") > 0) {
      try {
        effort_to_current_scale_[i] =
          std::stod(info_.joints[orig_idx].parameters.at("effort_scale"));
      } catch (...) {
        RCLCPP_WARN(rclcpp::get_logger(kDynamixelHardware),
                   "Invalid effort_scale for joint %s; use 1.0",
                    info_.joints[orig_idx].name.c_str());
      }
    }

  }

  if (
    info_.hardware_parameters.find("use_dummy") != info_.hardware_parameters.end() &&
    info_.hardware_parameters.at("use_dummy") == "true")
  {
    use_dummy_ = true;
    RCLCPP_INFO(rclcpp::get_logger(kDynamixelHardware), "dummy mode");

    std::fill(present_currents_A_.begin(), present_currents_A_.end(), 0.0);
    std::fill(hw_error_bits_.begin(),       hw_error_bits_.end(),       0);
    std::fill(hw_error_code_.begin(),       hw_error_code_.end(),       0.0);
    std::fill(external_port1_.begin(), external_port1_.end(), 0.0);
    std::fill(external_port2_.begin(), external_port2_.end(), 0.0);

    for (size_t i = 0; i < info_.joints.size(); ++i) {
      double limA = dummy_current_limit_A_;  // 例: 3.0A（既定値）
      auto it = info_.joints[i].parameters.find("current_limit_A");
      if (it != info_.joints[i].parameters.end()) {
        try { limA = std::stod(it->second); } catch (...) {}
      }
      current_limits_A_[i] = limA;
    }


    return CallbackReturn::SUCCESS;
  }

  if (
    info_.hardware_parameters.find("is_external_pos") != info_.hardware_parameters.end() &&
    info_.hardware_parameters.at("is_external_pos") == "true")
  {
    is_external_pos_ = true;
    RCLCPP_INFO(rclcpp::get_logger(kDynamixelHardware), "external position mode enabled");
  }

  auto usb_port = info_.hardware_parameters.at("usb_port");
  auto baud_rate = std::stoi(info_.hardware_parameters.at("baud_rate"));
  const char * log = nullptr;


  if (!dynamixel_workbench_.init(usb_port.c_str(), baud_rate, &log)) {
    RCLCPP_FATAL(rclcpp::get_logger(kDynamixelHardware), "%s", log);
    return CallbackReturn::ERROR;
  }

  for (uint i = 0; i < info_.joints.size(); ++i) {
    uint16_t model_number = 0;
    if (!dynamixel_workbench_.ping(joint_ids_[i], &model_number, &log)) {
      RCLCPP_FATAL(rclcpp::get_logger(kDynamixelHardware), "%s", log);
      return CallbackReturn::ERROR;
    }
  }

  enable_torque(false);
  set_operating_modes();  // Extended Position Control設定 (先に実行)
  set_joint_params();

  // 2グループSyncRead用のControlItem取得とハンドラー作成
  
  // Group 1: PH42/PM54 (ID1-2) - Pシリーズ共通テーブル
  const ControlItem * p_series_position = dynamixel_workbench_.getItemInfo(joint_ids_[0], kPresentPositionItem);
  if (p_series_position == nullptr) {
    RCLCPP_FATAL(rclcpp::get_logger(kDynamixelHardware), "P-series Present_Position not found");
    return CallbackReturn::ERROR;
  }
  const ControlItem * p_series_velocity = dynamixel_workbench_.getItemInfo(joint_ids_[0], kPresentVelocityItem);
  if (p_series_velocity == nullptr){
    RCLCPP_FATAL(rclcpp::get_logger(kDynamixelHardware), "P-series Present_Velocity not found");
    return CallbackReturn::ERROR;
  }
  const ControlItem * p_series_current = dynamixel_workbench_.getItemInfo(joint_ids_[0], kPresentCurrentItem);
  if (p_series_current == nullptr){
    RCLCPP_FATAL(rclcpp::get_logger(kDynamixelHardware), "P-series Present_Current not found");
    return CallbackReturn::ERROR;
  }
  const ControlItem * p_series_error = dynamixel_workbench_.getItemInfo(joint_ids_[0], kHardwareErrorStatusItem);
  if (p_series_error == nullptr){
    RCLCPP_FATAL(rclcpp::get_logger(kDynamixelHardware), "P-series Error_States not found");
    return CallbackReturn::ERROR;
  }
  
  // Group 2: XM540 (ID3-6) - Xシリーズテーブル  
  const ControlItem * x_series_position = dynamixel_workbench_.getItemInfo(joint_ids_[2], kPresentPositionItem);
  if (x_series_position == nullptr) {
    RCLCPP_FATAL(rclcpp::get_logger(kDynamixelHardware), "X-series Present_Position not found");
    return CallbackReturn::ERROR;
  }
  const ControlItem * x_series_velocity = dynamixel_workbench_.getItemInfo(joint_ids_[2], kPresentVelocityItem);
  if (x_series_velocity == nullptr) {
    RCLCPP_FATAL(rclcpp::get_logger(kDynamixelHardware), "X-series Present_Velocity not found");
    return CallbackReturn::ERROR;
  }
  const ControlItem * x_series_current = dynamixel_workbench_.getItemInfo(joint_ids_[2], kPresentCurrentItem);
  if (x_series_current == nullptr) {
    RCLCPP_FATAL(rclcpp::get_logger(kDynamixelHardware), "X-series Present_Current not found");
    return CallbackReturn::ERROR;
  }
  const ControlItem * x_series_error = dynamixel_workbench_.getItemInfo(joint_ids_[2], kHardwareErrorStatusItem);
  if (x_series_error == nullptr) {
    RCLCPP_FATAL(rclcpp::get_logger(kDynamixelHardware), "X-series Error_States not found");
    return CallbackReturn::ERROR;
  }

  // SyncReadハンドラー作成
  if (!dynamixel_workbench_.addSyncReadHandler(P_SR_START, P_SR_LEN, &log)) {
    RCLCPP_FATAL(rclcpp::get_logger(kDynamixelHardware), "P-series SyncRead handler failed: %s", log);
    return CallbackReturn::ERROR;
  }
  if (!dynamixel_workbench_.addSyncReadHandler(X_SR_START, X_SR_LEN, &log)) {
    RCLCPP_FATAL(rclcpp::get_logger(kDynamixelHardware), "X-series SyncRead handler failed: %s", log);
    return CallbackReturn::ERROR;
  }


  // SyncWriteハンドラー作成（Goal_Position用）
  const ControlItem * p_series_goal = dynamixel_workbench_.getItemInfo(joint_ids_[0], kGoalPositionItem);
  const ControlItem * x_series_goal = dynamixel_workbench_.getItemInfo(joint_ids_[2], kGoalPositionItem);
  
  if (p_series_goal && !dynamixel_workbench_.addSyncWriteHandler(
      p_series_goal->address, p_series_goal->data_length, &log)) {
    RCLCPP_FATAL(rclcpp::get_logger(kDynamixelHardware), "P-series SyncWrite handler failed: %s", log);
    return CallbackReturn::ERROR;
  }
  
  if (x_series_goal && !dynamixel_workbench_.addSyncWriteHandler(
      x_series_goal->address, x_series_goal->data_length, &log)) {
    RCLCPP_FATAL(rclcpp::get_logger(kDynamixelHardware), "X-series SyncWrite handler failed: %s", log);
    return CallbackReturn::ERROR;
  }

  if (!use_dummy_) {
    for (size_t i = 0; i < joint_ids_.size(); ++i) {
      int32_t raw = 0;
      if (dynamixel_workbench_.itemRead(joint_ids_[i], kCurrentLimitItem, &raw, &log)) {
        current_limits_A_[i] = dynamixel_workbench_.convertValue2Current(joint_ids_[i], raw);
      } else {
        RCLCPP_WARN(rclcpp::get_logger(kDynamixelHardware),
                    "Current_Limit read failed for ID %d: %s", joint_ids_[i], log ? log : "");
        current_limits_A_[i] = std::numeric_limits<double>::quiet_NaN();
      }
    }
  } else {
    std::fill(current_limits_A_.begin(), current_limits_A_.end(), dummy_current_limit_A_);
  }

  



  return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> DynamixelHardware::export_state_interfaces()
{
  
  std::vector<hardware_interface::StateInterface> state_interfaces;
  // 元の順序（URDF順）でエクスポート、データはID順配列から取得
  for (uint i = 0; i < info_.joints.size(); i++) {
    // info_.joints[i]から関節IDを取得し、対応するデータ配列のインデックスを見つける
    int joint_id = std::stoi(info_.joints[i].parameters.at("id"));
    size_t data_idx = 0;
    for (size_t j = 0; j < joint_ids_.size(); j++) {
      if (joint_ids_[j] == joint_id) {
        data_idx = j;
        break;
      }
    }
    
    state_interfaces.emplace_back(
      hardware_interface::StateInterface(
        info_.joints[i].name, hardware_interface::HW_IF_POSITION, &joints_[data_idx].state.position));
    state_interfaces.emplace_back(
      hardware_interface::StateInterface(
        info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &joints_[data_idx].state.velocity));
    state_interfaces.emplace_back(
      hardware_interface::StateInterface(
        info_.joints[i].name, hardware_interface::HW_IF_EFFORT, &joints_[data_idx].state.effort));
    state_interfaces.emplace_back(info_.joints[i].name, IF_EXT_PORT1, &external_port1_[data_idx]);
    state_interfaces.emplace_back(info_.joints[i].name, IF_EXT_PORT2, &external_port2_[data_idx]);
    state_interfaces.emplace_back(info_.joints[i].name, IF_PRESENT_CURRENT, &present_currents_A_[data_idx]);
    state_interfaces.emplace_back(info_.joints[i].name, IF_CURRENT_LIMIT, &current_limits_A_[data_idx]);
    state_interfaces.emplace_back(info_.joints[i].name, IF_HW_ERROR, &hw_error_code_[data_idx]);

  }

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> DynamixelHardware::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (uint i = 0; i < info_.joints.size(); i++) {
    int joint_id = std::stoi(info_.joints[i].parameters.at("id"));
    size_t data_idx = 0;
    for (size_t j = 0; j < joint_ids_.size(); j++) {
      if (joint_ids_[j] == joint_id) {
        data_idx = j;
        break;
      }
    }
    
    command_interfaces.emplace_back(
      hardware_interface::CommandInterface(
        info_.joints[i].name, hardware_interface::HW_IF_POSITION, &joints_[data_idx].command.position));
    command_interfaces.emplace_back(
      hardware_interface::CommandInterface(
        info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &joints_[data_idx].command.velocity));
    command_interfaces.emplace_back(
      hardware_interface::CommandInterface(
        info_.joints[i].name, hardware_interface::HW_IF_EFFORT, &joints_[data_idx].command.effort));
  }
  return command_interfaces;
}

CallbackReturn DynamixelHardware::on_configure(const rclcpp_lifecycle::State & /* previous_state */)
{
  for (uint i = 0; i < joints_.size(); i++) {
    if (use_dummy_ && std::isnan(joints_[i].state.position)) {
      joints_[i].state.position = 0.0;
      joints_[i].state.effort = 0.0;
    }
  }
  read(rclcpp::Time{}, rclcpp::Duration(0, 0));
  
  reset_command();
  
  // Safety check: Move dangerous joints to safe positions
  for (uint i = 0; i < joints_.size(); i++) {
    if (external_types_[i] == "potential") {
      // emergency_move_to_safe_position(i); 
    }
  }
  
  // Don't write initial position commands to avoid unexpected movement
  // write(rclcpp::Time{}, rclcpp::Duration(0, 0));

  enable_torque(true);

  return CallbackReturn::SUCCESS;
}

CallbackReturn DynamixelHardware::on_deactivate(
  const rclcpp_lifecycle::State & /* previous_state */)
{
  return CallbackReturn::SUCCESS;
}

return_type DynamixelHardware::read(
  const rclcpp::Time & /* time */,
  const rclcpp::Duration & /* period */)
{
  if (use_dummy_) {
    for (size_t i = 0; i < joints_.size(); ++i) {
      const double v = std::isfinite(joints_[i].state.velocity) ? joints_[i].state.velocity : 0.0;
      const double limit = std::isfinite(current_limits_A_[i]) ? current_limits_A_[i] : dummy_current_limit_A_;

      // Don't update current during post-reboot grace period to prevent immediate re-error
      if (post_reboot_grace_[i] == 0 && error_detection_suspend_[i] == 0) {
        double new_current = std::min(limit, std::abs(v) * dummy_current_slope_A_per_rad_s_);
        present_currents_A_[i] = new_current;

        // Debug: Log when current calculation resumes
        if (i == 0) {
          static auto last_normal_time = std::chrono::steady_clock::now();
          auto now = std::chrono::steady_clock::now();
          if (std::chrono::duration_cast<std::chrono::milliseconds>(now - last_normal_time).count() > 200) {
            RCLCPP_WARN(rclcpp::get_logger(kDynamixelHardware),
                       "Joint 1: NORMAL MODE - Velocity=%.2f, Current=%.2fA (calculated)",
                       v, new_current);
            last_normal_time = now;
          }
        }
      } else {
        // Debug: Log grace/suspend status for joint 1
        if (i == 0) {  // joint_1 = index 0
          static auto last_debug_time = std::chrono::steady_clock::now();
          auto now = std::chrono::steady_clock::now();
          if (std::chrono::duration_cast<std::chrono::milliseconds>(now - last_debug_time).count() > 200) {
            RCLCPP_INFO(rclcpp::get_logger(kDynamixelHardware),
                       "Joint 1: Grace=%d, Suspend=%d, Velocity=%.2f, Current=%.2fA (protected)",
                       post_reboot_grace_[i], error_detection_suspend_[i], v, present_currents_A_[i]);
            last_debug_time = now;
          }
        }
      }

      // Simplified error simulation - only generate error once, clear only by reboot
      if (!dummy_error_active_[i]) {
        // Clear error bits if no active dummy error
        hw_error_bits_[i] = 0;

        // Check for error conditions only if not already in error state
        // Trigger 1: High current (overload simulation)
        if (present_currents_A_[i] > (limit * 0.8)) {  // 80% of current limit
          dummy_error_countdown_[i]++;
          if (dummy_error_countdown_[i] > 50) {  // After 50 cycles (~0.5 seconds)
            dummy_error_active_[i] = true;
            hw_error_bits_[i] = 0x20;  // Overload error
            RCLCPP_WARN(rclcpp::get_logger(kDynamixelHardware),
                       "Dummy Mode: Overload error triggered for joint %d (current: %.2fA)",
                       joint_ids_[i], present_currents_A_[i]);
          }
        } else {
          dummy_error_countdown_[i] = 0;
        }

        // Trigger 2: Extreme position (mechanical limit simulation)
        const double pos = joints_[i].state.position;
        if (std::abs(pos) > 3.0) {  // Beyond ±3 radians (~172 degrees)
          dummy_error_active_[i] = true;
          hw_error_bits_[i] = 0x20;  // Overload error
          RCLCPP_WARN(rclcpp::get_logger(kDynamixelHardware),
                     "Dummy Mode: Position overload triggered for joint %d (pos: %.2f rad)",
                     joint_ids_[i], pos);
        }

        // Trigger 3: High velocity change (sudden shock simulation)
        static std::vector<double> prev_velocity(joints_.size(), 0.0);
        const double vel_change = std::abs(v - prev_velocity[i]);
        if (vel_change > 5.0) {  // Sudden velocity change > 5 rad/s
          dummy_error_active_[i] = true;
          hw_error_bits_[i] = 0x04;  // Electrical shock error
          RCLCPP_WARN(rclcpp::get_logger(kDynamixelHardware),
                     "Dummy Mode: Shock error triggered for joint %d (vel change: %.2f)",
                     joint_ids_[i], vel_change);
        }
        prev_velocity[i] = v;
      } else {
        // Error is active - maintain error state until cleared by reboot
        if (hw_error_bits_[i] != 0) {
          // Keep existing error bits
        } else {
          // Error bits were cleared by reboot - deactivate dummy error
          dummy_error_active_[i] = false;
          dummy_error_countdown_[i] = 0;
          RCLCPP_INFO(rclcpp::get_logger(kDynamixelHardware),
                     "Dummy Mode: Error cleared by reboot for joint %d", joint_ids_[i]);
        }
      }

      hw_error_code_[i] = static_cast<double>(hw_error_bits_[i]);
      external_port1_[i] = 0.0;
      external_port2_[i] = 0.0;
    }

    // Auto-reboot error detection (for dummy mode)
    for (size_t k = 0; k < joints_.size(); ++k) {
      // Skip error detection during suspend period
      if (error_detection_suspend_[k] > 0) {
        error_detection_suspend_[k]--;
        continue;
      }

      if (enable_auto_reboot_[k] && !reboot_requested_[k]) {
        // Critical errors that require reboot:
        // 0x04: Electrical Shock Error
        // 0x08: Motor Encoder Error
        // 0x10: Overheating Error
        // 0x20: Overload Error
        uint8_t critical_errors = 0x04 | 0x08 | 0x10 | 0x20;

        if (hw_error_bits_[k] & critical_errors) {
          RCLCPP_WARN(rclcpp::get_logger(kDynamixelHardware),
                     "Joint ID %d: Critical error detected (0x%02X). Requesting reboot.",
                     joint_ids_[k], hw_error_bits_[k]);
          reboot_requested_[k] = true;

          // Activate global stop mode
          global_stop_ = true;

          // Stop all joints immediately when reboot is requested
          for (size_t j = 0; j < joints_.size(); ++j) {
            joints_[j].command.velocity = 0.0;
            joints_[j].command.effort = 0.0;
            joints_[j].command.position = joints_[j].state.position;  // Hold current position
          }
          RCLCPP_WARN(rclcpp::get_logger(kDynamixelHardware),
                     "Emergency stop: All joint commands cleared due to joint %d reboot request", joint_ids_[k]);
        }
      }
    }

    return return_type::OK;
  }

  // 超高速化: static配列とループ最適化
  static const char * log = nullptr;
  static int32_t positions[7] = {0, 0, 0, 0, 0, 0, 0};
  static int32_t velocities[7] = {0, 0, 0, 0, 0, 0, 0};
  static int32_t currents_raw[7] = {0, 0, 0, 0, 0, 0, 0};
  static int32_t errors_raw[7] = {0, 0, 0, 0, 0, 0, 0};

  static int32_t ext1_raw[7] = {0};
  static int32_t ext2_raw[7] = {0};

  static uint8_t p_series_ids[2] = {1, 2};  // ID固定（constを削除）
  static uint8_t x_series_ids[5] = {3, 4, 5, 6, 7};  // ID固定（constを削除）
  
// ---- P series ----
dynamixel_workbench_.syncRead(0, p_series_ids, 2, &log);

// 位置
dynamixel_workbench_.getSyncReadData(0, p_series_ids, 2,
  P_ADDR_PRESENT_POSITION, P_LEN_PRESENT_POSITION, &positions[0], &log);

// 速度
dynamixel_workbench_.getSyncReadData(0, p_series_ids, 2,
  P_ADDR_PRESENT_VELOCITY, P_LEN_PRESENT_VELOCITY, &velocities[0], &log);

// 電流
dynamixel_workbench_.getSyncReadData(0, p_series_ids, 2,
  P_ADDR_PRESENT_CURRENT, P_LEN_PRESENT_CURRENT, &currents_raw[0], &log);

// エラー
dynamixel_workbench_.getSyncReadData(0, p_series_ids, 2,
  P_ADDR_HARDWARE_ERROR_STATUS, P_LEN_HARDWARE_ERROR_STATUS, &errors_raw[0], &log);

// 外部IO_1
dynamixel_workbench_.getSyncReadData(0, p_series_ids, 2, 
  P_ADDR_EXTERNAL_PORT_DATA_1, P_LEN_EXTERNAL_PORT_DATA_1, &ext1_raw[0], &log);

// 外部IO_2
dynamixel_workbench_.getSyncReadData(0, p_series_ids, 2, 
  P_ADDR_EXTERNAL_PORT_DATA_2, P_LEN_EXTERNAL_PORT_DATA_2, &ext2_raw[0], &log);


// ---- X series ----
dynamixel_workbench_.syncRead(1, x_series_ids, 5, &log);

// 位置
dynamixel_workbench_.getSyncReadData(1, x_series_ids, 5,
  X_ADDR_PRESENT_POSITION, X_LEN_PRESENT_POSITION, &positions[2], &log);

// 速度
dynamixel_workbench_.getSyncReadData(1, x_series_ids, 5,
  X_ADDR_PRESENT_VELOCITY, X_LEN_PRESENT_VELOCITY, &velocities[2], &log);

// 電流
dynamixel_workbench_.getSyncReadData(1, x_series_ids, 5,
  X_ADDR_PRESENT_CURRENT, X_LEN_PRESENT_CURRENT, &currents_raw[2], &log);

// エラー
dynamixel_workbench_.getSyncReadData(1, x_series_ids, 5,
  X_ADDR_HARDWARE_ERROR_STATUS, X_LEN_HARDWARE_ERROR_STATUS, &errors_raw[2], &log);

// 外部IO_1
dynamixel_workbench_.getSyncReadData(1, x_series_ids, 5, 
  X_ADDR_EXTERNAL_PORT_DATA_1, X_LEN_EXTERNAL_PORT_DATA_1, &ext1_raw[2], &log);

// 外部IO_2
dynamixel_workbench_.getSyncReadData(1, x_series_ids, 5, 
  X_ADDR_EXTERNAL_PORT_DATA_2, X_LEN_EXTERNAL_PORT_DATA_2, &ext2_raw[2], &log);



  // 結果設定最適化（ループ展開＋関数呼び出し削減）
   static const double inv_reductions[7] = {
     1.0 / mechanical_reductions_[0], 1.0 / mechanical_reductions_[1], 
     1.0 / mechanical_reductions_[2], 1.0 / mechanical_reductions_[3],
     1.0 / mechanical_reductions_[4], 1.0 / mechanical_reductions_[5],
     1.0 / mechanical_reductions_[6]
   };

  // ループ展開で高速化
  joints_[0].state.position = dynamixel_workbench_.convertValue2Radian(1, positions[0]) * inv_reductions[0];
  joints_[1].state.position = dynamixel_workbench_.convertValue2Radian(2, positions[1]) * inv_reductions[1];
  joints_[2].state.position = dynamixel_workbench_.convertValue2Radian(3, positions[2]) * inv_reductions[2];
  joints_[3].state.position = dynamixel_workbench_.convertValue2Radian(4, positions[3]) * inv_reductions[3];  
  joints_[4].state.position = dynamixel_workbench_.convertValue2Radian(5, positions[4]) * inv_reductions[4];
  joints_[5].state.position = dynamixel_workbench_.convertValue2Radian(6, positions[5]) * inv_reductions[5];
  joints_[6].state.position = dynamixel_workbench_.convertValue2Radian(7, positions[6]) * inv_reductions[6];
  
  joints_[0].state.velocity = dynamixel_workbench_.convertValue2Velocity(1, velocities[0]) * inv_reductions[0];
  joints_[1].state.velocity = dynamixel_workbench_.convertValue2Velocity(2, velocities[1]) * inv_reductions[1];
  joints_[2].state.velocity = dynamixel_workbench_.convertValue2Velocity(3, velocities[2]) * inv_reductions[2];
  joints_[3].state.velocity = dynamixel_workbench_.convertValue2Velocity(4, velocities[3]) * inv_reductions[3];  
  joints_[4].state.velocity = dynamixel_workbench_.convertValue2Velocity(5, velocities[4]) * inv_reductions[4];
  joints_[5].state.velocity = dynamixel_workbench_.convertValue2Velocity(6, velocities[5]) * inv_reductions[5];
  joints_[6].state.velocity = dynamixel_workbench_.convertValue2Velocity(7, velocities[6]) * inv_reductions[6];

  // velocity/effortは0固定（ループなし）
  joints_[0].state.effort = joints_[1].state.effort = joints_[2].state.effort = 0.0;
  joints_[3].state.effort = joints_[4].state.effort = joints_[5].state.effort = 0.0;
  joints_[6].state.effort = 0.0;

  if (is_external_pos_ && !use_dummy_) {
    const double TWO_PI = 2.0 * M_PI;
    for (size_t i = 0; i < joints_.size(); ++i) {
      if (external_types_[i] != "potential") continue;
      if (!std::isfinite(external_scale_rad_per_count_[i])) continue;

      const int32_t raw = static_cast<int32_t>(ext1_raw[i]);
      const double ext_angle = external_scale_rad_per_count_[i] * static_cast<double>(raw)
                            + external_offset_rad_[i];
      const double ref = std::isfinite(prev_command_positions_[i]) ?
                        prev_command_positions_[i] : joints_[i].state.position;

      const double k = std::round((ref - ext_angle) / TWO_PI);
      const double fused = ext_angle + k * TWO_PI;

      joints_[i].state.position = fused;
      prev_command_positions_[i] = fused;
    }
  }


  // Backlash compensation: Apply dead band filter to prevent drift accumulation
  for (size_t i = 0; i < joints_.size(); i++) {
    if (is_external_pos_ && external_types_[i] == "potential") continue;
    double delta = joints_[i].state.position - prev_command_positions_[i];
    if (std::abs(delta) < BACKLASH_DEAD_BAND) {
      // Within dead band - likely backlash, use previous command position
      joints_[i].state.position = prev_command_positions_[i];
    }
  }

  // Note: In dummy mode, hw_error_bits_ is set in the dummy mode section above
  // Don't overwrite it here to preserve error simulation
  for (size_t k = 0; k < joints_.size(); ++k) {
    present_currents_A_[k] = dynamixel_workbench_.convertValue2Current(joint_ids_[k], currents_raw[k]);
    // Only update hw_error_bits_ from real hardware in non-dummy mode
    // (In dummy mode, it's already set by error simulation above)
    hw_error_bits_[k] = static_cast<uint8_t>(errors_raw[k] & 0xFF);
    hw_error_code_[k] = static_cast<double>(hw_error_bits_[k]);

    external_port1_[k] = static_cast<double>(ext1_raw[k]);
    external_port2_[k] = static_cast<double>(ext2_raw[k]);
  }

  // Auto-reboot error detection (for real hardware mode)
  for (size_t k = 0; k < joints_.size(); ++k) {
    // Skip error detection during suspend period
    if (error_detection_suspend_[k] > 0) {
      error_detection_suspend_[k]--;
      continue;
    }

    if (enable_auto_reboot_[k] && !reboot_requested_[k]) {
      // Critical errors that require reboot:
      // 0x04: Electrical Shock Error
      // 0x08: Motor Encoder Error
      // 0x10: Overheating Error
      // 0x20: Overload Error
      uint8_t critical_errors = 0x04 | 0x08 | 0x10 | 0x20;

      if (hw_error_bits_[k] & critical_errors) {
        RCLCPP_WARN(rclcpp::get_logger(kDynamixelHardware),
                   "Joint ID %d: Critical error detected (0x%02X). Requesting reboot.",
                   joint_ids_[k], hw_error_bits_[k]);
        reboot_requested_[k] = true;

        // Activate global stop mode
        global_stop_ = true;

        // Stop all joints immediately when reboot is requested
        for (size_t j = 0; j < joints_.size(); ++j) {
          joints_[j].command.velocity = 0.0;
          joints_[j].command.effort = 0.0;
          joints_[j].command.position = joints_[j].state.position;  // Hold current position
        }
        RCLCPP_WARN(rclcpp::get_logger(kDynamixelHardware),
                   "Emergency stop: All joint commands cleared due to joint %d reboot request", joint_ids_[k]);
      }
    }
  }


  return return_type::OK;
}

return_type DynamixelHardware::write(
  const rclcpp::Time & /* time */,
  const rclcpp::Duration & period)
{
  // GLOBAL STOP: If ANY joint is rebooting, stop ALL joints
  if (global_stop_) {
    for (auto &joint : joints_) {
      joint.command.velocity = 0.0;
      joint.command.effort = 0.0;
      joint.command.position = joint.state.position;  // Hold current position
    }

    // Send zero commands to hardware
    if (!use_dummy_) {
      set_joint_velocities();
      set_joint_currents();
    }

    static auto last_global_stop_log = std::chrono::steady_clock::now();
    auto now = std::chrono::steady_clock::now();
    if (std::chrono::duration_cast<std::chrono::milliseconds>(now - last_global_stop_log).count() > 500) {
      RCLCPP_WARN(rclcpp::get_logger(kDynamixelHardware),
                 "Global stop: All joints stopped during reboot operation");
      last_global_stop_log = now;
    }

    // Continue to reboot logic below
  }

  // Handle auto-reboot requests
  for (size_t i = 0; i < joints_.size(); ++i) {
    if (reboot_requested_[i]) {
      RCLCPP_INFO(rclcpp::get_logger(kDynamixelHardware),
                 "Executing reboot for joint ID %d", joint_ids_[i]);

      if (!use_dummy_) {
        // Send zero commands to hardware before reboot
        for (size_t j = 0; j < joints_.size(); ++j) {
          joints_[j].command.velocity = 0.0;
          joints_[j].command.effort   = 0.0;
        }
        set_joint_velocities();
        set_joint_currents();
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
      }

      // Execute reboot
      if (reboot_joint(joint_ids_[i], i)) {
        RCLCPP_INFO(rclcpp::get_logger(kDynamixelHardware),
                   "Reboot successful for joint ID %d", joint_ids_[i]);
      } else {
        RCLCPP_ERROR(rclcpp::get_logger(kDynamixelHardware),
                    "Reboot failed for joint ID %d", joint_ids_[i]);
      }
      reboot_requested_[i] = false;
    }
  }

  if (use_dummy_) {
    const double dt = std::max(1e-6, period.seconds());
    for (size_t i = 0; i < joints_.size(); ++i) {
      joints_[i].prev_command.velocity = joints_[i].command.velocity;
      joints_[i].prev_command.effort = joints_[i].command.effort;

      double velocity_cmd, effort_cmd;

      // Skip commands during post-reboot grace period or if error is active (safety measure)
      if (post_reboot_grace_[i] > 0 || dummy_error_active_[i] || error_detection_suspend_[i] > 0) {
        if (post_reboot_grace_[i] > 0) {
          post_reboot_grace_[i]--;
        }
        velocity_cmd = 0.0;
        effort_cmd = 0.0;

        // Also clear command interfaces during protection period to prevent accumulation
        joints_[i].command.velocity = 0.0;
        joints_[i].command.effort = 0.0;
      } else {
        // Normal operation: use actual commands
        velocity_cmd = joints_[i].command.velocity;
        effort_cmd = joints_[i].command.effort;
      }

      // Dual limit sensor clamping (dummy mode)
      if (external_types_[i] == "dual_limit") {
        // Simulate limit sensor states based on position
        const double upper_limit = 0.5;  // Gripper fully open
        const double lower_limit = 0.1; // Gripper fully closed

        // Simulate limit sensor detection
        if (joints_[i].state.position >= upper_limit) {
          external_port2_[i] = 0.0; // Upper limit detected
          if (velocity_cmd > 0) velocity_cmd = 0.0;
          if (effort_cmd > 0) effort_cmd = 0.0;
        } else {
          external_port2_[i] = 1.0; // Not at upper limit
        }

        if (joints_[i].state.position <= lower_limit) {
          external_port1_[i] = 0.0; // Lower limit detected
          if (velocity_cmd < 0) velocity_cmd = 0.0;
          if (effort_cmd < 0) effort_cmd = 0.0;
        } else {
          external_port1_[i] = 1.0; // Not at lower limit
        }
      }

      joints_[i].state.velocity = velocity_cmd;
      joints_[i].state.effort = effort_cmd;

      // Position update: velocity command or effort-based velocity
      if (std::abs(velocity_cmd) > 1e-6) {
        // Velocity control mode
        joints_[i].state.position += velocity_cmd * dt;
      } else if (std::abs(effort_cmd) > 1e-6) {
        // Effort control mode: simulate velocity from effort
        double simulated_velocity = effort_cmd * 0.5; // Simple effort->velocity conversion

        // Simulate stall condition - reduce velocity as position approaches limits
        if (external_types_[i] == "dual_limit") {
          const double upper_limit = 1.0;
          const double lower_limit = -1.0;
          const double stall_zone = 0.1; // Stall within 0.1 rad of limits

          if (effort_cmd > 0 && joints_[i].state.position > (upper_limit - stall_zone)) {
            // Approaching upper limit - simulate increasing resistance
            double resistance_factor = 1.0 - (upper_limit - joints_[i].state.position) / stall_zone;
            simulated_velocity *= (1.0 - resistance_factor);
          } else if (effort_cmd < 0 && joints_[i].state.position < (lower_limit + stall_zone)) {
            // Approaching lower limit - simulate increasing resistance
            double resistance_factor = 1.0 - (joints_[i].state.position - lower_limit) / stall_zone;
            simulated_velocity *= (1.0 - resistance_factor);
          }
        }

        joints_[i].state.velocity = simulated_velocity;
        joints_[i].state.position += simulated_velocity * dt;
      }
    }

    // Reboot requests handled above in common section

    return return_type::OK;
  }

  // Reboot requests handled above in common section

  // Velocity mode(=1)
  set_joint_velocities();
  // Current mode(=0)
  set_joint_currents();

  // Critical hardware error check - disable all controllers if error detected
  for (size_t i = 0; i < joints_.size(); ++i) {
    // Critical errors that require immediate controller shutdown:
    // 0x04: Electrical Shock Error
    // 0x08: Motor Encoder Error
    // 0x10: Overheating Error
    // 0x20: Overload Error
    uint8_t critical_errors = 0x04 | 0x08 | 0x10 | 0x20;

    // Check for active errors and reboot requests (but not post-reboot grace periods)
    bool has_critical_error = (hw_error_bits_[i] & critical_errors) != 0;
    bool reboot_in_progress = reboot_requested_[i];

    if (has_critical_error || reboot_in_progress) {
      static auto last_error_time = std::chrono::steady_clock::now();
      auto now = std::chrono::steady_clock::now();
      if (std::chrono::duration_cast<std::chrono::milliseconds>(now - last_error_time).count() > 1000) {
        RCLCPP_ERROR(rclcpp::get_logger(kDynamixelHardware),
                     "Joint ID %d in critical state (error=0x%02X, reboot_requested=%s) - disabling all controllers",
                     joint_ids_[i], hw_error_bits_[i], reboot_in_progress ? "true" : "false");
        last_error_time = now;
      }

      // Return ERROR to disable all controllers via Controller Manager
      // This stops MoveIt Controller, Servo Bridge, MoveIt Servo, JTC - everything
      // Continue until ALL protection periods end
      return return_type::ERROR;
    }
  }

  // Check if global stop can be deactivated
  if (global_stop_) {
    bool still_protect = false;
    for (size_t i = 0; i < joints_.size(); ++i) {
      if (reboot_requested_[i] || post_reboot_grace_[i] > 0 || error_detection_suspend_[i] > 0) {
        still_protect = true;
        break;
      }
    }
    if (!still_protect) {
      global_stop_ = false;
      RCLCPP_INFO(rclcpp::get_logger(kDynamixelHardware),
                 "Global stop deactivated: All protection periods ended, normal operation resumed");
    }
  }

  return return_type::OK;
}

return_type DynamixelHardware::enable_torque(const bool enabled)
{
  const char * log = nullptr;

  if (enabled && !torque_enabled_) {
    for (uint i = 0; i < info_.joints.size(); ++i) {
      if (!dynamixel_workbench_.torqueOn(joint_ids_[i], &log)) {
        RCLCPP_FATAL(rclcpp::get_logger(kDynamixelHardware), "%s", log);
        return return_type::ERROR;
      }
    }

    RCLCPP_INFO(rclcpp::get_logger(kDynamixelHardware), "Torque enabled");
  } else if (!enabled && torque_enabled_) {
    for (uint i = 0; i < info_.joints.size(); ++i) {
      if (!dynamixel_workbench_.torqueOff(joint_ids_[i], &log)) {
        RCLCPP_FATAL(rclcpp::get_logger(kDynamixelHardware), "%s", log);
        return return_type::ERROR;
      }
    }
    RCLCPP_INFO(rclcpp::get_logger(kDynamixelHardware), "Torque disabled");
  }

  torque_enabled_ = enabled;
  return return_type::OK;
}

return_type DynamixelHardware::set_control_mode(const ControlMode & mode, const bool force_set)
{
  const char * log = nullptr;
  mode_changed_ = false;

  if (mode == ControlMode::Position && (force_set || control_mode_ != ControlMode::Position)) {
    RCLCPP_WARN(rclcpp::get_logger(kDynamixelHardware), 
                "Control mode change detected! force_set=%s, current_mode=%d", 
                force_set ? "true" : "false", static_cast<int>(control_mode_));
    bool torque_enabled = torque_enabled_;
    if (torque_enabled) {
      RCLCPP_WARN(rclcpp::get_logger(kDynamixelHardware), "Torque OFF for control mode change");
      // Preserve current position before torque off to prevent gravity drop
      read(rclcpp::Time{}, rclcpp::Duration(0, 0));  // Get latest position
      reset_command();  // Set command position = current position
      enable_torque(false);
    }

    for (uint i = 0; i < joint_ids_.size(); ++i) {
      if (!dynamixel_workbench_.setPositionControlMode(joint_ids_[i], &log)) {
        RCLCPP_FATAL(rclcpp::get_logger(kDynamixelHardware), "%s", log);
        return return_type::ERROR;
      }
    }
    RCLCPP_INFO(rclcpp::get_logger(kDynamixelHardware), "Position control");
    if (control_mode_ != ControlMode::Position) {
      mode_changed_ = true;
      control_mode_ = ControlMode::Position;
    }

    if (torque_enabled) {
      enable_torque(true);
    }
    return return_type::OK;
  }

  if (mode == ControlMode::Velocity && (force_set || control_mode_ != ControlMode::Velocity)) {
    bool torque_enabled = torque_enabled_;
    if (torque_enabled) {
      read(rclcpp::Time{}, rclcpp::Duration(0, 0));
      reset_command();
      enable_torque(false);
    }
    for (uint i = 0; i < joint_ids_.size(); ++i) {
      if (!dynamixel_workbench_.setVelocityControlMode(joint_ids_[i], &log)) {
        RCLCPP_FATAL(rclcpp::get_logger(kDynamixelHardware), "%s", log);
        return return_type::ERROR;
      }
    }
    RCLCPP_INFO(rclcpp::get_logger(kDynamixelHardware), "Velocity control");
    if (control_mode_ != ControlMode::Velocity) {
      mode_changed_ = true;
      control_mode_ = ControlMode::Velocity;
    }
    if (torque_enabled) {
      enable_torque(true);
    }
    return return_type::OK;
  }

  return return_type::OK;
}

return_type DynamixelHardware::reset_command()
{
  for (uint i = 0; i < joints_.size(); i++) {
    joints_[i].command.position = joints_[i].state.position;
    joints_[i].command.effort = 0.0;
    joints_[i].prev_command.position = joints_[i].command.position;
    joints_[i].prev_command.effort = joints_[i].command.effort;
  }

  return return_type::OK;
}

CallbackReturn DynamixelHardware::set_joint_positions()
{
  static const char * log = nullptr;
  static uint8_t p_series_ids[2] = {1, 2};
  static uint8_t x_series_ids[5] = {3, 4, 5, 6, 7};
  static int32_t commands[7];
  
  // prev_command更新（ループ展開）
  joints_[0].prev_command.position = joints_[0].command.position;
  joints_[1].prev_command.position = joints_[1].command.position;
  joints_[2].prev_command.position = joints_[2].command.position;
  joints_[3].prev_command.position = joints_[3].command.position;
  joints_[4].prev_command.position = joints_[4].command.position;
  joints_[5].prev_command.position = joints_[5].command.position;
  joints_[6].prev_command.position = joints_[6].command.position;
  
  // 目標位置計算（ループ展開）
  commands[0] = dynamixel_workbench_.convertRadian2Value(1, static_cast<float>(joints_[0].command.position * mechanical_reductions_[0]));
  commands[1] = dynamixel_workbench_.convertRadian2Value(2, static_cast<float>(joints_[1].command.position * mechanical_reductions_[1]));
  commands[2] = dynamixel_workbench_.convertRadian2Value(3, static_cast<float>(joints_[2].command.position * mechanical_reductions_[2]));
  commands[3] = dynamixel_workbench_.convertRadian2Value(4, static_cast<float>(joints_[3].command.position * mechanical_reductions_[3]));
  commands[4] = dynamixel_workbench_.convertRadian2Value(5, static_cast<float>(joints_[4].command.position * mechanical_reductions_[4]));
  commands[5] = dynamixel_workbench_.convertRadian2Value(6, static_cast<float>(joints_[5].command.position * mechanical_reductions_[5]));
  commands[6] = dynamixel_workbench_.convertRadian2Value(7, static_cast<float>(joints_[6].command.position * mechanical_reductions_[6]));
  
  // SyncWrite実行（2グループ）
  dynamixel_workbench_.syncWrite(0, p_series_ids, 2, &commands[0], 1, &log);  // P-series (ID1-2)
  dynamixel_workbench_.syncWrite(1, x_series_ids, 5, &commands[2], 1, &log);  // X-series (ID3-7)
  
  // Update previous command positions for backlash compensation
  for (size_t i = 0; i < joints_.size(); i++) {
    prev_command_positions_[i] = joints_[i].command.position;
  }
  
  return CallbackReturn::SUCCESS;
}


CallbackReturn DynamixelHardware::set_joint_params()
{
  const char * log = nullptr;
  for (uint i = 0; i < info_.joints.size(); ++i) {
    for (auto paramName : kExtraJointParameters) {
      if (info_.joints[i].parameters.find(paramName) != info_.joints[i].parameters.end()) {
        auto value = std::stoi(info_.joints[i].parameters.at(paramName));
        if (!dynamixel_workbench_.itemWrite(joint_ids_[i], paramName, value, &log)) {
          RCLCPP_FATAL(rclcpp::get_logger(kDynamixelHardware), "%s", log);
          return CallbackReturn::ERROR;
        }
        RCLCPP_INFO(
          rclcpp::get_logger(
            kDynamixelHardware), "%s set to %d for joint %d", paramName, value, i);
      }
    }
  }
  return CallbackReturn::SUCCESS;
}

CallbackReturn DynamixelHardware::set_joint_params_for_one(size_t joint_index)
{
  const char * log = nullptr;
  size_t orig_idx = id_sorted_indices_[joint_index];

  for (auto paramName : kExtraJointParameters) {
    auto it = info_.joints[orig_idx].parameters.find(paramName);
    if (it == info_.joints[orig_idx].parameters.end()) continue;

    int value = 0;
    try {
      value = std::stoi(it->second);
    } catch (...) {
      RCLCPP_WARN(rclcpp::get_logger(kDynamixelHardware),
                  "Invalid parameter %s for joint %d", paramName, joint_ids_[joint_index]);
      continue;
    }

    if (!dynamixel_workbench_.itemWrite(joint_ids_[joint_index], paramName, value, &log)) {
      RCLCPP_ERROR(rclcpp::get_logger(kDynamixelHardware),
                   "Reapply %s failed (ID:%d): %s",
                   paramName, joint_ids_[joint_index], log ? log : "unknown error");
      return CallbackReturn::ERROR;
    }

    RCLCPP_INFO(rclcpp::get_logger(kDynamixelHardware),
               "Reapplied %s = %d for joint ID %d after reboot",
               paramName, value, joint_ids_[joint_index]);
  }

  return CallbackReturn::SUCCESS;
}

CallbackReturn DynamixelHardware::set_joint_velocities()
{
  static uint32_t vel_error_count = 0;
  static const uint32_t ERROR_LOG_INTERVAL = 200;
  const char * log = nullptr;

  for (uint i = 0; i < info_.joints.size(); i++) {
    // Skip command during grace period after reboot
    if (post_reboot_grace_[i] > 0) {
      post_reboot_grace_[i]--;
      continue;
    }

    // Skip command if critical hardware error detected (safety measure)
    uint8_t critical_errors = 0x04 | 0x08 | 0x10 | 0x20;
    if (hw_error_bits_[i] & critical_errors) {
      continue;
    }

    // mode_check
    if (operating_modes_[i] != 1){
      continue;
    }

    joints_[i].prev_command.velocity = joints_[i].command.velocity;

    // Dual limit sensor velocity clamping
    if (external_types_[i] == "dual_limit") {
      double velocity = joints_[i].command.velocity;

      // data_2=0 (upper limit) -> clamp positive velocity to 0
      if (external_port2_[i] == 0 && velocity > 0) {
        velocity = 0.0;
      }

      // data_1=0 (lower limit) -> clamp negative velocity to 0
      if (external_port1_[i] == 0 && velocity < 0) {
        velocity = 0.0;
      }

      joints_[i].command.velocity = velocity;
    }

    int32_t goal_velocity = dynamixel_workbench_.convertVelocity2Value(
      joint_ids_[i], static_cast<float>(joints_[i].command.velocity * mechanical_reductions_[i]));
    
    if (!dynamixel_workbench_.itemWrite(joint_ids_[i], kGoalVelocityItem, goal_velocity, &log)) {
      if (++vel_error_count % ERROR_LOG_INTERVAL == 0) {
        RCLCPP_ERROR(rclcpp::get_logger(kDynamixelHardware), "Velocity write failed %u times", vel_error_count);
      }
    }
  }
  return CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn DynamixelHardware::set_joint_currents()
{
  const char * log = nullptr;
  static uint32_t cur_error_count = 0;
  static const uint32_t ERROR_LOG_INTERVAL = 200;

  for (uint i = 0; i < info_.joints.size(); ++i) {
    // Skip command during grace period after reboot
    if (post_reboot_grace_[i] > 0) {
      post_reboot_grace_[i]--;
      continue;
    }

    // Skip command if critical hardware error detected (safety measure)
    uint8_t critical_errors = 0x04 | 0x08 | 0x10 | 0x20;
    if (hw_error_bits_[i] & critical_errors) {
      continue;
    }

    // Mode check
    if (operating_modes_[i] != 0) {
      continue;
    }

    double current_A = joints_[i].command.effort * effort_to_current_scale_[i];

    // Dual limit sensor effort/current clamping
    if (external_types_[i] == "dual_limit") {
      // data_2=0 (upper limit) -> clamp positive effort/current to 0
      if (external_port2_[i] == 0 && current_A > 0) {
        current_A = 0.0;
      }

      // data_1=0 (lower limit) -> clamp negative effort/current to 0
      if (external_port1_[i] == 0 && current_A < 0) {
        current_A = 0.0;
      }
    }

    const double limit = std::isfinite(current_limits_A_[i]) ?
                          current_limits_A_[i] : dummy_current_limit_A_;
    if (std::isfinite(limit)) {
      if (current_A >  limit) current_A =  limit;
      if (current_A < -limit) current_A = -limit;
    }

    const int32_t goal_current_raw =
      dynamixel_workbench_.convertCurrent2Value(joint_ids_[i],
                                                static_cast<float>(current_A));

    if (!dynamixel_workbench_.itemWrite(joint_ids_[i],
                                        kGoalCurrentItem,
                                        goal_current_raw, &log)) {
      if (++cur_error_count % ERROR_LOG_INTERVAL == 0) {
        RCLCPP_ERROR(rclcpp::get_logger(kDynamixelHardware),
                     "Current write failed %u times", cur_error_count);
      }
    }

    joints_[i].prev_command.effort = joints_[i].command.effort;
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

void DynamixelHardware::set_operating_modes()
{
  const char * log = nullptr;

  for (uint i = 0; i < info_.joints.size(); ++i) {
    auto it = info_.joints[i].parameters.find("operating_mode");
    if (it == info_.joints[i].parameters.end()) {
      continue;
    }

    int mode = 0;
    try { mode = std::stoi(it->second); } catch (...) {
      RCLCPP_ERROR(rclcpp::get_logger(kDynamixelHardware),
                   "Invalid operating_mode for joint %s", info_.joints[i].name.c_str());
      continue;
    }

    if (!dynamixel_workbench_.setOperatingMode(joint_ids_[i], mode, &log)) {
      RCLCPP_ERROR(rclcpp::get_logger(kDynamixelHardware),
                   "Failed to set operating mode %d for joint %s (ID:%d): %s",
                   mode, info_.joints[i].name.c_str(), joint_ids_[i], log ? log : "");
    } else {
      operating_modes_[i] = mode;
      RCLCPP_INFO(rclcpp::get_logger(kDynamixelHardware),
                  "Operating mode %d set for joint %s (ID:%d)",
                  mode, info_.joints[i].name.c_str(), joint_ids_[i]);
    }
  }
}

bool DynamixelHardware::reboot_joint(uint8_t joint_id, size_t joint_index)
{
  // CRITICAL: Stop ALL joints during ANY reboot to prevent continued motion
  global_stop_ = true;
  RCLCPP_WARN(rclcpp::get_logger(kDynamixelHardware),
             "GLOBAL STOP ACTIVATED: All joints stopped during joint %d reboot", joint_id);

  if (use_dummy_) {
    RCLCPP_INFO(rclcpp::get_logger(kDynamixelHardware),
               "Dummy mode: Simulating reboot for joint ID %d", joint_id);

    // Clear all error states
    hw_error_bits_[joint_index] = 0;
    hw_error_code_[joint_index] = 0.0;
    dummy_error_active_[joint_index] = false;
    dummy_error_countdown_[joint_index] = 0;

    // Reset to safe values to prevent immediate re-error
    joints_[joint_index].state.position = 0.0;
    joints_[joint_index].state.velocity = 0.0;
    joints_[joint_index].state.effort = 0.0;
    present_currents_A_[joint_index] = 0.1;

    // Clear dangerous commands to prevent continued motion after reboot
    for (size_t j = 0; j < joints_.size(); ++j) {
      joints_[j].command.position = 0.0;
      joints_[j].command.velocity = 0.0;
      joints_[j].command.effort   = 0.0;
    }
    joints_[joint_index].prev_command.position = 0.0;
    joints_[joint_index].prev_command.velocity = 0.0;
    joints_[joint_index].prev_command.effort = 0.0;

    // Set grace periods (longer for dummy mode stability)
    post_reboot_grace_[joint_index] = 50;  // 50 cycles (~500ms)
    error_detection_suspend_[joint_index] = 100;  // 100 cycles (~1000ms)

    RCLCPP_INFO(rclcpp::get_logger(kDynamixelHardware),
               "Dummy mode: Reboot completed for joint ID %d (reset to safe state)", joint_id);
    return true;
  }

  const char* log = nullptr;

  // 1. torque_off (try, but don't fail reboot if it fails - servo may be in error state)
  if (!dynamixel_workbench_.torqueOff(joint_id, &log)) {
    RCLCPP_WARN(rclcpp::get_logger(kDynamixelHardware),
                "Torque off failed for ID %d: %s (continuing with reboot)", joint_id, log ? log : "unknown error");
  }

  // 2. send_reboot (try multiple approaches for stuck servos)
  bool reboot_success = false;

  // Try standard reboot first
  if (dynamixel_workbench_.reboot(joint_id, &log)) {
    reboot_success = true;
  } else {
    RCLCPP_WARN(rclcpp::get_logger(kDynamixelHardware),
                "Standard reboot failed for ID %d: %s (trying force reboot)", joint_id, log ? log : "unknown error");

    // Try multiple recovery strategies
    RCLCPP_WARN(rclcpp::get_logger(kDynamixelHardware),
               "Attempting emergency recovery for servo ID %d", joint_id);

    // Strategy 1: Try clearing error status
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
    if (dynamixel_workbench_.itemWrite(joint_id, "Hardware_Error_Status", 0, &log)) {
      RCLCPP_INFO(rclcpp::get_logger(kDynamixelHardware),
                 "Cleared hardware error status for ID %d, retrying reboot", joint_id);
      std::this_thread::sleep_for(std::chrono::milliseconds(100));

      if (dynamixel_workbench_.reboot(joint_id, &log)) {
        reboot_success = true;
        RCLCPP_INFO(rclcpp::get_logger(kDynamixelHardware),
                   "Recovery reboot successful for ID %d", joint_id);
      }
    }

    // Strategy 2: Try ping to re-establish communication
    if (!reboot_success) {
      uint16_t model_number;
      for (int ping_retry = 0; ping_retry < 3; ++ping_retry) {
        if (dynamixel_workbench_.ping(joint_id, &model_number, &log)) {
          RCLCPP_INFO(rclcpp::get_logger(kDynamixelHardware),
                     "Re-established communication with ID %d, trying reboot", joint_id);
          std::this_thread::sleep_for(std::chrono::milliseconds(100));

          if (dynamixel_workbench_.reboot(joint_id, &log)) {
            reboot_success = true;
            RCLCPP_INFO(rclcpp::get_logger(kDynamixelHardware),
                       "Post-ping reboot successful for ID %d", joint_id);
            break;
          }
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(300));
      }
    }

    if (!reboot_success) {
      RCLCPP_ERROR(rclcpp::get_logger(kDynamixelHardware),
                  "All reboot attempts failed for ID %d: %s", joint_id, log ? log : "unknown error");
      RCLCPP_WARN(rclcpp::get_logger(kDynamixelHardware),
                 "Servo ID %d may require manual power cycle. Disabling auto-reboot for this joint.", joint_id);

      // Disable auto-reboot for this joint to prevent infinite retry loops
      enable_auto_reboot_[joint_index] = false;

      // Clear flags and continue operation
      hw_error_bits_[joint_index] = 0;
      hw_error_code_[joint_index] = 0.0;
      post_reboot_grace_[joint_index] = 10;  // Extended grace period
      error_detection_suspend_[joint_index] = 50;  // Extended suspend period

      return false;  // Still return false to indicate reboot failure
    }
  }

  // 3. wait for reboot
  RCLCPP_INFO(rclcpp::get_logger(kDynamixelHardware),
             "Joint ID %d rebooting... (LED should be blinking)", joint_id);
  std::this_thread::sleep_for(std::chrono::milliseconds(2000));

  // 4. restore comunication
  uint16_t model_number = 0;
  for (int retry = 0; retry < 5; ++retry) {
    if (dynamixel_workbench_.ping(joint_id, &model_number, &log)) {
      RCLCPP_INFO(rclcpp::get_logger(kDynamixelHardware),
                 "Joint ID %d ping successful after reboot", joint_id);
      break;
    }
    if (retry == 4) {
      RCLCPP_ERROR(rclcpp::get_logger(kDynamixelHardware),
                  "Ping failed after reboot for ID %d", joint_id);
      return false;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
  }

  // 5. set_control_mode
  if (!dynamixel_workbench_.setOperatingMode(joint_id, operating_modes_[joint_index], &log)) {
    RCLCPP_WARN(rclcpp::get_logger(kDynamixelHardware),
               "Failed to restore operating mode for ID %d: %s", joint_id, log ? log : "unknown error");
  }

  // 6. torque_on
  if (!dynamixel_workbench_.torqueOn(joint_id, &log)) {
    RCLCPP_ERROR(rclcpp::get_logger(kDynamixelHardware),
                "Torque on failed after reboot for ID %d: %s", joint_id, log ? log : "unknown error");
    return false;
  }

  // 7. Re-apply RAM parameters (lost during reboot)
  if (set_joint_params_for_one(joint_index) != CallbackReturn::SUCCESS) {
    RCLCPP_WARN(rclcpp::get_logger(kDynamixelHardware),
               "Failed to reapply some parameters for joint ID %d after reboot", joint_id);
  }

  // 8. Startup grace period
  std::this_thread::sleep_for(std::chrono::milliseconds(100));

  // 9. Set grace period to skip next few command cycles
  post_reboot_grace_[joint_index] = 5;  // Skip 5 cycles (~50ms at 100Hz)

  // 10. Suspend error detection for recovery period
  error_detection_suspend_[joint_index] = 20;  // Suspend for 20 cycles (~200ms)

  // 11. Clear error flags
  hw_error_bits_[joint_index] = 0;
  hw_error_code_[joint_index] = 0.0;

  // 12. Clear dangerous commands to prevent continued motion after reboot
  for (size_t j = 0; j < joints_.size(); ++j) {
    joints_[j].command.position = joints_[j].state.position;
    joints_[j].command.velocity = 0.0;
    joints_[j].command.effort   = 0.0;
    joints_[j].prev_command     = joints_[j].command;
  }
  
  return true;
}

}  // namespace dynamixel_hardware

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(dynamixel_hardware::DynamixelHardware, hardware_interface::SystemInterface)
