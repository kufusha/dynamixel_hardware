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

#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace dynamixel_hardware
{
constexpr const char * kDynamixelHardware = "DynamixelHardware";
constexpr uint8_t kGoalPositionIndex = 0;
constexpr uint8_t kGoalVelocityIndex = 1;
constexpr uint8_t kPresentPositionCurrentIndex = 0;
constexpr uint8_t kExternalPortIndex = 1;
constexpr const char * kGoalPositionItem = "Goal_Position";
constexpr const char * kGoalVelocityItem = "Goal_Velocity";
constexpr const char * kPresentPositionItem = "Present_Position";
constexpr const char * kPresentVelocityItem = "Present_Velocity";
constexpr const char * kPresentCurrentItem = "Present_Current";
constexpr const char * kPresentLoadItem = "Present_Load";
constexpr const char * kExternalPortItem = "External_Port_Data_1";
constexpr const char * kCurrentLimitItem = "Current_Limit"; // 電流制限値
constexpr const char * kHardwareErrorStatusItem = "Hardware_Error_Status"; // HWエラーステータス(1bit)
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
  adc_filters_.resize(info_.joints.size(), EMAFilter(0.2));  // α=0.2 for moderate filtering
  prev_command_positions_.resize(info_.joints.size(), 0.0);  // Initialize previous command positions
  present_currents_A_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  current_limits_A_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_error_bits_.resize(info_.joints.size(), 0);
  hw_error_code_.resize(info_.joints.size(), 0.0);


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
    joints_[i].state.position = std::numeric_limits<double>::quiet_NaN();
    joints_[i].state.effort = std::numeric_limits<double>::quiet_NaN();
    joints_[i].command.position = std::numeric_limits<double>::quiet_NaN();
    joints_[i].command.effort = std::numeric_limits<double>::quiet_NaN();
    joints_[i].prev_command.position = joints_[i].command.position;
    joints_[i].prev_command.effort = joints_[i].command.effort;
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
  // Handler 0: P-series (ID1-2)
  if (!dynamixel_workbench_.addSyncReadHandler(
      p_series_position->address, p_series_position->data_length, &log)) {
    RCLCPP_FATAL(rclcpp::get_logger(kDynamixelHardware), "P-series SyncRead handler failed: %s", log);
    return CallbackReturn::ERROR;
  }
  
  // Handler 1: X-series (ID3-6)  
  if (!dynamixel_workbench_.addSyncReadHandler(
      x_series_position->address, x_series_position->data_length, &log)) {
    RCLCPP_FATAL(rclcpp::get_logger(kDynamixelHardware), "X-series SyncRead handler failed: %s", log);
    return CallbackReturn::ERROR;
  }

  // Handler 2: P-series-velocity (ID1-2)  
  if (!dynamixel_workbench_.addSyncReadHandler(
      p_series_velocity->address, p_series_velocity->data_length, &log)) {
    RCLCPP_FATAL(rclcpp::get_logger(kDynamixelHardware), "P-series SyncRead handler failed: %s", log);
    return CallbackReturn::ERROR;
  }

  // Handler 3: X-series-velocity (ID3-6)  
  if (!dynamixel_workbench_.addSyncReadHandler(
      x_series_velocity->address, x_series_velocity->data_length, &log)) {
    RCLCPP_FATAL(rclcpp::get_logger(kDynamixelHardware), "X-series SyncRead handler failed: %s", log);
    return CallbackReturn::ERROR;
  }

  // set data_address,lengths 
  addr_p_cur_ = p_series_current->address; len_p_cur_ = p_series_current->data_length;
  addr_x_cur_ = x_series_current->address; len_x_cur_ = x_series_current->data_length;
  addr_p_err_ = p_series_error->address;   len_p_err_ = p_series_error->data_length;
  addr_x_err_ = x_series_error->address;   len_x_err_ = x_series_error->data_length;

  int next = 4;
  if (!dynamixel_workbench_.addSyncReadHandler(addr_p_cur_, len_p_cur_, &log))  return CallbackReturn::ERROR;
  sr_idx_p_cur_ = next++;
  if (!dynamixel_workbench_.addSyncReadHandler(addr_x_cur_, len_x_cur_, &log))  return CallbackReturn::ERROR;
  sr_idx_x_cur_ = next++;
  if (!dynamixel_workbench_.addSyncReadHandler(addr_p_err_, len_p_err_, &log))  return CallbackReturn::ERROR;
  sr_idx_p_err_ = next++;
  if (!dynamixel_workbench_.addSyncReadHandler(addr_x_err_, len_x_err_, &log))  return CallbackReturn::ERROR;
  sr_idx_x_err_ = next++;
  

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

    state_interfaces.emplace_back(info_.joints[i].name, IF_PRESENT_CURRENT, &present_currents_A_[data_idx]);
    state_interfaces.emplace_back(info_.joints[i].name, IF_CURRENT_LIMIT, &current_limits_A_[data_idx]);
    state_interfaces.emplace_back(info_.joints[i].name, IF_HW_ERROR, &hw_error_code_[data_idx]);

  }

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> DynamixelHardware::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
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
    
    command_interfaces.emplace_back(
      hardware_interface::CommandInterface(
        info_.joints[i].name, hardware_interface::HW_IF_POSITION, &joints_[data_idx].command.position));
    command_interfaces.emplace_back(
      hardware_interface::CommandInterface(
        info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &joints_[data_idx].command.velocity));
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
      // emergency_move_to_safe_position(i); // ToDo 安全機能の実装
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
      present_currents_A_[i] = std::min(limit, std::abs(v) * dummy_current_slope_A_per_rad_s_);
      hw_error_bits_[i] = 0;
      hw_error_code_[i] = 0.0;
    }
    return return_type::OK;
  }

  // 超高速化: static配列とループ最適化
  static const char * log = nullptr;
  static int32_t positions[7] = {0};
  static int32_t velocities[7] = {0};
  static uint8_t p_series_ids[2] = {1, 2};  // ID固定（constを削除）
  static uint8_t x_series_ids[5] = {3, 4, 5, 6, 7};  // ID固定（constを削除）
  
  // SyncRead実行（最小限）
  dynamixel_workbench_.syncRead(0, p_series_ids, 2, &log);
  dynamixel_workbench_.getSyncReadData(0, p_series_ids, 2, 580, 4, &positions[0], &log);
  
  dynamixel_workbench_.syncRead(1, x_series_ids, 5, &log);
  dynamixel_workbench_.getSyncReadData(1, x_series_ids, 5, 132, 4, &positions[2], &log);

  dynamixel_workbench_.syncRead(2, p_series_ids, 2, &log);
  dynamixel_workbench_.getSyncReadData(2, p_series_ids, 2, 584, 4, &velocities[0], &log);

  dynamixel_workbench_.syncRead(3, x_series_ids, 5, &log);
  dynamixel_workbench_.getSyncReadData(3, x_series_ids, 5, 128, 4, &velocities[2], &log);

  static int32_t currents_raw[7] = {0};  // 長さは 2 or 4 byte だが int32_t に受けてOK
  dynamixel_workbench_.syncRead(sr_idx_p_cur_, p_series_ids, 2, &log);
  dynamixel_workbench_.getSyncReadData(sr_idx_p_cur_, p_series_ids, 2, addr_p_cur_, len_p_cur_, &currents_raw[0], &log);

  dynamixel_workbench_.syncRead(sr_idx_x_cur_, x_series_ids, 5, &log);
  dynamixel_workbench_.getSyncReadData(sr_idx_x_cur_, x_series_ids, 5, addr_x_cur_, len_x_cur_, &currents_raw[2], &log);

  static int32_t errors_raw[7] = {0};
  dynamixel_workbench_.syncRead(sr_idx_p_err_, p_series_ids, 2, &log);
  dynamixel_workbench_.getSyncReadData(sr_idx_p_err_, p_series_ids, 2, addr_p_err_, len_p_err_, &errors_raw[0], &log);

  dynamixel_workbench_.syncRead(sr_idx_x_err_, x_series_ids, 5, &log);
  dynamixel_workbench_.getSyncReadData(sr_idx_x_err_, x_series_ids, 5, addr_x_err_, len_x_err_, &errors_raw[2], &log);

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

  // 外部IO処理（必要に応じてコメントアウト解除）
  // if (is_external_pos_) {
  //   // 外部センサー読み取り処理
  //   for (uint i = 0; i < info_.joints.size(); i++) {
  //     if (external_types_[i] == "potential") {
  //       int32_t external_data = 0;
  //       if (dynamixel_workbench_.itemRead(joint_ids_[i], kExternalPortItem, &external_data, &log)) {
  //         external_data = static_cast<int32_t>(adc_filters_[i].update(static_cast<double>(external_data)));
  //         joints_[i].state.position = convert_external_sensor_to_angle(i, external_data);
  //       }
  //     }
  //   }
  //   smart_offset_management();
  // }

  // Backlash compensation: Apply dead band filter to prevent drift accumulation
  for (size_t i = 0; i < joints_.size(); i++) {
    double delta = joints_[i].state.position - prev_command_positions_[i];
    if (std::abs(delta) < BACKLASH_DEAD_BAND) {
      // Within dead band - likely backlash, use previous command position
      joints_[i].state.position = prev_command_positions_[i];
    }
  }

  for (size_t k = 0; k < joints_.size(); ++k) {
    present_currents_A_[k] = dynamixel_workbench_.convertValue2Current(joint_ids_[k], currents_raw[k]);
    hw_error_bits_[k] = static_cast<uint8_t>(errors_raw[k] & 0xFF);
    hw_error_code_[k] = static_cast<double>(hw_error_bits_[k]);
  }


  return return_type::OK;
}

return_type DynamixelHardware::write(
  const rclcpp::Time & /* time */,
  const rclcpp::Duration & period)
{
  if (use_dummy_) {
    const double dt = std::max(1e-6, period.seconds());
    for (auto & joint : joints_) {
      joint.prev_command.velocity = joint.command.velocity;
      joint.state.velocity = joint.command.velocity;
      joint.state.position += joint.command.velocity * dt;
    }
    return return_type::OK;
  }

  // Limit check for Hand motor
  // Limit_detect_function() // ToDo(Sasaki Tiger) create limit function

  set_control_mode(ControlMode::Velocity);
  set_joint_velocities();
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
  
  // Debug: joint5 command tracking
  static int debug_count = 0;
  if (++debug_count % 100 == 0) {  // Every 100 cycles
    RCLCPP_INFO(rclcpp::get_logger(kDynamixelHardware), 
                "Joint5 cmd=%.3f, prev=%.3f, dxl_cmd=%d", 
                joints_[4].command.position, joints_[4].prev_command.position, commands[4]);
  }
  
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

CallbackReturn DynamixelHardware::set_joint_velocities()
{
  static uint32_t vel_error_count = 0;
  static const uint32_t ERROR_LOG_INTERVAL = 200;
  const char * log = nullptr;

  for (uint i = 0; i < info_.joints.size(); i++) {
    joints_[i].prev_command.velocity = joints_[i].command.velocity;
    
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
      RCLCPP_INFO(rclcpp::get_logger(kDynamixelHardware),
                  "Operating mode %d set for joint %s (ID:%d)",
                  mode, info_.joints[i].name.c_str(), joint_ids_[i]);
    }
  }
}

}  // namespace dynamixel_hardware

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(dynamixel_hardware::DynamixelHardware, hardware_interface::SystemInterface)
