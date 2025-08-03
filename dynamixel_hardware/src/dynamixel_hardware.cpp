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
constexpr const char * const kExtraJointParameters[] = {
  "Profile_Velocity",
  "Profile_Acceleration",
  "Position_P_Gain",
  "Position_I_Gain",
  "Position_D_Gain",
  "Velocity_P_Gain",
  "Velocity_I_Gain",
};

CallbackReturn DynamixelHardware::on_init(const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS) {
    return CallbackReturn::ERROR;
  }

  // Initialize ROS2 node for services
  // if (!rclcpp::ok()) {
  //   rclcpp::init(0, nullptr);
  // }
  // node_ = rclcpp::Node::make_shared("dynamixel_hardware_services");
  // 
  // // Create torque enable service
  // torque_service_ = node_->create_service<std_srvs::srv::SetBool>(
  //   "torque_enable",
  //   std::bind(&DynamixelHardware::torque_enable_service_callback, this,
  //             std::placeholders::_1, std::placeholders::_2));

  joints_.resize(info_.joints.size(), Joint());
  joint_ids_.resize(info_.joints.size(), 0);
  mechanical_reductions_.resize(info_.joints.size(), 1.0);
  external_types_.resize(info_.joints.size(), "none");
  calibration_data_.resize(info_.joints.size(), CalibrationData());
  adc_filters_.resize(info_.joints.size(), EMAFilter(0.2));  // α=0.2 for moderate filtering

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
      calibration_data_[i].external_type = external_types_[i];
      
      // Load calibration parameters for potentiometer
      if (external_types_[i] == "potential") {
        if (info_.joints[orig_idx].parameters.count("angle_min") > 0) {
          calibration_data_[i].angle_min = std::stof(info_.joints[orig_idx].parameters.at("angle_min"));
        }
        if (info_.joints[orig_idx].parameters.count("adc_min") > 0) {
          calibration_data_[i].adc_min = std::stof(info_.joints[orig_idx].parameters.at("adc_min"));
        }
        if (info_.joints[orig_idx].parameters.count("angle_max") > 0) {
          calibration_data_[i].angle_max = std::stof(info_.joints[orig_idx].parameters.at("angle_max"));
        }
        if (info_.joints[orig_idx].parameters.count("adc_max") > 0) {
          calibration_data_[i].adc_max = std::stof(info_.joints[orig_idx].parameters.at("adc_max"));
        }
      }
      
      // Load calibration parameters for dual limit
      if (external_types_[i] == "dual_limit") {
        if (info_.joints[orig_idx].parameters.count("high_limit") > 0) {
          calibration_data_[i].high_limit = std::stof(info_.joints[orig_idx].parameters.at("high_limit"));
        }
        if (info_.joints[orig_idx].parameters.count("low_limit") > 0) {
          calibration_data_[i].low_limit = std::stof(info_.joints[orig_idx].parameters.at("low_limit"));
        }
        if (info_.joints[orig_idx].parameters.count("external_io_high") > 0) {
          calibration_data_[i].external_io_high = std::stoi(info_.joints[orig_idx].parameters.at("external_io_high"));
        }
        if (info_.joints[orig_idx].parameters.count("external_io_low") > 0) {
          calibration_data_[i].external_io_low = std::stoi(info_.joints[orig_idx].parameters.at("external_io_low"));
        }
        if (info_.joints[orig_idx].parameters.count("safety_margin") > 0) {
          calibration_data_[i].safety_margin = std::stof(info_.joints[orig_idx].parameters.at("safety_margin"));
        }
      }
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
  
  // Group 2: XM540 (ID3-6) - Xシリーズテーブル  
  const ControlItem * x_series_position = dynamixel_workbench_.getItemInfo(joint_ids_[2], kPresentPositionItem);
  if (x_series_position == nullptr) {
    RCLCPP_FATAL(rclcpp::get_logger(kDynamixelHardware), "X-series Present_Position not found");
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
  
  // Restore multiturn position from potential sensors (before torque enable)
  restore_multiturn_from_potential();
  
  // Calibrate potential sensor offsets
  calibrate_potential_offsets();
  
  // Safety check: Move dangerous joints to safe positions
  for (uint i = 0; i < joints_.size(); i++) {
    if (external_types_[i] == "potential") {
      emergency_move_to_safe_position(i);
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
    return return_type::OK;
  }

  // 超高速化: static配列とループ最適化
  static const char * log = nullptr;
  static int32_t positions[6] = {0};
  static uint8_t p_series_ids[2] = {1, 2};  // ID固定（constを削除）
  static uint8_t x_series_ids[4] = {3, 4, 5, 6};  // ID固定（constを削除）
  
  // SyncRead実行（最小限）
  dynamixel_workbench_.syncRead(0, p_series_ids, 2, &log);
  dynamixel_workbench_.getSyncReadData(0, p_series_ids, 2, 580, 4, &positions[0], &log);
  
  dynamixel_workbench_.syncRead(1, x_series_ids, 4, &log);
  dynamixel_workbench_.getSyncReadData(1, x_series_ids, 4, 132, 4, &positions[2], &log);
  
  // 結果設定最適化（ループ展開＋関数呼び出し削減）
  static const double inv_reductions[6] = {
    1.0 / mechanical_reductions_[0], 1.0 / mechanical_reductions_[1], 
    1.0 / mechanical_reductions_[2], 1.0 / mechanical_reductions_[3],
    1.0 / mechanical_reductions_[4], 1.0 / mechanical_reductions_[5]
  };
  
  // ループ展開で高速化
  joints_[0].state.position = dynamixel_workbench_.convertValue2Radian(1, positions[0]) * inv_reductions[0];
  joints_[1].state.position = dynamixel_workbench_.convertValue2Radian(2, positions[1]) * inv_reductions[1];
  joints_[2].state.position = dynamixel_workbench_.convertValue2Radian(3, positions[2]) * inv_reductions[2];
  joints_[3].state.position = dynamixel_workbench_.convertValue2Radian(4, positions[3]) * inv_reductions[3];  
  joints_[4].state.position = dynamixel_workbench_.convertValue2Radian(5, positions[4]) * inv_reductions[4];
  joints_[5].state.position = dynamixel_workbench_.convertValue2Radian(6, positions[5]) * inv_reductions[5];
  
  // velocity/effortは0固定（ループなし）
  joints_[0].state.velocity = joints_[1].state.velocity = joints_[2].state.velocity = 0.0;
  joints_[3].state.velocity = joints_[4].state.velocity = joints_[5].state.velocity = 0.0;
  joints_[0].state.effort = joints_[1].state.effort = joints_[2].state.effort = 0.0;
  joints_[3].state.effort = joints_[4].state.effort = joints_[5].state.effort = 0.0;

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

  return return_type::OK;
}

return_type DynamixelHardware::write(
  const rclcpp::Time & /* time */,
  const rclcpp::Duration & /* period */)
{
  if (use_dummy_) {
    for (auto & joint : joints_) {
      joint.prev_command.position = joint.command.position;
      joint.state.position = joint.command.position;
    }
    return return_type::OK;
  }

  // ID:7のリミットチェック
  if (joints_.size() > 6) {  // arm_joint_7が存在する場合
    if (!check_proximity_limit(6, joints_[6].command.position)) {
      // リミット検出時は現在位置を維持
      joints_[6].command.position = joints_[6].state.position;
      joints_[6].command.velocity = 0.0;
    }
  }

  // Force position control for all operations (joystick and MoveIt)
  // Check if velocity commands are set (non-zero) - disabled for position-only control
  bool has_velocity_commands = false;
  // for (const auto& joint : joints_) {
  //   if (std::abs(joint.command.velocity) > 1e-6) {
  //     has_velocity_commands = true;
  //     break;
  //   }
  // }

  if (has_velocity_commands) {
    set_joint_velocities();
  } else {
    set_joint_positions();  // Always use position control
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
    // Don't reset command during torque enable to avoid unexpected movement
    // reset_command();
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

  // Check if any joints have custom operating_mode parameters
  bool has_custom_operating_modes = false;
  for (uint i = 0; i < info_.joints.size(); ++i) {
    if (info_.joints[i].parameters.find("operating_mode") != info_.joints[i].parameters.end()) {
      has_custom_operating_modes = true;
      break;
    }
  }

  // If custom operating modes are set, skip default position control mode setting
  if (has_custom_operating_modes) {
    RCLCPP_INFO(rclcpp::get_logger(kDynamixelHardware), 
                "Custom operating modes detected, skipping default position control setup");
    control_mode_ = ControlMode::Position;  // Assume position-based control
    return return_type::OK;
  }

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

  if (control_mode_ != ControlMode::Position) {
    RCLCPP_FATAL(
      rclcpp::get_logger(kDynamixelHardware), "Only position control is implemented");
    return return_type::ERROR;
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
  static uint8_t x_series_ids[4] = {3, 4, 5, 6};
  static int32_t commands[6];
  
  // prev_command更新（ループ展開）
  joints_[0].prev_command.position = joints_[0].command.position;
  joints_[1].prev_command.position = joints_[1].command.position;
  joints_[2].prev_command.position = joints_[2].command.position;
  joints_[3].prev_command.position = joints_[3].command.position;
  joints_[4].prev_command.position = joints_[4].command.position;
  joints_[5].prev_command.position = joints_[5].command.position;
  
  // 目標位置計算（ループ展開）
  commands[0] = dynamixel_workbench_.convertRadian2Value(1, static_cast<float>(joints_[0].command.position * mechanical_reductions_[0]));
  commands[1] = dynamixel_workbench_.convertRadian2Value(2, static_cast<float>(joints_[1].command.position * mechanical_reductions_[1]));
  commands[2] = dynamixel_workbench_.convertRadian2Value(3, static_cast<float>(joints_[2].command.position * mechanical_reductions_[2]));
  commands[3] = dynamixel_workbench_.convertRadian2Value(4, static_cast<float>(joints_[3].command.position * mechanical_reductions_[3]));
  commands[4] = dynamixel_workbench_.convertRadian2Value(5, static_cast<float>(joints_[4].command.position * mechanical_reductions_[4]));
  commands[5] = dynamixel_workbench_.convertRadian2Value(6, static_cast<float>(joints_[5].command.position * mechanical_reductions_[5]));
  
  // Debug: joint5 command tracking
  static int debug_count = 0;
  if (++debug_count % 100 == 0) {  // Every 100 cycles
    RCLCPP_INFO(rclcpp::get_logger(kDynamixelHardware), 
                "Joint5 cmd=%.3f, prev=%.3f, dxl_cmd=%d", 
                joints_[4].command.position, joints_[4].prev_command.position, commands[4]);
  }
  
  // SyncWrite実行（2グループ）
  dynamixel_workbench_.syncWrite(0, p_series_ids, 2, &commands[0], 1, &log);  // P-series (ID1-2)
  dynamixel_workbench_.syncWrite(1, x_series_ids, 4, &commands[2], 1, &log);  // X-series (ID3-6)
  
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


double DynamixelHardware::convert_external_sensor_to_angle(int joint_index, int32_t adc_value)
{
  const auto& cal = calibration_data_[joint_index];
  
  if (cal.external_type == "potential") {
    // Simple linear interpolation between min and max points
    double adc_range = cal.adc_max - cal.adc_min;
    double angle_range = cal.angle_max - cal.angle_min;
    
    if (adc_range == 0.0) {
      RCLCPP_WARN(rclcpp::get_logger(kDynamixelHardware), 
                  "Joint %d: ADC range is zero, using default conversion", joint_index);
      return static_cast<double>(adc_value) / mechanical_reductions_[joint_index];
    }
    
    // Linear interpolation: angle = angle_min + (adc_value - adc_min) * (angle_range / adc_range)
    double angle = cal.angle_min + (adc_value - cal.adc_min) * (angle_range / adc_range);
    
    return angle;
  }
  
  // Default: return raw value scaled by mechanical reduction
  return static_cast<double>(adc_value) / mechanical_reductions_[joint_index];
}

bool DynamixelHardware::check_proximity_limit(int joint_index, double target_angle)
{
  const auto& cal = calibration_data_[joint_index];
  
  if (cal.external_type == "dual_limit") {
    // 外部I/O 1,2番から実際のリミット状態を読み取り
    bool high_limit_triggered = read_external_io(cal.external_io_high);  // I/O 1番 (High limit)
    bool low_limit_triggered = read_external_io(cal.external_io_low);    // I/O 2番 (Low limit)
    
    // High limit検出時：正方向への動作を制限
    if (high_limit_triggered && target_angle > (joints_[joint_index].state.position + 0.01)) {
      RCLCPP_WARN(rclcpp::get_logger(kDynamixelHardware), 
                  "Joint %d: High limit detected, cannot move positive direction", joint_index);
      return false;
    }
    
    // Low limit検出時：負方向への動作を制限
    if (low_limit_triggered && target_angle < (joints_[joint_index].state.position - 0.01)) {
      RCLCPP_WARN(rclcpp::get_logger(kDynamixelHardware),
                  "Joint %d: Low limit detected, cannot move negative direction", joint_index);
      return false;
    }
    
    return true;
  }
  
  return true;  // 他のジョイントは制限なし
}

bool DynamixelHardware::read_external_io(int io_number)
{
  if (use_dummy_) {
    return false;  // ダミーモードでは常にリミット無し
  }
  
  const char * log = nullptr;
  int32_t external_data = 0;
  
  // DynamixelのExternal_Port_Dataから読み取り
  if (!dynamixel_workbench_.itemRead(joint_ids_[6], "External_Port_Data_1", &external_data, &log)) {
    RCLCPP_ERROR(rclcpp::get_logger(kDynamixelHardware), 
                 "Failed to read External_Port_Data_1: %s", log);
    return false;
  }
  
  // 指定されたI/O番号のビットをチェック
  bool io_state = false;
  if (io_number == 1) {
    io_state = (external_data & 0x01) != 0;  // bit 0
  } else if (io_number == 2) {
    io_state = (external_data & 0x02) != 0;  // bit 1
  }
  
  return io_state;  // Highでリミット検出
}

void DynamixelHardware::restore_multiturn_from_potential()
{
  if (use_dummy_) {
    // ダミーモードでは模擬データでテスト
    RCLCPP_INFO(rclcpp::get_logger(kDynamixelHardware), "Dummy mode: Simulating multiturn restoration");
    for (uint i = 0; i < joints_.size(); i++) {
      if (external_types_[i] == "potential") {
        // 模擬データ: potentialから復元する真の角度
        double true_angle = -1.5 + i * 0.5;  // joint_2: -1.5, joint_3: -1.0
        double dxl_single_turn = 1.2 + i * 0.3;  // 異なる単一回転位置
        int turns = calculate_turn_offset(true_angle, dxl_single_turn);
        
        RCLCPP_INFO(rclcpp::get_logger(kDynamixelHardware), 
                    "Joint %d: Restored multiturn position - True: %.3f rad, Single: %.3f rad, Turns: %d", 
                    i, true_angle, dxl_single_turn, turns);
      }
    }
    multiturn_restored_ = true;
    RCLCPP_INFO(rclcpp::get_logger(kDynamixelHardware), 
                "Multiturn restoration completed. Switching to Dynamixel position feedback.");
    return;
  }
  
  // const char* log = nullptr;  // unused variable削除
  
  
  // Mark multiturn restoration as completed
  multiturn_restored_ = true;
  RCLCPP_INFO(rclcpp::get_logger(kDynamixelHardware), 
              "Multiturn restoration completed. Switching to Dynamixel position feedback.");
}

int DynamixelHardware::calculate_turn_offset(double true_angle, double single_turn_angle)
{
  // 最も近い整数回転数を計算
  double angle_diff = true_angle - single_turn_angle;
  return static_cast<int>(std::round(angle_diff / (2.0 * M_PI)));
}

void DynamixelHardware::set_operating_modes()
{
  const char * log = nullptr;
  
  for (uint i = 0; i < info_.joints.size(); ++i) {
    // operating_modeパラメータが設定されているかチェック
    auto param_it = info_.joints[i].parameters.find("operating_mode");
    if (param_it != info_.joints[i].parameters.end()) {
      int operating_mode = std::stoi(param_it->second);
      
      RCLCPP_INFO(rclcpp::get_logger(kDynamixelHardware), 
                  "Setting joint %s (ID:%d) to operating mode %d", 
                  info_.joints[i].name.c_str(), joint_ids_[i], operating_mode);
      
      if (!dynamixel_workbench_.setOperatingMode(joint_ids_[i], operating_mode, &log)) {
        RCLCPP_ERROR(rclcpp::get_logger(kDynamixelHardware), 
                     "Failed to set operating mode for joint %s: %s", 
                     info_.joints[i].name.c_str(), log);
      } else {
        RCLCPP_INFO(rclcpp::get_logger(kDynamixelHardware),
                    "Successfully set operating mode %d for joint %s", 
                    operating_mode, info_.joints[i].name.c_str());
      }
    }
  }
}

// void DynamixelHardware::torque_enable_service_callback(
//   const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
//   std::shared_ptr<std_srvs::srv::SetBool::Response> response)
// {
//   RCLCPP_INFO(rclcpp::get_logger(kDynamixelHardware), 
//               "Torque enable service called: %s", request->data ? "true" : "false");
//   
//   return_type result = enable_torque(request->data);
//   
//   response->success = (result == return_type::OK);
//   if (response->success) {
//     response->message = request->data ? "Torque enabled" : "Torque disabled";
//   } else {
//     response->message = "Failed to change torque state";
//   }
// }

void DynamixelHardware::calibrate_potential_offsets()
{
  if (use_dummy_) {
    // ダミーモードでは模擬オフセット
    RCLCPP_INFO(rclcpp::get_logger(kDynamixelHardware), "Dummy mode: Simulating offset calibration");
    for (uint i = 0; i < joints_.size(); i++) {
      if (external_types_[i] == "potential") {
        // 模擬オフセット（約-0.035 rad = -2度のずれ）
        potential_offset_map_[i] = -0.035;
        RCLCPP_INFO(rclcpp::get_logger(kDynamixelHardware), 
                    "Joint %d: Dummy offset calibrated: %.3f rad (%.1f deg)", 
                    i, potential_offset_map_[i], potential_offset_map_[i] * 180.0 / M_PI);
      }
    }
    offsets_calibrated_ = true;
    RCLCPP_INFO(rclcpp::get_logger(kDynamixelHardware), 
                "Offset calibration completed (dummy mode)");
    return;
  }

  const char* log = nullptr;
  RCLCPP_INFO(rclcpp::get_logger(kDynamixelHardware), "Calibrating potential sensor offsets...");
  
  for (uint i = 0; i < joints_.size(); i++) {
    if (external_types_[i] == "potential") {
      // 1. potentialセンサーから現在角度を取得
      int32_t external_data;
      if (!dynamixel_workbench_.itemRead(joint_ids_[i], "External_Port_Data_1", &external_data, &log)) {
        RCLCPP_ERROR(rclcpp::get_logger(kDynamixelHardware), 
                     "Failed to read external sensor for joint %d: %s", i, log);
        continue;
      }
      double potential_angle = convert_external_sensor_to_angle(i, external_data);
      
      // 2. DynamixelのPresent_Positionを取得
      int32_t dxl_raw_position;
      if (!dynamixel_workbench_.itemRead(joint_ids_[i], "Present_Position", &dxl_raw_position, &log)) {
        RCLCPP_ERROR(rclcpp::get_logger(kDynamixelHardware), 
                     "Failed to read Present_Position for joint %d: %s", i, log);
        continue;
      }
      double dynamixel_angle = dynamixel_workbench_.convertValue2Radian(joint_ids_[i], dxl_raw_position) / mechanical_reductions_[i];
      
      // 3. オフセット計算（potential基準 - Dynamixel基準）
      double offset = potential_angle - dynamixel_angle;
      potential_offset_map_[i] = offset;
      
      RCLCPP_INFO(rclcpp::get_logger(kDynamixelHardware), 
                  "Joint %d: Offset calibrated - Potential: %.3f rad, Dynamixel: %.3f rad, Offset: %.3f rad (%.1f deg)", 
                  i, potential_angle, dynamixel_angle, offset, offset * 180.0 / M_PI);
    }
  }
  
  offsets_calibrated_ = true;
  last_full_calibration_ = std::chrono::steady_clock::now();
  
  // 初期オフセット履歴を設定
  for (auto& [joint_index, offset] : potential_offset_map_) {
    offset_history_[joint_index] = offset;
  }
  
  RCLCPP_INFO(rclcpp::get_logger(kDynamixelHardware), 
              "Potential sensor offset calibration completed");
}

double DynamixelHardware::apply_potential_offset(int joint_index, double goal_position)
{
  if (!offsets_calibrated_ || external_types_[joint_index] != "potential") {
    return goal_position;  // オフセット未校正または非potentialジョイント
  }
  
  auto it = potential_offset_map_.find(joint_index);
  if (it == potential_offset_map_.end()) {
    return goal_position;  // オフセット情報なし
  }
  
  // MoveItからの目標位置（potential基準）をDynamixel基準に変換
  double corrected_goal = goal_position - it->second;
  
  RCLCPP_DEBUG(rclcpp::get_logger(kDynamixelHardware), 
               "Joint %d: Goal offset applied - Original: %.3f, Corrected: %.3f, Offset: %.3f", 
               joint_index, goal_position, corrected_goal, it->second);
  
  return corrected_goal;
}

double DynamixelHardware::get_corrected_dynamixel_position(int joint_index)
{
  const char* log = nullptr;
  
  // Dynamixelの生のPresent_Positionを取得
  int32_t dxl_raw_position;
  if (!dynamixel_workbench_.itemRead(joint_ids_[joint_index], "Present_Position", &dxl_raw_position, &log)) {
    RCLCPP_ERROR(rclcpp::get_logger(kDynamixelHardware), 
                 "Failed to read Present_Position for joint %d: %s", joint_index, log);
    return 0.0;
  }
  
  double dynamixel_angle = dynamixel_workbench_.convertValue2Radian(joint_ids_[joint_index], dxl_raw_position) / mechanical_reductions_[joint_index];
  
  // オフセットが校正済みの場合、potential基準に変換
  if (offsets_calibrated_ && external_types_[joint_index] == "potential") {
    auto it = potential_offset_map_.find(joint_index);
    if (it != potential_offset_map_.end()) {
      return dynamixel_angle + it->second;  // potential基準に変換
    }
  }
  
  return dynamixel_angle;  // Dynamixel生値
}

double DynamixelHardware::clamp_to_safe_range(int joint_index, double angle)
{
  // Joint-specific safety limits
  switch (joint_index) {
    case 1:  // arm_joint_2 (ID:2)
      // 可動域: 0° ~ -180° (-3.14159 rad)
      return std::clamp(angle, -M_PI, 0.0);
      
    case 2:  // arm_joint_3 (ID:3)
      // 可動域: 0° ~ 180° (3.14159 rad)
      return std::clamp(angle, 0.0, M_PI);
      
    // 他のジョイントも必要に応じて追加
    default:
      return angle;  // 制限なし
  }
}

bool DynamixelHardware::is_in_safe_range(int joint_index, double angle)
{
  double clamped = clamp_to_safe_range(joint_index, angle);
  double tolerance = 0.05;  // 3度程度の許容範囲
  return std::abs(angle - clamped) < tolerance;
}

void DynamixelHardware::emergency_move_to_safe_position(int joint_index)
{
  if (use_dummy_) {
    RCLCPP_WARN(rclcpp::get_logger(kDynamixelHardware), 
                "Dummy mode: Simulating emergency move for joint %d", joint_index);
    return;
  }

  const char* log = nullptr;
  
  // 現在の外部センサー値を取得
  double current_angle = 0.0;
  if (external_types_[joint_index] == "potential") {
    int32_t external_data;
    if (dynamixel_workbench_.itemRead(joint_ids_[joint_index], "External_Port_Data_1", &external_data, &log)) {
      current_angle = convert_external_sensor_to_angle(joint_index, external_data);
    }
  } else {
    // Dynamixel位置を使用
    current_angle = get_corrected_dynamixel_position(joint_index);
  }
  
  // 安全範囲内かチェック
  if (is_in_safe_range(joint_index, current_angle)) {
    return;  // 既に安全範囲内
  }
  
  // 最寄りの安全位置を計算
  double safe_position = clamp_to_safe_range(joint_index, current_angle);
  
  RCLCPP_WARN(rclcpp::get_logger(kDynamixelHardware), 
              "EMERGENCY: Joint %d at unsafe position %.3f rad (%.1f°), moving to safe position %.3f rad (%.1f°)",
              joint_index, current_angle, current_angle * 180.0 / M_PI,
              safe_position, safe_position * 180.0 / M_PI);
  
  // オフセット補正を適用して安全位置に移動
  double corrected_safe_position = apply_potential_offset(joint_index, safe_position);
  
  // 緊急移動実行
  int32_t goal_position = dynamixel_workbench_.convertRadian2Value(
    joint_ids_[joint_index], static_cast<float>(corrected_safe_position) * mechanical_reductions_[joint_index]);
  
  if (!dynamixel_workbench_.itemWrite(joint_ids_[joint_index], kGoalPositionItem, goal_position, &log)) {
    RCLCPP_ERROR(rclcpp::get_logger(kDynamixelHardware), 
                 "EMERGENCY MOVE FAILED for joint %d: %s", joint_index, log);
  } else {
    RCLCPP_INFO(rclcpp::get_logger(kDynamixelHardware), 
                "Emergency move completed for joint %d", joint_index);
  }
}

void DynamixelHardware::smart_offset_management()
{
  if (!offsets_calibrated_) {
    return;  // まだ初期校正されていない
  }
  
  auto now = std::chrono::steady_clock::now();
  
  // 1. 定期的な全体校正チェック（案2の要素）
  if (now - last_full_calibration_ > FULL_RECALIBRATION_INTERVAL) {
    RCLCPP_DEBUG(rclcpp::get_logger(kDynamixelHardware), 
                "Performing periodic full recalibration");
    full_recalibration();
    return;
  }
  
  // 2. 即応的偏差チェック（案1の要素）
  for (uint i = 0; i < joints_.size(); i++) {
    if (external_types_[i] == "potential") {
      update_offset_if_needed(i);
    }
  }
}

void DynamixelHardware::update_offset_if_needed(int joint_index)
{
  double current_offset = calculate_current_offset(joint_index);
  
  auto stored_it = potential_offset_map_.find(joint_index);
  if (stored_it == potential_offset_map_.end()) {
    return;  // オフセット未設定
  }
  
  double stored_offset = stored_it->second;
  double deviation = std::abs(current_offset - stored_offset);
  
  // 閾値を超えた偏差を検出
  if (deviation > OFFSET_DEVIATION_THRESHOLD) {
    RCLCPP_WARN(rclcpp::get_logger(kDynamixelHardware), 
                "Joint %d offset deviation detected: %.3f rad (%.1f deg), updating offset from %.3f to %.3f",
                joint_index, deviation, deviation * 180.0 / M_PI, 
                stored_offset, current_offset);
    
    // オフセット更新（スムージング適用）
    double smoothing_factor = 0.3;  // 30%の重み
    double new_offset = stored_offset * (1 - smoothing_factor) + current_offset * smoothing_factor;
    
    potential_offset_map_[joint_index] = new_offset;
    offset_history_[joint_index] = new_offset;
    
    RCLCPP_INFO(rclcpp::get_logger(kDynamixelHardware), 
                "Joint %d offset updated to %.3f rad (%.1f deg)", 
                joint_index, new_offset, new_offset * 180.0 / M_PI);
  }
}

void DynamixelHardware::full_recalibration()
{
  RCLCPP_INFO(rclcpp::get_logger(kDynamixelHardware), 
              "Starting full offset recalibration");
  
  bool any_changes = false;
  
  for (uint i = 0; i < joints_.size(); i++) {
    if (external_types_[i] == "potential") {
      double current_offset = calculate_current_offset(i);
      
      auto stored_it = potential_offset_map_.find(i);
      if (stored_it != potential_offset_map_.end()) {
        double old_offset = stored_it->second;
        double change = std::abs(current_offset - old_offset);
        
        if (change > 0.005) {  // 0.3度以上の変化
          RCLCPP_INFO(rclcpp::get_logger(kDynamixelHardware), 
                      "Joint %d full recalibration: %.3f -> %.3f rad (change: %.3f rad / %.1f deg)",
                      i, old_offset, current_offset, change, change * 180.0 / M_PI);
          
          potential_offset_map_[i] = current_offset;
          offset_history_[i] = current_offset;
          any_changes = true;
        }
      }
    }
  }
  
  last_full_calibration_ = std::chrono::steady_clock::now();
  
  if (any_changes) {
    RCLCPP_INFO(rclcpp::get_logger(kDynamixelHardware), 
                "Full recalibration completed with updates");
  } else {
    RCLCPP_DEBUG(rclcpp::get_logger(kDynamixelHardware), 
                 "Full recalibration completed - no significant changes");
  }
}

double DynamixelHardware::calculate_current_offset(int joint_index)
{
  if (use_dummy_) {
    // ダミーモードでは模擬的な変動
    return -0.035 + (std::rand() % 10 - 5) * 0.001;  // ±5度のランダム変動
  }
  
  const char* log = nullptr;
  
  // 1. potentialセンサーから現在角度を取得
  int32_t external_data;
  if (!dynamixel_workbench_.itemRead(joint_ids_[joint_index], "External_Port_Data_1", &external_data, &log)) {
    RCLCPP_ERROR(rclcpp::get_logger(kDynamixelHardware), 
                 "Failed to read external sensor for offset calculation: %s", log);
    return 0.0;
  }
  double potential_angle = convert_external_sensor_to_angle(joint_index, external_data);
  
  // 2. DynamixelのPresent_Positionを取得
  int32_t dxl_raw_position;
  if (!dynamixel_workbench_.itemRead(joint_ids_[joint_index], "Present_Position", &dxl_raw_position, &log)) {
    RCLCPP_ERROR(rclcpp::get_logger(kDynamixelHardware), 
                 "Failed to read Present_Position for offset calculation: %s", log);
    return 0.0;
  }
  double dynamixel_angle = dynamixel_workbench_.convertValue2Radian(joint_ids_[joint_index], dxl_raw_position) / mechanical_reductions_[joint_index];
  
  // 3. 現在のオフセット計算
  return potential_angle - dynamixel_angle;
}

}  // namespace dynamixel_hardware

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(dynamixel_hardware::DynamixelHardware, hardware_interface::SystemInterface)
