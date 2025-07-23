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
  RCLCPP_DEBUG(rclcpp::get_logger(kDynamixelHardware), "configure");
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

  for (uint i = 0; i < info_.joints.size(); i++) {
    joint_ids_[i] = std::stoi(info_.joints[i].parameters.at("id"));
    if (info_.joints[i].parameters.count("mechanical_reduction") > 0) {
      mechanical_reductions_[i] = std::stof(info_.joints[i].parameters.at("mechanical_reduction"));
    }
    if (info_.joints[i].parameters.count("external_type") > 0) {
      external_types_[i] = info_.joints[i].parameters.at("external_type");
      calibration_data_[i].external_type = external_types_[i];
      
      // Load calibration parameters for potentiometer
      if (external_types_[i] == "potential") {
        if (info_.joints[i].parameters.count("angle_min") > 0) {
          calibration_data_[i].angle_min = std::stof(info_.joints[i].parameters.at("angle_min"));
        }
        if (info_.joints[i].parameters.count("adc_min") > 0) {
          calibration_data_[i].adc_min = std::stof(info_.joints[i].parameters.at("adc_min"));
        }
        if (info_.joints[i].parameters.count("angle_max") > 0) {
          calibration_data_[i].angle_max = std::stof(info_.joints[i].parameters.at("angle_max"));
        }
        if (info_.joints[i].parameters.count("adc_max") > 0) {
          calibration_data_[i].adc_max = std::stof(info_.joints[i].parameters.at("adc_max"));
        }
      }
      
      // Load calibration parameters for dual limit
      if (external_types_[i] == "dual_limit") {
        if (info_.joints[i].parameters.count("high_limit") > 0) {
          calibration_data_[i].high_limit = std::stof(info_.joints[i].parameters.at("high_limit"));
        }
        if (info_.joints[i].parameters.count("low_limit") > 0) {
          calibration_data_[i].low_limit = std::stof(info_.joints[i].parameters.at("low_limit"));
        }
        if (info_.joints[i].parameters.count("external_io_high") > 0) {
          calibration_data_[i].external_io_high = std::stoi(info_.joints[i].parameters.at("external_io_high"));
        }
        if (info_.joints[i].parameters.count("external_io_low") > 0) {
          calibration_data_[i].external_io_low = std::stoi(info_.joints[i].parameters.at("external_io_low"));
        }
        if (info_.joints[i].parameters.count("safety_margin") > 0) {
          calibration_data_[i].safety_margin = std::stof(info_.joints[i].parameters.at("safety_margin"));
        }
      }
    }
    joints_[i].state.position = std::numeric_limits<double>::quiet_NaN();
    joints_[i].state.effort = std::numeric_limits<double>::quiet_NaN();
    joints_[i].command.position = std::numeric_limits<double>::quiet_NaN();
    joints_[i].command.effort = std::numeric_limits<double>::quiet_NaN();
    joints_[i].prev_command.position = joints_[i].command.position;
    joints_[i].prev_command.effort = joints_[i].command.effort;
    RCLCPP_INFO(rclcpp::get_logger(kDynamixelHardware), "joint_id %d: %d, external_type: %s", 
                i, joint_ids_[i], external_types_[i].c_str());
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

  RCLCPP_INFO(rclcpp::get_logger(kDynamixelHardware), "usb_port: %s", usb_port.c_str());
  RCLCPP_INFO(rclcpp::get_logger(kDynamixelHardware), "baud_rate: %d", baud_rate);

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

  const ControlItem * goal_position =
    dynamixel_workbench_.getItemInfo(joint_ids_[0], kGoalPositionItem);
  if (goal_position == nullptr) {
    return CallbackReturn::ERROR;
  }


  const ControlItem * present_position =
    dynamixel_workbench_.getItemInfo(joint_ids_[0], kPresentPositionItem);
  if (present_position == nullptr) {
    return CallbackReturn::ERROR;
  }


  const ControlItem * present_current =
    dynamixel_workbench_.getItemInfo(joint_ids_[0], kPresentCurrentItem);
  if (present_current == nullptr) {
    present_current = dynamixel_workbench_.getItemInfo(joint_ids_[0], kPresentLoadItem);
  }
  if (present_current == nullptr) {
    return CallbackReturn::ERROR;
  }

  control_items_[kGoalPositionItem] = goal_position;
  control_items_[kPresentPositionItem] = present_position;
  control_items_[kPresentCurrentItem] = present_current;

  if (is_external_pos_) {
    const ControlItem * external_port =
      dynamixel_workbench_.getItemInfo(joint_ids_[0], kExternalPortItem);
    if (external_port == nullptr) {
      RCLCPP_ERROR(rclcpp::get_logger(kDynamixelHardware), "External_Port_Data_1 not found");
      return CallbackReturn::ERROR;
    }
    control_items_[kExternalPortItem] = external_port;
  }

  if (!dynamixel_workbench_.addSyncWriteHandler(
      control_items_[kGoalPositionItem]->address, control_items_[kGoalPositionItem]->data_length,
      &log))
  {
    RCLCPP_FATAL(rclcpp::get_logger(kDynamixelHardware), "%s", log);
    return CallbackReturn::ERROR;
  }


  uint16_t start_address = std::min(
    control_items_[kPresentPositionItem]->address, control_items_[kPresentCurrentItem]->address);
  uint16_t read_length = control_items_[kPresentPositionItem]->data_length +
    control_items_[kPresentCurrentItem]->data_length + 2;
  if (!dynamixel_workbench_.addSyncReadHandler(start_address, read_length, &log)) {
    RCLCPP_FATAL(rclcpp::get_logger(kDynamixelHardware), "%s", log);
    return CallbackReturn::ERROR;
  }

  if (is_external_pos_) {
    if (!dynamixel_workbench_.addSyncReadHandler(
        control_items_[kExternalPortItem]->address, control_items_[kExternalPortItem]->data_length, &log)) {
      RCLCPP_FATAL(rclcpp::get_logger(kDynamixelHardware), "%s", log);
      return CallbackReturn::ERROR;
    }
  }

  return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> DynamixelHardware::export_state_interfaces()
{
  RCLCPP_DEBUG(rclcpp::get_logger(kDynamixelHardware), "export_state_interfaces");
  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (uint i = 0; i < info_.joints.size(); i++) {
    state_interfaces.emplace_back(
      hardware_interface::StateInterface(
        info_.joints[i].name, hardware_interface::HW_IF_POSITION, &joints_[i].state.position));
    state_interfaces.emplace_back(
      hardware_interface::StateInterface(
        info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &joints_[i].state.velocity));
    state_interfaces.emplace_back(
      hardware_interface::StateInterface(
        info_.joints[i].name, hardware_interface::HW_IF_EFFORT, &joints_[i].state.effort));
  }

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> DynamixelHardware::export_command_interfaces()
{
  RCLCPP_DEBUG(rclcpp::get_logger(kDynamixelHardware), "export_command_interfaces");
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (uint i = 0; i < info_.joints.size(); i++) {
    command_interfaces.emplace_back(
      hardware_interface::CommandInterface(
        info_.joints[i].name, hardware_interface::HW_IF_POSITION, &joints_[i].command.position));
    command_interfaces.emplace_back(
      hardware_interface::CommandInterface(
        info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &joints_[i].command.velocity));
  }

  return command_interfaces;
}

CallbackReturn DynamixelHardware::on_configure(const rclcpp_lifecycle::State & /* previous_state */)
{
  RCLCPP_DEBUG(rclcpp::get_logger(kDynamixelHardware), "start");
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
  
  // Don't write initial position commands to avoid unexpected movement
  // write(rclcpp::Time{}, rclcpp::Duration(0, 0));

  enable_torque(true);

  return CallbackReturn::SUCCESS;
}

CallbackReturn DynamixelHardware::on_deactivate(
  const rclcpp_lifecycle::State & /* previous_state */)
{
  RCLCPP_DEBUG(rclcpp::get_logger(kDynamixelHardware), "stop");
  return CallbackReturn::SUCCESS;
}

return_type DynamixelHardware::read(
  const rclcpp::Time & /* time */,
  const rclcpp::Duration & /* period */)
{
  if (use_dummy_) {
    return return_type::OK;
  }

  const char * log = nullptr;
  std::vector<int32_t> external_positions(info_.joints.size(), 0);

  // 個別Read方式に変更 - 異なる型番のサーボ(PH42/PM54/XM540)に対応
  for (uint i = 0; i < info_.joints.size(); i++) {
    int32_t position = 0;
    int32_t current = 0;
    
    // 各サーボから個別に位置情報を読み取り
    if (!dynamixel_workbench_.itemRead(joint_ids_[i], kPresentPositionItem, &position, &log)) {
      RCLCPP_ERROR(rclcpp::get_logger(kDynamixelHardware), "ID %d position read failed: %s", joint_ids_[i], log);
      continue;
    }
    
    // 各サーボから個別に電流情報を読み取り
    if (!dynamixel_workbench_.itemRead(joint_ids_[i], kPresentCurrentItem, &current, &log)) {
      // Present_Currentが無い場合はPresent_Loadを試行
      if (!dynamixel_workbench_.itemRead(joint_ids_[i], kPresentLoadItem, &current, &log)) {
        RCLCPP_ERROR(rclcpp::get_logger(kDynamixelHardware), "ID %d current/load read failed: %s", joint_ids_[i], log);
        current = 0;
      }
    }
    
    // 位置と電流値を設定
    joints_[i].state.position = dynamixel_workbench_.convertValue2Radian(joint_ids_[i], position) / mechanical_reductions_[i];
    joints_[i].state.effort = dynamixel_workbench_.convertValue2Current(current) / mechanical_reductions_[i];
  }

  // External position handling (if enabled)
  if (is_external_pos_) {
    for (uint i = 0; i < info_.joints.size(); i++) {
      int32_t external_data = 0;
      if (!dynamixel_workbench_.itemRead(joint_ids_[i], kExternalPortItem, &external_data, &log)) {
        RCLCPP_ERROR(rclcpp::get_logger(kDynamixelHardware), "ID %d external port read failed: %s", joint_ids_[i], log);
        external_data = 0;
      }
      
      // Apply EMA filter for potentiometer joints to reduce noise
      if (external_types_[i] == "potential") {
        external_data = static_cast<int32_t>(adc_filters_[i].update(static_cast<double>(external_data)));
      }
      
      external_positions[i] = external_data;
    }

    // Apply external position logic
    for (uint i = 0; i < info_.joints.size(); i++) {
      // Use external sensor for position based on configuration
      if (is_external_pos_ && external_types_[i] == "potential") {
        // Use external potentiometer reading with calibration
        joints_[i].state.position = convert_external_sensor_to_angle(i, external_positions[i]);
      }
      // For joints without external sensors (external_type != "potential"), 
      // keep using Dynamixel internal position (already set above)
    }
  }

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

  // Check if velocity commands are set (non-zero)
  bool has_velocity_commands = false;
  for (const auto& joint : joints_) {
    if (std::abs(joint.command.velocity) > 1e-6) {
      has_velocity_commands = true;
      break;
    }
  }

  if (has_velocity_commands) {
    set_joint_velocities();
  } else {
    set_joint_positions();
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
    bool torque_enabled = torque_enabled_;
    if (torque_enabled) {
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
  const char * log = nullptr;

  // 個別Write方式に変更 - 異なる型番のサーボ(PH42/PM54/XM540)に対応
  for (uint i = 0; i < info_.joints.size(); i++) {
    joints_[i].prev_command.position = joints_[i].command.position;
    
    // 各サーボへ個別にGoal_Positionを送信
    int32_t goal_position = dynamixel_workbench_.convertRadian2Value(
      joint_ids_[i], static_cast<float>(joints_[i].command.position) * mechanical_reductions_[i]);
    
    if (!dynamixel_workbench_.itemWrite(joint_ids_[i], kGoalPositionItem, goal_position, &log)) {
      RCLCPP_ERROR(rclcpp::get_logger(kDynamixelHardware), "ID %d goal position write failed: %s", joint_ids_[i], log);
    }
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
  const char * log = nullptr;

  // 個別Write方式に変更 - 異なる型番のサーボ(PH42/PM54/XM540)に対応
  for (uint i = 0; i < info_.joints.size(); i++) {
    joints_[i].prev_command.velocity = joints_[i].command.velocity;
    
    // 各サーボへ個別にGoal_Velocityを送信
    int32_t goal_velocity = dynamixel_workbench_.convertVelocity2Value(
      joint_ids_[i], static_cast<float>(joints_[i].command.velocity) * mechanical_reductions_[i]);
    
    if (!dynamixel_workbench_.itemWrite(joint_ids_[i], kGoalVelocityItem, goal_velocity, &log)) {
      RCLCPP_ERROR(rclcpp::get_logger(kDynamixelHardware), "ID %d goal velocity write failed: %s", joint_ids_[i], log);
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
  
  const char* log = nullptr;
  
  
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

}  // namespace dynamixel_hardware

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(dynamixel_hardware::DynamixelHardware, hardware_interface::SystemInterface)
