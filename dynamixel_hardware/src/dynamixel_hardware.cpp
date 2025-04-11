


#include "dynamixel_hardware/dynamixel_hardware.hpp"

#include <algorithm>
#include <limits>
#include <string>
#include <vector>

#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace dynamixel_hardware
{
constexpr const char * kDynamixelHardware = "DynamixelHardware";
constexpr const char * kGoalPositionItem = "Goal_Position";
constexpr const char * kGoalVelocityItem = "Goal_Velocity";
constexpr const char * kMovingSpeedItem = "Moving_Speed";
constexpr const char * kPresentPositionItem = "Present_Position";
constexpr const char * kPresentVelocityItem = "Present_Velocity";
constexpr const char * kPresentSpeedItem = "Present_Speed";
constexpr const char * kPresentCurrentItem = "Present_Current";
constexpr const char * kPresentLoadItem = "Present_Load";

constexpr const char * const kExtraJointParameters[] = {
  "Profile_Velocity", "Profile_Acceleration", "Position_P_Gain",
  "Position_I_Gain", "Position_D_Gain", "Velocity_P_Gain", "Velocity_I_Gain"
};

CallbackReturn DynamixelHardware::on_init(const hardware_interface::HardwareInfo & info)
{
  RCLCPP_DEBUG(rclcpp::get_logger(kDynamixelHardware), "configure");
  if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS) {
    return CallbackReturn::ERROR;
  }

  joints_.resize(info_.joints.size(), Joint());
  joint_ids_.resize(info_.joints.size(), 0);
  mechanical_reductions_.resize(info_.joints.size(), 1.0);
  joint_offsets_.resize(info_.joints.size(), 0.0);
  offsets_initialized_ = false;

  for (uint i = 0; i < info_.joints.size(); i++) {
    joint_ids_[i] = std::stoi(info_.joints[i].parameters.at("id"));
    if (info_.joints[i].parameters.count("mechanical_reduction") > 0) {
      mechanical_reductions_[i] = std::stof(info_.joints[i].parameters.at("mechanical_reduction"));
    }
    joints_[i].state.position = std::numeric_limits<double>::quiet_NaN();
    joints_[i].state.velocity = std::numeric_limits<double>::quiet_NaN();
    joints_[i].state.effort = std::numeric_limits<double>::quiet_NaN();
    joints_[i].command.position = std::numeric_limits<double>::quiet_NaN();
    joints_[i].command.velocity = std::numeric_limits<double>::quiet_NaN();
    joints_[i].command.effort = std::numeric_limits<double>::quiet_NaN();
    joints_[i].prev_command.position = joints_[i].command.position;
    joints_[i].prev_command.velocity = joints_[i].command.velocity;
    joints_[i].prev_command.effort = joints_[i].command.effort;
    RCLCPP_INFO(rclcpp::get_logger(kDynamixelHardware), "joint_id %d: %d", i, joint_ids_[i]);
  }

  if (info_.hardware_parameters.at("use_dummy") == "true") {
    use_dummy_ = true;
    RCLCPP_INFO(rclcpp::get_logger(kDynamixelHardware), "dummy mode");
    return CallbackReturn::SUCCESS;
  }

  const char * log = nullptr;
  auto usb_port = info_.hardware_parameters.at("usb_port");
  auto baud_rate = std::stoi(info_.hardware_parameters.at("baud_rate"));
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
  set_control_mode(ControlMode::Position, true);
  set_joint_params();

  return CallbackReturn::SUCCESS;
}

hardware_interface::return_type DynamixelHardware::read(
  const rclcpp::Time & /* time */,
  const rclcpp::Duration & /* period */)
{
  if (use_dummy_) {
    return return_type::OK;
  }
  const char * log = nullptr;

  for (uint i = 0; i < joint_ids_.size(); ++i){
    int32_t pos = 0, vel = 0, cur = 0;
    uint8_t id = joint_ids_[i];

    dynamixel_workbench_.itemRead(id, kPresentPositionItem, &pos, &log);
    dynamixel_workbench_.itemRead(id, kPresentVelocityItem, &vel, &log);
    dynamixel_workbench_.itemRead(id, kPresentCurrentItem, &cur, &log);

    double raw_position = dynamixel_workbench_.convertValue2Radian(id, pos) / mechanical_reductions_[i];
    
    
    if(i < 3 && offsets_initialized_){ // TODO:(Taiga SASAKI) マジックナンバーの削除
      joints_[i].state.position = raw_position - joint_offsets_[i];
    } else {
      joints_[i].state.position = raw_position;
    }
    
    joints_[i].state.velocity = dynamixel_workbench_.convertValue2Velocity(id, vel) / mechanical_reductions_[i];
    joints_[i].state.effort = dynamixel_workbench_.convertValue2Current(cur) / mechanical_reductions_[i];
  }
  return return_type::OK;
}



return_type DynamixelHardware::write(
  const rclcpp::Time & /* time */,
  const rclcpp::Duration & /* period */)
{
  if (use_dummy_) {
    for (auto & j : joints_) {
      j.prev_command = j.command;
      j.state.position = j.command.position;
    }
    return return_type::OK;
  }

  if (std::any_of(joints_.cbegin(), joints_.cend(), [](auto j) { return j.command.velocity != j.prev_command.velocity; })) {
    set_control_mode(ControlMode::Velocity);
    if (mode_changed_) set_joint_params();
    return set_joint_velocities();
  }

  if (std::any_of(joints_.cbegin(), joints_.cend(), [](auto j) { return j.command.position != j.prev_command.position; })) {
    set_control_mode(ControlMode::Position);
    if (mode_changed_) set_joint_params();
    return set_joint_positions();
  }

  return return_type::OK;
}

hardware_interface::return_type DynamixelHardware::set_joint_positions()
{
  const char * log = nullptr;
  for (uint i = 0; i < joint_ids_.size(); ++i) {
    joints_[i].prev_command.position = joints_[i].command.position;
    double target_position = joints_[i].command.position;

    if(i < 3 && offsets_initialized_)
    {
      target_position = target_position + joint_offsets_[i];
    }

    int32_t raw = dynamixel_workbench_.convertRadian2Value(
      joint_ids_[i], target_position * mechanical_reductions_[i]);
    dynamixel_workbench_.itemWrite(joint_ids_[i], kGoalPositionItem, raw, &log);
  }
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type DynamixelHardware::set_joint_velocities()
{
  const char * log = nullptr;
  for (uint i = 0; i < joint_ids_.size(); ++i) {
    joints_[i].prev_command.velocity = joints_[i].command.velocity;
    int32_t raw = dynamixel_workbench_.convertVelocity2Value(joint_ids_[i], joints_[i].command.velocity * mechanical_reductions_[i]);
    dynamixel_workbench_.itemWrite(joint_ids_[i], kGoalVelocityItem, raw, &log);
  }
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type DynamixelHardware::set_joint_params()
{
  const char * log = nullptr;
  for (uint i = 0; i < info_.joints.size(); ++i) {
    for (auto paramName : kExtraJointParameters) {
      if (info_.joints[i].parameters.find(paramName) != info_.joints[i].parameters.end()) {
        auto value = std::stoi(info_.joints[i].parameters.at(paramName));
        dynamixel_workbench_.itemWrite(joint_ids_[i], paramName, value, &log);
      }
    }
  }
  return hardware_interface::return_type::OK;
}

return_type DynamixelHardware::enable_torque(const bool enabled)
{
  const char * log = nullptr;
  for (uint i = 0; i < joint_ids_.size(); ++i) {
    if (enabled) {
      dynamixel_workbench_.torqueOn(joint_ids_[i], &log);
    } else {
      dynamixel_workbench_.torqueOff(joint_ids_[i], &log);
    }
  }
  torque_enabled_ = enabled;
  return return_type::OK;
}

CallbackReturn DynamixelHardware::on_configure(const rclcpp_lifecycle::State &)
{
  for (auto & j : joints_) {
    if (use_dummy_ && std::isnan(j.state.position)) {
      j.state.position = j.state.velocity = j.state.effort = 0.0;
    }
  }
  
  // 第1,2,3関節を0度に初期化（電源投入後の状態を無視して強制的に0度と仮定）
  // インデックスは0から始まるので注意
  if (!use_dummy_) {

    read(rclcpp::Time{}, rclcpp::Duration(0, 0));

    for (uint i = 0; i < 3 && i < joints_.size(); ++i) {
      joint_offsets_[i] = joints_[i].state.position;
      joints_[i].state.position = 0.0;  // 0度と仮定
      RCLCPP_INFO(rclcpp::get_logger(kDynamixelHardware), 
                "Set joint %d offset to %.4f, position now at 0.0", 
                i, joint_offsets_[i]); 
    }
    offsets_initialized_ = true;
  }
  
  reset_command();
  write(rclcpp::Time{}, rclcpp::Duration(0, 0));
  enable_torque(true);
  return CallbackReturn::SUCCESS;
}

CallbackReturn DynamixelHardware::on_deactivate(
  const rclcpp_lifecycle::State & /* previous_state */)
{
  RCLCPP_DEBUG(rclcpp::get_logger(kDynamixelHardware), "stop");
  return CallbackReturn::SUCCESS;
}

return_type DynamixelHardware::reset_command()
{
  for (auto & j : joints_) {
    j.command.position = j.state.position;
    j.command.velocity = 0.0;
    j.command.effort = 0.0;
    j.prev_command = j.command;
  }
  return return_type::OK;
}

std::vector<hardware_interface::StateInterface> DynamixelHardware::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (uint i = 0; i < info_.joints.size(); i++) {
    state_interfaces.emplace_back(info_.joints[i].name, hardware_interface::HW_IF_POSITION, &joints_[i].state.position);
    state_interfaces.emplace_back(info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &joints_[i].state.velocity);
    state_interfaces.emplace_back(info_.joints[i].name, hardware_interface::HW_IF_EFFORT, &joints_[i].state.effort);
  }
  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> DynamixelHardware::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (uint i = 0; i < info_.joints.size(); i++) {
    command_interfaces.emplace_back(info_.joints[i].name, hardware_interface::HW_IF_POSITION, &joints_[i].command.position);
    command_interfaces.emplace_back(info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &joints_[i].command.velocity);
  }
  return command_interfaces;
}

return_type DynamixelHardware::set_control_mode(const ControlMode & mode, const bool force_set)
{
  const char * log = nullptr;
  mode_changed_ = false;

  if (mode == ControlMode::Velocity && (force_set || control_mode_ != ControlMode::Velocity)) {
    bool torque_enabled = torque_enabled_;
    if (torque_enabled) {
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

  if (mode == ControlMode::Position && (force_set || control_mode_ != ControlMode::Position)) {
    bool torque_enabled = torque_enabled_;
    if (torque_enabled) {
      enable_torque(false);
    }

    for (uint i = 0; i < joint_ids_.size(); ++i) {
      if (!dynamixel_workbench_.setExtendedPositionControlMode(joint_ids_[i], &log)) {
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

  // その他のモードについてのチェック
  if (control_mode_ != ControlMode::Velocity && control_mode_ != ControlMode::Position) {
    RCLCPP_FATAL(
      rclcpp::get_logger(kDynamixelHardware), "Only position/velocity control are implemented");
    return return_type::ERROR;
  }

  return return_type::OK;
}

}  // namespace dynamixel_hardware

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(dynamixel_hardware::DynamixelHardware, hardware_interface::SystemInterface)
