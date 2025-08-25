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
#include <chrono>

#include <hardware_interface/handle.hpp>
#include <hardware_interface/hardware_info.hpp>
#include <hardware_interface/system_interface.hpp>
#include <rclcpp_lifecycle/state.hpp>

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
  Torque,
  Currrent,
  ExtendedPosition,
  MultiTurn,
  CurrentBasedPosition,
  PWM,
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

  return_type reset_command();

  CallbackReturn set_joint_positions();
  CallbackReturn set_joint_velocities();
  CallbackReturn set_joint_params();

  DynamixelWorkbench dynamixel_workbench_;
  std::map<const char * const, const ControlItem *> control_items_;
  std::vector<Joint> joints_;
  std::vector<uint8_t> joint_ids_;
  std::vector<double> mechanical_reductions_;
  bool torque_enabled_{false};
  ControlMode control_mode_{ControlMode::Position};
  bool mode_changed_{false};
  bool use_dummy_{false};

  // ===== 追加メンバ（速度固定運用のため） =====
  bool fixed_velocity_mode_ = true;                  // 速度モード固定
  double alpha_ = 0.8;                               // 速度LPF係数（0.7〜0.9程度）
  double dv_max_ = 0.2;                              // 1周期あたりの速度変化上限 [rad/s/step]
  size_t read_divider_ = 10;                         // read間引き（N周期に1回）
  size_t read_count_ = 0;                            // カウンタ
  rclcpp::Time last_cmd_time_;                       // 最終コマンド時刻
  std::chrono::milliseconds watchdog_timeout_{100};  // 入力ロスト監視 [ms]
  std::vector<double> v_out_;                        // 平滑後の送出速度
  std::vector<double> vel_limit_;                    // 任意：各関節の速度上限 [rad/s]

  // ===== ヘルパ関数 =====
  // LPF＋Δv制限＋飽和をかけて v_out_ を更新（write()から呼ぶ）
  void update_velocity_commands_with_filters(const rclcpp::Duration & period);
  // フィルタ済み速度をGoal_VelocityでSyncWrite送信
  CallbackReturn set_joint_velocities_filtered(const std::vector<double>& vel);

  // 既存: set_joint_velocities()/set_joint_positions() 等はそのまま保持

  // 速度固定時にPosition切替要求を無視するなら set_control_mode 内で判定（cpp側diff参照）


};
}  // namespace dynamixel_hardware

#endif  // DYNAMIXEL_HARDWARE__DYNAMIXEL_HARDWARE_HPP_
