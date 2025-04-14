# dynamixel_hardware

The [`ros2_control`](https://github.com/ros-controls/ros2_control) implementation for any kind of [ROBOTIS Dynamixel](https://emanual.robotis.com/docs/en/dxl/) robots.

The `dynamixel_hardware` package is the [`SystemInterface`](https://github.com/ros-controls/ros2_control/blob/master/hardware_interface/include/hardware_interface/system_interface.hpp) implementation for the multiple ROBOTIS Dynamixel servos.

It is hopefully compatible any configuration of ROBOTIS Dynamixel servos thanks to the `ros2_control`'s flexible architecture.

## Set up

First [install ROS 2 Rolling on Ubuntu 22.04](http://docs.ros.org/en/rolling/Installation/Ubuntu-Install-Debians.html). Then follow the instruction below.

```shell
$ source /opt/ros/rolling/setup.bash
$ mkdir -p ~/ros/rolling && cd ~/ros/rolling/src
$ git clone https://github.com/youtalk/dynamixel_hardware.git
$ git clone https://github.com/youtalk/dynamixel_hardware_examples.git
$ cd -
$ rosdep install --from-paths src --ignore-src -r -y
$ colcon build --symlink-install --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
$ . install/setup.bash
```

## Demo with real ROBOTIS OpenManipulator-X

### Configure Dynamixel motor parameters

Update the `usb_port`, `baud_rate`, and `joint_ids` parameters on [`open_manipulator_x_description/urdf/open_manipulator_x.ros2_control.xacro`](https://github.com/youtalk/dynamixel_hardware_examples/blob/main/open_manipulator_x_description/urdf/open_manipulator_x.ros2_control.xacro#L9-L12) to correctly communicate with Dynamixel motors.
The `use_dummy` parameter is required if you don't have a real OpenManipulator-X.

Note that `joint_ids` parameters must be splited by `,`.

```xml
<hardware>
  <plugin>dynamixel_hardware/DynamixelHardware</plugin>
  <param name="usb_port">/dev/ttyUSB0</param>
  <param name="baud_rate">1000000</param>
  <!-- <param name="use_dummy">true</param> -->
</hardware>
```

- Terminal 1

Launch the `ros2_control` manager for the OpenManipulator-X.

```shell
$ ros2 launch open_manipulator_x_description open_manipulator_x.launch.py
```

- Terminal 2

Start the `joint_trajectory_controller` and send a `/joint_trajectory_controller/follow_joint_trajectory` goal to move the OpenManipulator-X.

```shell
$ ros2 control switch_controllers --activate joint_state_broadcaster --activate joint_trajectory_controller --deactivate velocity_controller
$ ros2 action send_goal /joint_trajectory_controller/follow_joint_trajectory control_msgs/action/FollowJointTrajectory -f "{
  trajectory: {
    joint_names: [joint1, joint2, joint3, joint4, gripper],
    points: [
      { positions: [0.1, 0.1, 0.1, 0.1, 0], time_from_start: { sec: 2 } },
      { positions: [-0.1, -0.1, -0.1, -0.1, 0], time_from_start: { sec: 4 } },
      { positions: [0, 0, 0, 0, 0], time_from_start: { sec: 6 } }
    ]
  }
}"
```

If you would like to use the velocity control instead, switch to the `velocity_controller` and publish a `/velocity_controller/commands` message to move the OpenManipulator-X.

```shell
$ ros2 control switch_controllers --activate joint_state_broadcaster --deactivate joint_trajectory_controller --activate velocity_controller
$ ros2 topic pub /velocity_controller/commands std_msgs/msg/Float64MultiArray "data: [0.1, 0.1, 0.1, 0.1, 0]"
```

[![dynamixel_control: the ros2_control implementation for any kind of ROBOTIS Dynamixel robots](https://img.youtube.com/vi/EZtBaU-otzI/0.jpg)](https://www.youtube.com/watch?v=EZtBaU-otzI)

## Demo with dummy ROBOTIS OpenManipulator-X

The `use_dummy` parameter is required if you use the dummy OpenManipulator-X.

```diff
diff --git a/open_manipulator_x_description/urdf/open_manipulator_x.ros2_control.xacro b/open_manipulator_x_description/urdf/open_manipulator_x.ros2_control.xacro
index c6cdb74..111846d 100644
--- a/open_manipulator_x_description/urdf/open_manipulator_x.ros2_control.xacro
+++ b/open_manipulator_x_description/urdf/open_manipulator_x.ros2_control.xacro
@@ -9,7 +9,7 @@
         <param name="usb_port">/dev/ttyUSB0</param>
         <param name="baud_rate">1000000</param>
-        <!-- <param name="use_dummy">true</param> -->
+        <param name="use_dummy">true</param>
       </hardware>
       <joint name="joint1">
         <param name="id">11</param>
```

Then follow the same instruction of the real robot one.

Note that the dummy implementation has no interpolation so far.
If you sent a joint message, the robot would move directly to the joints without interpolation.

## Advanced Features

### Joint Offset Management

This implementation includes a special feature for managing joint offsets, particularly useful for robot arms that need to initialize from arbitrary positions:

```cpp
// Example from dynamixel_hardware.cpp
if(i < 3 && offsets_initialized_) {
  joints_[i].state.position = raw_position - joint_offsets_[i];
} else {
  joints_[i].state.position = raw_position;
}
```

The first three joints (typically the base, shoulder, and elbow of a robot arm) automatically have their initial positions recorded as offsets during initialization, treating them as the "zero" position. This feature helps:

- Initialize the robot from any physical position
- Maintain consistent coordinate frames regardless of power-cycle position
- Simplify trajectory planning by establishing a reliable zero position

### Control Mode Switching

The hardware interface supports automatic switching between position and velocity control modes:

```cpp
// Automatically switches based on command type
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
```

This allows you to:
- Use the same hardware interface for both position and velocity control
- Seamlessly transition between control strategies
- Support multiple ROS 2 controller types with the same hardware

### Mechanical Reduction Support

For robots with gearing or other mechanical reductions, you can specify reduction ratios per joint:

```xml
<joint name="joint1">
  <param name="id">11</param>
  <param name="mechanical_reduction">1.0</param>
</joint>
```

The interface handles the conversion between motor angles and actual joint angles based on these ratios.

### Extended Joint Parameters

Each joint can be configured with additional performance parameters:

```cpp
constexpr const char * const kExtraJointParameters[] = {
  "Profile_Velocity", "Profile_Acceleration", "Position_P_Gain",
  "Position_I_Gain", "Position_D_Gain", "Velocity_P_Gain", "Velocity_I_Gain"
};
```

Example configuration in URDF:

```xml
<joint name="joint1">
  <param name="id">11</param>
  <param name="mechanical_reduction">1.0</param>
  <param name="Profile_Velocity">100</param>
  <param name="Position_P_Gain">800</param>
</joint>
```

This allows fine-tuning of joint control parameters without modifying the code.