// Copyright 2020 the Autoware Foundation
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//
// Co-developed by Tier IV, Inc. and Apex.AI, Inc.
/// \file
/// \brief This file defines a class for extended square root covariance filter
#ifndef JOYSTICK_VEHICLE_INTERFACE__JOYSTICK_VEHICLE_INTERFACE_NODE_HPP_
#define JOYSTICK_VEHICLE_INTERFACE__JOYSTICK_VEHICLE_INTERFACE_NODE_HPP_

#include <joystick_vehicle_interface/visibility_control.hpp>

#include <autoware_auto_msgs/msg/high_level_control_command.hpp>
#include <autoware_auto_msgs/msg/raw_control_command.hpp>
#include <autoware_auto_msgs/msg/vehicle_control_command.hpp>
#include <autoware_auto_msgs/msg/vehicle_state_command.hpp>
#include <mpark_variant_vendor/variant.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <std_msgs/msg/u_int8.hpp>
#include <common/types.hpp>

#include <map>
#include <string>

using autoware::common::types::bool8_t;

namespace joystick_vehicle_interface
{

/// Continuously varying control commands
enum class Axes
{
  THROTTLE,  ///< For raw control
  BRAKE,  ///< For raw control
  FRONT_STEER,  ///< For all control
  REAR_STEER,  ///< For all control
  CURVATURE,  ///< For high level control
  ACCELERATION  ///< For normal control
};

/// Discretely varying control commands; not all of VehicleStateCommand is here
enum class Buttons
{
  VELOCITY_UP,  ///< For high level control
  VELOCITY_DOWN,   ///< For high level control
  AUTONOMOUS_TOGGLE,
  HEADLIGHTS_TOGGLE,
  HAND_BRAKE_TOGGLE,
  HORN_TOGGLE,
  WIPER_TOGGLE,
  GEAR_DRIVE,
  GEAR_REVERSE,
  GEAR_PARK,
  GEAR_NEUTRAL,
  GEAR_LOW,
  BLINKER_LEFT,
  BLINKER_RIGHT,
  BLINKER_HAZARD,
  RECORDREPLAY_START_RECORD,
  RECORDREPLAY_START_REPLAY,
  RECORDREPLAY_STOP
};

enum class Recordreplay : uint8_t
{
  NOOP = 0u,
  START_RECORD,
  START_REPLAY,
  STOP
};

/// A node which translates sensor_msgs/msg/Joy messages into messages compatible with the vehicle
/// interface. All participants use SensorDataQoS
class JOYSTICK_VEHICLE_INTERFACE_PUBLIC JoystickVehicleInterfaceNode : public ::rclcpp::Node
{
public:
  using AxisValue = decltype(sensor_msgs::msg::Joy::axes)::value_type;
  using AxisMap = std::map<Axes, decltype(sensor_msgs::msg::Joy::axes)::size_type>;
  using AxisScaleMap = std::map<Axes, AxisValue>;
  using ButtonMap = std::map<Buttons, decltype(sensor_msgs::msg::Joy::buttons)::size_type>;
  static constexpr AxisValue DEFAULT_SCALE = 100.0F;
  static constexpr AxisValue DEFAULT_OFFSET = 0.0F;
  static constexpr AxisValue VELOCITY_INCREMENT = 1.0F;
  /// ROS 2 parameter constructor
  explicit JoystickVehicleInterfaceNode(const rclcpp::NodeOptions & node_options);

private:
  JOYSTICK_VEHICLE_INTERFACE_LOCAL void init(
    const std::string & control_command,
    const std::string & state_command_topic,
    const std::string & joy_topic,
    const bool8_t & recordreplay_command_enabled,
    const AxisMap & axis_map,
    const AxisScaleMap & axis_scale_map,
    const AxisScaleMap & axis_offset_map,
    const ButtonMap & button_map);

  /// Callback for joystick subscription: compute control and state command and publish
  JOYSTICK_VEHICLE_INTERFACE_LOCAL void on_joy(const sensor_msgs::msg::Joy::SharedPtr msg);
  /// Compute state command
  JOYSTICK_VEHICLE_INTERFACE_LOCAL bool8_t update_state_command(const sensor_msgs::msg::Joy & msg);
  /// Given an active button, update the state command
  JOYSTICK_VEHICLE_INTERFACE_LOCAL bool8_t handle_active_button(Buttons button);
  /// Convert raw axis value with affine transform for type
  template<typename T>
  JOYSTICK_VEHICLE_INTERFACE_LOCAL
  void axis_value(const sensor_msgs::msg::Joy & msg, Axes axis, T & value) const
  {
    const auto axis_it = m_axis_map.find(axis);
    if (m_axis_map.end() == axis_it) {
      return;
    }
    const auto axis_idx = axis_it->second;
    if (axis_idx >= msg.axes.size()) {
      return;
    }
    const auto scale_it = m_axis_scale_map.find(axis);
    const auto scale = m_axis_scale_map.end() == scale_it ? DEFAULT_SCALE : scale_it->second;
    const auto val_raw = msg.axes[axis_idx] * scale;
    const auto offset_it = m_axis_offset_map.find(axis);
    const auto offset = m_axis_offset_map.end() == offset_it ? DEFAULT_OFFSET : offset_it->second;
    using ValT = std::decay_t<decltype(value)>;
    value = static_cast<ValT>(val_raw + offset);
  }
  /// Compute control command
  template<typename T>
  JOYSTICK_VEHICLE_INTERFACE_LOCAL T compute_command(const sensor_msgs::msg::Joy & msg);

  using HighLevelControl = autoware_auto_msgs::msg::HighLevelControlCommand;
  using BasicControl = autoware_auto_msgs::msg::VehicleControlCommand;
  using RawControl = autoware_auto_msgs::msg::RawControlCommand;
  template<typename T>
  using PubT = typename rclcpp::Publisher<T>::SharedPtr;

  using ControlPub = mpark::variant<PubT<RawControl>, PubT<BasicControl>, PubT<HighLevelControl>>;

  ControlPub m_cmd_pub{};
  rclcpp::Publisher<autoware_auto_msgs::msg::VehicleStateCommand>::SharedPtr m_state_cmd_pub{};
  rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr m_recordreplay_cmd_pub{};
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr m_joy_sub{nullptr};

  AxisMap m_axis_map{};
  AxisScaleMap m_axis_scale_map{};
  AxisScaleMap m_axis_offset_map{};
  ButtonMap m_button_map{};
  bool8_t m_autonomous{false};
  bool8_t m_wipers_on{false};
  bool8_t m_headlights_on{false};
  bool8_t m_hand_brake_on{false};
  bool8_t m_horn_on{false};
  decltype(HighLevelControl::velocity_mps) m_velocity{};

  autoware_auto_msgs::msg::VehicleStateCommand m_state_command{};
  std_msgs::msg::UInt8 m_recordreplay_command{};
};  // class JoystickVehicleInterfaceNode
}  // namespace joystick_vehicle_interface

#endif  // JOYSTICK_VEHICLE_INTERFACE__JOYSTICK_VEHICLE_INTERFACE_NODE_HPP_
