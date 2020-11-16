// Copyright 2020-2021 the Autoware Foundation
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
#ifndef JOYSTICK_VEHICLE_INTERFACE_NODES__JOYSTICK_VEHICLE_INTERFACE_NODE_HPP_
#define JOYSTICK_VEHICLE_INTERFACE_NODES__JOYSTICK_VEHICLE_INTERFACE_NODE_HPP_

#include <joystick_vehicle_interface/joystick_vehicle_interface.hpp>
#include <joystick_vehicle_interface_nodes/visibility_control.hpp>

#include <mpark_variant_vendor/variant.hpp>
#include <rclcpp/rclcpp.hpp>

#include <memory>
#include <string>

using autoware::common::types::bool8_t;

using joystick_vehicle_interface::Axes;
using joystick_vehicle_interface::Buttons;
using joystick_vehicle_interface::AxisMap;
using joystick_vehicle_interface::AxisScaleMap;
using joystick_vehicle_interface::ButtonMap;

namespace joystick_vehicle_interface_nodes
{

/// A node which translates sensor_msgs/msg/Joy messages into messages compatible with the vehicle
/// interface. All participants use SensorDataQoS
class JOYSTICK_VEHICLE_INTERFACE_NODES_PUBLIC JoystickVehicleInterfaceNode : public ::rclcpp::Node
{
public:
  /// ROS 2 parameter constructor
  explicit JoystickVehicleInterfaceNode(const rclcpp::NodeOptions & node_options);

private:
  std::unique_ptr<joystick_vehicle_interface::JoystickVehicleInterface> m_core;
  JOYSTICK_VEHICLE_INTERFACE_NODES_LOCAL void init(
    const std::string & control_command,
    const std::string & state_command_topic,
    const std::string & joy_topic,
    const bool8_t & recordreplay_command_enabled,
    const AxisMap & axis_map,
    const AxisScaleMap & axis_scale_map,
    const AxisScaleMap & axis_offset_map,
    const ButtonMap & button_map);

  /// Callback for joystick subscription: compute control and state command and publish
  JOYSTICK_VEHICLE_INTERFACE_NODES_LOCAL void on_joy(const sensor_msgs::msg::Joy::SharedPtr msg);

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
};  // class JoystickVehicleInterfaceNode
}  // namespace joystick_vehicle_interface_nodes

#endif  // JOYSTICK_VEHICLE_INTERFACE_NODES__JOYSTICK_VEHICLE_INTERFACE_NODE_HPP_
