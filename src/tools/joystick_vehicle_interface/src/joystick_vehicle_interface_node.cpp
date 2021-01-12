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
#include <common/types.hpp>
#include <joystick_vehicle_interface/joystick_vehicle_interface_node.hpp>
#include <rclcpp_components/register_node_macro.hpp>

#include <string>
#include <type_traits>

using autoware::common::types::bool8_t;
using autoware::common::types::float64_t;

namespace joystick_vehicle_interface
{

JoystickVehicleInterfaceNode::JoystickVehicleInterfaceNode(
  const rclcpp::NodeOptions & node_options)
: Node{"joystick_vehicle_interface", node_options}
{
  // topics
  const auto control_command =
    declare_parameter("control_command").get<std::string>();
  const bool recordreplay_command_enabled =
    declare_parameter("recordreplay_command_enabled").get<bool8_t>();

  // maps
  const auto check_set = [this](auto & map, auto key, const std::string & param_name) {
      const auto param = declare_parameter(param_name);
      if (param.get_type() != rclcpp::ParameterType::PARAMETER_NOT_SET) {
        using MapT = std::remove_reference_t<decltype(map)>;
        using ValT = typename MapT::mapped_type;
        const auto val_raw =
          param.get<std::conditional_t<std::is_floating_point<ValT>::value, float64_t, int64_t>>();
        map[key] = static_cast<ValT>(val_raw);
      }
    };
  // axis map
  AxisMap axis_map{};
  check_set(axis_map, Axes::THROTTLE, "axes.throttle");
  check_set(axis_map, Axes::BRAKE, "axes.brake");
  check_set(axis_map, Axes::FRONT_STEER, "axes.front_steer");
  check_set(axis_map, Axes::REAR_STEER, "axes.rear_steer");
  check_set(axis_map, Axes::CURVATURE, "axes.curvature");
  check_set(axis_map, Axes::ACCELERATION, "axes.acceleration");
  // axis scale map
  AxisScaleMap axis_scale_map{};
  check_set(axis_scale_map, Axes::THROTTLE, "axis_scale.throttle");
  check_set(axis_scale_map, Axes::BRAKE, "axis_scale.brake");
  check_set(axis_scale_map, Axes::FRONT_STEER, "axis_scale.front_steer");
  check_set(axis_scale_map, Axes::REAR_STEER, "axis_scale.rear_steer");
  check_set(axis_scale_map, Axes::CURVATURE, "axis_scale.curvature");
  check_set(axis_scale_map, Axes::ACCELERATION, "axis_scale.acceleration");
  // axis offset map
  AxisScaleMap axis_offset_map{};
  check_set(axis_offset_map, Axes::THROTTLE, "axis_offset.throttle");
  check_set(axis_offset_map, Axes::BRAKE, "axis_offset.brake");
  check_set(axis_offset_map, Axes::FRONT_STEER, "axis_offset.front_steer");
  check_set(axis_offset_map, Axes::REAR_STEER, "axis_offset.rear_steer");
  check_set(axis_offset_map, Axes::CURVATURE, "axis_offset.curvature");
  check_set(axis_offset_map, Axes::ACCELERATION, "axis_offset.acceleration");
  // button map
  ButtonMap button_map{};
  check_set(button_map, Buttons::AUTONOMOUS_TOGGLE, "buttons.autonomous");
  check_set(button_map, Buttons::HEADLIGHTS_TOGGLE, "buttons.headlights");
  check_set(button_map, Buttons::WIPER_TOGGLE, "buttons.wiper");
  check_set(button_map, Buttons::GEAR_DRIVE, "buttons.gear_drive");
  check_set(button_map, Buttons::GEAR_REVERSE, "buttons.gear_reverse");
  check_set(button_map, Buttons::GEAR_PARK, "buttons.gear_park");
  check_set(button_map, Buttons::GEAR_NEUTRAL, "buttons.gear_neutral");
  check_set(button_map, Buttons::GEAR_LOW, "buttons.gear_low");
  check_set(button_map, Buttons::BLINKER_LEFT, "buttons.blinker_left");
  check_set(button_map, Buttons::BLINKER_RIGHT, "buttons.blinker_right");
  check_set(button_map, Buttons::BLINKER_HAZARD, "buttons.blinker_hazard");
  check_set(button_map, Buttons::VELOCITY_UP, "buttons.velocity_up");
  check_set(button_map, Buttons::VELOCITY_DOWN, "buttons.velocity_down");
  check_set(button_map, Buttons::RECORDREPLAY_START_RECORD, "buttons.recordreplay_start_record");
  check_set(button_map, Buttons::RECORDREPLAY_START_REPLAY, "buttons.recordreplay_start_replay");
  check_set(button_map, Buttons::RECORDREPLAY_STOP, "buttons.recordreplay_stop");

  // Control commands
  if (control_command == "high_level") {
    m_cmd_pub = create_publisher<HighLevelControl>(
      "high_level_command",
      rclcpp::QoS{10U}.reliable().durability_volatile());
  } else if (control_command == "raw") {
    m_cmd_pub = create_publisher<RawControl>(
      "raw_command",
      rclcpp::QoS{10U}.reliable().durability_volatile());
  } else if (control_command == "basic") {
    m_cmd_pub = create_publisher<BasicControl>(
      "basic_command",
      rclcpp::QoS{10U}.reliable().durability_volatile());
  } else {
    throw std::domain_error
          {"JoystickVehicleInterface does not support " + control_command + "command control mode"};
  }
  // State commands
  m_state_cmd_pub =
    create_publisher<autoware_auto_msgs::msg::VehicleStateCommand>(
    "state_command",
    rclcpp::QoS{10U}.reliable().durability_volatile());

  // Recordreplay command
  if (recordreplay_command_enabled) {
    m_recordreplay_cmd_pub = create_publisher<std_msgs::msg::UInt8>("recordreplay_cmd", 10);
  }

  // Joystick
  m_joy_sub = create_subscription<sensor_msgs::msg::Joy>(
    "joy", rclcpp::SensorDataQoS{},
    [this](const sensor_msgs::msg::Joy::SharedPtr msg) {on_joy(msg);});
  // Maps
  m_axis_map = axis_map;
  m_axis_scale_map = axis_scale_map;
  m_axis_offset_map = axis_offset_map;
  m_button_map = button_map;
}

////////////////////////////////////////////////////////////////////////////////
template<>
JoystickVehicleInterfaceNode::HighLevelControl
JoystickVehicleInterfaceNode::compute_command(const sensor_msgs::msg::Joy & msg)
{
  HighLevelControl ret{};
  {
    ret.stamp = msg.header.stamp;
    ret.velocity_mps = m_velocity;
    axis_value(msg, Axes::CURVATURE, ret.curvature);
  }
  return ret;
}

////////////////////////////////////////////////////////////////////////////////
template<>
JoystickVehicleInterfaceNode::RawControl
JoystickVehicleInterfaceNode::compute_command(const sensor_msgs::msg::Joy & msg)
{
  RawControl ret{};
  ret.stamp = msg.header.stamp;
  axis_value(msg, Axes::BRAKE, ret.brake);
  axis_value(msg, Axes::THROTTLE, ret.throttle);
  axis_value(msg, Axes::FRONT_STEER, ret.front_steer);
  axis_value(msg, Axes::REAR_STEER, ret.rear_steer);
  return ret;
}

////////////////////////////////////////////////////////////////////////////////
template<>
JoystickVehicleInterfaceNode::BasicControl
JoystickVehicleInterfaceNode::compute_command(const sensor_msgs::msg::Joy & msg)
{
  BasicControl ret{};
  {
    ret.stamp = msg.header.stamp;
    axis_value(msg, Axes::ACCELERATION, ret.long_accel_mps2);
    axis_value(msg, Axes::FRONT_STEER, ret.front_wheel_angle_rad);
    axis_value(msg, Axes::REAR_STEER, ret.rear_wheel_angle_rad);
  }
  return ret;
}


////////////////////////////////////////////////////////////////////////////////
void JoystickVehicleInterfaceNode::on_joy(const sensor_msgs::msg::Joy::SharedPtr msg)
{
  // State command: modify state first
  if (update_state_command(*msg)) {
    m_state_cmd_pub->publish(m_state_command);
  }
  // Command publish
  const auto compute_publish_command = [this, &msg](auto && pub) -> void {
      using MessageT =
        typename std::decay_t<decltype(pub)>::element_type::MessageUniquePtr::element_type;
      const auto cmd = compute_command<MessageT>(*msg);
      pub->publish(cmd);
    };
  mpark::visit(compute_publish_command, m_cmd_pub);

  if (m_recordreplay_cmd_pub != nullptr && m_recordreplay_command.data > 0u) {
    m_recordreplay_cmd_pub->publish(m_recordreplay_command);
    m_recordreplay_command.data =
      static_cast<decltype(std_msgs::msg::UInt8::data)>(Recordreplay::NOOP);
  }
}

////////////////////////////////////////////////////////////////////////////////
bool8_t JoystickVehicleInterfaceNode::update_state_command(const sensor_msgs::msg::Joy & msg)
{
  auto ret = false;
  m_state_command = decltype(m_state_command) {};
  m_state_command.stamp = msg.header.stamp;
  for (const auto & button_idx : m_button_map) {
    const auto idx = button_idx.second;
    // Check if button is in range and active
    if (idx < msg.buttons.size()) {
      if (1 == msg.buttons[idx]) {
        ret = handle_active_button(button_idx.first) || ret;
      }
    }
  }
  if (ret) {
    m_state_command.hand_brake = m_hand_brake_on;
    m_state_command.horn = m_horn_on;
  }
  return ret;
}

////////////////////////////////////////////////////////////////////////////////
bool8_t JoystickVehicleInterfaceNode::handle_active_button(Buttons button)
{
  auto ret = true;
  using VSC = decltype(m_state_command);
  switch (button) {
    case Buttons::VELOCITY_UP:  ///< For high level control
      ret = false;
      m_velocity += VELOCITY_INCREMENT;
      break;
    case Buttons::VELOCITY_DOWN:   ///< For high level control
      ret = false;
      m_velocity -= VELOCITY_INCREMENT;
      break;
    case Buttons::AUTONOMOUS_TOGGLE:
      m_state_command.mode = m_autonomous ? VSC::MODE_MANUAL : VSC::MODE_AUTONOMOUS;
      m_autonomous = !m_autonomous;
      break;
    case Buttons::HEADLIGHTS_TOGGLE:
      m_state_command.headlight = m_headlights_on ? VSC::HEADLIGHT_OFF : VSC::HEADLIGHT_ON;
      m_headlights_on = !m_headlights_on;
      break;
    case Buttons::WIPER_TOGGLE:
      m_state_command.wiper = m_wipers_on ? VSC::WIPER_OFF : VSC::WIPER_LOW;
      m_wipers_on = !m_wipers_on;
      break;
    case Buttons::HAND_BRAKE_TOGGLE:
      m_hand_brake_on = !m_hand_brake_on;
      break;
    case Buttons::HORN_TOGGLE:
      m_horn_on = !m_horn_on;
      break;
    case Buttons::GEAR_DRIVE:
      m_state_command.gear = VSC::GEAR_DRIVE;
      break;
    case Buttons::GEAR_REVERSE:
      m_state_command.gear = VSC::GEAR_REVERSE;
      break;
    case Buttons::GEAR_PARK:
      m_state_command.gear = VSC::GEAR_PARK;
      break;
    case Buttons::GEAR_NEUTRAL:
      m_state_command.gear = VSC::GEAR_NEUTRAL;
      break;
    case Buttons::GEAR_LOW:
      m_state_command.gear = VSC::GEAR_LOW;
      break;
    case Buttons::BLINKER_LEFT:
      m_state_command.blinker = VSC::BLINKER_LEFT;
      break;
    case Buttons::BLINKER_RIGHT:
      m_state_command.blinker = VSC::BLINKER_RIGHT;
      break;
    case Buttons::BLINKER_HAZARD:
      m_state_command.blinker = VSC::BLINKER_HAZARD;
      break;
    case Buttons::RECORDREPLAY_START_RECORD:
      m_recordreplay_command.data =
        static_cast<decltype(std_msgs::msg::UInt8::data)>(Recordreplay::START_RECORD);
      break;
    case Buttons::RECORDREPLAY_START_REPLAY:
      m_recordreplay_command.data =
        static_cast<decltype(std_msgs::msg::UInt8::data)>(Recordreplay::START_REPLAY);
      break;
    case Buttons::RECORDREPLAY_STOP:
      m_recordreplay_command.data =
        static_cast<decltype(std_msgs::msg::UInt8::data)>(Recordreplay::STOP);
      break;
    default:
      throw std::logic_error{"Impossible button was pressed"};
  }
  return ret;
}

}  // namespace joystick_vehicle_interface

RCLCPP_COMPONENTS_REGISTER_NODE(joystick_vehicle_interface::JoystickVehicleInterfaceNode)
