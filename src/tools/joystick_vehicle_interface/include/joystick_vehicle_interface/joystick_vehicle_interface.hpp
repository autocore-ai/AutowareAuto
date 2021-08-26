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
/// \brief This file defines a core class for the joystick vehicle interface node.
#ifndef JOYSTICK_VEHICLE_INTERFACE__JOYSTICK_VEHICLE_INTERFACE_HPP_
#define JOYSTICK_VEHICLE_INTERFACE__JOYSTICK_VEHICLE_INTERFACE_HPP_

#include <autoware_auto_msgs/msg/headlights_command.hpp>
#include <autoware_auto_msgs/msg/wipers_command.hpp>
#include <autoware_auto_msgs/msg/high_level_control_command.hpp>
#include <autoware_auto_msgs/msg/raw_control_command.hpp>
#include <autoware_auto_msgs/msg/vehicle_control_command.hpp>
#include <autoware_auto_msgs/msg/vehicle_state_command.hpp>
#include <joystick_vehicle_interface/visibility_control.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <std_msgs/msg/u_int8.hpp>
#include <common/types.hpp>

#include <map>

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
  ACCELERATION,  ///< For normal control
  VELOCITY  ///< For nomal control
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

using AxisValue = decltype(sensor_msgs::msg::Joy::axes)::value_type;
using AxisMap = std::map<Axes, decltype(sensor_msgs::msg::Joy::axes)::size_type>;
using AxisScaleMap = std::map<Axes, AxisValue>;
using ButtonMap = std::map<Buttons, decltype(sensor_msgs::msg::Joy::buttons)::size_type>;

static constexpr AxisValue DEFAULT_SCALE = 100.0F;
static constexpr AxisValue DEFAULT_OFFSET = 0.0F;
static constexpr AxisValue VELOCITY_INCREMENT = 1.0F;

using autoware_auto_msgs::msg::HeadlightsCommand;
using autoware_auto_msgs::msg::WipersCommand;
using autoware_auto_msgs::msg::VehicleStateCommand;

/// A core class which performs all basic functions which are not ROS-related for
/// joystick_vehicle_interface.
class JOYSTICK_VEHICLE_INTERFACE_PUBLIC JoystickVehicleInterface
{
public:
  JoystickVehicleInterface(
    const AxisMap & axis_map,
    const AxisScaleMap & axis_scale_map,
    const AxisScaleMap & axis_offset_map,
    const ButtonMap & button_map);
  /// Compute state command
  bool8_t update_state_command(const sensor_msgs::msg::Joy & msg);
  /// Compute control command
  template<typename T>
  T compute_command(const sensor_msgs::msg::Joy & msg);
  void reset_recordplay();
  const VehicleStateCommand & get_state_command();
  const VehicleStateCommand & get_previous_state_command();
  const std_msgs::msg::UInt8 & get_recordreplay_command();
  void update_headlights_state(
    const autoware_auto_msgs::msg::HeadlightsCommand & headlights_cmd);

private:
  /// Given an active button, update the state command
  JOYSTICK_VEHICLE_INTERFACE_LOCAL bool8_t handle_active_button(Buttons button);
  /// Convert raw axis value with affine transform for type
  template<typename T>
  JOYSTICK_VEHICLE_INTERFACE_LOCAL void axis_value(
    const sensor_msgs::msg::Joy & msg,
    Axes axis, T & value) const;

  using HighLevelControl = autoware_auto_msgs::msg::HighLevelControlCommand;
  using BasicControl = autoware_auto_msgs::msg::VehicleControlCommand;
  using RawControl = autoware_auto_msgs::msg::RawControlCommand;

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

  VehicleStateCommand m_state_command{};
  VehicleStateCommand m_previous_state_command{};
  std_msgs::msg::UInt8 m_recordreplay_command{};
};  // class JoystickVehicleInterface
}  // namespace joystick_vehicle_interface

#endif  // JOYSTICK_VEHICLE_INTERFACE__JOYSTICK_VEHICLE_INTERFACE_HPP_
