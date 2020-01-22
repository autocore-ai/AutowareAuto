// Copyright 2020 Apex.AI, Inc.
// Co-developed by Tier IV, Inc. and Apex.AI, Inc.
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
#include "lgsvl_interface/lgsvl_interface.hpp"

#include <algorithm>
#include <cmath>
#include <string>

namespace lgsvl_interface
{

LgsvlInterface::LgsvlInterface(
  rclcpp::Node & node,
  const std::string & sim_cmd_topic,
  const std::string & sim_can_topic,
  const Config & config)
: m_config{config}
{
  if (m_config.m_brake_scale <= 0.0) {
    throw std::domain_error{"Brake scale must be positive"};
  }
  if (m_config.m_throttle_scale <= 0.0) {
    throw std::domain_error{"Throttle scale must be positive"};
  }
  if (m_config.m_steer_scale <= 0.0) {
    throw std::domain_error{"Steer scale must be positive"};
  }

  m_cmd_pub = node.create_publisher<VehicleCmd>(sim_cmd_topic, rclcpp::QoS{10});
  (void)sim_can_topic;  // message type not implemented
}

////////////////////////////////////////////////////////////////////////////////
bool LgsvlInterface::update(std::chrono::nanoseconds timeout)
{
  (void)timeout;
  // Not yet implemented: missing messages!
  return true;
}

////////////////////////////////////////////////////////////////////////////////
bool LgsvlInterface::send_state_command(const autoware_auto_msgs::msg::VehicleStateCommand & msg)
{
  // Only gear data
  using autoware_auto_msgs::msg::VehicleStateCommand;
  m_msg.gear = msg.gear == VehicleStateCommand::GEAR_REVERSE ? 63 : 64;
  // 64 -> gear up, !64 and !0 -> gear down
  return true;
}

////////////////////////////////////////////////////////////////////////////////
bool LgsvlInterface::send_control_command(
  const autoware_auto_msgs::msg::VehicleControlCommand & msg)
{
  publish_and_update(msg.stamp, msg.long_accel_mps2, msg.front_wheel_angle_rad);
  return true;
}

////////////////////////////////////////////////////////////////////////////////
bool LgsvlInterface::send_control_command(const autoware_auto_msgs::msg::RawControlCommand & msg)
{
  const auto brake_val = static_cast<double>(msg.brake) * m_config.m_brake_scale;
  const auto throttle_val = static_cast<double>(msg.throttle) * m_config.m_throttle_scale;
  const auto accel = throttle_val - brake_val;
  const auto steer = static_cast<double>(msg.front_steer) * m_config.m_steer_scale;
  publish_and_update(msg.stamp, accel, steer);
  return true;
}

////////////////////////////////////////////////////////////////////////////////
void LgsvlInterface::publish_and_update(
  const builtin_interfaces::msg::Time stamp,
  const double accel_mps2,
  const double steer_rad)
{
  // Update state
  const auto t = std::chrono::seconds{stamp.sec} + std::chrono::nanoseconds{stamp.nanosec};
  const auto dt = std::chrono::duration_cast<std::chrono::duration<double>>(t - m_last_stamp);
  m_last_stamp = t;
  m_velocity_mps += dt.count() * accel_mps2;
  m_velocity_mps =
    std::min(std::max(m_velocity_mps, m_config.m_velocity_min), m_config.m_velocity_max);
  // Fill message
  m_msg.ctrl_cmd.linear_acceleration = accel_mps2;
  m_msg.twist_cmd.twist.linear.x = m_velocity_mps;
  // tan is only continuous on -PI/2 to PI/2
  constexpr auto PI_2 = 1.57079632679;
  const auto steer_clamped = std::min(std::max(steer_rad, -PI_2), PI_2);
  m_msg.ctrl_cmd.steering_angle = steer_clamped;
  // Kinematic bicycle model
  {
    const auto tanb = (m_config.m_cg_to_rear * std::tan(steer_clamped)) /
      (m_config.m_cg_to_rear + m_config.m_cg_to_front);
    m_msg.twist_cmd.twist.angular.z =
      (m_velocity_mps * std::sin(std::atan(tanb))) / m_config.m_cg_to_rear;
  }
  // Publish
  m_cmd_pub->publish(m_msg);
  // reset state
  m_msg.gear = {};
}

}  // namespace lgsvl_interface
