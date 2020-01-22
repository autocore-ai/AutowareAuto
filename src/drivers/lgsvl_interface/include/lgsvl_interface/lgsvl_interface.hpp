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
/// \file
/// \brief Implementation of interface for LGSVL simulator
#ifndef LGSVL_INTERFACE__LGSVL_INTERFACE_HPP_
#define LGSVL_INTERFACE__LGSVL_INTERFACE_HPP_

#include <lgsvl_interface/visibility_control.hpp>

#include <autoware_msgs/msg/vehicle_cmd.hpp>
#include <builtin_interfaces/msg/time.hpp>
// TODO(c.ho) CanBusData or apollo::canbus::Chassis

#include <rclcpp/rclcpp.hpp>
#include <vehicle_interface/platform_interface.hpp>

#include <chrono>
#include <string>

namespace lgsvl_interface
{

using VehicleCmd = autoware_msgs::msg::VehicleCmd;

// TODO(c.ho) Make this a proper class...
struct Config
{
  double m_throttle_scale{1.0};
  double m_brake_scale{1.0};
  double m_steer_scale{1.0};
  double m_velocity_max{35.0};
  double m_velocity_min{-5.0};
  double m_cg_to_front{1.0};
  double m_cg_to_rear{1.0};
};  // struct Config

class LGSVL_INTERFACE_PUBLIC LgsvlInterface
  : public ::autoware::drivers::vehicle_interface::PlatformInterface
{
public:
  LgsvlInterface(
    rclcpp::Node & node,
    const std::string & sim_cmd_topic,
    const std::string & sim_can_topic,
    const Config & config);

  ~LgsvlInterface() noexcept override = default;
  /// Receives data from ROS 2 subscriber, and updates output messages.
  /// Not yet implemented
  bool update(std::chrono::nanoseconds timeout) override;
  /// Queues up data to be sent along with the next control command.
  /// Only gear shifting between drive and reverse is supported at this time.
  bool send_state_command(const autoware_auto_msgs::msg::VehicleStateCommand & msg) override;
  /// Send control command data with whatever state data came along last
  bool send_control_command(const autoware_auto_msgs::msg::VehicleControlCommand & msg) override;
  /// Send control data with whatever state data came along last; applies scaling here too.
  /// If both brake and throttle is nonzero, decide based on config
  bool send_control_command(const autoware_auto_msgs::msg::RawControlCommand & msg) override;

private:
  void publish_and_update(
    const builtin_interfaces::msg::Time stamp,
    const double accel_mps2,
    const double steer_rad);

  rclcpp::Publisher<VehicleCmd>::SharedPtr m_cmd_pub{};
  // TODO(c.ho) subscriber
  Config m_config;
  VehicleCmd m_msg{};

  double m_velocity_mps{};
  std::chrono::nanoseconds m_last_stamp{};
};  // class LgsvlInterface

}  // namespace lgsvl_interface

#endif  // LGSVL_INTERFACE__LGSVL_INTERFACE_HPP_
