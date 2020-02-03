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

#include <autoware_auto_msgs/msg/raw_control_command.hpp>
#include <autoware_auto_msgs/msg/vehicle_kinematic_state.hpp>
#include <autoware_auto_msgs/msg/vehicle_state_command.hpp>
#include <autoware_auto_msgs/msg/vehicle_state_report.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2_msgs/msg/tf_message.hpp>

#include <helper_functions/lookup_table.hpp>
#include <rclcpp/rclcpp.hpp>
#include <vehicle_interface/platform_interface.hpp>

#include <chrono>
#include <string>

namespace lgsvl_interface
{

using Table1D = ::autoware::common::helper_functions::LookupTable1D<double>;

class LGSVL_INTERFACE_PUBLIC LgsvlInterface
  : public ::autoware::drivers::vehicle_interface::PlatformInterface
{
public:
  LgsvlInterface(
    rclcpp::Node & node,
    const std::string & sim_cmd_topic,
    const std::string & sim_state_cmd_topic,
    const std::string & sim_state_report_topic,
    const std::string & sim_odom_topic,
    const std::string & kinematic_state_topic,
    Table1D && throttle_table,
    Table1D && brake_table,
    Table1D && steer_table);

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
  // Convert odometry into vehicle kinematic state and tf
  void on_odometry(const nav_msgs::msg::Odometry & msg);

  rclcpp::Publisher<autoware_auto_msgs::msg::RawControlCommand>::SharedPtr m_cmd_pub{};
  rclcpp::Publisher<autoware_auto_msgs::msg::VehicleStateCommand>::SharedPtr m_state_pub{};
  rclcpp::Publisher<autoware_auto_msgs::msg::VehicleKinematicState>::SharedPtr
    m_kinematic_state_pub{};
  rclcpp::Publisher<tf2_msgs::msg::TFMessage>::SharedPtr m_tf_pub{};
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr m_odom_sub{};
  rclcpp::Subscription<autoware_auto_msgs::msg::VehicleStateReport>::SharedPtr m_state_sub{};

  Table1D m_throttle_table;
  Table1D m_brake_table;
  Table1D m_steer_table;

  bool m_odom_set{false};  // TODO(c.ho) this should be optional<Vector3>
  geometry_msgs::msg::Vector3 m_odom_zero{};
};  // class LgsvlInterface

}  // namespace lgsvl_interface

#endif  // LGSVL_INTERFACE__LGSVL_INTERFACE_HPP_
