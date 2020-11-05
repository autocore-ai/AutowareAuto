// Copyright 2020 Apex.AI, Inc.
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
/// \brief Implementation of interface for LGSVL simulator
#ifndef LGSVL_INTERFACE__LGSVL_INTERFACE_HPP_
#define LGSVL_INTERFACE__LGSVL_INTERFACE_HPP_

#include <lgsvl_interface/visibility_control.hpp>

#include <autoware_auto_msgs/msg/raw_control_command.hpp>
#include <autoware_auto_msgs/msg/vehicle_kinematic_state.hpp>
#include <autoware_auto_msgs/msg/vehicle_state_command.hpp>
#include <autoware_auto_msgs/msg/vehicle_state_report.hpp>
#include <autoware_auto_msgs/srv/autonomy_mode_change.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2_msgs/msg/tf_message.hpp>

#include <geometry/lookup_table.hpp>
#include <rclcpp/rclcpp.hpp>
#include <vehicle_interface/platform_interface.hpp>

#include <chrono>
#include <string>
#include <unordered_map>
#include <utility>

namespace lgsvl_interface
{

using Table1D = ::autoware::common::helper_functions::LookupTable1D<double>;

// initialise default covariance for each measurement
// if simulator does not provide estimate of a state variable
// variance should be set high
constexpr static double COV_X_VAR = 0.1;  // ros covariance array is float64 = double
constexpr static double COV_Y_VAR = 0.1;
constexpr static double COV_Z_VAR = 0.1;
constexpr static double COV_RX_VAR = 1000.0;
constexpr static double COV_RY_VAR = 1000.0;
constexpr static double COV_RZ_VAR = 1000.0;

// Covariance array index values
constexpr static int32_t COV_X = 0;
constexpr static int32_t COV_Y = 7;
constexpr static int32_t COV_Z = 14;
constexpr static int32_t COV_RX = 21;
constexpr static int32_t COV_RY = 28;
constexpr static int32_t COV_RZ = 35;

constexpr bool PUBLISH = true;
constexpr bool NO_PUBLISH = false;

// in lgsvl 0 is drive and 1 is reverse https://github.com/lgsvl/simulator/blob/cb937deb8e633573f6c0cc76c9f451398b8b9eff/Assets/Scripts/Sensors/VehicleStateSensor.cs#L70
using VSC = autoware_auto_msgs::msg::VehicleStateCommand;
using GEAR_TYPE = decltype(VSC::gear);
enum class LGSVL_GEAR : GEAR_TYPE
{
  DRIVE = 0u,
  REVERSE = 1u,
};

/// Platform interface implementation for LGSVL. Bridges data to and from the simulator
/// where custom logic is required to get simulator data to adhere to ROS conventions.
/// For a full list of behaviors, see \ref lgsvl
class LGSVL_INTERFACE_PUBLIC LgsvlInterface
  : public ::autoware::drivers::vehicle_interface::PlatformInterface
{
public:
  LgsvlInterface(
    rclcpp::Node & node,
    const std::string & sim_cmd_topic,
    const std::string & sim_state_cmd_topic,
    const std::string & sim_state_report_topic,
    const std::string & sim_nav_odom_topic,
    const std::string & sim_veh_odom_topic,
    const std::string & kinematic_state_topic,
    Table1D && throttle_table,
    Table1D && brake_table,
    Table1D && steer_table,
    bool publish_tf = NO_PUBLISH,
    bool publish_pose = PUBLISH);

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
  /// Respond to request for changing autonomy mode. For LGSVL, this means nothing.
  bool handle_mode_change_request(
    autoware_auto_msgs::srv::AutonomyModeChange_Request::SharedPtr request) override;

private:
  // Mapping from Autoware Gear to LGSVL Gear values
  static const std::unordered_map<GEAR_TYPE, GEAR_TYPE> autoware_to_lgsvl_gear;

  // Convert odometry into vehicle kinematic state and pose
  void on_odometry(const nav_msgs::msg::Odometry & msg);

  // store state_report with gear value correction
  void on_state_report(const autoware_auto_msgs::msg::VehicleStateReport & msg);

  rclcpp::Publisher<autoware_auto_msgs::msg::RawControlCommand>::SharedPtr m_cmd_pub{};
  rclcpp::Publisher<autoware_auto_msgs::msg::VehicleStateCommand>::SharedPtr m_state_pub{};
  rclcpp::Publisher<autoware_auto_msgs::msg::VehicleKinematicState>::SharedPtr
    m_kinematic_state_pub{};
  rclcpp::Publisher<tf2_msgs::msg::TFMessage>::SharedPtr m_tf_pub{};
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr m_pose_pub{};
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr m_nav_odom_sub{};
  rclcpp::Subscription<autoware_auto_msgs::msg::VehicleStateReport>::SharedPtr m_state_sub{};
  rclcpp::Subscription<autoware_auto_msgs::msg::VehicleOdometry>::SharedPtr m_veh_odom_sub{};

  Table1D m_throttle_table;
  Table1D m_brake_table;
  Table1D m_steer_table;

  bool m_odom_set{false};  // TODO(c.ho) this should be optional<Vector3>
  geometry_msgs::msg::Vector3 m_odom_zero{};

  rclcpp::Logger m_logger;
};  // class LgsvlInterface

}  // namespace lgsvl_interface

#endif  // LGSVL_INTERFACE__LGSVL_INTERFACE_HPP_
