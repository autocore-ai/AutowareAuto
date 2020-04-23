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
#include <common/types.hpp>
#include <tf2/LinearMath/Quaternion.h>

#include <algorithm>
#include <cmath>
#include <limits>
#include <string>
#include <unordered_map>
#include <utility>


#include "lgsvl_interface/lgsvl_interface.hpp"

using autoware::common::types::bool8_t;
using autoware::common::types::float32_t;

namespace lgsvl_interface
{

const std::unordered_map<GEAR_TYPE, GEAR_TYPE> LgsvlInterface::autoware_to_lgsvl_gear {
  {VSC::GEAR_DRIVE, static_cast<GEAR_TYPE>(LGSVL_GEAR::DRIVE)},               // Drive
  {VSC::GEAR_REVERSE, static_cast<GEAR_TYPE>(LGSVL_GEAR::REVERSE)},           // Reverse
};

LgsvlInterface::LgsvlInterface(
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
  bool publish_tf,
  bool publish_pose)
: m_throttle_table{throttle_table},
  m_brake_table{brake_table},
  m_steer_table{steer_table},
  m_logger{node.get_logger()}
{
  const auto check = [](const auto value, const auto ref) -> bool8_t {
      return std::fabs(value - ref) > std::numeric_limits<decltype(value)>::epsilon();
    };
  // check throttle table
  if (check(m_throttle_table.domain().front(), 0.0)) {
    throw std::domain_error{"Throttle table domain must be [0, ...)"};
  }
  if (check(m_throttle_table.range().front(), 0.0) ||
    check(m_throttle_table.range().back(), 100.0))
  {
    throw std::domain_error{"Throttle table range must go from 0 to 100"};
  }
  for (const auto val : m_throttle_table.range()) {
    if (val < 0.0) {
      throw std::domain_error{"Throttle table must map to nonnegative accelerations"};
    }
  }
  // Check brake table
  if (check(m_brake_table.domain().back(), 0.0)) {
    throw std::domain_error{"Brake table domain must be [..., 0)"};
  }
  if (check(m_brake_table.range().front(), 100.0) || check(m_brake_table.range().back(), 0.0)) {
    throw std::domain_error{"Brake table must go from 100 to 0"};
  }
  for (const auto val : m_brake_table.domain()) {
    if (val > 0.0) {
      throw std::domain_error{"Brake table must map negative accelerations to 0-100 values"};
    }
  }
  // Check steer table
  if (check(m_steer_table.range().front(), -100.0) || check(m_steer_table.range().back(), 100.0)) {
    throw std::domain_error{"Steer table must go from -100 to 100"};
  }
  if ((m_steer_table.domain().front() >= 0.0) ||  // Should be negative...
    (m_steer_table.domain().back() <= 0.0) ||  // to positive...
    check(m_steer_table.domain().back(), -m_steer_table.domain().front()))  // with symmetry
  {
    // Warn if steer domain is not equally straddling zero: could be right, but maybe not
    RCLCPP_WARN(node.get_logger(), "Steer table domain is not symmetric across zero. Is this ok?");
  }

  // Make publishers
  m_cmd_pub = node.create_publisher<autoware_auto_msgs::msg::RawControlCommand>(
    sim_cmd_topic, rclcpp::QoS{10});
  m_state_pub = node.create_publisher<autoware_auto_msgs::msg::VehicleStateCommand>(
    sim_state_cmd_topic, rclcpp::QoS{10});
  // Make subscribers
  if (!sim_nav_odom_topic.empty() && ("null" != sim_nav_odom_topic)) {
    m_nav_odom_sub = node.create_subscription<nav_msgs::msg::Odometry>(sim_nav_odom_topic,
        rclcpp::QoS{10},
        [this](nav_msgs::msg::Odometry::SharedPtr msg) {on_odometry(*msg);});
    // Ground truth state/pose publishers only work if there's a ground truth input
    m_kinematic_state_pub = node.create_publisher<autoware_auto_msgs::msg::VehicleKinematicState>(
      kinematic_state_topic, rclcpp::QoS{10});

    if (publish_pose) {
      m_pose_pub = node.create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
        "/gnss/pose", rclcpp::QoS{10});
    }

    if (publish_tf) {
      m_tf_pub = node.create_publisher<tf2_msgs::msg::TFMessage>("/tf", rclcpp::QoS{10});
    }
  }

  m_state_sub = node.create_subscription<autoware_auto_msgs::msg::VehicleStateReport>(
    sim_state_report_topic,
    rclcpp::QoS{10},
    [this](autoware_auto_msgs::msg::VehicleStateReport::SharedPtr msg) {on_state_report(*msg);});

  m_veh_odom_sub = node.create_subscription<autoware_auto_msgs::msg::VehicleOdometry>(
    sim_veh_odom_topic,
    rclcpp::QoS{10},
    [this](autoware_auto_msgs::msg::VehicleOdometry::SharedPtr msg) {odometry() = *msg;});
}

////////////////////////////////////////////////////////////////////////////////
bool8_t LgsvlInterface::update(std::chrono::nanoseconds timeout)
{
  (void)timeout;
  // Not implemented: API is not needed since everything is handled by subscription callbacks
  return true;
}

////////////////////////////////////////////////////////////////////////////////
bool8_t LgsvlInterface::send_state_command(const autoware_auto_msgs::msg::VehicleStateCommand & msg)
{
  auto msg_corrected = msg;

  // in autoware_auto_msgs::msg::VehicleStateCommand 1 is drive, 2 is reverse, https://gitlab.com/autowarefoundation/autoware.auto/AutowareAuto/-/blob/9744f6dc/src/messages/autoware_auto_msgs/msg/VehicleStateCommand.msg#L32
  // in lgsvl 0 is drive and 1 is reverse https://github.com/lgsvl/simulator/blob/cb937deb8e633573f6c0cc76c9f451398b8b9eff/Assets/Scripts/Sensors/VehicleStateSensor.cs#L70

  auto const iter = autoware_to_lgsvl_gear.find(msg.gear);

  if (iter != autoware_to_lgsvl_gear.end()) {
    msg_corrected.gear = iter->second;
  } else {
    msg_corrected.gear = static_cast<uint8_t>(LGSVL_GEAR::DRIVE);
    RCLCPP_WARN(m_logger, "Unsupported gear value in state command, defaulting to Drive");
  }

  // Correcting blinker, they are shifted down by one,
  // as the first value BLINKER_NO_COMMAND does not exisit in LGSVL
  if (msg.blinker == VSC::BLINKER_NO_COMMAND) {
    msg_corrected.blinker = get_state_report().blinker;
  }
  msg_corrected.blinker--;

  m_state_pub->publish(msg_corrected);
  return true;
}

////////////////////////////////////////////////////////////////////////////////
bool8_t LgsvlInterface::send_control_command(
  const autoware_auto_msgs::msg::VehicleControlCommand & msg)
{
  autoware_auto_msgs::msg::RawControlCommand raw_msg;
  raw_msg.stamp = msg.stamp;
  raw_msg.throttle = 0;
  raw_msg.brake = 0;

  using VSR = autoware_auto_msgs::msg::VehicleStateReport;
  const auto directional_accel = get_state_report().gear ==
    VSR::GEAR_REVERSE ? -msg.long_accel_mps2 : msg.long_accel_mps2;

  if (directional_accel >= decltype(msg.long_accel_mps2) {}) {
    // TODO(c.ho)  cast to double...
    raw_msg.throttle =
      static_cast<decltype(raw_msg.throttle)>(m_throttle_table.lookup(directional_accel));
  } else {
    raw_msg.brake =
      static_cast<decltype(raw_msg.brake)>(m_brake_table.lookup(directional_accel));
  }
  raw_msg.front_steer =
    static_cast<decltype(raw_msg.front_steer)>(m_steer_table.lookup(msg.front_wheel_angle_rad));
  raw_msg.rear_steer = 0;

  return send_control_command(raw_msg);
}

////////////////////////////////////////////////////////////////////////////////
bool8_t LgsvlInterface::send_control_command(const autoware_auto_msgs::msg::RawControlCommand & msg)
{
  // Front steer semantically is z up, ccw positive, but LGSVL thinks its the opposite
  auto msg_corrected = msg;
  msg_corrected.front_steer = -msg.front_steer;
  m_cmd_pub->publish(msg_corrected);
  return true;
}

////////////////////////////////////////////////////////////////////////////////
void LgsvlInterface::on_odometry(const nav_msgs::msg::Odometry & msg)
{
  if (!m_odom_set) {
    m_odom_zero.x = msg.pose.pose.position.x;
    m_odom_zero.y = msg.pose.pose.position.y;
    m_odom_zero.z = msg.pose.pose.position.z;
    m_odom_set = true;
  }
  decltype(msg.pose.pose.orientation) q{};
  {
    // Convert from LHS system to RHS system: Y forward, Z up
    tf2::Quaternion q_rhs{
      -msg.pose.pose.orientation.z,
      msg.pose.pose.orientation.x,
      -msg.pose.pose.orientation.y,
      msg.pose.pose.orientation.w};
    // rotate +90 degrees around +z axis to get X forward
    tf2::Quaternion q90{};
    q90.setRPY(0.0, 0.0, 90.0 * (M_PI / 180.0));
    const auto q_x_forward = q90 * q_rhs;
    q.x = q_x_forward.getX();
    q.y = q_x_forward.getY();
    q.z = q_x_forward.getZ();
    q.w = q_x_forward.getW();
  }
  const auto px = msg.pose.pose.position.x - m_odom_zero.x;
  const auto py = msg.pose.pose.position.y - m_odom_zero.y;
  const auto pz = msg.pose.pose.position.z - m_odom_zero.z;
  {
    autoware_auto_msgs::msg::VehicleKinematicState vse{};
    vse.header = msg.header;
    vse.state.x = static_cast<decltype(vse.state.x)>(px);
    vse.state.y = static_cast<decltype(vse.state.y)>(py);
    {
      const auto inv_mag = 1.0 / std::sqrt((q.z * q.z) + (q.w * q.w));
      vse.state.heading.real = static_cast<decltype(vse.state.heading.real)>(q.w * inv_mag);
      vse.state.heading.imag = static_cast<decltype(vse.state.heading.imag)>(q.z * inv_mag);
      // LGSVL is y-up and left handed
    }

    // Get values from vehicle odometry
    vse.state.longitudinal_velocity_mps = get_odometry().velocity_mps;
    vse.state.front_wheel_angle_rad = get_odometry().front_wheel_angle_rad;
    vse.state.rear_wheel_angle_rad = get_odometry().rear_wheel_angle_rad;

    vse.state.lateral_velocity_mps =
      static_cast<decltype(vse.state.lateral_velocity_mps)>(msg.twist.twist.linear.y);
    // TODO(jitrc): populate with correct value when acceleration is available from simulator
    vse.state.acceleration_mps2 = 0.0F;
    vse.state.heading_rate_rps =
      static_cast<decltype(vse.state.heading_rate_rps)>(msg.twist.twist.angular.z);

    m_kinematic_state_pub->publish(vse);
  }

  if (m_pose_pub) {
    geometry_msgs::msg::PoseWithCovarianceStamped pose{};
    pose.header = msg.header;
    pose.pose.pose.position = msg.pose.pose.position;
    pose.pose.pose.orientation = q;

    constexpr auto EPS = std::numeric_limits<float32_t>::epsilon();
    if (std::fabs(msg.pose.covariance[COV_X]) > EPS ||
      std::fabs(msg.pose.covariance[COV_Y]) > EPS ||
      std::fabs(msg.pose.covariance[COV_Z]) > EPS ||
      std::fabs(msg.pose.covariance[COV_RX]) > EPS ||
      std::fabs(msg.pose.covariance[COV_RY]) > EPS ||
      std::fabs(msg.pose.covariance[COV_RZ]) > EPS)
    {
      pose.pose.covariance = {
        COV_X_VAR, 0.0, 0.0, 0.0, 0.0, 0.0,
        0.0, COV_Y_VAR, 0.0, 0.0, 0.0, 0.0,
        0.0, 0.0, COV_Z_VAR, 0.0, 0.0, 0.0,
        0.0, 0.0, 0.0, COV_RX_VAR, 0.0, 0.0,
        0.0, 0.0, 0.0, 0.0, COV_RY_VAR, 0.0,
        0.0, 0.0, 0.0, 0.0, 0.0, COV_RX_VAR};
    } else {
      pose.pose.covariance = msg.pose.covariance;
    }

    m_pose_pub->publish(pose);
  }

  if (m_tf_pub) {
    geometry_msgs::msg::TransformStamped tf{};
    tf.header = msg.header;
    tf.child_frame_id = msg.child_frame_id;
    tf.transform.translation.x = px;
    tf.transform.translation.y = py;
    tf.transform.translation.z = pz;
    tf.transform.rotation = q;

    tf2_msgs::msg::TFMessage tf_msg{};
    tf_msg.transforms.emplace_back(std::move(tf));
    m_tf_pub->publish(tf_msg);
  }
}

////////////////////////////////////////////////////////////////////////////////
void LgsvlInterface::on_state_report(const autoware_auto_msgs::msg::VehicleStateReport & msg)
{
  auto corrected_report = msg;

  // in autoware_auto_msgs::msg::VehicleStateCommand 1 is drive, 2 is reverse, https://gitlab.com/autowarefoundation/autoware.auto/AutowareAuto/-/blob/9744f6dc/src/messages/autoware_auto_msgs/msg/VehicleStateCommand.msg#L32
  // in lgsvl 0 is drive and 1 is reverse https://github.com/lgsvl/simulator/blob/cb937deb8e633573f6c0cc76c9f451398b8b9eff/Assets/Scripts/Sensors/VehicleStateSensor.cs#L70


  // Find autoware gear via inverse mapping
  const auto value_same = [&msg](const auto & kv) -> bool {  // also do some capture
      return msg.gear == kv.second;
    };
  const auto it = std::find_if(autoware_to_lgsvl_gear.begin(),
      autoware_to_lgsvl_gear.end(), value_same);

  if (it != autoware_to_lgsvl_gear.end()) {
    corrected_report.gear = it->first;
  } else {
    corrected_report.gear = msg.GEAR_NEUTRAL;
    RCLCPP_WARN(m_logger, "Invalid gear value in state report from LGSVL simulator");
  }

  // Correcting blinker value, they are shifted up by one,
  // as the first value BLINKER_NO_COMMAND does not exisit in LGSVL
  // not setting  VSC::BLINKER_NO_COMMAND, when get.state_report.blinker == msg.blinker
  // instead reporting true blinker status
  corrected_report.blinker++;

  state_report() = corrected_report;
}

}  // namespace lgsvl_interface
