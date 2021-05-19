// Copyright 2021 Apex.AI, Inc.
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

#include <gnss_conversion_nodes/gnss_conversion_node.hpp>

#include <common/types.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>

#include <tf2/buffer_core.h>
#include <tf2_eigen/tf2_eigen.h>

#include <Eigen/Core>

#include <limits>
#include <memory>
#include <string>
#include <vector>

namespace
{
static constexpr auto kDefaultHistorySize = 10;
static constexpr auto kNumOfCovarianceEntries = 9UL;
static constexpr auto kHistorySizeTag = "history_size";
static constexpr auto kOutputTopic = "gnss_position";
static constexpr auto kGnssFixInputTopic = "wgs84_position";
static constexpr auto kFrameIdTag = "output_frame_id";
static constexpr auto kChildFrameIdTag = "child_frame_id";
static constexpr auto kDefaultChildFrameIdTag = "base_link";
static constexpr auto kOverrideCovarianceTag = "override_variances";
static constexpr auto kDefaultFrameId = "earth";
static constexpr auto kDefaultLoggingInterval = 1000;  // Milliseconds.

using autoware_auto_msgs::msg::RelativePositionWithCovarianceStamped;

template<typename ArrayT>
Eigen::Matrix3d to_eigen(const ArrayT & covariance)
{
  if (covariance.size() < 9UL) {
    throw std::domain_error(
            "The array representing the covariance matrix does not have enough elements. "
            "Expected " + std::to_string(9UL) + " but has " + std::to_string(covariance.size()));
  }
  return Eigen::Matrix3d{Eigen::Map<const Eigen::Matrix3d>{&covariance[0]}};
}

template<typename ArrayT>
void fill_covariance_from_eigen(ArrayT & covariance, const Eigen::Matrix3d & matrix)
{
  if (covariance.size() < 9UL) {
    throw std::domain_error(
            "The array representing the covariance matrix does not have enough elements. "
            "Expected " + std::to_string(9UL) + " but has " + std::to_string(covariance.size()));
  }
  Eigen::Map<Eigen::Matrix3d>{&covariance[0]} = matrix;
}

Eigen::Matrix3d rotate_covariance(
  const Eigen::Isometry3d & tf__new__old,
  const Eigen::Matrix3d & covariance)
{
  return tf__new__old.rotation() * covariance * tf__new__old.rotation().transpose();
}

void switch_frames_if_needed(
  autoware_auto_msgs::msg::RelativePositionWithCovarianceStamped & msg,
  const std_msgs::msg::Header::_frame_id_type & new_frame_id,
  const tf2::BufferCore & buffer)
{
  if (new_frame_id == msg.header.frame_id) {return;}
  const Eigen::Isometry3d tf__new_frame_id__msg_frame_id = tf2::transformToEigen(
    buffer.lookupTransform(
      new_frame_id, msg.header.frame_id, tf2_ros::fromMsg(msg.header.stamp)));
  Eigen::Vector3d position;
  tf2::fromMsg(msg.position, position);
  msg.position = tf2::toMsg2(tf__new_frame_id__msg_frame_id * position);

  // Update position covariance
  fill_covariance_from_eigen(
    msg.covariance, rotate_covariance(tf__new_frame_id__msg_frame_id, to_eigen(msg.covariance)));
  msg.header.frame_id = new_frame_id;
}

}  // namespace


namespace autoware
{
namespace gnss_conversion_nodes
{

GnssConversionNode::GnssConversionNode(const rclcpp::NodeOptions & options)
:  Node("gnss_conversion_nodes", options),
  m_wgs84_to_ecef_convertor{
    GeographicLib::Constants::WGS84_a(),
    GeographicLib::Constants::WGS84_f()},
  m_tf_listener(m_tf_buffer, std::shared_ptr<rclcpp::Node>(this, [](auto) {}), false)
{
  const auto history_size{declare_parameter(kHistorySizeTag, kDefaultHistorySize)};
  if (history_size <= 0) {throw std::domain_error("History size must be positive.");}
  m_frame_id = declare_parameter(kFrameIdTag, kDefaultFrameId);
  m_child_frame_id = declare_parameter(kChildFrameIdTag, kDefaultChildFrameIdTag);
  const auto qos = rclcpp::QoS{static_cast<std::size_t>(history_size)};

  m_publisher = create_publisher<RelativePositionWithCovarianceStamped>(kOutputTopic, qos);
  m_gnss_nav_fix_subscription = create_subscription<sensor_msgs::msg::NavSatFix>(
    kGnssFixInputTopic, qos,
    std::bind(&GnssConversionNode::nav_sat_fix_callback, this, std::placeholders::_1));
  m_override_variances_diagonal =
    declare_parameter(kOverrideCovarianceTag, std::vector<common::types::float64_t>{});
  if (m_override_variances_diagonal.size() != 3UL) {
    throw std::runtime_error("Override covariance must have exactly 3 entries.");
  }
}

void GnssConversionNode::nav_sat_fix_callback(
  const sensor_msgs::msg::NavSatFix::ConstSharedPtr msg) const
{
  if (msg->status.status == msg->status.STATUS_NO_FIX) {
    RCLCPP_WARN_THROTTLE(
      get_logger(),
      m_steady_clock,
      kDefaultLoggingInterval,
      "GNSS has no fix, so nothing is published.");
    return;
  }
  RelativePositionWithCovarianceStamped out_msg{};
  out_msg.header.stamp = msg->header.stamp;
  out_msg.header.frame_id = kDefaultFrameId;
  out_msg.child_frame_id = m_child_frame_id;
  out_msg.covariance.resize(kNumOfCovarianceEntries);
  m_wgs84_to_ecef_convertor.Forward(
    msg->latitude, msg->longitude, msg->altitude,
    out_msg.position.x, out_msg.position.y, out_msg.position.z);
  try {
    switch_frames_if_needed(out_msg, m_frame_id, m_tf_buffer);
  } catch (const tf2::LookupException & exception) {
    RCLCPP_WARN_THROTTLE(
      get_logger(),
      m_steady_clock,
      kDefaultLoggingInterval,
      "Skipping publishing of a GNSS pose message.\n"
      "Could not look up transformation between " +
      out_msg.header.frame_id + " and " +
      m_frame_id + " with the exception: " + exception.what());
    return;
  }
  if (!m_override_variances_diagonal.empty()) {
    const Eigen::Vector3d variances_diagonal{
      Eigen::Map<const Eigen::Vector3d>{m_override_variances_diagonal.data()}};
    const auto override_covariance = variances_diagonal.array().square().matrix().asDiagonal();
    auto pose_covariance = Eigen::Map<Eigen::Matrix3d>{&out_msg.covariance.front()};
    pose_covariance.topLeftCorner(
      override_covariance.rows(), override_covariance.cols()) = override_covariance;
  }
  m_publisher->publish(out_msg);
}

}  // namespace gnss_conversion_nodes
}  // namespace autoware

RCLCPP_COMPONENTS_REGISTER_NODE(autoware::gnss_conversion_nodes::GnssConversionNode)
