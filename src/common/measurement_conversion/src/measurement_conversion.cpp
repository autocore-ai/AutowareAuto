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

/// \copyright Copyright 2021 Apex.AI, Inc.
/// All rights reserved.

#include <measurement_conversion/measurement_conversion.hpp>

#include <common/types.hpp>

namespace
{
constexpr auto kCovarianceMatrixRows = 6U;
constexpr auto kIndexX = 0U;
constexpr auto kIndexXY = 1U;
constexpr auto kIndexY = kCovarianceMatrixRows + 1U;
constexpr auto kIndexYX = kCovarianceMatrixRows;
constexpr auto kCovarianceMatrixRowsSquared = kCovarianceMatrixRows * kCovarianceMatrixRows;
static_assert(
  std::tuple_size<
    geometry_msgs::msg::PoseWithCovariance::_covariance_type>::value ==
  kCovarianceMatrixRowsSquared, "We expect the covariance matrix to have 36 entries.");

/// Convert the ROS timestamp to chrono time point.
std::chrono::system_clock::time_point to_time_point(const rclcpp::Time & time)
{
  return std::chrono::system_clock::time_point{std::chrono::nanoseconds{time.nanoseconds()}};
}
}  // namespace

namespace autoware
{
namespace common
{
namespace state_estimation
{

template<>
Measurement2dSpeed message_to_measurement(
  const geometry_msgs::msg::TwistWithCovariance & msg)
{
  using FloatT = common::types::float32_t;
  Eigen::Matrix2d covariance;
  covariance <<
    msg.covariance[kIndexX], msg.covariance[kIndexXY],
    msg.covariance[kIndexYX], msg.covariance[kIndexY];
  return Measurement2dSpeed{
    Eigen::Vector2f{msg.twist.linear.x, msg.twist.linear.y},
    covariance.cast<FloatT>()};
}

template<>
Measurement2dPose message_to_measurement(
  const geometry_msgs::msg::PoseWithCovariance & msg)
{
  using FloatT = common::types::float32_t;
  Eigen::Matrix2d covariance;
  covariance <<
    msg.covariance[kIndexX], msg.covariance[kIndexXY],
    msg.covariance[kIndexYX], msg.covariance[kIndexY];
  return Measurement2dPose{Eigen::Vector2f{
      msg.pose.position.x, msg.pose.position.y},
    covariance.cast<FloatT>()};
}


template<>
StampedMeasurement2dSpeed message_to_measurement(
  const geometry_msgs::msg::TwistWithCovarianceStamped & msg)
{
  return StampedMeasurement2dSpeed{
    to_time_point(msg.header.stamp),
    message_to_measurement<Measurement2dSpeed>(msg.twist)
  };
}

template<>
StampedMeasurement2dPose message_to_measurement(
  const geometry_msgs::msg::PoseWithCovarianceStamped & msg)
{
  return StampedMeasurement2dPose{
    to_time_point(msg.header.stamp),
    message_to_measurement<Measurement2dPose>(msg.pose)
  };
}

template<>
StampedMeasurement2dPoseAndSpeed message_to_measurement(
  const nav_msgs::msg::Odometry & msg)
{
  using FloatT = common::types::float32_t;
  const Eigen::Vector2f pos_state {
    static_cast<FloatT>(msg.pose.pose.position.x),
    static_cast<FloatT>(msg.pose.pose.position.y),
  };
  const Eigen::Vector2f speed_state{
    static_cast<FloatT>(msg.twist.twist.linear.x),
    static_cast<FloatT>(msg.twist.twist.linear.y),
  };
  Eigen::Matrix4d covariance;
  covariance.topLeftCorner(2, 2) <<
    msg.pose.covariance[kIndexX], msg.pose.covariance[kIndexXY],
    msg.pose.covariance[kIndexYX], msg.pose.covariance[kIndexY];
  covariance.bottomRightCorner(2, 2) <<
    msg.twist.covariance[kIndexX], msg.twist.covariance[kIndexXY],
    msg.twist.covariance[kIndexYX], msg.twist.covariance[kIndexY];

  return StampedMeasurement2dPoseAndSpeed{
    to_time_point(msg.header.stamp),
    Measurement2dPoseAndSpeed{
      (Eigen::Vector4f{} << pos_state, speed_state).finished(),
      covariance.cast<FloatT>()}};
}

}  // namespace state_estimation
}  // namespace common
}  // namespace autoware
