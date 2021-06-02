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
#include <measurement_conversion/eigen_utils.hpp>
#include <tf2_eigen/tf2_eigen.h>

namespace
{
using autoware::common::types::float32_t;
using autoware::common::types::float64_t;

constexpr auto kCovarianceMatrixRows = 6U;
constexpr auto kCovarianceMatrixRowsRelativePos = 3U;
constexpr auto kIndexX = 0U;
constexpr auto kIndexXY = 1U;
constexpr auto kIndexY = kCovarianceMatrixRows + 1U;
constexpr auto kIndexYX = kCovarianceMatrixRows;
constexpr auto kIndexXRelativePos = 0U;
constexpr auto kIndexXYRelativePos = 1U;
constexpr auto kIndexYRelativePos = kCovarianceMatrixRowsRelativePos + 1U;
constexpr auto kIndexYXRelativePos = kCovarianceMatrixRowsRelativePos;
constexpr auto kCovarianceMatrixRowsSquared = kCovarianceMatrixRows * kCovarianceMatrixRows;
static_assert(
  std::tuple_size<
    geometry_msgs::msg::PoseWithCovariance::_covariance_type>::value ==
  kCovarianceMatrixRowsSquared, "We expect the covariance matrix to have 36 entries.");
// TODO(#789 autoware_auto_msgs) add a static assert once the RelativePosition message covariance is
// represented by an std::array.

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
Measurement2dSpeed64 message_to_measurement(
  const geometry_msgs::msg::TwistWithCovariance & msg)
{
  Eigen::Vector2d mean{msg.twist.linear.x, msg.twist.linear.y};
  Eigen::Matrix2d covariance;
  covariance <<
    msg.covariance[kIndexX], msg.covariance[kIndexXY],
    msg.covariance[kIndexYX], msg.covariance[kIndexY];
  return Measurement2dSpeed64{
    mean,
    covariance};
}

template<>
Measurement2dPose64 message_to_measurement(
  const geometry_msgs::msg::PoseWithCovariance & msg)
{
  Eigen::Vector2d mean{msg.pose.position.x, msg.pose.position.y};
  Eigen::Matrix2d covariance;
  covariance <<
    msg.covariance[kIndexX], msg.covariance[kIndexXY],
    msg.covariance[kIndexYX], msg.covariance[kIndexY];
  return Measurement2dPose64{
    mean,
    covariance};
}

template<>
StampedMeasurement2dSpeed64 message_to_measurement(
  const geometry_msgs::msg::TwistWithCovarianceStamped & msg)
{
  return StampedMeasurement2dSpeed64{
    to_time_point(msg.header.stamp),
    message_to_measurement<Measurement2dSpeed64>(msg.twist)
  };
}

template<>
StampedMeasurement2dPose64 message_to_measurement(
  const geometry_msgs::msg::PoseWithCovarianceStamped & msg)
{
  return StampedMeasurement2dPose64{
    to_time_point(msg.header.stamp),
    message_to_measurement<Measurement2dPose64>(msg.pose)
  };
}

template<>
StampedMeasurement2dPose64 message_to_measurement(
  const autoware_auto_msgs::msg::RelativePositionWithCovarianceStamped & msg)
{
  Eigen::Vector2d mean{msg.position.x, msg.position.y};
  Eigen::Matrix2d covariance;
  covariance <<
    msg.covariance[kIndexXRelativePos], msg.covariance[kIndexXYRelativePos],
    msg.covariance[kIndexYXRelativePos], msg.covariance[kIndexYRelativePos];
  return StampedMeasurement2dPose64{
    to_time_point(msg.header.stamp),
    Measurement2dPose64{mean, covariance}};
}

template<>
StampedMeasurement2dPoseAndSpeed64 message_to_measurement(
  const nav_msgs::msg::Odometry & msg)
{
  Eigen::Isometry3d tf__msg_frame_id__msg_child_frame_id;
  tf2::fromMsg(msg.pose.pose, tf__msg_frame_id__msg_child_frame_id);
  const Eigen::Matrix2d rx__msg_frame_id__msg_child_frame_id = downscale_isometry<2>(
    tf__msg_frame_id__msg_child_frame_id).rotation();

  const Eigen::Vector2d pos_state {
    msg.pose.pose.position.x,
    msg.pose.pose.position.y,
  };
  const Eigen::Vector2d speed_in_child_frame{
    msg.twist.twist.linear.x,
    msg.twist.twist.linear.y,
  };
  const Eigen::Vector2d speed{rx__msg_frame_id__msg_child_frame_id * speed_in_child_frame};
  Eigen::Matrix4d covariance{Eigen::Matrix4d::Zero()};
  covariance.topLeftCorner(2, 2) <<
    msg.pose.covariance[kIndexX], msg.pose.covariance[kIndexXY],
    msg.pose.covariance[kIndexYX], msg.pose.covariance[kIndexY];
  covariance.bottomRightCorner(2, 2) <<
    msg.twist.covariance[kIndexX], msg.twist.covariance[kIndexXY],
    msg.twist.covariance[kIndexYX], msg.twist.covariance[kIndexY];
  // Rotate the speed covariance as the speed is now in frame_id frame and not in child_frame_id.
  covariance.bottomRightCorner(2, 2) =
    rx__msg_frame_id__msg_child_frame_id *
    covariance.bottomRightCorner(2, 2) *
    rx__msg_frame_id__msg_child_frame_id.transpose();

  const Eigen::Vector4d mean = (Eigen::Vector4d{} << pos_state, speed).finished();
  return StampedMeasurement2dPoseAndSpeed64{
    to_time_point(msg.header.stamp),
    Measurement2dPoseAndSpeed64{mean, covariance}};
}


}  // namespace state_estimation
}  // namespace common
}  // namespace autoware
