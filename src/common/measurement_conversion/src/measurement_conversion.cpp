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
StampedMeasurement2dPose message_to_measurement(
  const autoware_auto_msgs::msg::RelativePositionWithCovarianceStamped & msg)
{
  using common::types::float32_t;
  Eigen::Matrix2d covariance;
  covariance <<
    msg.covariance[kIndexXRelativePos], msg.covariance[kIndexXYRelativePos],
    msg.covariance[kIndexYXRelativePos], msg.covariance[kIndexYRelativePos];
  return StampedMeasurement2dPose{
    to_time_point(msg.header.stamp),
    Measurement2dPose{
      Eigen::Vector2f{msg.position.x, msg.position.y},
      covariance.cast<float32_t>()}};
}

template<>
StampedMeasurement2dPoseAndSpeed message_to_measurement(const nav_msgs::msg::Odometry & msg)
{
  using common::types::float32_t;
  Eigen::Isometry3d tf__msg_frame_id__msg_child_frame_id;
  tf2::fromMsg(msg.pose.pose, tf__msg_frame_id__msg_child_frame_id);
  const Eigen::Matrix2f rx__msg_frame_id__msg_child_frame_id = downscale_isometry<2>(
    tf__msg_frame_id__msg_child_frame_id).cast<float32_t>().rotation();

  const Eigen::Vector2f pos_state {
    static_cast<float32_t>(msg.pose.pose.position.x),
    static_cast<float32_t>(msg.pose.pose.position.y),
  };
  const Eigen::Vector2f speed_in_child_frame{
    static_cast<float32_t>(msg.twist.twist.linear.x),
    static_cast<float32_t>(msg.twist.twist.linear.y),
  };
  const Eigen::Vector2f speed{rx__msg_frame_id__msg_child_frame_id * speed_in_child_frame};
  Eigen::Matrix4d covariance_double{Eigen::Matrix4d::Zero()};
  covariance_double.topLeftCorner(2, 2) <<
    msg.pose.covariance[kIndexX], msg.pose.covariance[kIndexXY],
    msg.pose.covariance[kIndexYX], msg.pose.covariance[kIndexY];
  covariance_double.bottomRightCorner(2, 2) <<
    msg.twist.covariance[kIndexX], msg.twist.covariance[kIndexXY],
    msg.twist.covariance[kIndexYX], msg.twist.covariance[kIndexY];
  Eigen::Matrix4f covariance{covariance_double.cast<float32_t>()};
  // Rotate the speed covariance as the speed is now in frame_id frame and not in child_frame_id.
  covariance.bottomRightCorner(2, 2) =
    rx__msg_frame_id__msg_child_frame_id *
    covariance.bottomRightCorner(2, 2) *
    rx__msg_frame_id__msg_child_frame_id.transpose();

  return StampedMeasurement2dPoseAndSpeed{
    to_time_point(msg.header.stamp),
    Measurement2dPoseAndSpeed{(Eigen::Vector4f{} << pos_state, speed).finished(), covariance}};
}

}  // namespace state_estimation
}  // namespace common
}  // namespace autoware
