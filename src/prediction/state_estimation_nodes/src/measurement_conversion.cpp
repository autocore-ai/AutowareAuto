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

#include <state_estimation_nodes/measurement_conversion.hpp>

namespace
{
constexpr auto kCovarianceMatrixRows = 6U;
constexpr auto kIndexX = 0U;
constexpr auto kIndexY = kCovarianceMatrixRows + 1U;
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
namespace prediction
{

template<>
MeasurementPoseAndSpeed message_to_measurement(
  const nav_msgs::msg::Odometry & msg,
  const Eigen::Isometry3f & tf__world__frame_id,
  const Eigen::Isometry3f & tf__world__child_frame_id)
{
  using FloatT = common::types::float32_t;
  const auto converted_tf__world__frame_id = downscale_isometry<2>(tf__world__frame_id);
  const auto converted_tf__world__child_frame_id = downscale_isometry<2>(tf__world__child_frame_id);
  const Eigen::Vector2f pos_state = converted_tf__world__frame_id * Eigen::Vector2f{
    static_cast<FloatT>(msg.pose.pose.position.x),
    static_cast<FloatT>(msg.pose.pose.position.y),
  };
  const Eigen::Vector2f speed_state =
    converted_tf__world__child_frame_id.rotation() * Eigen::Vector2f{
    static_cast<FloatT>(msg.twist.twist.linear.x),
    static_cast<FloatT>(msg.twist.twist.linear.y),
  };
  const auto x_variance{static_cast<FloatT>(msg.pose.covariance[kIndexX])};
  const auto y_variance{static_cast<FloatT>(msg.pose.covariance[kIndexY])};
  const auto x_speed_variance{static_cast<FloatT>(msg.twist.covariance[kIndexX])};
  const auto y_speed_variance{static_cast<FloatT>(msg.twist.covariance[kIndexY])};

  return MeasurementPoseAndSpeed{
    to_time_point(msg.header.stamp),
    (Eigen::Vector4f{} << pos_state, speed_state).finished(),
    {x_variance, y_variance, x_speed_variance, y_speed_variance}};
}

template<>
MeasurementSpeed message_to_measurement(
  const geometry_msgs::msg::TwistWithCovarianceStamped & msg,
  const Eigen::Isometry3f & tf__world__frame_id)
{
  using FloatT = common::types::float32_t;
  const auto converted_tf__world__frame_id = downscale_isometry<2>(tf__world__frame_id);
  return MeasurementSpeed{
    to_time_point(msg.header.stamp),
    converted_tf__world__frame_id.rotation() * Eigen::Vector2f{
      msg.twist.twist.linear.x, msg.twist.twist.linear.y},
    {static_cast<FloatT>(msg.twist.covariance[kIndexX]),
      static_cast<FloatT>(msg.twist.covariance[kIndexY])}
  };
}

template<>
MeasurementPose message_to_measurement(
  const geometry_msgs::msg::PoseWithCovarianceStamped & msg,
  const Eigen::Isometry3f & tf__world__frame_id)
{
  const auto converted_tf__world__frame_id = downscale_isometry<2>(tf__world__frame_id);
  return MeasurementPose{
    to_time_point(msg.header.stamp),
    converted_tf__world__frame_id * Eigen::Vector2f{
      msg.pose.pose.position.x, msg.pose.pose.position.y},
    {static_cast<common::types::float32_t>(msg.pose.covariance[kIndexX]),
      static_cast<common::types::float32_t>(msg.pose.covariance[kIndexY])}
  };
}


}  // namespace prediction
}  // namespace autoware
