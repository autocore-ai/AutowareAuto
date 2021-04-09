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

#ifndef STATE_ESTIMATION_NODES__MEASUREMENT_CONVERSION_HPP_
#define STATE_ESTIMATION_NODES__MEASUREMENT_CONVERSION_HPP_

#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp>
#include <motion_model/constant_acceleration.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/time.hpp>
#include <state_estimation_nodes/measurement.hpp>

#include <Eigen/Geometry>

namespace autoware
{
namespace prediction
{

using MeasurementPose = Measurement<common::types::float32_t,
    motion::motion_model::ConstantAcceleration::States::POSE_X,
    motion::motion_model::ConstantAcceleration::States::POSE_Y>;

using MeasurementPoseAndSpeed = Measurement<common::types::float32_t,
    motion::motion_model::ConstantAcceleration::States::POSE_X,
    motion::motion_model::ConstantAcceleration::States::POSE_Y,
    motion::motion_model::ConstantAcceleration::States::VELOCITY_X,
    motion::motion_model::ConstantAcceleration::States::VELOCITY_Y>;

using MeasurementSpeed = Measurement<common::types::float32_t,
    motion::motion_model::ConstantAcceleration::States::VELOCITY_X,
    motion::motion_model::ConstantAcceleration::States::VELOCITY_Y>;

///
/// @brief      Interface for converting a message into a measurement.
///
/// @tparam     MeasurementT  Type of measurement.
/// @tparam     MessageT      Type of ROS 2 message.
///
/// @return     The measurement created from a message.
///
template<typename MeasurementT, typename MessageT>
MeasurementT message_to_measurement(const MessageT &, const Eigen::Isometry3f &)
{
  static_assert(
    sizeof(MessageT) == 0,
    "Only specializations for message_to_measurement() function are allowed!");
}

///
/// @brief      Interface for converting a message into a measurement that accepts an additional
///             transformation.
///
/// @details    This function is needed to transform messages like Odometry, where different parts
///             of the measurement must be transformed with different transformation matrices.
///
/// @tparam     MeasurementT  Type of measurement.
/// @tparam     MessageT      Type of ROS 2 message.
///
/// @return     The measurement created from a message.
///
template<typename MeasurementT, typename MessageT>
MeasurementT message_to_measurement(
  const MessageT &, const Eigen::Isometry3f &, const Eigen::Isometry3f &)
{
  static_assert(
    sizeof(MessageT) == 0,
    "Only specializations for message_to_measurement() function are allowed!");
}

///
/// @brief      Downscale the isometry to a lower dimension if needed.
///
/// @param[in]  isometry              The isometry transform
///
/// @tparam     kStateDimentionality  Dimensionality of the space.
/// @tparam     FloatT                Type of scalar.
///
/// @return     Downscaled isometry.
///
template<std::int32_t kStateDimentionality, typename FloatT>
static constexpr Eigen::Transform<
  FloatT, kStateDimentionality, Eigen::TransformTraits::Isometry> downscale_isometry(
  const Eigen::Transform<FloatT, 3, Eigen::TransformTraits::Isometry> & isometry)
{
  static_assert(kStateDimentionality <= 3, "We only handle scaling the isometry down.");
  using Isometry = Eigen::Transform<
    FloatT, kStateDimentionality, Eigen::TransformTraits::Isometry>;
  Isometry result{Isometry::Identity()};
  result.linear() = isometry.rotation()
    .template block<kStateDimentionality, kStateDimentionality>(0, 0);
  result.translation() = isometry.translation().topRows(kStateDimentionality);
  return result;
}

///
/// @brief      Specialization of message_to_measurement for odometry message.
///
/// @param[in]  msg                        The odometry message.
/// @param[in]  tf__world__frame_id        A transform from message frame_id to world frame.
/// @param[in]  tf__world__child_frame_id  A transform from message frame_id to child_frame_id.
///
/// @return     The measurement containing pose and speed.
///
template<>
STATE_ESTIMATION_NODES_PUBLIC MeasurementPoseAndSpeed message_to_measurement(
  const nav_msgs::msg::Odometry & msg,
  const Eigen::Isometry3f & tf__world__frame_id,
  const Eigen::Isometry3f & tf__world__child_frame_id);

///
/// @brief      Specialization of message_to_measurement for twist message.
///
/// @param[in]  msg                  The twist message.
/// @param[in]  tf__world__frame_id  A transform from message frame_id to world frame.
///
/// @return     The measurement containing speed.
///
template<>
STATE_ESTIMATION_NODES_PUBLIC MeasurementSpeed message_to_measurement(
  const geometry_msgs::msg::TwistWithCovarianceStamped & msg,
  const Eigen::Isometry3f & tf__world__frame_id);

///
/// @brief      Specialization of message_to_measurement for pose message.
///
/// @param[in]  msg                  The pose message.
/// @param[in]  tf__world__frame_id  A transform from message frame_id to world frame.
///
/// @return     The measurement containing pose.
///
template<>
STATE_ESTIMATION_NODES_PUBLIC MeasurementPose message_to_measurement(
  const geometry_msgs::msg::PoseWithCovarianceStamped & msg,
  const Eigen::Isometry3f & tf__world__frame_id);

}  // namespace prediction
}  // namespace autoware


#endif  // STATE_ESTIMATION_NODES__MEASUREMENT_CONVERSION_HPP_
