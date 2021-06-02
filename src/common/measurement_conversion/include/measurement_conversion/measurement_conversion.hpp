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

#ifndef MEASUREMENT_CONVERSION__MEASUREMENT_CONVERSION_HPP_
#define MEASUREMENT_CONVERSION__MEASUREMENT_CONVERSION_HPP_

#include <autoware_auto_msgs/msg/relative_position_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp>
#include <measurement_conversion/measurement_typedefs.hpp>
#include <measurement_conversion/visibility_control.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/time.hpp>

#include <Eigen/Geometry>

namespace autoware
{
namespace common
{
namespace state_estimation
{

///
/// @brief      Interface for converting a message into a measurement.
///
/// @tparam     MeasurementT  Type of measurement.
/// @tparam     MessageT      Type of ROS 2 message.
///
/// @return     The measurement created from a message.
///
template<typename MeasurementT, typename MessageT>
MEASUREMENT_CONVERSION_PUBLIC MeasurementT message_to_measurement(const MessageT &)
{
  static_assert(
    sizeof(MessageT) == 0,
    "Only specializations for message_to_measurement() function are allowed!");
}

///
/// @brief      Specialization of message_to_measurement for twist message.
///
/// @param[in]  msg                  The twist message.
///
/// @return     The measurement containing speed.
///
template<>
MEASUREMENT_CONVERSION_PUBLIC Measurement2dSpeed64 message_to_measurement(
  const geometry_msgs::msg::TwistWithCovariance & msg);

///
/// @brief      Specialization of message_to_measurement for pose message.
///
/// @param[in]  msg                  The pose message.
///
/// @return     The measurement containing pose.
///
template<>
MEASUREMENT_CONVERSION_PUBLIC Measurement2dPose64 message_to_measurement(
  const geometry_msgs::msg::PoseWithCovariance & msg);

///
/// @brief      Specialization of message_to_measurement for stamped twist message.
///
/// @param[in]  msg                  The stamped twist message.
///
/// @return     The measurement containing speed.
///
template<>
MEASUREMENT_CONVERSION_PUBLIC StampedMeasurement2dSpeed64 message_to_measurement(
  const geometry_msgs::msg::TwistWithCovarianceStamped & msg);

///
/// @brief      Specialization of message_to_measurement for stamped pose message.
///
/// @param[in]  msg                  The stamped pose message.
///
/// @return     The measurement containing pose.
///
template<>
MEASUREMENT_CONVERSION_PUBLIC StampedMeasurement2dPose64 message_to_measurement(
  const geometry_msgs::msg::PoseWithCovarianceStamped & msg);

///
/// @brief      Specialization of message_to_measurement for stamped relative position message.
///
/// @param[in]  msg                  The stamped relative position message.
///
/// @return     The measurement containing the relative position.
///
template<>
MEASUREMENT_CONVERSION_PUBLIC StampedMeasurement2dPose64 message_to_measurement(
  const autoware_auto_msgs::msg::RelativePositionWithCovarianceStamped & msg);

///
/// @brief      Specialization of message_to_measurement for odometry message.
///
/// @param[in]  msg                  The odometry message.
///
/// @return     The measurement containing pose and speed.
///
template<>
MEASUREMENT_CONVERSION_PUBLIC StampedMeasurement2dPoseAndSpeed64 message_to_measurement(
  const nav_msgs::msg::Odometry & msg);

}  // namespace state_estimation
}  // namespace common
}  // namespace autoware


#endif  // MEASUREMENT_CONVERSION__MEASUREMENT_CONVERSION_HPP_
