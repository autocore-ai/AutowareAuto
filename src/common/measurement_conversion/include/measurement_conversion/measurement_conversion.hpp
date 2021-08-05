// Copyright 2021 Apex.AI, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//    http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//
// Co-developed by Tier IV, Inc. and Apex.AI, Inc.

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
#include <time_utils/time_utils.hpp>

#include <Eigen/Geometry>

namespace autoware
{
namespace common
{
namespace state_estimation
{

namespace detail
{
inline const geometry_msgs::msg::PoseWithCovariance & unstamp(
  const geometry_msgs::msg::PoseWithCovarianceStamped & msg)
{
  return msg.pose;
}
inline const autoware_auto_msgs::msg::RelativePositionWithCovarianceStamped & unstamp(
  const autoware_auto_msgs::msg::RelativePositionWithCovarianceStamped & msg) {return msg;}

}  // namespace detail

///
/// @brief      A default template structure used for message conversion. Only its specializations
///             are to be used.
///
/// @tparam     MeasurementT  A measurement into which the conversion takes place.
///
template<typename MeasurementT>
struct MEASUREMENT_CONVERSION_PUBLIC convert_to
{
  static_assert(
    autoware::common::type_traits::impossible_branch<MeasurementT>(),
    "This struct should always have a specialization.");
};

///
/// @brief      A specialization of `convert_to` to convert to any `Stamped` measurement.
///
/// @tparam     MeasurementT  A non-stamped measurement into which the conversion takes place.
///
template<typename MeasurementT>
struct MEASUREMENT_CONVERSION_PUBLIC convert_to<Stamped<MeasurementT>>
{
  ///
  /// @brief      A general function to be called for all `Stamped` measurements.
  ///
  /// @details    This function forwards the call to the conversion function that produces a
  ///             non-stamped version of the measurement and wraps it into the `Stamped` struct.
  ///
  /// @note       detail::unstamp function must exist for every input MsgT in order for this
  ///             function to compile.
  ///
  /// @param[in]  msg   The message to be converted to a stamped measurement.
  ///
  /// @tparam     MsgT  A message type that must have a header.
  ///
  /// @return     A stamped measurement.
  ///
  template<typename MsgT>
  static Stamped<MeasurementT> from(const MsgT & msg)
  {
    return Stamped<MeasurementT>{
      time_utils::from_message(msg.header.stamp),
      convert_to<MeasurementT>::from(detail::unstamp(msg))
    };
  }
};

/// A specialization for PoseMeasurementXYZRPY64.
template<>
struct MEASUREMENT_CONVERSION_PUBLIC convert_to<PoseMeasurementXYZRPY64>
{
  static PoseMeasurementXYZRPY64 from(const geometry_msgs::msg::PoseWithCovariance & msg);
};

/// A specialization for PoseMeasurementXYZ64.
template<>
struct MEASUREMENT_CONVERSION_PUBLIC convert_to<PoseMeasurementXYZ64>
{
  static PoseMeasurementXYZ64 from(
    const autoware_auto_msgs::msg::RelativePositionWithCovarianceStamped & msg);
};

}  // namespace state_estimation
}  // namespace common
}  // namespace autoware


#endif  // MEASUREMENT_CONVERSION__MEASUREMENT_CONVERSION_HPP_
