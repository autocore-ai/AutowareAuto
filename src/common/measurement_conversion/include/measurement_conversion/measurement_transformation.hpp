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

#ifndef MEASUREMENT_CONVERSION__MEASUREMENT_TRANSFORMATION_HPP_
#define MEASUREMENT_CONVERSION__MEASUREMENT_TRANSFORMATION_HPP_

#include <measurement_conversion/eigen_utils.hpp>
#include <measurement_conversion/measurement_conversion.hpp>
#include <measurement_conversion/measurement_typedefs.hpp>
#include <measurement_conversion/visibility_control.hpp>

#include <Eigen/Geometry>

namespace autoware
{
namespace common
{
namespace state_estimation
{

///
/// @brief      Interface for transforming a measurement into a different coordinate system.
///
/// @tparam     MeasurementT  Type of measurement.
///
/// @return     The measurement, but transformed
///
template<typename MeasurementT>
MeasurementT transform_measurement(
  const MeasurementT &,
  const Eigen::Isometry3d &)
{
  static_assert(
    sizeof(MeasurementT) == 0,
    "Only specializations for transform_measurement() function are allowed!");
}

///
/// @brief      Convenience function for converting a message into a measurement and transforming
///             it into a different coordinate frame.
///
/// @tparam     MeasurementT  Type of measurement.
/// @tparam     MessageT      Type of ROS 2 message.
///
/// @return     The measurement created from a message.
///
template<typename MeasurementT, typename MessageT>
MeasurementT message_to_transformed_measurement(
  const MessageT & msg,
  const Eigen::Isometry3d & tf__world__frame_id)
{
  const auto measurement = convert_to<MeasurementT>::from(msg);
  return transform_measurement(measurement, tf__world__frame_id);
}


// Doxygen is buggy when the parameters are repeated here, so they are omitted.

///
/// @brief      Specialization of transform_measurement for pose measurement.
///
template<>
MEASUREMENT_CONVERSION_PUBLIC PoseMeasurementXYZ64 transform_measurement(
  const PoseMeasurementXYZ64 & measurement,
  const Eigen::Isometry3d & tf__world__frame_id);

///
/// @brief      Specialization of transform_measurement for stamped pose measurement.
///
template<>
MEASUREMENT_CONVERSION_PUBLIC Stamped<PoseMeasurementXYZ64> transform_measurement(
  const Stamped<PoseMeasurementXYZ64> & measurement,
  const Eigen::Isometry3d & tf__world__frame_id);

}  // namespace state_estimation
}  // namespace common
}  // namespace autoware


#endif  // MEASUREMENT_CONVERSION__MEASUREMENT_TRANSFORMATION_HPP_
