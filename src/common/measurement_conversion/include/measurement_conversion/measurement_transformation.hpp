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

#ifndef MEASUREMENT_CONVERSION__MEASUREMENT_TRANSFORMATION_HPP_
#define MEASUREMENT_CONVERSION__MEASUREMENT_TRANSFORMATION_HPP_

#include <measurement_conversion/measurement_typedefs.hpp>
#include <measurement_conversion/measurement_conversion.hpp>
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
/// @param[in]  measurement          The measurement.
/// @param[in]  tf__world__frame_id  A transform from frame_id to world frame.
///
/// @return     The measurement, but transformed
///
template<typename MeasurementT>
MeasurementT transform_measurement(
  const MeasurementT & measurement,
  const Eigen::Isometry3f & tf__world__frame_id)
{
  static_assert(
    sizeof(MeasurementT) == 0,
    "Only specializations for transform_measurement() function are allowed!");
}

///
/// @brief      Interface for transforming a measurement into a different coordinate system.
///
/// @details    This function is needed to transform messages like Odometry, where different parts
///             of the measurement must be transformed with different transformation matrices.
///
/// @tparam     MeasurementT  Type of measurement.
///
/// @return     The measurement, but transformed
///
template<typename MeasurementT>
MeasurementT transform_measurement(
  const MeasurementT &,
  const Eigen::Isometry3f &,
  const Eigen::Isometry3f &)
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
  const Eigen::Isometry3f & tf__world__frame_id)
{
  const auto measurement = message_to_measurement<MeasurementT, MessageT>(msg);
  return transform_measurement(measurement, tf__world__frame_id);
}

///
/// @brief      Convenience function for converting a message into a measurement and transforming
///             it into a different coordinate frame.
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
MeasurementT message_to_transformed_measurement(
  const MessageT & msg, const Eigen::Isometry3f & tf__world__frame_id,
  const Eigen::Isometry3f & tf__world__child_frame_id)
{
  const auto measurement = message_to_measurement<MeasurementT, MessageT>(msg);
  return transform_measurement(measurement, tf__world__frame_id, tf__world__child_frame_id);
}

///
/// @brief      Downscale the isometry to a lower dimension if needed.
///
/// @param[in]  isometry              The isometry transform
///
/// @tparam     kStateDimensionality  Dimensionality of the space.
/// @tparam     FloatT                Type of scalar.
///
/// @return     Downscaled isometry.
///
template<std::int32_t kStateDimensionality, typename FloatT>
static constexpr Eigen::Transform<
  FloatT, kStateDimensionality, Eigen::TransformTraits::Isometry> downscale_isometry(
  const Eigen::Transform<FloatT, 3, Eigen::TransformTraits::Isometry> & isometry)
{
  static_assert(kStateDimensionality <= 3, "We only handle scaling the isometry down.");
  using Isometry = Eigen::Transform<
    FloatT, kStateDimensionality, Eigen::TransformTraits::Isometry>;
  Isometry result{Isometry::Identity()};
  result.linear() = isometry.rotation()
    .template block<kStateDimensionality, kStateDimensionality>(0, 0);
  result.translation() = isometry.translation().topRows(kStateDimensionality);
  return result;
}

// Doxygen is buggy when the parameters are repeated here, so they are omitted.

///
/// @brief      Specialization of transform_measurement for speed measurement.
///
template<>
MEASUREMENT_CONVERSION_PUBLIC Measurement2dSpeed transform_measurement(
  const Measurement2dSpeed & measurement,
  const Eigen::Isometry3f & tf__world__frame_id);

///
/// @brief      Specialization of transform_measurement for pose measurement.
///
template<>
MEASUREMENT_CONVERSION_PUBLIC Measurement2dPose transform_measurement(
  const Measurement2dPose & measurement,
  const Eigen::Isometry3f & tf__world__frame_id);

///
/// @brief      Specialization of transform_measurement for stamped speed measurement.
///
template<>
MEASUREMENT_CONVERSION_PUBLIC StampedMeasurement2dSpeed transform_measurement(
  const StampedMeasurement2dSpeed & measurement,
  const Eigen::Isometry3f & tf__world__frame_id);

///
/// @brief      Specialization of transform_measurement for stamped pose measurement.
///
template<>
MEASUREMENT_CONVERSION_PUBLIC StampedMeasurement2dPose transform_measurement(
  const StampedMeasurement2dPose & measurement,
  const Eigen::Isometry3f & tf__world__frame_id);

///
/// @brief      Specialization of transform_measurement for stamped pose and speed measurement.
///
template<>
MEASUREMENT_CONVERSION_PUBLIC StampedMeasurement2dPoseAndSpeed transform_measurement(
  const StampedMeasurement2dPoseAndSpeed & measurement,
  const Eigen::Isometry3f & tf__world__frame_id,
  const Eigen::Isometry3f & tf__world__child_frame_id);

}  // namespace state_estimation
}  // namespace common
}  // namespace autoware


#endif  // MEASUREMENT_CONVERSION__MEASUREMENT_TRANSFORMATION_HPP_
