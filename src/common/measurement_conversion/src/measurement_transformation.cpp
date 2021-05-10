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

#include <measurement_conversion/measurement_transformation.hpp>

namespace autoware
{
namespace common
{
namespace state_estimation
{

template<>
Measurement2dSpeed transform_measurement(
  const Measurement2dSpeed & measurement,
  const Eigen::Isometry3f & tf__world__frame_id)
{
  const auto converted_tf__world__frame_id = downscale_isometry<2>(tf__world__frame_id);
  return Measurement2dSpeed{
    converted_tf__world__frame_id.rotation() * measurement.state().vector(),
    Eigen::Matrix2f{converted_tf__world__frame_id.rotation() * measurement.covariance().matrix() *
      converted_tf__world__frame_id.rotation().transpose()}};
}

template<>
Measurement2dPose transform_measurement(
  const Measurement2dPose & measurement,
  const Eigen::Isometry3f & tf__world__frame_id)
{
  const auto converted_tf__world__frame_id = downscale_isometry<2>(tf__world__frame_id);
  return Measurement2dPose{
    converted_tf__world__frame_id * measurement.state().vector(),
    Eigen::Matrix2f
    {
      converted_tf__world__frame_id.rotation() *
        measurement.covariance().matrix() *
        converted_tf__world__frame_id.rotation().transpose()
    }
  };
}

template<>
StampedMeasurement2dSpeed transform_measurement(
  const StampedMeasurement2dSpeed & measurement,
  const Eigen::Isometry3f & tf__world__frame_id)
{
  return StampedMeasurement2dSpeed {
    measurement.timestamp,
    transform_measurement(measurement.measurement, tf__world__frame_id)};
}

template<>
StampedMeasurement2dPose transform_measurement(
  const StampedMeasurement2dPose & measurement,
  const Eigen::Isometry3f & tf__world__frame_id)
{
  return StampedMeasurement2dPose {
    measurement.timestamp,
    transform_measurement(measurement.measurement, tf__world__frame_id)};
}


template<>
StampedMeasurement2dPoseAndSpeed transform_measurement(
  const StampedMeasurement2dPoseAndSpeed & measurement,
  const Eigen::Isometry3f & tf__world__frame_id)
{
  const auto converted_tf__world__frame_id = downscale_isometry<2>(tf__world__frame_id);
  const auto & state = measurement.measurement.state().vector();
  const Eigen::Vector2f pos_state = converted_tf__world__frame_id *
    Eigen::Vector2f{state(0), state(1)};
  const Eigen::Vector2f speed_state = converted_tf__world__frame_id.rotation() *
    Eigen::Vector2f{state(2), state(3)};

  return StampedMeasurement2dPoseAndSpeed{
    measurement.timestamp,
    Measurement2dPoseAndSpeed{
      (Eigen::Vector4f{} << pos_state, speed_state).finished(),
      measurement.measurement.covariance()}
  };
}

}  // namespace state_estimation
}  // namespace common
}  // namespace autoware
