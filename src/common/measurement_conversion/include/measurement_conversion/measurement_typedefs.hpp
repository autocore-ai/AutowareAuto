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

#ifndef MEASUREMENT_CONVERSION__MEASUREMENT_TYPEDEFS_HPP_
#define MEASUREMENT_CONVERSION__MEASUREMENT_TYPEDEFS_HPP_

#include <common/types.hpp>
#include <state_estimation/measurement/linear_measurement.hpp>
#include <state_vector/common_variables.hpp>

namespace autoware
{
namespace common
{
namespace state_estimation
{

template<typename MeasurementT>
struct Stamped
{
  std::chrono::system_clock::time_point timestamp;
  MeasurementT measurement;

  template<typename NewScalarT>
  auto cast() const noexcept
  {
    using NewMeasurementT = decltype(measurement.template cast<NewScalarT>());
    return Stamped<NewMeasurementT> {
      timestamp,
      measurement.template cast<NewScalarT>()
    };
  }
};

template<typename ScalarT>
using Measurement2dPose = LinearMeasurement<
  state_vector::GenericState<ScalarT,
  state_vector::variable::X, state_vector::variable::Y>>;
using Measurement2dPose32 = Measurement2dPose<common::types::float32_t>;
using Measurement2dPose64 = Measurement2dPose<common::types::float64_t>;

template<typename ScalarT>
using Measurement2dSpeed = LinearMeasurement<
  state_vector::GenericState<ScalarT,
  state_vector::variable::X_VELOCITY, state_vector::variable::Y_VELOCITY>>;
using Measurement2dSpeed32 = Measurement2dSpeed<common::types::float32_t>;
using Measurement2dSpeed64 = Measurement2dSpeed<common::types::float64_t>;

template<typename ScalarT>
using Measurement2dPoseAndSpeed = LinearMeasurement<
  state_vector::GenericState<ScalarT,
  state_vector::variable::X, state_vector::variable::Y,
  state_vector::variable::X_VELOCITY, state_vector::variable::Y_VELOCITY>>;
using Measurement2dPoseAndSpeed32 = Measurement2dPoseAndSpeed<common::types::float32_t>;
using Measurement2dPoseAndSpeed64 = Measurement2dPoseAndSpeed<common::types::float64_t>;

template<typename ScalarT>
using StampedMeasurement2dPose = Stamped<Measurement2dPose<ScalarT>>;
using StampedMeasurement2dPose32 = StampedMeasurement2dPose<common::types::float32_t>;
using StampedMeasurement2dPose64 = StampedMeasurement2dPose<common::types::float64_t>;

template<typename ScalarT>
using StampedMeasurement2dSpeed = Stamped<Measurement2dSpeed<ScalarT>>;
using StampedMeasurement2dSpeed32 = StampedMeasurement2dSpeed<common::types::float32_t>;
using StampedMeasurement2dSpeed64 = StampedMeasurement2dSpeed<common::types::float64_t>;

template<typename ScalarT>
using StampedMeasurement2dPoseAndSpeed = Stamped<Measurement2dPoseAndSpeed<ScalarT>>;
using StampedMeasurement2dPoseAndSpeed32 =
  StampedMeasurement2dPoseAndSpeed<common::types::float32_t>;
using StampedMeasurement2dPoseAndSpeed64 =
  StampedMeasurement2dPoseAndSpeed<common::types::float64_t>;

}  // namespace state_estimation
}  // namespace common
}  // namespace autoware

#endif  // MEASUREMENT_CONVERSION__MEASUREMENT_TYPEDEFS_HPP_
