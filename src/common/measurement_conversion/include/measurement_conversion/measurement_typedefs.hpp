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
using PoseMeasurementXYZ = LinearMeasurement<state_vector::GenericState<ScalarT,
    state_vector::variable::X, state_vector::variable::Y, state_vector::variable::Z>>;
using PoseMeasurementXYZ32 = PoseMeasurementXYZ<common::types::float32_t>;
using PoseMeasurementXYZ64 = PoseMeasurementXYZ<common::types::float64_t>;

template<typename ScalarT>
using PoseMeasurementXYZRPY = LinearMeasurement<state_vector::GenericState<ScalarT,
    state_vector::variable::X, state_vector::variable::Y, state_vector::variable::Z,
    state_vector::variable::ROLL, state_vector::variable::PITCH, state_vector::variable::YAW>>;
using PoseMeasurementXYZRPY32 = PoseMeasurementXYZRPY<common::types::float32_t>;
using PoseMeasurementXYZRPY64 = PoseMeasurementXYZRPY<common::types::float64_t>;


}  // namespace state_estimation
}  // namespace common
}  // namespace autoware

#endif  // MEASUREMENT_CONVERSION__MEASUREMENT_TYPEDEFS_HPP_
