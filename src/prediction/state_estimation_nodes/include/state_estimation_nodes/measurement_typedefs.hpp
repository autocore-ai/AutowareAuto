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

#ifndef STATE_ESTIMATION_NODES__MEASUREMENT_TYPEDEFS_HPP_
#define STATE_ESTIMATION_NODES__MEASUREMENT_TYPEDEFS_HPP_

#include <measurement/linear_measurement.hpp>

namespace autoware
{
namespace prediction
{

template<typename MeasurementT>
struct Stamped
{
  std::chrono::system_clock::time_point timestamp;
  MeasurementT measurement;
};

using MeasurementPose = LinearMeasurement<
  FloatState<variable::X, variable::Y>>;
using MeasurementSpeed = LinearMeasurement<
  FloatState<variable::X_VELOCITY, variable::Y_VELOCITY>>;
using MeasurementPoseAndSpeed = LinearMeasurement<
  FloatState<variable::X, variable::Y, variable::X_VELOCITY, variable::Y_VELOCITY>>;

using StampedMeasurementPose = Stamped<MeasurementPose>;
using StampedMeasurementSpeed = Stamped<MeasurementSpeed>;
using StampedMeasurementPoseAndSpeed = Stamped<MeasurementPoseAndSpeed>;

}  // namespace prediction
}  // namespace autoware

#endif  // STATE_ESTIMATION_NODES__MEASUREMENT_TYPEDEFS_HPP_
