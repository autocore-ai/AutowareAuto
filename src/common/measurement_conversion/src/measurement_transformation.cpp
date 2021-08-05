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

#include <measurement_conversion/measurement_transformation.hpp>

namespace autoware
{
namespace common
{
namespace state_estimation
{

template<>
PoseMeasurementXYZ64 transform_measurement(
  const PoseMeasurementXYZ64 & measurement,
  const Eigen::Isometry3d & tf__world__frame_id)
{
  return PoseMeasurementXYZ64{
    tf__world__frame_id * measurement.state().vector(),
      tf__world__frame_id.rotation() *
      measurement.covariance().matrix() *
      tf__world__frame_id.rotation().transpose()
  };
}

template<>
Stamped<PoseMeasurementXYZ64> transform_measurement(
  const Stamped<PoseMeasurementXYZ64> & measurement,
  const Eigen::Isometry3d & tf__world__frame_id)
{
  return Stamped<PoseMeasurementXYZ64> {
    measurement.timestamp,
    transform_measurement(measurement.measurement, tf__world__frame_id)};
}

}  // namespace state_estimation
}  // namespace common
}  // namespace autoware
