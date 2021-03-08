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

#include <common/types.hpp>
#include <motion_model/constant_acceleration.hpp>
#include <state_estimation_nodes/measurement.hpp>

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

}  // namespace prediction
}  // namespace autoware

#endif  // STATE_ESTIMATION_NODES__MEASUREMENT_TYPEDEFS_HPP_
