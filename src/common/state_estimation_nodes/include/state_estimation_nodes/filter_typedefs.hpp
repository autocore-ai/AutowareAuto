// Copyright 2021 the Autoware Foundation
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
//
// Developed by Apex.AI, Inc.

/// \copyright Copyright 2021 the Autoware Foundation
/// All rights reserved.
/// \file

#ifndef STATE_ESTIMATION_NODES__FILTER_TYPEDEFS_HPP_
#define STATE_ESTIMATION_NODES__FILTER_TYPEDEFS_HPP_

#include <motion_model/linear_motion_model.hpp>
#include <state_estimation/kalman_filter/kalman_filter.hpp>
#include <state_estimation/noise_model/wiener_noise.hpp>
#include <state_vector/common_states.hpp>

namespace autoware
{
namespace common
{
namespace state_estimation
{
using ConstAccelerationKalmanFilterXY = KalmanFilter<
  motion_model::LinearMotionModel<state_vector::ConstAccelerationXY32>,
  WienerNoise<common::state_vector::ConstAccelerationXY32>>;

using ConstAccelerationKalmanFilterXYZRPY = KalmanFilter<
  motion_model::LinearMotionModel<state_vector::ConstAccelerationXYZRPY32>,
  WienerNoise<common::state_vector::ConstAccelerationXYZRPY32>>;
}  // namespace state_estimation
}  // namespace common
}  // namespace autoware


#endif  // STATE_ESTIMATION_NODES__FILTER_TYPEDEFS_HPP_
