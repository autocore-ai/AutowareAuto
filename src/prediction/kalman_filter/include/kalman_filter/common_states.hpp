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
/// \brief This file holds a collection of states that are commonly used in this package.

#ifndef KALMAN_FILTER__COMMON_STATES_HPP_
#define KALMAN_FILTER__COMMON_STATES_HPP_

#include <kalman_filter/common_variables.hpp>
#include <kalman_filter/generic_state.hpp>

namespace autoware
{
namespace prediction
{
namespace state
{

///
/// @brief      A 2D state with no rotation.
///
///             All variables have their value, velocity and acceleration. All of the variables are
///             assumed to be independent here.
using ConstAccelerationXY =
  FloatState<
  variable::X, variable::X_VELOCITY, variable::X_ACCELERATION,
  variable::Y, variable::Y_VELOCITY, variable::Y_ACCELERATION>;

///
/// @brief      A 2D state with a CCW rotation.
///
///             All variables in this state have their value, velocity and acceleration. All of
///             these variables are assumed to be independent here.
using ConstAccelerationXYYaw =
  FloatState<
  variable::X, variable::X_VELOCITY, variable::X_ACCELERATION,
  variable::Y, variable::Y_VELOCITY, variable::Y_ACCELERATION,
  variable::YAW, variable::YAW_CHANGE_RATE, variable::YAW_CHANGE_ACCELERATION>;

///
/// @brief      A state consisting of a 2D position, CCW orientation, speed along the orientation
///             vector, angle change rate and acceleration along the rotation vector.
///
///             This state is usually used for the CATR motion model: i.e., a model in which
///             Constant Acceleration and Turn Rate are assumed for a differential drive base.
using ConstantAccelerationAndTurnRate =
  FloatState<
  variable::X, variable::Y, variable::YAW,
  variable::XY_VELOCITY, variable::YAW_CHANGE_RATE,
  variable::XY_ACCELERATION>;

///
/// @brief      A state consisting of a 2D position, CCW orientation, speed along the orientation
///             vector, and an orientation change rate.
///
///             This state is usually used for the CVTR motion model: i.e., a model in which
///             Constant Velocity and Turn Rate are assumed for a differential drive base.
using ConstantVelocityAndTurnRate =
  FloatState<
  variable::X, variable::Y, variable::YAW,
  variable::XY_VELOCITY, variable::YAW_CHANGE_RATE>;

}  // namespace state
}  // namespace prediction
}  // namespace autoware

#endif  // KALMAN_FILTER__COMMON_STATES_HPP_
