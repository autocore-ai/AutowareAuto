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
//
// Developed by Apex.AI, Inc.

/// \copyright Copyright 2021 the Autoware Foundation
/// All rights reserved.
/// \file
/// \brief This file holds a collection of states that are commonly used in this package.

#ifndef STATE_VECTOR__COMMON_STATES_HPP_
#define STATE_VECTOR__COMMON_STATES_HPP_

#include <common/types.hpp>
#include <state_vector/common_variables.hpp>
#include <state_vector/generic_state.hpp>

namespace autoware
{
namespace common
{
namespace state_vector
{

///
/// @brief      A 2D state with no rotation.
///
///             All variables have their value, velocity and acceleration. All of the variables are
///             assumed to be independent here.
template<typename ScalarT>
using ConstAccelerationXY =
  GenericState<ScalarT,
    variable::X, variable::X_VELOCITY, variable::X_ACCELERATION,
    variable::Y, variable::Y_VELOCITY, variable::Y_ACCELERATION>;
using ConstAccelerationXY32 = ConstAccelerationXY<common::types::float32_t>;
using ConstAccelerationXY64 = ConstAccelerationXY<common::types::float64_t>;

///
/// @brief      A 2D state with a CCW rotation.
///
///             All variables in this state have their value, velocity and acceleration. All of
///             these variables are assumed to be independent here.
template<typename ScalarT>
using ConstAccelerationXYYaw =
  GenericState<ScalarT,
    variable::X, variable::X_VELOCITY, variable::X_ACCELERATION,
    variable::Y, variable::Y_VELOCITY, variable::Y_ACCELERATION,
    variable::YAW, variable::YAW_CHANGE_RATE, variable::YAW_CHANGE_ACCELERATION>;
using ConstAccelerationXYYaw32 = ConstAccelerationXYYaw<common::types::float32_t>;
using ConstAccelerationXYYaw64 = ConstAccelerationXYYaw<common::types::float64_t>;

///
/// @brief      A state consisting of a 2D position, CCW orientation, speed along the orientation
///             vector, angle change rate and acceleration along the rotation vector.
///
///             This state is usually used for the CATR motion model: i.e., a model in which
///             Constant Acceleration and Turn Rate are assumed for a differential drive base.
template<typename ScalarT>
using ConstantAccelerationAndTurnRate =
  GenericState<ScalarT,
    variable::X, variable::Y, variable::YAW,
    variable::XY_VELOCITY, variable::YAW_CHANGE_RATE,
    variable::XY_ACCELERATION>;
using ConstantAccelerationAndTurnRate32 = ConstantAccelerationAndTurnRate<common::types::float32_t>;
using ConstantAccelerationAndTurnRate64 = ConstantAccelerationAndTurnRate<common::types::float64_t>;

///
/// @brief      A state consisting of a 2D position, CCW orientation, speed along the orientation
///             vector, and an orientation change rate.
///
///             This state is usually used for the CVTR motion model: i.e., a model in which
///             Constant Velocity and Turn Rate are assumed for a differential drive base.
template<typename ScalarT>
using ConstantVelocityAndTurnRate =
  GenericState<ScalarT,
    variable::X, variable::Y, variable::YAW,
    variable::XY_VELOCITY, variable::YAW_CHANGE_RATE>;
using ConstantVelocityAndTurnRate32 = ConstantVelocityAndTurnRate<common::types::float32_t>;
using ConstantVelocityAndTurnRate64 = ConstantVelocityAndTurnRate<common::types::float64_t>;

}  // namespace state_vector
}  // namespace common
}  // namespace autoware

#endif  // STATE_VECTOR__COMMON_STATES_HPP_
