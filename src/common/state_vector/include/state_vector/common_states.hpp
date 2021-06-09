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
/// @details    All variables have their value, velocity and acceleration. All of the variables are
///             assumed to be independent here.
template<typename ScalarT>
using ConstAccelerationXY =
  GenericState<ScalarT,
    variable::X, variable::X_VELOCITY, variable::X_ACCELERATION,
    variable::Y, variable::Y_VELOCITY, variable::Y_ACCELERATION>;
using ConstAccelerationXY32 = ConstAccelerationXY<common::types::float32_t>;
using ConstAccelerationXY64 = ConstAccelerationXY<common::types::float64_t>;

///
/// @brief      A 3D state with no rotation.
///
/// @details    All variables have their value, velocity and acceleration. All of the variables are
///             assumed to be independent here.
template<typename ScalarT>
using ConstAccelerationXYZ =
  GenericState<ScalarT,
    variable::X, variable::X_VELOCITY, variable::X_ACCELERATION,
    variable::Y, variable::Y_VELOCITY, variable::Y_ACCELERATION,
    variable::Z, variable::Z_VELOCITY, variable::Z_ACCELERATION>;
using ConstAccelerationXYZ32 = ConstAccelerationXYZ<common::types::float32_t>;
using ConstAccelerationXYZ64 = ConstAccelerationXYZ<common::types::float64_t>;

///
/// @brief      A 2D state with a CCW rotation.
///
/// @details    All variables in this state have their value, velocity and acceleration. All of
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
/// @brief      A 3D state with a CCW yaw rotation.
///
/// @details    All variables in this state have their value, velocity and acceleration. All of
///             these variables are assumed to be independent here.
template<typename ScalarT>
using ConstAccelerationXYZYaw =
  GenericState<ScalarT,
    variable::X, variable::X_VELOCITY, variable::X_ACCELERATION,
    variable::Y, variable::Y_VELOCITY, variable::Y_ACCELERATION,
    variable::Z, variable::Z_VELOCITY, variable::Z_ACCELERATION,
    variable::YAW, variable::YAW_CHANGE_RATE, variable::YAW_CHANGE_ACCELERATION>;
using ConstAccelerationXYZYaw32 = ConstAccelerationXYZYaw<common::types::float32_t>;
using ConstAccelerationXYZYaw64 = ConstAccelerationXYZYaw<common::types::float64_t>;

///
/// @brief      A 3D state with a roll, pitch and yaw rotation. All rotations are CCW.
///
/// @details    All variables in this state have their value, velocity and acceleration. All of
///             these variables are assumed to be independent here.
///
/// @note       While any values can be stored in ROLL, PITCH and YAW variables and, strictly
///             speaking, the state itself does not enforce any convention, throughout our code base
///             we use the convention in which ROLL represents a rotation around X axis, PITCH -
///             around Y axis and YAW - around Z axis. Furthermore, they usually represent intrinsic
///             rotation and get applied in ZYX order, i.e., first, the rotation around Z axis is
///             applied, then the rotation around the resulting Y axis, then the rotation around the
///             resulting X axis.
///
template<typename ScalarT>
using ConstAccelerationXYZRPY =
  GenericState<ScalarT,
    variable::X, variable::X_VELOCITY, variable::X_ACCELERATION,
    variable::Y, variable::Y_VELOCITY, variable::Y_ACCELERATION,
    variable::Z, variable::Z_VELOCITY, variable::Z_ACCELERATION,
    variable::ROLL, variable::ROLL_CHANGE_RATE, variable::ROLL_CHANGE_ACCELERATION,
    variable::PITCH, variable::PITCH_CHANGE_RATE, variable::PITCH_CHANGE_ACCELERATION,
    variable::YAW, variable::YAW_CHANGE_RATE, variable::YAW_CHANGE_ACCELERATION>;
using ConstAccelerationXYZRPY32 = ConstAccelerationXYZRPY<common::types::float32_t>;
using ConstAccelerationXYZRPY64 = ConstAccelerationXYZRPY<common::types::float64_t>;

///
/// @brief      A state consisting of a 2D position, CCW orientation, speed along the orientation
///             vector, angle change rate and acceleration along the rotation vector.
///
/// @details    This state is usually used for the CATR motion model: i.e., a model in which
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
/// @details    This state is usually used for the CVTR motion model: i.e., a model in which
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
