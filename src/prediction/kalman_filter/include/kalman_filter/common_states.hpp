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

using ConstAccelerationXY =
  FloatState<
  variable::X, variable::X_VELOCITY, variable::X_ACCELERATION,
  variable::Y, variable::Y_VELOCITY, variable::Y_ACCELERATION>;

using ConstAccelerationXYYaw =
  FloatState<
  variable::X, variable::X_VELOCITY, variable::X_ACCELERATION,
  variable::Y, variable::Y_VELOCITY, variable::Y_ACCELERATION,
  variable::YAW, variable::YAW_CHANGE_RATE, variable::YAW_CHANGE_ACCELERATION>;

}  // namespace state
}  // namespace prediction
}  // namespace autoware

#endif  // KALMAN_FILTER__COMMON_STATES_HPP_
