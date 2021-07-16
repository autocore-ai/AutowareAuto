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
// Co-developed by Tier IV, Inc. and Apex.AI, Inc.

/// @file
/// \brief This file has a number of explicit instantiations that are used throughout the code base.

#include <state_vector/common_states.hpp>

using autoware::common::types::float32_t;
using autoware::common::types::float64_t;

namespace autoware
{
namespace common
{
namespace state_vector
{

template class GenericState<float32_t,
    variable::X, variable::X_VELOCITY, variable::X_ACCELERATION,
    variable::Y, variable::Y_VELOCITY, variable::Y_ACCELERATION>;
template class GenericState<float64_t,
    variable::X, variable::X_VELOCITY, variable::X_ACCELERATION,
    variable::Y, variable::Y_VELOCITY, variable::Y_ACCELERATION>;


template class GenericState<float32_t,
    variable::X, variable::X_VELOCITY, variable::X_ACCELERATION,
    variable::Y, variable::Y_VELOCITY, variable::Y_ACCELERATION,
    variable::YAW, variable::YAW_CHANGE_RATE, variable::YAW_CHANGE_ACCELERATION>;
template class GenericState<float64_t,
    variable::X, variable::X_VELOCITY, variable::X_ACCELERATION,
    variable::Y, variable::Y_VELOCITY, variable::Y_ACCELERATION,
    variable::YAW, variable::YAW_CHANGE_RATE, variable::YAW_CHANGE_ACCELERATION>;

template class GenericState<float32_t,
    variable::X, variable::Y, variable::YAW,
    variable::XY_VELOCITY, variable::YAW_CHANGE_RATE,
    variable::XY_ACCELERATION>;
template class GenericState<float64_t,
    variable::X, variable::Y, variable::YAW,
    variable::XY_VELOCITY, variable::YAW_CHANGE_RATE,
    variable::XY_ACCELERATION>;

template class GenericState<float32_t,
    variable::X, variable::Y, variable::YAW,
    variable::XY_VELOCITY, variable::YAW_CHANGE_RATE>;
template class GenericState<float64_t,
    variable::X, variable::Y, variable::YAW,
    variable::XY_VELOCITY, variable::YAW_CHANGE_RATE>;

}  // namespace state_vector
}  // namespace common
}  // namespace autoware
