// Copyright 2020 Embotech AG, Zurich, Switzerland. All rights reserved.
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
#ifndef PARKING_PLANNER__CONFIGURATION_HPP_
#define PARKING_PLANNER__CONFIGURATION_HPP_

#include <common/types.hpp>

namespace autoware
{

namespace motion
{

namespace planning
{


namespace parking_planner
{
using autoware::common::types::float64_t;
/// NLP horizon length, roughly proportional to the maximum useful trajectory length
constexpr std::size_t HORIZON_LENGTH = 30;

/// Maximum number of obstacles supported by the NLP planner
constexpr std::size_t MAX_NUMBER_OF_OBSTACLES = 10;

/// Maximum number of hyperplanes per obstacle supported by the NLP planner
constexpr std::size_t MAX_HYPERPLANES_PER_OBSTACLE = 4;

/// Maximum number of ego vehicle hyperplanes supported by the NLP planner
constexpr std::size_t MAX_EGO_HYPERPLANES = 4;

/// Step size (in seconds) for the RK4 integration
constexpr float64_t INTEGRATION_STEP_SIZE = 0.3;

/// \brief Number of (sub-)steps performed INTEGRATION_STEP_SIZE for the RK4 integration.
constexpr std::size_t NUMBER_OF_INTEGRATION_STEPS = 4;

}  // namespace parking_planner
}  // namespace planning
}  // namespace motion
}  // namespace autoware

#endif  // PARKING_PLANNER__CONFIGURATION_HPP_
