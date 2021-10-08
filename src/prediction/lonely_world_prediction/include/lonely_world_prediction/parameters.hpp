// Copyright 2021 The Autoware Foundation
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

// Co-developed by Tier IV, Inc. and Apex.AI, Inc.

/// \copyright Copyright 2021 The Autoware Foundation
/// \file
/// \brief Define configuration parameters

#ifndef LONELY_WORLD_PREDICTION__PARAMETERS_HPP_
#define LONELY_WORLD_PREDICTION__PARAMETERS_HPP_

#include <chrono>

#include "lonely_world_prediction/visibility_control.hpp"

namespace autoware
{
namespace prediction
{
class Parameters
{
public:
  /**
   * parameters for lonely-world prediction
   *
   * Enforced constraints:
   * - step > 0
   * - horizon > 0
   * - horizon >= step
   *
   * @throw `std::invalid_argument` if constraints are not satisfied
   * @param time_step time difference between two consecutive predicted states
   * @param time_horizon time difference from now into the future until which to predict objects
   */
  LONELY_WORLD_PREDICTION_PUBLIC Parameters(
    std::chrono::microseconds time_step, std::chrono::microseconds time_horizon);

  /**
   * getter for the time step
   */
  inline std::chrono::microseconds LONELY_WORLD_PREDICTION_PUBLIC time_step() const noexcept
  {
    return m_time_step;
  }

  /**
   * getter for the time horizon
   */
  inline std::chrono::microseconds LONELY_WORLD_PREDICTION_PUBLIC time_horizon() const noexcept
  {
    return m_time_horizon;
  }

private:
  std::chrono::microseconds m_time_step, m_time_horizon;
};
}  // namespace prediction
}  // namespace autoware

#endif  // LONELY_WORLD_PREDICTION__PARAMETERS_HPP_
