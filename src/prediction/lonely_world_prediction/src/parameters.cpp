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

#include "lonely_world_prediction/parameters.hpp"

#include <stdexcept>

namespace autoware
{
namespace prediction
{
using namespace std::literals;

Parameters::Parameters(std::chrono::microseconds time_step, std::chrono::microseconds time_horizon)
: m_time_step(time_step), m_time_horizon(time_horizon)
{
  if (time_step <= 0s) {
    throw std::invalid_argument(
            "prediction time step > 0 required. Got "s + std::to_string(time_step.count()));
  }
  if (time_horizon <= 0s) {
    throw std::invalid_argument(
            "prediction time horizon > 0 required. Got "s + std::to_string(time_horizon.count()));
  }
  if (time_horizon < time_step) {
    throw std::invalid_argument(
            "prediction time horizon >= time step required. Got "s +
            std::to_string(time_horizon.count()) + " and " + std::to_string(time_step.count()));
  }
}

}  // namespace prediction
}  // namespace autoware
