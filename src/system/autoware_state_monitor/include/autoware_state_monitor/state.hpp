// Copyright 2021 The Autoware Foundation
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//
// Co-developed by Tier IV, Inc. and Robotec.AI sp. z o.o.

#ifndef AUTOWARE_STATE_MONITOR__STATE_HPP_
#define AUTOWARE_STATE_MONITOR__STATE_HPP_

#include <string>

#include "autoware_auto_msgs/msg/autoware_state.hpp"

namespace autoware
{
namespace state_monitor
{

/// \brief Defines states of the Autoware system
using State = uint8_t;

/// \brief Converts AutowareState to string
inline std::string toString(const State state)
{
  using autoware_auto_msgs::msg::AutowareState;

  switch (state) {
    case AutowareState::INITIALIZING:
      return "Initializing";
    case AutowareState::WAITING_FOR_ROUTE:
      return "WaitingForRoute";
    case AutowareState::PLANNING:
      return "Planning";
    case AutowareState::WAITING_FOR_ENGAGE:
      return "WaitingForEngage";
    case AutowareState::DRIVING:
      return "Driving";
    case AutowareState::ARRIVED_GOAL:
      return "ArrivedGoal";
    case AutowareState::FINALIZING:
      return "Finalizing";
    default:
      throw std::runtime_error("Invalid state: " + std::to_string(state));
  }
}

}  // namespace state_monitor
}  // namespace autoware

#endif  // AUTOWARE_STATE_MONITOR__STATE_HPP_
