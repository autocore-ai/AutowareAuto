// Copyright 2021 Robotec.ai
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

#include "autoware_state_monitor/state.hpp"

#include <memory>

#include "gtest/gtest.h"

#include "autoware_auto_msgs/msg/autoware_state.hpp"

#include "test_utils.hpp"

using autoware::state_monitor::State;
using autoware::state_monitor::toString;
using autoware_auto_msgs::msg::AutowareState;

TEST(StateTest, autoware_state_to_string)
{
  EXPECT_EQ(toString(AutowareState::INITIALIZING), "Initializing");
  EXPECT_EQ(toString(AutowareState::WAITING_FOR_ROUTE), "WaitingForRoute");
  EXPECT_EQ(toString(AutowareState::PLANNING), "Planning");
  EXPECT_EQ(toString(AutowareState::WAITING_FOR_ENGAGE), "WaitingForEngage");
  EXPECT_EQ(toString(AutowareState::DRIVING), "Driving");
  EXPECT_EQ(toString(AutowareState::ARRIVED_GOAL), "ArrivedGoal");
  EXPECT_EQ(toString(AutowareState::FINALIZING), "Finalizing");
  EXPECT_ANY_THROW(toString(static_cast<State>(10)));
}
