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
/// \brief This file defines tests for the generic state.

#include <kalman_filter/generic_state.hpp>

#include <common/types.hpp>
#include <gtest/gtest.h>

#include <tuple>

using autoware::prediction::GenericState;
using autoware::prediction::FloatState;
using autoware::prediction::AngleVariable;
using autoware::prediction::is_state;
using autoware::prediction::variable::X;
using autoware::prediction::variable::Y;
using autoware::prediction::variable::YAW;
using autoware::common::types::float32_t;
using StateXY = GenericState<float32_t, X, Y>;

struct NotAState {};

struct CustomVariable : AngleVariable {};

/// @test Create an empty state.
TEST(KalmanFilterGenericStateTest, CreateEmpty) {
  EXPECT_EQ(2, StateXY::size());
  StateXY state{};
  EXPECT_TRUE(state.vector().isApproxToConstant(0.0F));
  EXPECT_TRUE(is_state<StateXY>::value);
  EXPECT_TRUE((is_state<FloatState<X, Y, CustomVariable>>::value));
  EXPECT_FALSE(is_state<NotAState>::value);
}

/// @test Create a non-empty state and access its values.
TEST(KalmanFilterGenericStateTest, CreateAndAccess) {
  StateXY state{{42.0F, 23.0F}};
  ASSERT_EQ(2, state.size());
  EXPECT_FLOAT_EQ(state[0], 42.0F);
  EXPECT_FLOAT_EQ(state[StateXY::index_of<X>()], 42.0F);
  EXPECT_FLOAT_EQ(state.at<X>(), 42.0F);
  EXPECT_FLOAT_EQ(state.at(X{}), 42.0F);
  EXPECT_FLOAT_EQ(state[1], 23.0F);
  EXPECT_FLOAT_EQ(state[StateXY::index_of<Y>()], 23.0F);
  EXPECT_FLOAT_EQ(state.at<Y>(), 23.0F);

  state[0] = 42.42F;
  EXPECT_FLOAT_EQ(state[0], 42.42F);
  state.at<X>() = 23.23F;
  EXPECT_FLOAT_EQ(state[0], 23.23F);
}

/// @test We are able to print the state to string.
TEST(KalmanFilterGenericStateTest, Print) {
  StateXY state{{42.0F, 23.0F}};
  std::stringstream str_stream;
  str_stream << state << std::endl;
  EXPECT_EQ(
    str_stream.str(),
    "State:\n  autoware::prediction::variable::X: 42\n  autoware::prediction::variable::Y: 23\n");
}

/// @test We are able to copy the state into a bigger state.
TEST(KalmanFilterGenericStateTest, CopyIntoAnotherState) {
  using StateXY = FloatState<X, Y>;
  using StateXYYaw = FloatState<X, Y, YAW>;
  StateXY state{{23.0F, 42.0F}};
  EXPECT_EQ(state.copy_into<StateXY>(), (StateXY{{23.0F, 42.0F}}));
  StateXYYaw state_with_yaw{{42.0F, 23.0F, 42.23F}};
  EXPECT_EQ(state.copy_into(state_with_yaw), (StateXYYaw{{23.0F, 42.0F, 42.23F}}));
  EXPECT_EQ(state_with_yaw.copy_into<StateXY>(), (StateXY{{42.0F, 23.0F}}));
  EXPECT_EQ(state_with_yaw.copy_into<StateXYYaw>(), state_with_yaw);
  using StateYX = FloatState<Y, X>;
  EXPECT_EQ(state.copy_into<StateYX>(), (StateYX{{42.0F, 23.0F}}));
}
