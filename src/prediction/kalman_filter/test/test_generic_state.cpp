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
using autoware::common::types::float32_t;

struct NotAState {};

struct CustomVariable : AngleVariable {};

/// @test Create an empty state.
TEST(KalmanFilterGenericStateTest, CreateEmpty) {
  using StateXY = GenericState<float32_t, X, Y>;
  EXPECT_EQ(2, StateXY::size());
  StateXY state{};
  EXPECT_TRUE(state.vector().isApproxToConstant(0.0F));
  EXPECT_TRUE(is_state<StateXY>::value);
  EXPECT_TRUE((is_state<FloatState<X, Y, CustomVariable>>::value));
  EXPECT_FALSE(is_state<NotAState>::value);
}

/// @test Create a non-empty state and access its values.
TEST(KalmanFilterGenericStateTest, CreateAndAccess) {
  using StateXY = GenericState<float32_t, X, Y>;
  StateXY state{{42.0F, 23.0F}};
  ASSERT_EQ(2, state.size());
  EXPECT_FLOAT_EQ(state[0], 42.0F);
  EXPECT_FLOAT_EQ(state[StateXY::index_of<X>()], 42.0F);
  EXPECT_FLOAT_EQ(state.at<X>(), 42.0F);
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
  using StateXY = GenericState<float32_t, X, Y>;
  StateXY state{{42.0F, 23.0F}};
  std::stringstream str_stream;
  str_stream << state << std::endl;
  EXPECT_EQ(
    str_stream.str(),
    "State:\n  autoware::prediction::variable::X: 42\n  autoware::prediction::variable::Y: 23\n");
}
