// Copyright 2021 the Autoware Foundation
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//    http://www.apache.org/licenses/LICENSE-2.0
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

#include <state_vector/generic_state.hpp>

#include <common/types.hpp>
#include <gtest/gtest.h>

#include <tuple>

using autoware::common::state_vector::GenericState;
using autoware::common::state_vector::FloatState;
using autoware::common::state_vector::AngleVariable;
using autoware::common::state_vector::is_state;
using autoware::common::state_vector::variable::X;
using autoware::common::state_vector::variable::Y;
using autoware::common::state_vector::variable::YAW;
using autoware::common::types::float32_t;
using StateXY = GenericState<float32_t, X, Y>;
using StateXYaw = GenericState<float32_t, X, YAW>;

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
    "State:\n  autoware::common::state_vector::variable::X: 42\n"
    "  autoware::common::state_vector::variable::Y: 23\n");
}

/// @test We are able to copy the state into a bigger state.
TEST(KalmanFilterGenericStateTest, CopyIntoAnotherState) {
  using StateXY = FloatState<X, Y>;
  using StateXYaw = FloatState<X, YAW>;
  StateXY state{{23.0F, 42.0F}};
  const StateXY empty_state{};
  EXPECT_EQ(state.copy_into<StateXY>(), (StateXY{{23.0F, 42.0F}}));
  StateXYaw state_with_yaw{{42.0F, 42.23F}};
  EXPECT_EQ(state.copy_into(state_with_yaw), (StateXYaw{{23.0F, 42.23F}}));
  EXPECT_EQ(state.copy_into<StateXYaw>(), (StateXYaw{{23.0F, 0.0F}}));
  EXPECT_EQ(state_with_yaw.copy_into<StateXY>(), (StateXY{{42.0F, 0.0F}}));
  EXPECT_EQ(state_with_yaw.copy_into<StateXYaw>(), state_with_yaw);
  EXPECT_EQ(state_with_yaw.copy_into(empty_state), (StateXY{{42.0F, 0.0F}}));
  using StateYX = FloatState<Y, X>;
  EXPECT_EQ(state.copy_into<StateYX>(), (StateYX{{42.0F, 23.0F}}));
}

/// @test The states can be used with operators.
TEST(KalmanFilterGenericStateTest, Operators) {
  StateXYaw state{{0.0F, 0.0F}};
  state += StateXYaw{{42.0F, 42.0F}};
  EXPECT_EQ((StateXYaw{{42.0F, 42.0F}}), state);
  EXPECT_EQ(state, (StateXYaw{{22.0F, 22.0F}} + StateXYaw{{20.0F, 20.0F}}));
  state -= StateXYaw{{42.0F, 42.0F}};
  EXPECT_EQ((StateXYaw{{0.0F, 0.0F}}), state);
  EXPECT_EQ(state, (StateXYaw{{42.0F, 42.0F}} - StateXYaw{{42.0F, 42.0F}}));

  state += Eigen::Vector2f{42.0F, 42.0F};
  EXPECT_EQ((StateXYaw{{42.0F, 42.0F}}), state);
  state -= Eigen::Vector2f{42.0F, 42.0F};
  EXPECT_EQ((StateXYaw{{0.0F, 0.0F}}), state);
}

/// @test The angles are wrapped correctly if needed.
TEST(KalmanFilterGenericStateTest, WrapAngle) {
  StateXYaw state{{42.0F, 0.0F}};
  state.wrap_all_angles();
  EXPECT_EQ((StateXYaw{{42.0F, 0.0F}}), state);
  state.at<YAW>() = 2.0F * M_PIf32;
  state.wrap_all_angles();
  EXPECT_EQ((StateXYaw{{42.0F, 0.0F}}), state);
  state.at<YAW>() = M_PIf32 + 0.42F;
  EXPECT_EQ((StateXYaw{{42.0F, -M_PIf32 + 0.42F}}), wrap_all_angles(state));
  state.at<YAW>() = -M_PIf32 - 0.42F;
  state.wrap_all_angles();
  EXPECT_EQ((StateXYaw{{42.0F, M_PIf32 - 0.42F}}), state);

  // Check that wrapping works over the typical angle wrapping points: 0, pi, -pi, 2 * pi.
  const auto eps = 0.42F;
  StateXYaw angle_epsilon_plus{{0.0F, eps}};
  StateXYaw angle_epsilon_minus{{0.0F, -eps}};
  EXPECT_EQ(
    (StateXYaw{{0.0F, 2.0F * eps}}), wrap_all_angles(angle_epsilon_plus - angle_epsilon_minus));
  angle_epsilon_plus.at<YAW>() = M_PIf32 + eps;
  angle_epsilon_plus.wrap_all_angles();
  angle_epsilon_minus.at<YAW>() = M_PIf32 - eps;
  angle_epsilon_minus.wrap_all_angles();
  EXPECT_EQ(
    (StateXYaw{{0.0F, 2.0F * eps}}), wrap_all_angles(angle_epsilon_plus - angle_epsilon_minus));
  angle_epsilon_plus.at<YAW>() = -M_PIf32 + eps;
  angle_epsilon_plus.wrap_all_angles();
  angle_epsilon_minus.at<YAW>() = -M_PIf32 - eps;
  angle_epsilon_minus.wrap_all_angles();
  EXPECT_EQ(
    (StateXYaw{{0.0F, 2.0F * eps}}), wrap_all_angles(angle_epsilon_plus - angle_epsilon_minus));
  angle_epsilon_plus.at<YAW>() = -2.0F * M_PIf32 + eps;
  angle_epsilon_plus.wrap_all_angles();
  angle_epsilon_minus.at<YAW>() = 2.0F * M_PIf32 - eps;
  angle_epsilon_minus.wrap_all_angles();
  EXPECT_EQ(
    (StateXYaw{{0.0F, 2.0F * eps}}), wrap_all_angles(angle_epsilon_plus - angle_epsilon_minus));
}
