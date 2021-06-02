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
// Developed by Apex.AI, Inc.

#include <motion_model/differential_drive_motion_model.hpp>
#include <state_vector/common_states.hpp>

#include <gtest/gtest.h>

#include <array>
#include <chrono>
#include <cmath>

using autoware::common::motion_model::CatrMotionModel32;
using autoware::common::motion_model::CvtrMotionModel32;
using autoware::common::state_vector::variable::X;
using autoware::common::state_vector::variable::Y;
using autoware::common::state_vector::variable::YAW;
using autoware::common::state_vector::variable::YAW_CHANGE_RATE;
using autoware::common::state_vector::variable::XY_VELOCITY;
using autoware::common::state_vector::variable::XY_ACCELERATION;
using autoware::common::types::float32_t;
using std::sqrt;
using std::sin;
using std::cos;

namespace
{
constexpr auto kEpsilon = 1.0e-6F;
}  // namespace

/// @test Make sure a static object stays static.
TEST(CvtrMotionModelTest, PredictStaticObject) {
  CvtrMotionModel32 model;
  CvtrMotionModel32::State initial_state{CvtrMotionModel32::State{}};
  initial_state.at<X>() = 42.0F;
  initial_state.at<Y>() = 42.0F;
  initial_state.at<YAW>() = 1.0F;
  EXPECT_EQ(
    initial_state, model.predict(initial_state, std::chrono::milliseconds{100LL}));
}


/// @test Make sure a static object stays static.
TEST(CatrMotionModelTest, PredictStaticObject) {
  CatrMotionModel32 model;
  CatrMotionModel32::State initial_state{CatrMotionModel32::State{}};
  initial_state.at<X>() = 42.0F;
  initial_state.at<Y>() = 42.0F;
  initial_state.at<YAW>() = 1.0F;
  EXPECT_EQ(
    initial_state, model.predict(initial_state, std::chrono::milliseconds{100LL}));
}

/// @test Check that the Jacobian matches one computed symbolically when turn rate is not zero.
TEST(CvtrMotionModelTest, TestJacobianNonZeroTurnRate) {
  CvtrMotionModel32::State state{CvtrMotionModel32::State{}};
  state.at<X>() = 42.0F;
  state.at<Y>() = 23.0F;
  state.at<YAW>() = 0.5F;
  state.at<XY_VELOCITY>() = 2.0F;
  state.at<YAW_CHANGE_RATE>() = 2.0F;

  // Computed with SymPy.
  CvtrMotionModel32::State::Matrix expected_jacobian{(CvtrMotionModel32::State::Matrix{} <<
      1.0F, 0.0F, -0.112740374605884F, 0.082396074316744F, -0.00591185558829518F,
      0.0F, 1.0F, 0.164792148633488F, 0.0563701873029421F, 0.00805158142082696F,
      0.0F, 0.0F, 1.0F, 0.0F, 0.0F,
      0.0F, 0.0F, 0.0F, 1.0F, 0.1F,
      0.0F, 0.0F, 0.0F, 0.0F, 1.0F).finished()};

  CvtrMotionModel32 model;
  const auto jacobian = model.jacobian(state, std::chrono::milliseconds{100LL});
  EXPECT_TRUE(expected_jacobian.isApprox(jacobian, kEpsilon)) <<
    "Jacobians don't match: \nExpected:\n" << expected_jacobian << "\nActual:\n" << jacobian;
}

/// @test Check that the Jacobian matches one computed symbolically when turn rate is not zero.
TEST(CatrMotionModelTest, TestJacobianNonZeroTurnRate) {
  CatrMotionModel32::State state{CatrMotionModel32::State{}};
  state.at<X>() = 42.0F;
  state.at<Y>() = 23.0F;
  state.at<YAW>() = 0.5F;
  state.at<XY_VELOCITY>() = 2.0F;
  state.at<YAW_CHANGE_RATE>() = 2.0F;
  state.at<XY_ACCELERATION>() = 2.0F;

  // Computed with SymPy.
  CatrMotionModel32::State::Matrix expected_jacobian{(CatrMotionModel32::State::Matrix{} <<
      1.0F, 0.0F, -0.1186522F, 0.0823960F, -0.0063150F, 0.0040257F,
      0.0F, 1.0F, 0.1728437F, 0.0563701F, 0.0085819F, 0.0029559277F,
      0.0F, 0.0F, 1.0F, 0.0F, 0.1F, 0.0F,
      0.0F, 0.0F, 0.0F, 1.0F, 0.0F, 0.1F,
      0.0F, 0.0F, 0.0F, 0.0F, 1.0F, 0.0F,
      0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 1.0F).finished()};


  CatrMotionModel32 model;
  const auto jacobian = model.jacobian(state, std::chrono::milliseconds{100LL});
  EXPECT_TRUE(expected_jacobian.isApprox(jacobian, kEpsilon)) <<
    "Jacobians don't match: \nExpected:\n" << expected_jacobian << "\nActual:\n" << jacobian;
}

/// @test Check that the Jacobian matches one computed symbolically when turn rate is zero.
TEST(CvtrMotionModelTest, TestJacobianZeroTurnRate) {
  CvtrMotionModel32::State state{CvtrMotionModel32::State{}};
  state.at<X>() = 42.0F;
  state.at<Y>() = 23.0F;
  state.at<YAW>() = 0.5F;
  state.at<XY_VELOCITY>() = 2.0F;
  state.at<YAW_CHANGE_RATE>() = 0.0F;

  // Computed with SymPy.
  CvtrMotionModel32::State::Matrix expected_jacobian{(CvtrMotionModel32::State::Matrix{} <<
      1.0F, 0.0F, -0.0958851077208406F, 0.0877582561890373F, -0.00958851077208406F,
      0.0F, 1.0F, 0.175516512378075F, 0.0479425538604203F, 0.0175516512378075F,
      0.0F, 0.0F, 1.0F, 0.0F, 0.0F,
      0.0F, 0.0F, 0.0F, 1.0F, 0.0F,
      0.0F, 0.0F, 0.0F, 0.0F, 1.0F).finished()};

  CvtrMotionModel32 model;
  const auto jacobian = model.jacobian(state, std::chrono::milliseconds{100LL});
  EXPECT_TRUE(expected_jacobian.isApprox(jacobian, kEpsilon)) <<
    "Jacobians don't match: \nExpected:\n" << expected_jacobian << "\nActual:\n" << jacobian;
}

/// @test Check that the Jacobian matches one computed symbolically when turn rate is zero.
TEST(CatrMotionModelTest, TestJacobianZeroTurnRate) {
  CatrMotionModel32::State state{CatrMotionModel32::State{}};
  state.at<X>() = 42.0F;
  state.at<Y>() = 23.0F;
  state.at<YAW>() = 0.5F;
  state.at<XY_VELOCITY>() = 2.0F;
  state.at<YAW_CHANGE_RATE>() = 0.0F;
  state.at<XY_ACCELERATION>() = 2.0F;

  // Computed with SymPy.
  CatrMotionModel32::State::Matrix expected_jacobian{(CatrMotionModel32::State::Matrix{} <<
      1.0F, 0.0F, -0.1006793F, 0.0877582F, -0.0100679F, 0.0043879F,
      0.0F, 1.0F, 0.1842923F, 0.0479425F, 0.0184292F, 0.0023971F,
      0.0F, 0.0F, 1.0F, 0.0F, 0.0F, 0.0F,
      0.0F, 0.0F, 0.0F, 1.0F, 0.0F, 0.1F,
      0.0F, 0.0F, 0.0F, 0.0F, 1.0F, 0.0F,
      0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 1.0F).finished()};

  CatrMotionModel32 model;
  const auto jacobian = model.jacobian(state, std::chrono::milliseconds{100LL});
  EXPECT_TRUE(expected_jacobian.isApprox(jacobian, kEpsilon)) <<
    "Jacobians don't match: \nExpected:\n" << expected_jacobian << "\nActual:\n" << jacobian;
}

/// @test Predict the linear movement with zero turn rate.
TEST(CvtrMotionModelTest, PredictLinearMovementWithZeroTurnRate) {
  CvtrMotionModel32 model;
  CvtrMotionModel32::State initial_state{};

  // Movement in X direction.
  initial_state = CvtrMotionModel32::State{};
  initial_state.at<XY_VELOCITY>() = 1.0F;
  CvtrMotionModel32::State expected_state{initial_state};
  expected_state.at<X>() += 1.0F;
  EXPECT_EQ(expected_state, model.predict(initial_state, std::chrono::seconds{1LL}));
  // Movement in negative X direction.
  initial_state = CvtrMotionModel32::State{};
  initial_state.at<XY_VELOCITY>() = -1.0F;
  expected_state = initial_state;
  expected_state.at<X>() -= 1.0F;
  EXPECT_EQ(expected_state, model.predict(initial_state, std::chrono::seconds{1LL}));

  // Movement in Y direction.
  initial_state = CvtrMotionModel32::State{};
  initial_state.at<XY_VELOCITY>() = 1.0F;
  initial_state.at<YAW>() = 0.5F * M_PIf32;
  expected_state = initial_state;
  expected_state.at<Y>() += 1.0F;
  EXPECT_EQ(expected_state, model.predict(initial_state, std::chrono::seconds{1LL}));
  // Movement in negative Y direction.
  initial_state = CvtrMotionModel32::State{};
  initial_state.at<XY_VELOCITY>() = 1.0F;
  initial_state.at<YAW>() = -0.5F * M_PIf32;
  expected_state = initial_state;
  expected_state.at<Y>() -= 1.0F;
  EXPECT_EQ(expected_state, model.predict(initial_state, std::chrono::seconds{1LL}));

  // Movement in XY direction.
  initial_state = CvtrMotionModel32::State{};
  initial_state.at<XY_VELOCITY>() = 1.0F;
  initial_state.at<YAW>() = 0.25F * M_PIf32;
  expected_state = initial_state;
  expected_state.at<X>() += 0.5F * sqrt(2.0F);
  expected_state.at<Y>() += 0.5F * sqrt(2.0F);
  EXPECT_EQ(expected_state, model.predict(initial_state, std::chrono::seconds{1LL}));
  // Movement in negative XY direction.
  initial_state = CvtrMotionModel32::State{};
  initial_state.at<XY_VELOCITY>() = 1.0F;
  initial_state.at<YAW>() = -0.75F * M_PIf32;
  expected_state = initial_state;
  expected_state.at<X>() -= 0.5F * sqrt(2.0F);
  expected_state.at<Y>() -= 0.5F * sqrt(2.0F);
  EXPECT_EQ(expected_state, model.predict(initial_state, std::chrono::seconds{1LL}));
}

/// @test Predict the linear movement with zero turn rate.
TEST(CatrMotionModelTest, PredictLinearMovementWithZeroTurnRate) {
  CatrMotionModel32 model;
  CatrMotionModel32::State initial_state{};

  // Movement in X direction.
  initial_state = CatrMotionModel32::State{};
  initial_state.at<XY_VELOCITY>() = 1.0F;
  initial_state.at<XY_ACCELERATION>() = 1.0F;
  const auto time_difference{std::chrono::seconds{1LL}};
  const auto dt{std::chrono::duration<float32_t>{time_difference}.count()};
  CatrMotionModel32::State expected_state{initial_state};
  expected_state.at<X>() +=
    dt * initial_state.at<XY_VELOCITY>() + 0.5F * dt * dt * initial_state.at<XY_ACCELERATION>();
  expected_state.at<XY_VELOCITY>() += dt * initial_state.at<XY_ACCELERATION>();
  EXPECT_EQ(expected_state, model.predict(initial_state, time_difference));
  // Movement in negative X direction.
  initial_state = CatrMotionModel32::State{};
  initial_state.at<XY_VELOCITY>() = -1.0F;
  initial_state.at<XY_ACCELERATION>() = -1.0F;
  expected_state = initial_state;
  expected_state.at<X>() +=
    dt * initial_state.at<XY_VELOCITY>() + 0.5F * dt * dt * initial_state.at<XY_ACCELERATION>();
  expected_state.at<XY_VELOCITY>() += dt * initial_state.at<XY_ACCELERATION>();
  EXPECT_EQ(expected_state, model.predict(initial_state, time_difference));

  // Movement in Y direction.
  initial_state = CatrMotionModel32::State{};
  initial_state.at<XY_VELOCITY>() = 1.0F;
  initial_state.at<XY_ACCELERATION>() = 1.0F;
  initial_state.at<YAW>() = 0.5F * M_PIf32;
  expected_state = initial_state;
  expected_state.at<Y>() +=
    dt * initial_state.at<XY_VELOCITY>() + 0.5F * dt * dt * initial_state.at<XY_ACCELERATION>();
  expected_state.at<XY_VELOCITY>() += dt * initial_state.at<XY_ACCELERATION>();
  EXPECT_EQ(expected_state, model.predict(initial_state, time_difference));
  // Movement in negative Y direction.
  initial_state = CatrMotionModel32::State{};
  initial_state.at<XY_VELOCITY>() = 1.0F;
  initial_state.at<XY_ACCELERATION>() = 1.0F;
  initial_state.at<YAW>() = -0.5F * M_PIf32;
  expected_state = initial_state;
  expected_state.at<Y>() -=
    dt * initial_state.at<XY_VELOCITY>() + 0.5F * dt * dt * initial_state.at<XY_ACCELERATION>();
  expected_state.at<XY_VELOCITY>() += dt * initial_state.at<XY_ACCELERATION>();
  EXPECT_EQ(expected_state, model.predict(initial_state, time_difference));

  // Movement in XY direction.
  initial_state = CatrMotionModel32::State{};
  initial_state.at<XY_VELOCITY>() = 1.0F;
  initial_state.at<XY_ACCELERATION>() = 1.0F;
  initial_state.at<YAW>() = 0.25F * M_PIf32;
  expected_state = initial_state;
  const auto distance =
    dt * initial_state.at<XY_VELOCITY>() + 0.5F * dt * dt * initial_state.at<XY_ACCELERATION>();
  expected_state.at<X>() += sqrt(0.5F * distance * distance);
  expected_state.at<Y>() += sqrt(0.5F * distance * distance);
  expected_state.at<XY_VELOCITY>() += dt * initial_state.at<XY_ACCELERATION>();
  EXPECT_EQ(expected_state, model.predict(initial_state, time_difference));
  // Movement in negative XY direction.
  initial_state = CatrMotionModel32::State{};
  initial_state.at<XY_VELOCITY>() = 1.0F;
  initial_state.at<XY_ACCELERATION>() = 1.0F;
  initial_state.at<YAW>() = -0.75F * M_PIf32;
  expected_state = initial_state;
  expected_state.at<X>() -= sqrt(0.5F * distance * distance);
  expected_state.at<Y>() -= sqrt(0.5F * distance * distance);
  expected_state.at<XY_VELOCITY>() += dt * initial_state.at<XY_ACCELERATION>();
  EXPECT_EQ(expected_state, model.predict(initial_state, time_difference));
}

/// @test Predict the linear movement with non-zero turn rate.
TEST(CvtrMotionModelTest, PredictLinearMovementWithNonzeroTurnRate) {
  CvtrMotionModel32 model;
  CvtrMotionModel32::State initial_state{};
  initial_state.at<X>() = 42.0F;
  initial_state.at<Y>() = 23.0F;
  initial_state.at<YAW>() = 0.5F;
  initial_state.at<XY_VELOCITY>() = 2.0F;
  initial_state.at<YAW_CHANGE_RATE>() = 2.0F;
  CvtrMotionModel32::State expected_state{initial_state};
  // Values computed from a symbolic math derivation.
  expected_state.at<X>() = 42.1647921486335F;
  expected_state.at<Y>() = 23.1127403746059F;
  expected_state.at<YAW>() = 0.7F;
  EXPECT_EQ(expected_state, model.predict(initial_state, std::chrono::milliseconds{100LL}));
}


/// @test Predict the linear movement with non-zero turn rate.
TEST(CatrMotionModelTest, PredictLinearMovementWithNonzeroTurnRate) {
  CatrMotionModel32 model;
  CatrMotionModel32::State initial_state{};
  initial_state.at<X>() = 42.0F;
  initial_state.at<Y>() = 23.0F;
  initial_state.at<YAW>() = 0.5F;
  initial_state.at<XY_VELOCITY>() = 2.0F;
  initial_state.at<YAW_CHANGE_RATE>() = 2.0F;
  initial_state.at<XY_ACCELERATION>() = 2.0F;
  CatrMotionModel32::State expected_state{initial_state};
  // Values computed from a symbolic math derivation.
  expected_state.at<X>() = 42.1728437300543F;
  expected_state.at<Y>() = 23.1186522301942F;
  expected_state.at<YAW>() = 0.7F;
  expected_state.at<XY_VELOCITY>() = 2.2F;
  EXPECT_EQ(expected_state, model.predict(initial_state, std::chrono::milliseconds{100LL}));
}
