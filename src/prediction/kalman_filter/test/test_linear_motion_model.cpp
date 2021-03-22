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
/// \brief This file defines tests for the linear motion model.

#include <kalman_filter/common_states.hpp>
#include <motion_model/linear_motion_model.hpp>

#include <gtest/gtest.h>

using autoware::prediction::state::ConstAccelerationXY;
using autoware::prediction::state::ConstAccelerationXYYaw;
using autoware::prediction::LinearMotionModel;
using autoware::prediction::variable::X;
using autoware::prediction::variable::Y;
using autoware::prediction::variable::YAW;
using autoware::prediction::variable::X_VELOCITY;
using autoware::prediction::variable::Y_VELOCITY;
using autoware::prediction::variable::YAW_CHANGE_RATE;
using autoware::prediction::variable::X_ACCELERATION;
using autoware::prediction::variable::Y_ACCELERATION;
using autoware::prediction::variable::YAW_CHANGE_ACCELERATION;


/// @test Test that prediction on independent x, y, yaw with constant acceleration works.
TEST(LinearMotionModel, PredictConstAccelerationXYYaw) {
  ConstAccelerationXYYaw state{ConstAccelerationXYYaw::Vector::Ones()};
  LinearMotionModel<ConstAccelerationXYYaw> motion_model{};
  state = motion_model.predict(state, std::chrono::milliseconds{100});
  EXPECT_FLOAT_EQ(1.105F, state.at<X>());
  EXPECT_FLOAT_EQ(1.1F, state.at<X_VELOCITY>());
  EXPECT_FLOAT_EQ(1.0F, state.at<X_ACCELERATION>());
  EXPECT_FLOAT_EQ(1.105F, state.at<Y>());
  EXPECT_FLOAT_EQ(1.1F, state.at<Y_VELOCITY>());
  EXPECT_FLOAT_EQ(1.0F, state.at<Y_ACCELERATION>());
  EXPECT_FLOAT_EQ(1.105F, state.at<YAW>());
  EXPECT_FLOAT_EQ(1.1F, state.at<YAW_CHANGE_RATE>());
  EXPECT_FLOAT_EQ(1.0F, state.at<YAW_CHANGE_ACCELERATION>());
}


/// @test Test that prediction on independent x, y with constant acceleration works.
TEST(LinearMotionModel, PredictConstAccelerationXY) {
  ConstAccelerationXY state{ConstAccelerationXY::Vector::Ones()};
  LinearMotionModel<ConstAccelerationXY> motion_model{};
  state = motion_model.predict(state, std::chrono::milliseconds{100});
  EXPECT_FLOAT_EQ(1.105F, state.at<X>());
  EXPECT_FLOAT_EQ(1.1F, state.at<X_VELOCITY>());
  EXPECT_FLOAT_EQ(1.0F, state.at<X_ACCELERATION>());
  EXPECT_FLOAT_EQ(1.105F, state.at<Y>());
  EXPECT_FLOAT_EQ(1.1F, state.at<Y_VELOCITY>());
  EXPECT_FLOAT_EQ(1.0F, state.at<Y_ACCELERATION>());
}
