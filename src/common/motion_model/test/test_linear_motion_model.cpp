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

#include <motion_model/linear_motion_model.hpp>
#include <state_vector/common_states.hpp>

#include <gtest/gtest.h>

using autoware::common::motion_model::LinearMotionModel;
using autoware::common::state_vector::ConstAccelerationXY32;
using autoware::common::state_vector::ConstAccelerationXYYaw32;
using autoware::common::state_vector::variable::X;
using autoware::common::state_vector::variable::Y;
using autoware::common::state_vector::variable::YAW;
using autoware::common::state_vector::variable::X_VELOCITY;
using autoware::common::state_vector::variable::Y_VELOCITY;
using autoware::common::state_vector::variable::YAW_CHANGE_RATE;
using autoware::common::state_vector::variable::X_ACCELERATION;
using autoware::common::state_vector::variable::Y_ACCELERATION;
using autoware::common::state_vector::variable::YAW_CHANGE_ACCELERATION;


/// @test Test that prediction on independent x, y, yaw with constant acceleration works.
TEST(LinearMotionModel, PredictConstAccelerationXYYaw32) {
  ConstAccelerationXYYaw32 state{ConstAccelerationXYYaw32::Vector::Ones()};
  LinearMotionModel<ConstAccelerationXYYaw32> motion_model{};
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
  ConstAccelerationXY32 state{ConstAccelerationXY32::Vector::Ones()};
  LinearMotionModel<ConstAccelerationXY32> motion_model{};
  state = motion_model.predict(state, std::chrono::milliseconds{100});
  EXPECT_FLOAT_EQ(1.105F, state.at<X>());
  EXPECT_FLOAT_EQ(1.1F, state.at<X_VELOCITY>());
  EXPECT_FLOAT_EQ(1.0F, state.at<X_ACCELERATION>());
  EXPECT_FLOAT_EQ(1.105F, state.at<Y>());
  EXPECT_FLOAT_EQ(1.1F, state.at<Y_VELOCITY>());
  EXPECT_FLOAT_EQ(1.0F, state.at<Y_ACCELERATION>());
}
