// Copyright 2018 Apex.AI, Inc.
// Co-developed by Tier IV, Inc. and Apex.AI, Inc.
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

#ifndef TEST_CONSTANT_VELOCITY_HPP_
#define TEST_CONSTANT_VELOCITY_HPP_

#include <common/types.hpp>
#include <motion_model/constant_velocity.hpp>

using autoware::motion::motion_model::ConstantVelocity;
using Eigen::Matrix;
using autoware::common::types::float32_t;

TEST(constant_velocity, basic)
{
  ConstantVelocity model;
  const uint64_t sz = 4U;
  Matrix<float32_t, sz, 1U> x, y;
  x(0) = 0.0F;
  x(1) = 0.0F;
  x(2) = 1.0F;
  x(3) = 2.0F;

  // prepare duration objects for testing
  std::chrono::nanoseconds seconds(std::chrono::seconds(1));
  std::chrono::nanoseconds milliseconds_50(std::chrono::milliseconds(50));
  std::chrono::nanoseconds milliseconds_100(std::chrono::milliseconds(100));
  std::chrono::nanoseconds milliseconds_500(std::chrono::milliseconds(500));
  // set state
  const float32_t vx = 1.0F;
  const float32_t vy = 2.0F;
  model.reset(x);
  ASSERT_FLOAT_EQ(model[ConstantVelocity::States::POSE_X], 0.0F);
  ASSERT_FLOAT_EQ(model[ConstantVelocity::States::POSE_Y], 0.0F);
  ASSERT_FLOAT_EQ(model[ConstantVelocity::States::VELOCITY_X], vx);
  ASSERT_FLOAT_EQ(model[ConstantVelocity::States::VELOCITY_Y], vy);
  // push prediction eslewhere
  model.predict(y, seconds);
  ASSERT_FLOAT_EQ(y(0), 1.0F);
  ASSERT_FLOAT_EQ(y(1), 2.0F);
  ASSERT_FLOAT_EQ(y(2), vx);
  ASSERT_FLOAT_EQ(y(3), vy);
  model.predict(y, milliseconds_500);
  ASSERT_FLOAT_EQ(y(0), 0.5F);
  ASSERT_FLOAT_EQ(y(1), 1.0F);
  ASSERT_FLOAT_EQ(y(2), vx);
  ASSERT_FLOAT_EQ(y(3), vy);
  // compute jacobian
  Matrix<float32_t, sz, sz> F;
  model.compute_jacobian(F, milliseconds_100);
  for (uint32_t idx = 0U; idx < sz; ++idx) {
    for (uint32_t jdx = 0U; jdx < sz; ++jdx) {
      if (idx == jdx) {
        ASSERT_FLOAT_EQ(F(idx, idx), 1.0F);
      } else if (idx == 0U && jdx == 2U) {
        ASSERT_FLOAT_EQ(F(idx, jdx), 0.1F);
      } else if (idx == 1U && jdx == 3U) {
        ASSERT_FLOAT_EQ(F(idx, jdx), 0.1F);
      } else {
        ASSERT_FLOAT_EQ(F(idx, jdx), 0.0F);
      }
    }
  }
  // y should not change
  model.predict(milliseconds_100);
  ASSERT_FLOAT_EQ(y(0), 0.5F);
  ASSERT_FLOAT_EQ(y(1), 1.0F);
  ASSERT_FLOAT_EQ(y(2), vx);
  ASSERT_FLOAT_EQ(y(3), vy);
  ASSERT_FLOAT_EQ(model[ConstantVelocity::States::POSE_X], 0.1F);
  ASSERT_FLOAT_EQ(model[ConstantVelocity::States::POSE_Y], 0.2F);
  ASSERT_FLOAT_EQ(model[ConstantVelocity::States::VELOCITY_X], vx);
  ASSERT_FLOAT_EQ(model[ConstantVelocity::States::VELOCITY_Y], vy);
  // combined change
  model.compute_jacobian_and_predict(F, milliseconds_50);
  for (uint32_t idx = 0U; idx < sz; ++idx) {
    for (uint32_t jdx = 0U; jdx < sz; ++jdx) {
      if (idx == jdx) {
        ASSERT_FLOAT_EQ(F(idx, idx), 1.0F);
      } else if (idx == 0U && jdx == 2U) {
        ASSERT_FLOAT_EQ(F(idx, jdx),  0.05F);
      } else if (idx == 1U && jdx == 3U) {
        ASSERT_FLOAT_EQ(F(idx, jdx), 0.05F);
      } else {
        ASSERT_FLOAT_EQ(F(idx, jdx), 0.0F);
      }
    }
  }
  ASSERT_FLOAT_EQ(model[ConstantVelocity::States::POSE_X], 0.15F);
  ASSERT_FLOAT_EQ(model[ConstantVelocity::States::POSE_Y], 0.3F);
  ASSERT_FLOAT_EQ(model[ConstantVelocity::States::VELOCITY_X], vx);
  ASSERT_FLOAT_EQ(model[ConstantVelocity::States::VELOCITY_Y], vy);
}

#endif  // TEST_CONSTANT_VELOCITY_HPP_
