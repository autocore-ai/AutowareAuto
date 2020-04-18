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

#ifndef TEST_CONSTANT_ACCELERATION_HPP_
#define TEST_CONSTANT_ACCELERATION_HPP_

#include <common/types.hpp>
#include <motion_model/constant_acceleration.hpp>

using autoware::common::types::float32_t;

using autoware::motion::motion_model::ConstantAcceleration;
using Eigen::Matrix;

TEST(constant_acceleration, basic)
{
  ConstantAcceleration model;
  const uint64_t sz = 6U;
  // set state
  const float32_t vx = -1.0F;
  const float32_t vy = 2.0F;
  const float32_t ax = 2.0F;
  const float32_t ay = -1.0F;
  Matrix<float32_t, sz, 1U> x, y;
  x << 0.0F, 0.0F, vx, vy, ax, ay;
  // prepare duration objects for testing
  std::chrono::nanoseconds seconds(std::chrono::seconds(1));
  std::chrono::nanoseconds milliseconds_50(std::chrono::milliseconds(50));
  std::chrono::nanoseconds milliseconds_100(std::chrono::milliseconds(100));
  std::chrono::nanoseconds milliseconds_500(std::chrono::milliseconds(500));

  model.reset(x);
  ASSERT_FLOAT_EQ(model[ConstantAcceleration::States::POSE_X], 0.0F);
  ASSERT_FLOAT_EQ(model[ConstantAcceleration::States::POSE_Y], 0.0F);
  ASSERT_FLOAT_EQ(model[ConstantAcceleration::States::VELOCITY_X], vx);
  ASSERT_FLOAT_EQ(model[ConstantAcceleration::States::VELOCITY_Y], vy);
  ASSERT_FLOAT_EQ(model[ConstantAcceleration::States::ACCELERATION_X], ax);
  ASSERT_FLOAT_EQ(model[ConstantAcceleration::States::ACCELERATION_Y], ay);
  // push prediction eslewhere

  model.predict(y, seconds);
  const float32_t x1 = vx + (0.5F * 1.0F * 1.0F * ax);
  const float32_t y1 = vy + (0.5F * 1.0F * 1.0F * ay);
  const float32_t vx1 = vx + ax;
  const float32_t vy1 = vy + ay;
  ASSERT_FLOAT_EQ(y(0), x1);
  ASSERT_FLOAT_EQ(y(1), y1);
  ASSERT_FLOAT_EQ(y(2), vx1);
  ASSERT_FLOAT_EQ(y(3), vy1);
  ASSERT_FLOAT_EQ(y(4), ax);
  ASSERT_FLOAT_EQ(y(5), ay);
  model.predict(y, milliseconds_500);
  const float32_t x2 = 0.5F * vx + (0.5F * 0.5F * 0.5F * ax);
  const float32_t y2 = 0.5F * vy + (0.5F * 0.5F * 0.5F * ay);
  const float32_t vx2 = vx + 0.5F * ax;
  const float32_t vy2 = vy + 0.5F * ay;
  ASSERT_FLOAT_EQ(y(0), x2);
  ASSERT_FLOAT_EQ(y(1), y2);
  ASSERT_FLOAT_EQ(y(2), vx2);
  ASSERT_FLOAT_EQ(y(3), vy2);
  ASSERT_FLOAT_EQ(y(4), ax);
  ASSERT_FLOAT_EQ(y(5), ay);
  // compute jacobian
  Matrix<float32_t, sz, sz> F;
  model.compute_jacobian(F, milliseconds_100);
  for (uint32_t idx = 0U; idx < sz; ++idx) {
    for (uint32_t jdx = 0U; jdx < sz; ++jdx) {
      // diagonal terms
      if (idx == jdx) {
        ASSERT_FLOAT_EQ(F(idx, idx), 1.0F);
      // first order terms
      } else if (ConstantAcceleration::States::POSE_X == idx &&
                 ConstantAcceleration::States::VELOCITY_X == jdx) {
        ASSERT_FLOAT_EQ(F(idx, jdx), 0.1F);
      } else if (ConstantAcceleration::States::POSE_Y == idx &&
                 ConstantAcceleration::States::VELOCITY_Y == jdx) {
        ASSERT_FLOAT_EQ(F(idx, jdx), 0.1F);
      } else if (ConstantAcceleration::States::VELOCITY_X == idx &&
                 ConstantAcceleration::States::ACCELERATION_X == jdx) {
        ASSERT_FLOAT_EQ(F(idx, jdx), 0.1F);
      } else if (ConstantAcceleration::States::VELOCITY_Y == idx &&
                 ConstantAcceleration::States::ACCELERATION_Y == jdx) {
        ASSERT_FLOAT_EQ(F(idx, jdx), 0.1F);
      // second order terms
      } else if (ConstantAcceleration::States::POSE_X == idx &&
                 ConstantAcceleration::States::ACCELERATION_X == jdx) {
        ASSERT_FLOAT_EQ(F(idx, jdx), 0.1F * 0.1F * 0.5F);
      } else if (ConstantAcceleration::States::POSE_Y == idx &&
                 ConstantAcceleration::States::ACCELERATION_Y == jdx) {
        ASSERT_FLOAT_EQ(F(idx, jdx), 0.1F * 0.1F * 0.5F);
      // other terms
      } else {
        ASSERT_FLOAT_EQ(F(idx, jdx), 0.0F);
      }
    }
  }
  // y should not change
  model.predict(milliseconds_100);
  ASSERT_FLOAT_EQ(y(0), x2);
  ASSERT_FLOAT_EQ(y(1), y2);
  ASSERT_FLOAT_EQ(y(2), vx2);
  ASSERT_FLOAT_EQ(y(3), vy2);
  ASSERT_FLOAT_EQ(y(4), ax);
  ASSERT_FLOAT_EQ(y(5), ay);
  const float32_t x3 = 0.1F * vx + (0.5F * 0.1F * 0.1F * ax);
  const float32_t y3 = 0.1F * vy + (0.5F * 0.1F * 0.1F * ay);
  const float32_t vx3 = vx + 0.1F * ax;
  const float32_t vy3 = vy + 0.1F * ay;
  ASSERT_FLOAT_EQ(model[ConstantAcceleration::States::POSE_X], x3);
  ASSERT_FLOAT_EQ(model[ConstantAcceleration::States::POSE_Y], y3);
  ASSERT_FLOAT_EQ(model[ConstantAcceleration::States::VELOCITY_X], vx3);
  ASSERT_FLOAT_EQ(model[ConstantAcceleration::States::VELOCITY_Y], vy3);
  ASSERT_FLOAT_EQ(model[ConstantAcceleration::States::ACCELERATION_X], ax);
  ASSERT_FLOAT_EQ(model[ConstantAcceleration::States::ACCELERATION_Y], ay);
  // combined change
  model.compute_jacobian_and_predict(F, milliseconds_50);
  for (uint32_t idx = 0U; idx < sz; ++idx) {
    for (uint32_t jdx = 0U; jdx < sz; ++jdx) {
      // diagonal terms
      if (idx == jdx) {
        ASSERT_FLOAT_EQ(F(idx, idx), 1.0F);
      // first order terms
      } else if (ConstantAcceleration::States::POSE_X == idx &&
                 ConstantAcceleration::States::VELOCITY_X == jdx) {
        ASSERT_FLOAT_EQ(F(idx, jdx), 0.05F);
      } else if (ConstantAcceleration::States::POSE_Y == idx &&
                 ConstantAcceleration::States::VELOCITY_Y == jdx) {
        ASSERT_FLOAT_EQ(F(idx, jdx), 0.05F);
      } else if (ConstantAcceleration::States::VELOCITY_X == idx &&
                 ConstantAcceleration::States::ACCELERATION_X == jdx) {
        ASSERT_FLOAT_EQ(F(idx, jdx), 0.05F);
      } else if (ConstantAcceleration::States::VELOCITY_Y == idx &&
                 ConstantAcceleration::States::ACCELERATION_Y == jdx) {
        ASSERT_FLOAT_EQ(F(idx, jdx), 0.05F);
      // second order terms
      } else if (ConstantAcceleration::States::POSE_X == idx &&
                 ConstantAcceleration::States::ACCELERATION_X == jdx) {
        ASSERT_FLOAT_EQ(F(idx, jdx), 0.05F * 0.05F * 0.5F);
      } else if (ConstantAcceleration::States::POSE_Y == idx &&
                 ConstantAcceleration::States::ACCELERATION_Y == jdx) {
        ASSERT_FLOAT_EQ(F(idx, jdx), 0.05F * 0.05F * 0.5F);
      // other terms
      } else {
        ASSERT_FLOAT_EQ(F(idx, jdx), 0.0F);
      }
    }
  }
  const float32_t x4 = x3 + 0.05F * vx3 + (0.5F * 0.05F * 0.05F * ax);
  const float32_t y4 = y3 + 0.05F * vy3 + (0.5F * 0.05F * 0.05F * ay);
  const float32_t vx4 = vx3 + 0.05F * ax;
  const float32_t vy4 = vy3 + 0.05F * ay;
  ASSERT_FLOAT_EQ(model[ConstantAcceleration::States::POSE_X], x4);
  ASSERT_FLOAT_EQ(model[ConstantAcceleration::States::POSE_Y], y4);
  ASSERT_FLOAT_EQ(model[ConstantAcceleration::States::VELOCITY_X], vx4);
  ASSERT_FLOAT_EQ(model[ConstantAcceleration::States::VELOCITY_Y], vy4);
  ASSERT_FLOAT_EQ(model[ConstantAcceleration::States::ACCELERATION_X], ax);
  ASSERT_FLOAT_EQ(model[ConstantAcceleration::States::ACCELERATION_Y], ay);
}
#endif  // TEST_CONSTANT_ACCELERATION_HPP_
