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

#include <state_estimation/noise_model/wiener_noise.hpp>
#include <state_vector/common_states.hpp>

#include <gtest/gtest.h>

using autoware::common::state_vector::ConstAccelerationXYYaw32;
using autoware::common::state_estimation::WienerNoise;

namespace
{
float squared(float num)
{
  return num * num;
}
}  // namespace

/// @test Test that the noise model is correctly created.
TEST(WienerNoiseModel, Create) {
  const auto sigma_x = 1.0F;
  const auto sigma_y = 2.0F;
  const auto sigma_yaw = 3.0F;
  WienerNoise<ConstAccelerationXYYaw32> noise{{sigma_x, sigma_y, sigma_yaw}};
  const auto dt = std::chrono::milliseconds{100LL};
  const auto covariance = noise.covariance(dt);
  const auto t = std::chrono::duration<float>{dt}.count();

  const auto position_noise_gain = 0.5F * t * t;
  const auto velocity_noise_gain = t;
  const auto acceleration_noise_gain = 1.0F;
  EXPECT_FLOAT_EQ(covariance(0, 0), squared(position_noise_gain) * squared(sigma_x));
  EXPECT_FLOAT_EQ(covariance(1, 1), squared(velocity_noise_gain) * squared(sigma_x));
  EXPECT_FLOAT_EQ(covariance(2, 2), squared(acceleration_noise_gain) * squared(sigma_x));
  EXPECT_FLOAT_EQ(covariance(3, 3), squared(position_noise_gain) * squared(sigma_y));
  EXPECT_FLOAT_EQ(covariance(4, 4), squared(velocity_noise_gain) * squared(sigma_y));
  EXPECT_FLOAT_EQ(covariance(5, 5), squared(acceleration_noise_gain) * squared(sigma_y));
  EXPECT_FLOAT_EQ(covariance(6, 6), squared(position_noise_gain) * squared(sigma_yaw));
  EXPECT_FLOAT_EQ(covariance(7, 7), squared(velocity_noise_gain) * squared(sigma_yaw));
  EXPECT_FLOAT_EQ(covariance(8, 8), squared(acceleration_noise_gain) * squared(sigma_yaw));
}
