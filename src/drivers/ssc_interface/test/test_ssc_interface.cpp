// Copyright 2020 The Autoware Foundation
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

#include <common/types.hpp>

#include <autoware_auto_msgs/msg/vehicle_kinematic_state.hpp>

#include <cmath>
#include <memory>
#include <iostream>
#include <fstream>

#include "gtest/gtest.h"
#include "ssc_interface/ssc_interface.hpp"

using autoware_auto_msgs::msg::VehicleKinematicState;

using autoware::common::types::float32_t;

static constexpr float32_t kFrontAxleToCogM = 1.5f;
static constexpr float32_t kRearAxleToCogM = 0.5f;

static bool is_close(const float32_t a, const float32_t b, const float32_t tol = 1e-5F)
{
  return std::abs(a - b) < tol;
}

TEST(TestSscInterface, TestDriveStraight) {
  VehicleKinematicState vks;
  vks.state.longitudinal_velocity_mps = 2.0;
  constexpr float32_t kYaw = 1.0471975511965976F;  // 60 deg
  vks.state.heading.real = 0.86602540378F;  // cos(30 deg)
  vks.state.heading.imag = 0.5F;  // sin(30 deg)
  constexpr float32_t dt = 0.1F;

  for (float32_t i = 0; i < 50.0F; ++i) {
    const float32_t x_nominal = std::cos(kYaw) * i * dt * 2.0;
    EXPECT_TRUE(is_close(x_nominal, vks.state.x)) <<
      "should be " << x_nominal << ", is " << vks.state.x;
    const float32_t y_nominal = std::sin(kYaw) * i * dt * 2.0;
    EXPECT_TRUE(is_close(y_nominal, vks.state.y)) <<
      "should be " << y_nominal << ", is " << vks.state.y;
    ssc_interface::SscInterface::kinematic_bicycle_model(
      dt, kRearAxleToCogM, kFrontAxleToCogM,
      &vks);
  }
}

TEST(TestSscInterface, TestConstantSteering) {
  constexpr float32_t kFrontWheelAngleRad = 0.4f;
  constexpr float32_t kFrontAxleToCogM = 1.5f;
  constexpr float32_t kRearAxleToCogM = 0.5f;
  constexpr float32_t kWheelbaseM = kFrontAxleToCogM + kRearAxleToCogM;
  constexpr float32_t kScalingToPOI = kRearAxleToCogM / kWheelbaseM;
  constexpr float32_t kPi = 3.14159265359f;
  constexpr float32_t kLongitudinalVelocityMps = kPi;
  constexpr int32_t kNumSteps = 1000;
  VehicleKinematicState vks;
  // This is sort of duplicated, but I don't want to instantiate a node for this test.
  vks.state.front_wheel_angle_rad = kFrontWheelAngleRad;
  vks.state.longitudinal_velocity_mps = kLongitudinalVelocityMps;
  vks.state.lateral_velocity_mps = kScalingToPOI *
    kLongitudinalVelocityMps * std::tan(kFrontWheelAngleRad);

  // We should be doing a circle of a certain radius
  // But what is the radius? Well, sin(β)*r = kRearAxleToCogM.
  // β is the angle of the velocity vector relative to the car's main axis
  const float32_t beta_rad = std::atan2(
    vks.state.lateral_velocity_mps,
    vks.state.longitudinal_velocity_mps);
  const float32_t radius_m = kRearAxleToCogM / std::sin(beta_rad);
  const double radius_m_squared = radius_m * radius_m;
  const float32_t dt = 2.0 * radius_m / kNumSteps;

  // Make the circle go around (0, 0) – we're headed right at the start, and
  // turning left (positive angle).
  // So the starting position needs to be below the origin.
  vks.state.x = 0;
  vks.state.y = -radius_m;
  // Make sure the velocity vector is horizontal at the beginning.
  vks.state.heading.real = std::cos(-beta_rad / 2.0f);
  vks.state.heading.imag = std::sin(-beta_rad / 2.0f);
  for (int i = 0; i < kNumSteps; ++i) {
    ssc_interface::SscInterface::kinematic_bicycle_model(
      dt, kRearAxleToCogM, kFrontAxleToCogM,
      &vks);
    // Check that we're driving a circle. It would be nicer to explicitly
    // calculate a nominal x and y position for each step and compare.
    const float32_t dist_squared = vks.state.x * vks.state.x + vks.state.y * vks.state.y;
    EXPECT_TRUE(is_close(dist_squared, radius_m_squared, 1e-2)) <<
      "should be " << dist_squared << ", is " << radius_m_squared;
  }
}
