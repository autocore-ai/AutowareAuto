// Copyright 2019 Christopher Ho
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
#include <apex_test_tools/apex_test_tools.hpp>

#include <controller_common/controller_base.hpp>
#include <motion_common/motion_common.hpp>
#include <motion_testing/motion_testing.hpp>
#include <time_utils/time_utils.hpp>

#include <chrono>

using motion::control::controller_common::BehaviorConfig;
using motion::control::controller_common::ControlReference;
using motion::control::controller_common::ControllerBase;
using motion::control::controller_common::Command;
using motion::control::controller_common::State;
using motion::control::controller_common::Trajectory;
using motion::motion_common::Point;
using motion::motion_common::to_angle;
using motion::motion_testing::make_state;
using motion::motion_testing::constant_velocity_trajectory;

using time_utils::from_message;

using std::chrono::milliseconds;

class MiscTestController : public ControllerBase
{
public:
  MiscTestController()
  : ControllerBase{BehaviorConfig{3.0F, milliseconds(100LL), ControlReference::SPATIAL}}
  {
  }
  using ControllerBase::predict;

protected:
  Command compute_command_impl(const State & state) override
  {
    (void)state;
    return Command{};
  }
};

class MotionModel : public ::testing::Test
{
protected:
  MiscTestController controller_{};
};

// These are generic and/or not represented by the CATR model
void generic_checks(const Point & s, const Point & p, std::chrono::nanoseconds dt, float TOL)
{
  EXPECT_LT(std::fabs(s.lateral_velocity_mps - p.lateral_velocity_mps), TOL);
  EXPECT_LT(std::fabs(s.front_wheel_angle_rad - p.front_wheel_angle_rad), TOL);
  EXPECT_LT(std::fabs(s.rear_wheel_angle_rad - p.rear_wheel_angle_rad), TOL);
  EXPECT_TRUE((dt < milliseconds(1)) && (dt > milliseconds(-1))) <<
    std::chrono::duration_cast<milliseconds>(dt).count();
}

TEST_F(MotionModel, ConstantVelocity)
{
  constexpr auto TOL = 1.0E-3F;
  const auto s =
    make_state(5.0F, 10.0F, -1.0F, 10.0F, 0.0F, 0.0F, std::chrono::system_clock::now()).state;
  apex_test_tools::memory_test::start_paused();
  // Just have one of these in a compilation unit to make sure everything is hunky-dory
#ifndef __aarch64__
  ASSERT_TRUE(osrf_testing_tools_cpp::memory_tools::is_working());
#endif
  apex_test_tools::memory_test::resume();
  // zero
  {
    const auto p = controller_.predict(s, milliseconds(0));
    EXPECT_LT(std::fabs(s.x - p.x), TOL);
    EXPECT_LT(std::fabs(s.y - p.y), TOL);
    EXPECT_LT(std::fabs(to_angle(s.heading) - to_angle(p.heading)), TOL);
    EXPECT_LT(std::fabs(s.longitudinal_velocity_mps - p.longitudinal_velocity_mps), TOL);
    EXPECT_LT(std::fabs(s.acceleration_mps2 - p.acceleration_mps2), TOL);
    EXPECT_LT(std::fabs(s.heading_rate_rps - p.heading_rate_rps), TOL);
    const auto dt = from_message(p.time_from_start) - from_message(s.time_from_start);
    generic_checks(p, s, dt, TOL);
  }
  // positive time step
  {
    const auto p = controller_.predict(s, milliseconds(10));
    EXPECT_GT(p.x, s.x);
    EXPECT_LT(p.y, s.y);
    EXPECT_LT(std::fabs(to_angle(s.heading) - to_angle(p.heading)), TOL);
    EXPECT_LT(std::fabs(s.longitudinal_velocity_mps - p.longitudinal_velocity_mps), TOL);
    EXPECT_LT(std::fabs(s.acceleration_mps2 - p.acceleration_mps2), TOL);
    EXPECT_LT(std::fabs(s.heading_rate_rps - p.heading_rate_rps), TOL);
    const auto dt =
      (from_message(p.time_from_start) - from_message(s.time_from_start)) - milliseconds(10);
    generic_checks(p, s, dt, TOL);
  }
  // negative time step
  {
    const auto p = controller_.predict(s, milliseconds(-10));
    EXPECT_LT(p.x, s.x);
    EXPECT_GT(p.y, s.y);
    EXPECT_LT(std::fabs(to_angle(s.heading) - to_angle(p.heading)), TOL);
    EXPECT_LT(std::fabs(s.longitudinal_velocity_mps - p.longitudinal_velocity_mps), TOL);
    EXPECT_LT(std::fabs(s.acceleration_mps2 - p.acceleration_mps2), TOL);
    EXPECT_LT(std::fabs(s.heading_rate_rps - p.heading_rate_rps), TOL);
    const auto dt =
      (from_message(p.time_from_start) - from_message(s.time_from_start)) - milliseconds(-10);
    generic_checks(p, s, dt, TOL);
  }
  apex_test_tools::memory_test::stop();
}

TEST_F(MotionModel, ConstantAcceleration)
{
  constexpr auto TOL = 1.0E-3F;
  const auto s =
    make_state(-5.0F, -10.0F, 1.0F, 1.0F, -1.0F, 0.0F, std::chrono::system_clock::now()).state;
  apex_test_tools::memory_test::start();
  // positive
  {
    const auto p = controller_.predict(s, milliseconds(100));
    EXPECT_GT(p.x, s.x);
    EXPECT_GT(p.y, s.y);
    EXPECT_LT(std::fabs(to_angle(s.heading) - to_angle(p.heading)), TOL);
    EXPECT_LT(p.longitudinal_velocity_mps, s.longitudinal_velocity_mps);
    EXPECT_LT(std::fabs(s.acceleration_mps2 - p.acceleration_mps2), TOL);
    EXPECT_LT(std::fabs(s.heading_rate_rps - p.heading_rate_rps), TOL);
    const auto dt =
      (from_message(p.time_from_start) - from_message(s.time_from_start)) - milliseconds(100);
    generic_checks(p, s, dt, TOL);
  }
  // more positive, should be backwards
  {
    const auto p = controller_.predict(s, milliseconds(1100));
    EXPECT_GT(p.x, s.x);
    EXPECT_GT(p.y, s.y);
    EXPECT_LT(std::fabs(to_angle(s.heading) - to_angle(p.heading)), TOL);
    EXPECT_LT(p.longitudinal_velocity_mps, 0.0F);
    EXPECT_LT(std::fabs(s.acceleration_mps2 - p.acceleration_mps2), TOL);
    EXPECT_LT(std::fabs(s.heading_rate_rps - p.heading_rate_rps), TOL);
    const auto dt =
      (from_message(p.time_from_start) - from_message(s.time_from_start)) - milliseconds(1100);
    generic_checks(p, s, dt, TOL);
  }
  // negative
  {
    const auto p = controller_.predict(s, milliseconds(-100));
    EXPECT_LT(p.x, s.x);
    EXPECT_LT(p.y, s.y);
    EXPECT_LT(std::fabs(to_angle(s.heading) - to_angle(p.heading)), TOL);
    EXPECT_GT(p.longitudinal_velocity_mps, s.longitudinal_velocity_mps);
    EXPECT_LT(std::fabs(s.acceleration_mps2 - p.acceleration_mps2), TOL);
    EXPECT_LT(std::fabs(s.heading_rate_rps - p.heading_rate_rps), TOL);
    const auto dt =
      (from_message(p.time_from_start) - from_message(s.time_from_start)) - milliseconds(-100);
    generic_checks(p, s, dt, TOL);
  }
  apex_test_tools::memory_test::stop();
}

TEST_F(MotionModel, ConstantTurnRate)
{
  constexpr auto TOL = 1.0E-3F;
  const auto s =
    make_state(0.0F, 0.0F, 0.0F, 1.0F, 0.0F, 0.1F, std::chrono::system_clock::now()).state;
  apex_test_tools::memory_test::start();
  // positive
  {
    const auto p = controller_.predict(s, milliseconds(100));
    EXPECT_GT(p.x, s.x);
    EXPECT_GT(p.y, s.y);
    EXPECT_LT(to_angle(s.heading), to_angle(p.heading));
    EXPECT_LT(std::fabs(p.longitudinal_velocity_mps - s.longitudinal_velocity_mps), TOL);
    EXPECT_LT(std::fabs(s.acceleration_mps2 - p.acceleration_mps2), TOL);
    EXPECT_LT(std::fabs(s.heading_rate_rps - p.heading_rate_rps), TOL);
    const auto dt =
      (from_message(p.time_from_start) - from_message(s.time_from_start)) - milliseconds(100);
    generic_checks(p, s, dt, TOL);
  }
  // negative
  {
    const auto p = controller_.predict(s, milliseconds(-100));
    EXPECT_LT(p.x, s.x);
    EXPECT_GT(p.y, s.y);
    EXPECT_GT(to_angle(s.heading), to_angle(p.heading));
    EXPECT_LT(std::fabs(p.longitudinal_velocity_mps - s.longitudinal_velocity_mps), TOL);
    EXPECT_LT(std::fabs(s.acceleration_mps2 - p.acceleration_mps2), TOL);
    EXPECT_LT(std::fabs(s.heading_rate_rps - p.heading_rate_rps), TOL);
    const auto dt =
      (from_message(p.time_from_start) - from_message(s.time_from_start)) - milliseconds(-100);
    generic_checks(p, s, dt, TOL);
  }
  apex_test_tools::memory_test::stop();
}
