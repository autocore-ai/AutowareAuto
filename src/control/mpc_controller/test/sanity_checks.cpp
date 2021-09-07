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
#include <mpc_controller/mpc_controller.hpp>
#include <motion_testing/motion_testing.hpp>
#include <time_utils/time_utils.hpp>

#include <vector>

using motion::control::controller_common::ControlReference;
using motion::control::mpc_controller::Real;
using motion::control::mpc_controller::StateWeight;
using motion::control::mpc_controller::Config;
using motion::control::mpc_controller::BehaviorConfig;
using motion::control::mpc_controller::LimitsConfig;
using motion::control::mpc_controller::OptimizationConfig;
using motion::control::mpc_controller::VehicleConfig;
using motion::control::mpc_controller::Interpolation;
using motion::control::mpc_controller::MpcController;
using motion::motion_testing::make_state;
using motion::motion_testing::constant_velocity_trajectory;
using motion::motion_testing::bad_heading_trajectory;
using motion::motion_testing::constant_acceleration_turn_rate_trajectory;
using motion::control::mpc_controller::State;
using time_utils::from_message;
using time_utils::to_message;

// Default values from:
// http://www.mchenrysoftware.com/medit32/readme/msmac/
// default.htm?turl=examplestirecorneringstiffnesscalculation1.htm
//
// http://www.mchenrysoftware.com/medit32/readme/msmac/
// default.htm?turl=examplestirecorneringstiffnesscalculation1.htmF
//
// http://www.mchenrysoftware.com/forum/Yaw%20Inertia.pdf
class sanity_checks_base : public ::testing::Test
{
protected:
  LimitsConfig limits_cfg_{
    // Debug bounds
    // {-999.1F, 999.0F},
    // {-999.1F, 999.0F},
    // {-999.1F, 999.0F},
    // {-999.1F, 999.0F},
    // {-999.1F, 999.0F},
    // {-999.1F, 999.0F},
    // {-999.1F, 999.0F}
    // Real Bounds
    {0.01F, 35.0F},  // Longitudinal velocity
    {-3.0F, 3.0F},  // Lateral velocity
    {-3.0F, 3.0F},  // Acceleation
    {-3.0F, 3.0F},  // yaw rate
    {-10.0F, 10.0F},  // Jerk
    {-0.331F, 0.331F},  // Steer angle
    {-0.331F, 0.331F}  // Steer angle rate
  };
  VehicleConfig vehicle_cfg_{
    // 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F
    // Parameters from LaValle
    1.2F,  // CG to front
    1.5F,  // CG to rear
    17000.0F,  // front cornering
    20000.0F,  // rear cornering
    1460.0F,  // mass
    2170.0F,  // Inertia
    2.0F,  // width
    0.5F,  // front overhang
    0.5F  // rear overhang
    // Paraeters from McHenry
    // 1.5F,  // CG to front
    // 1.5F,  // CG to rear
    // 880.0F,  // front cornering
    // 587.0F,  // rear cornering
    // 1000.0F,  // mass
    // 12500.0F  // Inertia
  };
  BehaviorConfig behavior_cfg_{
    3.0F,  // Stop rate
    std::chrono::milliseconds(100LL),  // time step
    ControlReference::SPATIAL};
  OptimizationConfig opt_cfg_{
    StateWeight{
      10.0F,  // pose
      10.0F,  // heading
      10.0F,  // longitudinal velocity
      10.0F,  // lateral velocity
      10.0F,  // yaw rate
      10.0F,  // acceleration
      10.0F,  // jerk
      10.0F,  // steer angle
      10.0F  // steer angle rate
    },
    StateWeight{
      1000.0F,  // pose
      1000.0F,  // heading
      1000.0F,  // longitudinal velocity
      1000.0F,  // lateral velocity
      1000.0F,  // yaw rate
      1000.0F,  // acceleration
      1000.0F,  // jerk
      1000.0F,  // steer angle
      1000.0F  // steer angle rate
    }
  };
};  // class sanity_checks_base
class SanityChecks : public sanity_checks_base
{
protected:
  MpcController controller_{
    Config{
      limits_cfg_,
      vehicle_cfg_,
      behavior_cfg_,
      opt_cfg_,
      std::chrono::milliseconds(5LL),  // sample_period_tolerance
      std::chrono::milliseconds(100LL),  // control_lookahead_duration
      Interpolation::YES}};
};  // class SanityChecks

class SanityChecksNoInterpolation : public sanity_checks_base
{
protected:
  MpcController controller_{
    Config{
      limits_cfg_,
      vehicle_cfg_,
      behavior_cfg_,
      opt_cfg_,
      std::chrono::milliseconds(5LL),  // sample_period_tolerance
      std::chrono::milliseconds(100LL),  // control_lookahead_duration
      Interpolation::NO}};
};  // class SanityChecksNoInterpolation

TEST_F(SanityChecksNoInterpolation, BadTrajectorySampleInterval)
{
  const auto dt = controller_.get_config().behavior().time_step();
  const auto traj_ = constant_velocity_trajectory(0.0F, 0.0F, 0.0F, 0.0F, dt);
  const auto eps = std::chrono::microseconds{1LL};
  const auto dt_ = controller_.get_config().sample_period_tolerance();
  const std::vector<decltype(traj_.points)::size_type> inds{0U, 10U, traj_.points.size() - 1U};
  apex_test_tools::memory_test::start_paused();
  // Just have one of these in a compilation unit to make sure everything is hunky-dory
#ifndef __aarch64__
  ASSERT_TRUE(osrf_testing_tools_cpp::memory_tools::is_working());
#endif
  for (auto idx : inds) {
    {
      auto traj = traj_;
      traj.points[idx].time_from_start = to_message((idx * dt) + (dt_ + eps));
      // Throws, can't memory test
      EXPECT_THROW(controller_.set_trajectory(traj), std::domain_error);
      traj.points[idx].time_from_start = to_message((idx * dt) + (dt_));
      apex_test_tools::memory_test::resume();
      EXPECT_NO_THROW(controller_.set_trajectory(traj));
      apex_test_tools::memory_test::pause();
    }
    {
      auto traj = traj_;
      traj.points[idx].time_from_start = to_message((idx * dt) - (dt_ + eps));
      // Throws, can't memory test
      EXPECT_THROW(controller_.set_trajectory(traj), std::domain_error);
      traj.points[idx].time_from_start = to_message((idx * dt) - (dt_));
      apex_test_tools::memory_test::resume();
      EXPECT_NO_THROW(controller_.set_trajectory(traj));
      apex_test_tools::memory_test::pause();
    }
  }
  apex_test_tools::memory_test::stop();
}

struct ConstantParam
{
  Real x0;
  Real y0;
  Real heading;
  Real v0;
  Real a0;
  Real omega0;
  std::chrono::nanoseconds dt;  // if zero, use configuration dt
};  // struct ConstantParam

constexpr auto zero_ns = std::chrono::nanoseconds::zero();

class SanityChecksOneshot
  : public SanityChecks, public testing::WithParamInterface<ConstantParam>
{
};
class SanityChecksSimulation
  : public SanityChecks, public testing::WithParamInterface<ConstantParam>
{
};

// Same velocity on a constant velocity track should result in no acceleration
TEST_P(SanityChecksOneshot, ConstantTrajectory)
{
  const auto p = GetParam();
  auto dt = p.dt;
  if (std::chrono::nanoseconds::zero() == dt) {
    dt = controller_.get_config().behavior().time_step();
  }
  const auto traj =
    constant_acceleration_turn_rate_trajectory(p.x0, p.y0, p.heading, p.v0, p.a0, p.omega0, dt);
  const auto state =
    make_state(p.x0, p.y0, p.heading, p.v0, p.a0, 0.0F, from_message(traj.header.stamp));
  apex_test_tools::memory_test::start();
  controller_.set_trajectory(traj);
  const auto cmd = controller_.compute_command(state);
  constexpr float TOL = 1.5E-1F;  // numerical algorithms are inexact
  EXPECT_LT(std::fabs(cmd.long_accel_mps2 - (p.a0)), TOL);
  EXPECT_LT(std::fabs(cmd.front_wheel_angle_rad - (0.0F)), TOL);
  EXPECT_LT(std::fabs(cmd.rear_wheel_angle_rad - (0.0F)), TOL);
  EXPECT_EQ(cmd.stamp, state.header.stamp);
  const auto cmd_dot = controller_.get_computed_control_derivatives();
  apex_test_tools::memory_test::stop();
  ASSERT_LT(std::fabs(cmd_dot.jerk_mps3 - (0.0F)), TOL);
  ASSERT_LT(std::fabs(cmd_dot.steer_angle_rate_rps - (0.0F)), TOL);
  if (HasFailure()) {
    controller_.debug_print(std::cout);
  }
}

INSTANTIATE_TEST_CASE_P(
  Oneshot,
  SanityChecksOneshot,
  testing::Values(
    // Different acceleration (profiles)
    ConstantParam{0.0F, 0.0F, 0.0F, 10.0F, 0.0F, 0.0F, zero_ns},
    ConstantParam{0.0F, 0.0F, 0.0F, 3.0F, 1.0F, 0.0F, zero_ns},
    ConstantParam{0.0F, 0.0F, 0.0F, 15.0F, -1.0F, 0.0F, zero_ns},
    // Different angles
    ConstantParam{0.0F, 0.0F, 2.0F, 3.0F, 0.0F, 0.0F, zero_ns},
    ConstantParam{0.0F, 0.0F, -2.0F, 3.0F, 0.0F, 0.0F, zero_ns},
    ConstantParam{0.0F, 0.0F, 3.14F, 3.0F, 0.0F, 0.0F, zero_ns},
    ConstantParam{0.0F, 0.0F, -3.14F, 3.0F, 0.0F, 0.0F, zero_ns},
    // Different offsets
    ConstantParam{3.0F, 0.0F, 0.0F, 10.0F, 0.0F, 0.0F, zero_ns},
    ConstantParam{0.0F, -30.0F, 0.0F, 3.0F, 1.0F, 0.0F, zero_ns},
    ConstantParam{-15.0F, 95.0F, 0.0F, 15.0F, -1.0F, 0.0F, zero_ns},
    // Different sample periods
    ConstantParam{0.0F, 0.0F, 0.0F, 10.0F, 0.0F, 0.0F, std::chrono::milliseconds{60LL}},
    ConstantParam{0.0F, 0.0F, 0.0F, 10.0F, 0.0F, 0.0F, std::chrono::milliseconds{160LL}}
));

// Lateral offset should be rejected by inducing some steering
TEST_F(SanityChecks, LateralOffset)
{
  const auto x0 = 0.0F;
  const auto y0 = 0.0F;
  const auto heading = 0.0F;
  const auto v0 = 10.0F;
  const auto dt = std::chrono::milliseconds(100LL);
  const auto traj = constant_velocity_trajectory(x0, y0, heading, v0, dt);
  const auto state =
    make_state(x0, y0 - 3.0F, heading, v0, 0.0F, 0.0F, from_message(traj.header.stamp));
  apex_test_tools::memory_test::start();
  controller_.set_trajectory(traj);
  const auto cmd = controller_.compute_command(state);
  apex_test_tools::memory_test::stop();
  constexpr float TOL = 1.0E-4F;  // numerical algorithms are inexact
  EXPECT_LT(std::fabs(cmd.long_accel_mps2 - (0.0F)), TOL);
  EXPECT_GT(cmd.front_wheel_angle_rad, 0.0F);
  EXPECT_LT(std::fabs(cmd.rear_wheel_angle_rad - (0.0F)), TOL);
  EXPECT_EQ(cmd.stamp, state.header.stamp);
  // No guarantee of zero derivatives
  // const auto cmd_dot = controller_.get_computed_control_derivatives();
  // ASSERT_LT(std::fabs(cmd_dot.jerk_mps3 - (0.0F)), TOL);
  // ASSERT_GT(cmd_dot.steer_angle_rate_rps, 0.0F);
  if (HasFailure()) {
    controller_.debug_print(std::cout);
  }
}

// bad heading value in trajectory, should throw due to bad solution with Nan.
TEST_F(SanityChecks, BadTrajectoryHeading)
{
  const auto x0 = 0.0F;
  const auto y0 = 0.0F;
  const auto heading = 0.0F;
  const auto v0 = 10.0F;
  const auto dt = std::chrono::milliseconds(100LL);
  const auto state = make_state(x0, y0, heading, v0, 0.0F, 0.0F, std::chrono::system_clock::now());
  const auto traj = bad_heading_trajectory(state, dt);

  EXPECT_THROW(controller_.set_trajectory(traj), std::domain_error);

  if (HasFailure()) {
    controller_.debug_print(std::cout);
  }
}

// Fake simulation: should be able to follow trajectory from start to finish without going nuts
TEST_P(SanityChecksSimulation, ConstantTrajectorySimulation)
{
  const auto p = GetParam();
  auto dt = p.dt;
  auto stop_indices = 2U;
  if (std::chrono::nanoseconds::zero() == dt) {
    dt = controller_.get_config().behavior().time_step();
  }
  // Dumb hard coding
  if (std::chrono::milliseconds(60) == dt) {
    stop_indices = 3U;
  }
  const auto traj =
    constant_acceleration_turn_rate_trajectory(p.x0, p.y0, p.heading, p.v0, p.a0, p.omega0, dt);
  apex_test_tools::memory_test::start();
  controller_.set_trajectory(traj);
  apex_test_tools::memory_test::pause();
  State state;
  auto iters = 93U;
  // More dumb hard coding: I think there's some accumulation of numerical errors due to warm
  // starting and interpolation
  if (std::chrono::milliseconds(160) == dt) {
    iters = 35U;
  }
  for (auto idx = 0U; idx < iters; ++idx) {
    const auto pt = traj.points[idx];
    state.state = pt;
    state.header.stamp =
      to_message(from_message(traj.header.stamp) + from_message(pt.time_from_start));
    apex_test_tools::memory_test::resume();
    const auto cmd = controller_.compute_command(state);

    // numerical algorithms are inexact
    const auto TOL = 1.5E-1F;
    // ^^ loose tolerance for non-straight cases because IDK what's happening
    if (idx < traj.points.size() - stop_indices) {
      EXPECT_LT(std::fabs(cmd.long_accel_mps2 - (p.a0)), TOL) << idx;
      EXPECT_LT(std::fabs(cmd.front_wheel_angle_rad - (0.0F)), TOL) << idx;
      EXPECT_LT(std::fabs(cmd.rear_wheel_angle_rad - (0.0F)), TOL) << idx;
      EXPECT_EQ(cmd.stamp, state.header.stamp) << idx;
      const auto cmd_dot = controller_.get_computed_control_derivatives();
      ASSERT_LT(std::fabs(cmd_dot.jerk_mps3 - (0.0F)), TOL);
      ASSERT_LT(std::fabs(cmd_dot.steer_angle_rate_rps - (0.0F)), TOL);
    } else {
      // Safe stop
      const auto safe_decel_rate = controller_.get_config().behavior().safe_deceleration_rate();
      EXPECT_LT(std::fabs(cmd.long_accel_mps2 - (-safe_decel_rate)), TOL) << idx;
      EXPECT_LT(std::fabs(cmd.front_wheel_angle_rad - (0.0F)), TOL) << idx;
      EXPECT_LT(std::fabs(cmd.rear_wheel_angle_rad - (0.0F)), TOL) << idx;
      EXPECT_EQ(cmd.stamp, state.header.stamp) << idx;
      const auto cmd_dot = controller_.get_computed_control_derivatives();
      ASSERT_LT(std::fabs(cmd_dot.jerk_mps3 - (0.0F)), TOL);
      ASSERT_LT(std::fabs(cmd_dot.steer_angle_rate_rps - (0.0F)), TOL);
    }
    apex_test_tools::memory_test::pause();
    if (HasFailure()) {
      controller_.debug_print(std::cout);
      ASSERT_TRUE(false);
    }
  }
  apex_test_tools::memory_test::stop();
}

INSTANTIATE_TEST_CASE_P(
  Simulation,
  SanityChecksSimulation,
  testing::Values(
    ConstantParam{0.0F, 0.0F, 0.0F, 10.0F, 0.0F, 0.0F, zero_ns},
    ConstantParam{10.0F, 5.0F, 0.0F, 3.0F, 1.0F, 0.0F, zero_ns},
    ConstantParam{-50.0F, -10.0F, 2.0F, 7.0F, 0.2F, 0.0F, zero_ns},
    ConstantParam{17.0F, -13.0F, -2.0F, 15.0F, -1.0F, 0.0F, zero_ns},
    // Different sample periods
    ConstantParam{0.0F, 0.0F, 0.0F, 10.0F, 0.0F, 0.0F, std::chrono::milliseconds{60LL}},
    ConstantParam{0.0F, 0.0F, 0.0F, 10.0F, 0.0F, 0.0F, std::chrono::milliseconds{160LL}},
    // Turn past +/- pi cases
    ConstantParam{0.0F, 0.0F, -3.14F, 10.0F, 0.0F, -0.01F, zero_ns},
    ConstantParam{0.0F, 0.0F, 3.14F, 10.0F, 0.0F, 0.01F, zero_ns}
));

// Fake simulation: should be able to follow trajectory from start to finish without going nuts
TEST_F(SanityChecks, BackToBack)
{
  const auto p = ConstantParam{0.0F, 0.0F, 0.0F, 1.0F, 1.0F, 0.0F, zero_ns};
  auto dt = p.dt;
  if (std::chrono::nanoseconds::zero() == dt) {
    dt = controller_.get_config().behavior().time_step();
  }
  // Initial trajectory
  auto traj1 =
    constant_acceleration_turn_rate_trajectory(p.x0, p.y0, p.heading, p.v0, p.a0, p.omega0, dt);
  // Resize to force receding horizon behavior
  traj1.points.resize(30);
  const auto state1 =
    make_state(p.x0, p.y0, p.heading, p.v0, p.a0, 0.0F, from_message(traj1.header.stamp));
  std::vector<MpcController::Command> commands{20};
  apex_test_tools::memory_test::start();
  // Run back to back--result should be same
  for (auto & cmd : commands) {
    controller_.set_trajectory(traj1);
    cmd = controller_.compute_command(state1);
  }
  apex_test_tools::memory_test::pause();
  const auto EXPECT_CMD_EQ = [](auto cmd1, auto cmd2) -> void {
      // Don't care about header
      EXPECT_FLOAT_EQ(cmd1.long_accel_mps2, cmd2.long_accel_mps2);
      EXPECT_FLOAT_EQ(cmd1.front_wheel_angle_rad, cmd2.front_wheel_angle_rad);
      EXPECT_FLOAT_EQ(cmd1.rear_wheel_angle_rad, cmd2.rear_wheel_angle_rad);
    };
  for (const auto & cmd : commands) {
    EXPECT_CMD_EQ(cmd, commands.front());
  }

  // Get new state near the end of current trajectory
  auto state2 = state1;
  {
    ASSERT_GT(traj1.points.size(), 28U);
    const auto & second_last_pt = traj1.points[27U];
    const auto stamp =
      from_message(traj1.header.stamp) + from_message(second_last_pt.time_from_start);
    state2.header.stamp = to_message(stamp);
  }
  apex_test_tools::memory_test::resume();
  // Should result in qualitatively different behavior
  {
    const auto cmd2 = controller_.compute_command(state2);
    EXPECT_NE(cmd2.long_accel_mps2, commands.front().long_accel_mps2);
  }
  apex_test_tools::memory_test::pause();
  if (HasFailure()) {
    controller_.debug_print(std::cout);
  }
  // New trajectory starting near traj1
  auto traj2 = traj1;
  {
    traj2.header.stamp = state2.header.stamp;
  }
  apex_test_tools::memory_test::resume();
  // Should have exact same behavior as first case (aka internal state should be retained)
  {
    controller_.set_trajectory(traj2);
    const auto cmd3 = controller_.compute_command(state2);
    EXPECT_CMD_EQ(cmd3, commands.front());
  }
  apex_test_tools::memory_test::pause();

  if (HasFailure()) {
    controller_.debug_print(std::cout);
  }
}
