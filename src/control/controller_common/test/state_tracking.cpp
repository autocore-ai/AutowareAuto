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
#include <motion_testing/motion_testing.hpp>
#include <time_utils/time_utils.hpp>

#include <chrono>
#include <utility>

using motion::control::controller_common::State;
using motion::control::controller_common::Command;
using motion::control::controller_common::BehaviorConfig;
using motion::control::controller_common::ControlReference;
using motion::control::controller_common::ControllerBase;
using motion::motion_testing::make_state;
using motion::motion_testing::constant_velocity_trajectory;
using time_utils::from_message;

class TestStateController : public ControllerBase
{
public:
  explicit TestStateController(const BehaviorConfig & config)
  : ControllerBase{config} {}

  // expose API for testing purposes
  using ControllerBase::get_current_state_spatial_index;
  using ControllerBase::get_current_state_temporal_index;

protected:
  Command compute_command_impl(const State & state) override
  {
    (void)state;
    return Command{};
  }
};  // class TestController

class StateTracking : public ::testing::Test
{
public:
  StateTracking()
  : controller_{BehaviorConfig{3.0F, std::chrono::milliseconds(100LL), ControlReference::SPATIAL}}
  {
  }

protected:
  TestStateController controller_;
};  // class StateTracking

// Should throw if no trajectory is present or if trajectory is of size 0
TEST_F(StateTracking, NoTrajectory)
{
  EXPECT_THROW(controller_.get_current_state_spatial_index(), std::domain_error);
  EXPECT_THROW(controller_.get_current_state_temporal_index(), std::domain_error);
  auto state = make_state(10.0F, -30.0F, 0.0F, 3.0F, 0.0F, 0.0F, std::chrono::system_clock::now());
  state.header.frame_id = "foo";
  apex_test_tools::memory_test::start_paused();
  // Just have one of these in a compilation unit to make sure everything is hunky-dory
#ifndef __aarch64__
  ASSERT_TRUE(osrf_testing_tools_cpp::memory_tools::is_working());
#endif
  apex_test_tools::memory_test::resume();
  (void)controller_.compute_command(state);
  apex_test_tools::memory_test::stop();
  EXPECT_THROW(controller_.get_current_state_spatial_index(), std::domain_error);
  EXPECT_THROW(controller_.get_current_state_temporal_index(), std::domain_error);
}

// Should throw if no trajectory is present or if trajectory is of size 0
TEST_F(StateTracking, SizeZeroTrajectory)
{
  EXPECT_THROW(controller_.get_current_state_spatial_index(), std::domain_error);
  EXPECT_THROW(controller_.get_current_state_temporal_index(), std::domain_error);
  const auto dt = std::chrono::milliseconds(100LL);
  auto state = make_state(10.0F, -30.0F, 0.0F, 3.0F, 0.0F, 0.0F, std::chrono::system_clock::now());
  state.header.frame_id = "foo";
  auto traj = constant_velocity_trajectory(0.0F, 0.0F, 3.0F, 1.0F, dt);
  traj.header.frame_id = "foo";
  traj.points.resize(0U);
  EXPECT_THROW(controller_.set_trajectory(std::move(traj)), std::domain_error);
  apex_test_tools::memory_test::start();
  (void)controller_.compute_command(state);
  apex_test_tools::memory_test::stop();
  EXPECT_THROW(controller_.get_current_state_temporal_index(), std::domain_error);
  EXPECT_THROW(controller_.get_current_state_spatial_index(), std::domain_error);
}

// New trajectory should result in a reference index of 0
// subsequent calls should give the index of the point the current state is just after
// So if points are at 0, 1, 2..., then 0, and 0.5 should result in reference indices of 0
// and 1.0 and 1.5 should give 1
TEST_F(StateTracking, Basic)
{
  const auto dt = std::chrono::milliseconds(100LL);
  auto traj = constant_velocity_trajectory(0.0F, 0.0F, 0.0F, 10.0F, dt);
  traj.header.frame_id = "foo";
  apex_test_tools::memory_test::start();
  controller_.set_trajectory(traj);
  ASSERT_EQ(controller_.get_current_state_spatial_index(), 0U);
  ASSERT_EQ(controller_.get_current_state_temporal_index(), 0U);
  apex_test_tools::memory_test::pause();
  const auto t0 = from_message(traj.header.stamp);
  for (auto idx = 0U; idx < 100U; ++idx) {
    const auto x = 0.5F * static_cast<float>(idx);
    const auto t = t0 + (idx * (dt / 2));
    auto state = make_state(x, 0.0F, 0.0F, 10.0F, 0.0F, 0.0F, t);
    state.header.frame_id = "foo";
    apex_test_tools::memory_test::resume();
    (void)controller_.compute_command(state);
    ASSERT_EQ(controller_.get_current_state_spatial_index(), idx / 2U);
    ASSERT_EQ(controller_.get_current_state_temporal_index(), idx / 2U);
    apex_test_tools::memory_test::pause();
  }
  apex_test_tools::memory_test::stop();
}

// Going past the end of the trajectory should keep you fixed at the end, even if
// you go backwards
TEST_F(StateTracking, PastEnd)
{
  const auto dt = std::chrono::milliseconds(100LL);
  auto traj = constant_velocity_trajectory(0.0F, 0.0F, 3.14159F, 1.0F, dt);
  traj.header.frame_id = "foo";
  apex_test_tools::memory_test::start();
  controller_.set_trajectory(traj);
  EXPECT_EQ(controller_.get_current_state_spatial_index(), 0U);
  ASSERT_EQ(controller_.get_current_state_temporal_index(), 0U);
  apex_test_tools::memory_test::pause();
  const auto t0 = from_message(traj.header.stamp);
  const auto t0_ = t0 + std::chrono::microseconds(1LL);  // the 1.01 of time
  // First one is normal
  {
    auto state = make_state(-1.0F, 0.0F, 0.0F, 10.0F, 0.0F, 0.0F, t0 + (dt * 10U));
    state.header.frame_id = "foo";
    apex_test_tools::memory_test::resume();
    (void)controller_.compute_command(state);
    EXPECT_EQ(controller_.get_current_state_spatial_index(), 10U);
    EXPECT_EQ(controller_.get_current_state_temporal_index(), 10U);
    apex_test_tools::memory_test::pause();
  }
  // Normal
  {
    auto state = make_state(-9.01F, 0.0F, 0.0F, 10.0F, 0.0F, 0.0F, t0_ + (dt * 90U));
    state.header.frame_id = "foo";
    apex_test_tools::memory_test::resume();
    (void)controller_.compute_command(state);
    EXPECT_EQ(controller_.get_current_state_spatial_index(), 90U);
    EXPECT_EQ(controller_.get_current_state_temporal_index(), 90U);
    apex_test_tools::memory_test::pause();
  }
  // End
  {
    auto state = make_state(-10.0F, 0.0F, 0.0F, 10.0F, 0.0F, 0.0F, t0 + (dt * 100U));
    state.header.frame_id = "foo";
    apex_test_tools::memory_test::resume();
    (void)controller_.compute_command(state);
    EXPECT_EQ(controller_.get_current_state_spatial_index(), 99U);
    EXPECT_EQ(controller_.get_current_state_temporal_index(), 99U);
    EXPECT_EQ(traj.points.size(), 99U + 1U);
    apex_test_tools::memory_test::pause();
  }
  // Past end
  {
    auto state = make_state(-15.0F, 0.0F, 0.0F, 10.0F, 0.0F, 0.0F, t0 + (dt * 150U));
    state.header.frame_id = "foo";
    apex_test_tools::memory_test::resume();
    (void)controller_.compute_command(state);
    EXPECT_EQ(controller_.get_current_state_spatial_index(), 99U);
    EXPECT_EQ(controller_.get_current_state_temporal_index(), 99U);
    apex_test_tools::memory_test::pause();
  }
  // Backwards
  {
    auto state = make_state(-5.0F, 0.0F, 0.0F, 10.0F, 0.0F, 0.0F, t0 + (dt * 50U));
    state.header.frame_id = "foo";
    apex_test_tools::memory_test::resume();
    (void)controller_.compute_command(state);
    EXPECT_EQ(controller_.get_current_state_spatial_index(), 99U);
    EXPECT_EQ(controller_.get_current_state_temporal_index(), 99U);
    apex_test_tools::memory_test::pause();
  }
  // New trajectory resets things TODO(c.ho) separate test case
  apex_test_tools::memory_test::resume();
  controller_.set_trajectory(traj);
  EXPECT_EQ(controller_.get_current_state_spatial_index(), 0U);
  EXPECT_EQ(controller_.get_current_state_temporal_index(), 0U);
  {
    apex_test_tools::memory_test::pause();
    auto state = make_state(-0.11F, 0.0F, 0.0F, 10.0F, 0.0F, 0.0F, t0_ + dt);
    state.header.frame_id = "foo";
    apex_test_tools::memory_test::resume();
    (void)controller_.compute_command(state);
    EXPECT_EQ(controller_.get_current_state_spatial_index(), 1U);
    EXPECT_EQ(controller_.get_current_state_temporal_index(), 1U);
    apex_test_tools::memory_test::pause();
  }
  apex_test_tools::memory_test::stop();
}

// TODO(c.ho) U-bend test
