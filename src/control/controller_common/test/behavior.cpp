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

using motion::control::controller_common::State;
using motion::control::controller_common::Command;
using motion::control::controller_common::BehaviorConfig;
using motion::control::controller_common::ControlReference;
using motion::control::controller_common::ControllerBase;
using motion::motion_testing::make_state;
using motion::motion_testing::constant_velocity_trajectory;
using time_utils::from_message;

class TestController : public ControllerBase
{
public:
  explicit TestController(const BehaviorConfig & config)
  : ControllerBase{config} {}

protected:
  Command compute_command_impl(const State & state) override
  {
    (void)state;
    return Command{};
  }
};  // class TestController

class Behavior : public ::testing::Test
{
public:
  Behavior()
  : controller_{BehaviorConfig{3.0F, std::chrono::milliseconds(100LL), ControlReference::SPATIAL}}
  {
  }

protected:
  TestController controller_;
};  // class Behavior

// Vehicle should come to a stop if there's no trajectory present
TEST_F(Behavior, NoTrajectory)
{
  // TODO(c.ho) checks on stop rate
  auto state = make_state(10.0F, -30.0F, 0.0F, 3.0F, 0.0F, 0.0F, std::chrono::system_clock::now());
  state.header.frame_id = "foo";
  apex_test_tools::memory_test::start_paused();
  // Just have one of these in a compilation unit to make sure everything is hunky-dory
#ifndef __aarch64__
  ASSERT_TRUE(osrf_testing_tools_cpp::memory_tools::is_working());
#endif
  apex_test_tools::memory_test::resume();
  const auto cmd = controller_.compute_command(state);
  apex_test_tools::memory_test::stop();
  ASSERT_FLOAT_EQ(cmd.long_accel_mps2, -3.0F);
  ASSERT_FLOAT_EQ(cmd.front_wheel_angle_rad, 0.0F);
  ASSERT_FLOAT_EQ(cmd.rear_wheel_angle_rad, 0.0F);
  ASSERT_EQ(cmd.stamp, state.header.stamp);
}

// Vehicle should stay stopped if the vehicle is nearly stopped
TEST_F(Behavior, NoTrajectorySlow)
{
  auto state = make_state(15.0F, 10.0F, 4.0F, 0.1F, 0.0F, 0.0F, std::chrono::system_clock::now());
  state.header.frame_id = "foo";
  apex_test_tools::memory_test::start();
  const auto cmd = controller_.compute_command(state);
  apex_test_tools::memory_test::stop();
  ASSERT_FLOAT_EQ(cmd.long_accel_mps2, -1.0F);
  ASSERT_FLOAT_EQ(cmd.front_wheel_angle_rad, 0.0F);
  ASSERT_FLOAT_EQ(cmd.rear_wheel_angle_rad, 0.0F);
  ASSERT_EQ(cmd.stamp, state.header.stamp);
}

// Vehicle should stay stopped if the vehicle is stopped
TEST_F(Behavior, NoTrajectoryStopped)
{
  auto state = make_state(-10.0F, -30.0F, 1.0F, 0.0F, 0.0F, 0.0F, std::chrono::system_clock::now());
  state.header.frame_id = "foo";
  apex_test_tools::memory_test::start();
  const auto cmd = controller_.compute_command(state);
  apex_test_tools::memory_test::stop();
  ASSERT_FLOAT_EQ(cmd.long_accel_mps2, 0.0F);
  ASSERT_FLOAT_EQ(cmd.front_wheel_angle_rad, 0.0F);
  ASSERT_FLOAT_EQ(cmd.rear_wheel_angle_rad, 0.0F);
  ASSERT_EQ(cmd.stamp, state.header.stamp);
}

// If the state is newest than the end of the trajectory, come to a stop
TEST_F(Behavior, OldTrajectory)
{
  const auto dt = std::chrono::milliseconds(100LL);
  auto traj = constant_velocity_trajectory(0.0F, 0.0F, 3.0F, 1.0F, dt);
  traj.header.frame_id = "foo";
  auto state =
    make_state(0.0F, 0.0F, 0.0F, 1.0F, 0.0F, 0.0F, std::chrono::system_clock::time_point::max());
  state.header.frame_id = "foo";
  apex_test_tools::memory_test::start();
  controller_.set_trajectory(traj);
  const auto cmd = controller_.compute_command(state);
  apex_test_tools::memory_test::stop();
  ASSERT_FLOAT_EQ(cmd.long_accel_mps2, -3.0F);
  ASSERT_FLOAT_EQ(cmd.front_wheel_angle_rad, 0.0F);
  ASSERT_FLOAT_EQ(cmd.rear_wheel_angle_rad, 0.0F);
  ASSERT_EQ(cmd.stamp, state.header.stamp);
}

// Vehicle should come to a stop after physically being past the last point of the trajectory
TEST_F(Behavior, PastTrajectory)
{
  const auto dt = std::chrono::milliseconds(100LL);
  auto traj = constant_velocity_trajectory(0.0F, 0.0F, 0.0F, 1.0F, dt);
  traj.header.frame_id = "foo";
  auto state = make_state(11.0F, 0.0F, 0.0F, 1.0F, 0.0F, 0.0F, from_message(traj.header.stamp));
  state.header.frame_id = "foo";
  apex_test_tools::memory_test::start();
  controller_.set_trajectory(traj);
  const auto cmd = controller_.compute_command(state);
  apex_test_tools::memory_test::stop();
  ASSERT_FLOAT_EQ(cmd.long_accel_mps2, -3.0F);
  ASSERT_FLOAT_EQ(cmd.front_wheel_angle_rad, 0.0F);
  ASSERT_FLOAT_EQ(cmd.rear_wheel_angle_rad, 0.0F);
  ASSERT_EQ(cmd.stamp, state.header.stamp);
}

// Fail on inconsistent header
TEST_F(Behavior, WrongFrame)
{
  const auto dt = std::chrono::milliseconds(100LL);
  auto traj = constant_velocity_trajectory(0.0F, 0.0F, 3.0F, 1.0F, dt);
  traj.header.frame_id = "foo";
  controller_.set_trajectory(traj);
  auto state =
    make_state(0.0F, 0.0F, 0.0F, 1.0F, 0.0F, 0.0F, std::chrono::system_clock::time_point::max());
  state.header.frame_id = "bar";
  ASSERT_NE(traj.header.frame_id, state.header.frame_id);
  ASSERT_THROW(controller_.compute_command(state), std::domain_error);
  // TODO(c.ho) no memory test here: no access to static exceptions
}

// TODO(c.ho) past_trajectory_curved (i.e. u-turn maneuver, but past the end somehow)
