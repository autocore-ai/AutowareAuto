// Copyright 2020 Embotech AG, Zurich, Switzerland, inspired by Christopher Ho's mpc code
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

#include <gtest/gtest.h>
#include <recordreplay_planner/recordreplay_planner.hpp>
#include <motion_testing/motion_testing.hpp>
#include <autoware_auto_msgs/msg/trajectory.hpp>

#include <chrono>
#include <algorithm>

using motion::planning::recordreplay_planner::RecordReplayPlanner;
using std::chrono::system_clock;
using motion::motion_testing::make_state;
using Trajectory = autoware_auto_msgs::msg::Trajectory;

class sanity_checks_base : public ::testing::Test
{
protected:
  RecordReplayPlanner planner_{};
};


//------------------ Test basic properties of a recorded, then replayed trajectory
struct PropertyTestParameters
{
  uint64_t time_spacing_ms;
};

class sanity_checks_trajectory_properties
  : public sanity_checks_base, public testing::WithParamInterface<PropertyTestParameters>
{};

TEST_P(sanity_checks_trajectory_properties, basicproperties)
{
  const auto t = system_clock::now();
  const auto p = GetParam();
  auto t0 = system_clock::from_time_t({});

  // Build a trajectory
  constexpr auto N = 10;
  const auto time_increment = p.time_spacing_ms * std::chrono::milliseconds{1LL};
  for (uint32_t k = {}; k < N; ++k) {
    const auto next_state = make_state(1.0F * k, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
        t0 + k * time_increment);
    planner_.record_state(next_state);
  }

  // Test: Check that the length is equal to the number of states we fed in
  EXPECT_EQ(planner_.get_record_length(), N);

  // Test: Check that the plan returned has the expected time length
  auto trajectory = planner_.plan(make_state(0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, t0));
  double trajectory_time_length = trajectory.points[N - 1].time_from_start.sec + 1e-9F *
    trajectory.points[N - 1].time_from_start.nanosec;
  EXPECT_EQ(std::chrono::duration<float>(trajectory_time_length), 1.0F * (N - 1) * time_increment);
}

INSTANTIATE_TEST_CASE_P(
  trajectory_properties,
  sanity_checks_trajectory_properties,
  testing::Values(
    PropertyTestParameters{100},
    PropertyTestParameters{200}
));


//------------------ Test that length cropping properly works
struct LengthTestParameters
{
  // The number of points to be recorded
  uint32_t number_of_points;
};


class sanity_checks_trajectory_length
  : public sanity_checks_base, public testing::WithParamInterface<LengthTestParameters>
{};

TEST_P(sanity_checks_trajectory_length, length)
{
  const auto t = system_clock::now();
  const auto p = GetParam();
  const auto N = p.number_of_points;
  const auto dummy_state = make_state(0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
      system_clock::from_time_t({}));

  for (uint32_t k = {}; k < N; ++k) {
    planner_.record_state(dummy_state);
  }

  // Test: Check that the length is equal to the number of states we fed in
  EXPECT_EQ(planner_.get_record_length(), N);
  auto trajectory = planner_.plan(dummy_state);

  EXPECT_EQ(trajectory.points.size(),
    std::min(N, static_cast<uint32_t>(trajectory.points.max_size())));
}

INSTANTIATE_TEST_CASE_P(
  trajectory_length,
  sanity_checks_trajectory_length,
  testing::Values(
    LengthTestParameters{80},
    LengthTestParameters{200}
));


//------------------ Test that "receding horizon" planning properly works
TEST(recordreplay_sanity_checks, receding_horizon)
{
  auto planner = RecordReplayPlanner();

  // Record some states
  const auto t0 = system_clock::now();
  const auto N = 3;
  for (uint32_t k = {}; k < N; ++k) {
    planner.record_state(make_state(1.0F * k, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
      t0 + k * std::chrono::milliseconds{100LL}));
  }

  // Call "plan" multiple times in sequence, expecting the states to come back out in order
  for (uint32_t k = {}; k < N; ++k) {
    auto trajectory = planner.plan(make_state(0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, t0));
    // normally don't check float equality but we _just_ pushed this float so it ought not
    // to have changed
    EXPECT_EQ(1.0F * k, trajectory.points[0].x);
  }
}
