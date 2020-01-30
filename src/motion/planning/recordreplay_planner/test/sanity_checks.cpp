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

using motion::planning::recordreplay_planner::RecordReplayPlanner;
using std::chrono::system_clock;
using motion::motion_testing::make_state;
using Trajectory = autoware_auto_msgs::msg::Trajectory;

class sanity_checks_base : public ::testing::Test
{
protected:
  RecordReplayPlanner planner_{};
};

struct RecordTestParameters
{
  uint64_t time_spacing_ms;
};

class sanity_checks_constraint_free
  : public sanity_checks_base, public testing::WithParamInterface<RecordTestParameters>
{
};

// TODO(s.me) more tests, also of from_record()
TEST_P(sanity_checks_constraint_free, sometest)
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
  constraint_free,
  sanity_checks_constraint_free,
  testing::Values(
    RecordTestParameters{100},
    RecordTestParameters{200}
));
