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
#include <gtest/gtest.h>

#include <planning_common/planner_base.hpp>
#include <motion_testing/motion_testing.hpp>
#include <time_utils/time_utils.hpp>

#include <chrono>
#include <string>
#include <utility>

using motion::planning::planning_common::State;
using motion::planning::planning_common::Trajectory;
using motion::planning::planning_common::Transform;
using motion::planning::planning_common::Transforms;
using motion::planning::planning_common::EnvironmentConfig;
using motion::planning::planning_common::PlannerBase;
using motion::planning::planning_common::PlanningContext;
using motion::planning::planning_common::PlanningEnvironment;
using motion::motion_testing::make_state;
using time_utils::to_message;

class planner_test_error : public std::runtime_error
{
public:
  planner_test_error()
  : runtime_error{"test: planner logic not needed"} {}
};

class TestPlanner : public PlannerBase
{
public:
  const Trajectory & plan_impl(const PlanningContext & context) override
  {
    (void)context;
    throw planner_test_error{};
  }
};  // class TestPlanner

TEST(planner_base, past_target)
{
  PlanningEnvironment env{EnvironmentConfig{std::chrono::milliseconds{100LL}}};
  // Add stuff
  const auto frame = "test_frame";
  const auto t0 = std::chrono::system_clock::now();
  const auto t1 = t0 + std::chrono::milliseconds{100LL};
  const auto tm = t0 + ((t1 - t0) / 2);
  auto target = make_state(-1.0F, 2.0F, 2.0F, 0.0F, 0.0F, 0.0F, tm);
  auto s1 = make_state(0.0F, 0.0F, 2.0F, 0.0F, 0.0F, 0.0F, t0);
  auto s2 = make_state(-2.0F, 4.0F, 2.0F, 0.0F, 0.0F, 0.0F, t1);
  s1.header.frame_id = frame;
  s2.header.frame_id = frame;
  target.header.frame_id = frame;
  env.add_ego_state(s1);
  env.add_ego_state(s2);
  env.add_target_state(target);

  // Nominal behavior should hit
  TestPlanner planner{};
  EXPECT_THROW(planner.plan(env.context(frame, t0)), planner_test_error);
  // Stopping behavior
  const auto traj = planner.plan(env.context(frame, t1));
  EXPECT_EQ(traj.header.stamp, time_utils::to_message(t1));
  EXPECT_EQ(traj.header.frame_id, frame);
  EXPECT_TRUE(traj.points.empty());
}

// TODO(c.ho) test edge cases
