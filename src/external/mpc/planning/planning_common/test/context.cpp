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
using motion::planning::planning_common::PlanningContext;
using motion::planning::planning_common::PlanningEnvironment;
using motion::motion_testing::make_state;
using time_utils::from_message;

class context : public ::testing::Test
{
protected:
  PlanningEnvironment env{EnvironmentConfig{std::chrono::milliseconds{100LL}}};
  const std::chrono::system_clock::time_point t0{std::chrono::system_clock::now()};
  const std::string frame{"foo"};
};

TEST_F(context, simple_alignment)
{
  // Empty
  EXPECT_FALSE(env.has_valid_context(frame, t0));
  EXPECT_THROW(env.context(frame, t0), std::domain_error);
  // Not empty
  {
    auto ego_state = make_state(0.0F, 1.0F, 2.0F, 3.0F, 4.0F, 5.0F, t0);
    auto target_state = make_state(0.0F, -1.0F, -2.0F, -3.0F, -4.0F, -5.0F, t0);
    ego_state.header.frame_id = frame;
    target_state.header.frame_id = frame;

    env.add_target_state(std::move(target_state));
    // Missing ego
    EXPECT_FALSE(env.has_valid_context(frame, t0));
    EXPECT_THROW(env.context(frame, t0), std::domain_error);
    env.add_ego_state(std::move(ego_state));
    EXPECT_TRUE(env.has_valid_context(frame, t0));
    const auto ctx = env.context(frame, t0);
    EXPECT_EQ(ctx.ego_state(), ego_state);
    EXPECT_EQ(ctx.target_state(), target_state);
    // No transform
    EXPECT_FALSE(env.has_valid_context("bar", t0));
    EXPECT_THROW(env.context("bar", t0), std::domain_error);
  }
  // Empty again
  env.clear_before(t0 + std::chrono::milliseconds(1));
  EXPECT_FALSE(env.has_valid_context(frame, t0));
  EXPECT_THROW(env.context(frame, t0), std::domain_error);
}

TEST_F(context, interpolation)
{
  const auto t1 = t0 + std::chrono::milliseconds{100LL};
  const auto tm = t0 + ((t1 - t0) / 4);
  auto target1 = make_state(-3.0F, -3.0F, 0.0F, 0.0F, 0.0F, 0.0F, t0);
  auto target2 = make_state(3.0F, 3.0F, 0.0F, 0.0F, 0.0F, 0.0F, t1);
  auto s1 = make_state(0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, t0);
  auto s2 = make_state(1.0F, -1.0F, 0.0F, 0.0F, 0.0F, 0.0F, t1);
  s1.header.frame_id = frame;
  s2.header.frame_id = frame;
  target1.header.frame_id = frame;
  target2.header.frame_id = frame;
  env.add_ego_state(s1);
  env.add_ego_state(s2);
  // Missing target
  EXPECT_FALSE(env.has_valid_context(frame, t0));
  EXPECT_THROW(env.context(frame, t0), std::domain_error);
  env.add_target_state(target1);
  env.add_target_state(target2);
  // State at t0 and t1; mean ok, ends ok
  {
    EXPECT_TRUE(env.has_valid_context(frame, tm));
    const auto ctx = env.context(frame, tm);
    EXPECT_EQ(ctx.target_state(), target1);
    EXPECT_LT(std::fabs(ctx.ego_state().state.x - 0.25F), 1.0E-4F);
    EXPECT_LT(std::fabs(ctx.ego_state().state.y + 0.25F), 1.0E-4F);
  }
  {
    EXPECT_TRUE(env.has_valid_context(frame, t0));
    const auto ctx = env.context(frame, t0);
    EXPECT_EQ(ctx.target_state(), target1);
    EXPECT_EQ(ctx.ego_state(), s1);
  }
  {
    EXPECT_TRUE(env.has_valid_context(frame, t1));
    const auto ctx = env.context(frame, t1);
    EXPECT_EQ(ctx.target_state(), target2);
    EXPECT_EQ(ctx.ego_state(), s2);
  }
  // Just outside of range
  constexpr auto dt_eps = std::chrono::microseconds{1LL};
  {
    EXPECT_FALSE(env.has_valid_context(frame, t1 + dt_eps));
    EXPECT_THROW(env.context(frame, t1 + dt_eps), std::domain_error);
  }
  {
    EXPECT_FALSE(env.has_valid_context(frame, t0 - dt_eps));
    EXPECT_THROW(env.context(frame, t0 - dt_eps), std::domain_error);
  }
  // Limits
  {
    EXPECT_FALSE(env.has_valid_context(frame, std::chrono::system_clock::time_point::max()));
    EXPECT_THROW(env.context(frame, std::chrono::system_clock::time_point::max()),
      std::domain_error);
  }
  {
    EXPECT_FALSE(env.has_valid_context(frame, std::chrono::system_clock::time_point::min()));
    EXPECT_THROW(env.context(frame, std::chrono::system_clock::time_point::min()),
      std::domain_error);
  }
}

TEST_F(context, transform)
{
  const auto t1 = t0 + std::chrono::milliseconds{100LL};
  const auto tm = t0 + ((t1 - t0) / 2);
  const auto child = "bar";
  auto target = make_state(-3.0F, -3.0F, 0.0F, 0.0F, 0.0F, 0.0F, tm);
  // Same location, just different frame
  auto s1 = make_state(1.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, t0);
  auto s2 = make_state(-2.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, t1);
  // transform: [0. 0. 0 1]. [-1. 0. 0]
  s1.header.frame_id = frame;
  s2.header.frame_id = child;
  target.header.frame_id = frame;
  env.add_ego_state(s1);
  env.add_ego_state(s2);
  env.add_target_state(target);
  {
    Transform tf1;
    tf1.transform.translation.x = -1.0;
    tf1.transform.rotation.x = {};
    tf1.transform.rotation.z = 1.0;
    tf1.child_frame_id = child;
    tf1.header.frame_id = frame;
    tf1.header.stamp = time_utils::to_message(t0);
    auto tf2 = tf1;
    tf2.header.stamp = time_utils::to_message(t1);
    env.add_transform(tf1);
    env.add_transform(std::move(tf2));
  }
  // TODO(c.ho) add transforms
  // State at t0 and t1; mean ok, ends ok
  {
    EXPECT_TRUE(env.has_valid_context(frame, tm));
    const auto ctx = env.context(frame, tm);
    EXPECT_EQ(ctx.target_state(), target);
    EXPECT_LT(std::fabs(ctx.ego_state().state.x - 1.0F), 1.0E-4F);
    EXPECT_LT(std::fabs(ctx.ego_state().state.y - 0.0F), 1.0E-4F);
  }
  {
    EXPECT_TRUE(env.has_valid_context(frame, t0));
    const auto ctx = env.context(child, t0);
    EXPECT_LT(std::fabs(ctx.target_state().state.x - 2.0F), 1.0E-4F);
    EXPECT_LT(std::fabs(ctx.target_state().state.y - 3.0F), 1.0E-4F);
    EXPECT_LT(std::fabs(ctx.ego_state().state.x + 2.0F), 1.0E-4F);
    EXPECT_LT(std::fabs(ctx.ego_state().state.y - 0.0F), 1.0E-4F);
  }
  // No transform
}

// Test that things that can't be interpolated throw errors when requested time is too far away
TEST(context_configed, discrete_range)
{
  const auto dt_range = std::chrono::milliseconds{20LL};
  const auto dt_bad = dt_range + std::chrono::microseconds{1LL};
  PlanningEnvironment env{EnvironmentConfig{dt_range}};
  const auto t0 = std::chrono::system_clock::now();
  const auto t1 = t0 + std::chrono::milliseconds{100LL};
  const auto tm = t0 + ((t1 - t0) / 4);
  const auto frame = "foo";
  const auto t_target0 = t0 + dt_bad;
  const auto t_target1 = t1 - dt_bad;
  auto target1 = make_state(-3.0F, -3.0F, 0.0F, 0.0F, 0.0F, 0.0F, t_target0);
  auto target2 = make_state(3.0F, 3.0F, 0.0F, 0.0F, 0.0F, 0.0F, t_target1);
  auto s1 = make_state(0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, t0);
  auto s2 = make_state(1.0F, -1.0F, 0.0F, 0.0F, 0.0F, 0.0F, t1);
  s1.header.frame_id = frame;
  s2.header.frame_id = frame;
  target1.header.frame_id = frame;
  target2.header.frame_id = frame;
  env.add_ego_state(s1);
  env.add_ego_state(s2);
  env.add_target_state(target1);
  env.add_target_state(target2);
  // State at t0 and t1; mean ok, ends ok
  {
    EXPECT_TRUE(env.has_valid_context(frame, tm));
    const auto ctx = env.context(frame, tm);
    EXPECT_EQ(ctx.target_state(), target1);
    EXPECT_LT(std::fabs(ctx.ego_state().state.x - 0.25F), 1.0E-4F);
    EXPECT_LT(std::fabs(ctx.ego_state().state.y + 0.25F), 1.0E-4F);
  }
  // A fraction outside the target range (but still within interpolation
  {
    // Assertion to check test invariants
    // |--| represents receptive field for targets
    // t0  1|2---tar0---3|4         5|6---tar1---7|8      t1
    const auto check_t = [&](const auto stamp, bool expect_throw) -> void {
        ASSERT_LE(stamp, t1);
        ASSERT_GE(stamp, t0);
        if (expect_throw) {
          EXPECT_FALSE(env.has_valid_context(frame, stamp));
          EXPECT_THROW(env.context(frame, stamp), std::domain_error);
        } else {
          EXPECT_TRUE(env.has_valid_context(frame, stamp));
          EXPECT_NO_THROW(env.context(frame, stamp));
        }
      };
    check_t(t0, true);
    // Cases 1-8 in order from here down
    check_t(t_target0 - dt_bad, true);
    check_t(t_target0 - dt_range, false);
    check_t(t_target0 + dt_range, false);
    check_t(t_target0 + dt_bad, true);
    check_t(t_target1 - dt_bad, true);
    check_t(t_target1 - dt_range, false);
    check_t(t_target1 + dt_range, false);
    check_t(t_target1 + dt_bad, true);
    check_t(t1, true);
  }
}
