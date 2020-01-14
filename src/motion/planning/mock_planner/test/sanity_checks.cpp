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
#include <mock_planner/mock_planner.hpp>
#include <motion_testing/motion_testing.hpp>
#include <time_utils/time_utils.hpp>

#include <chrono>

using motion::planning::mock_planner::MockPlanner;
using motion::planning::mock_planner::PlannerConfig;
using motion::planning::mock_planner::Trajectory;
using motion::planning::planning_common::State;
using motion::planning::planning_common::EnvironmentConfig;
using motion::planning::planning_common::PlanningEnvironment;
using motion::planning::planning_common::PlanningContext;
using motion::motion_testing::make_state;

using std::chrono::system_clock;

class sanity_checks_base : public ::testing::Test
{
protected:
  MockPlanner planner_{
    PlannerConfig{
      motion::motion_common::LimitsConfig{
        {0.01F, 35.0F},  // Longitudinal velocity
        {-3.0F, 3.0F},  // Lateral velocity
        {-3.0F, 3.0F},  // Acceleation
        {-3.0F, 3.0F},  // yaw rate
        {-10.0F, 10.0F},  // Jerk
        {-0.331F, 0.331F},  // Steer angle
        {-0.331F, 0.331F}  // Steer angle rate
      },
      motion::motion_common::VehicleConfig{
        // Parameters from LaValle
        1.2F,  // CG to front
        1.5F,  // CG to rear
        17000.0F,  // front cornering
        20000.0F,  // rear cornering
        1460.0F,  // mass
        2170.0F  // Inertia
      },
      motion::motion_common::OptimizationConfig{
        motion::motion_common::StateWeight{
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
        motion::motion_common::StateWeight{
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
      }
    }
  };
  PlanningEnvironment environment_{EnvironmentConfig{std::chrono::seconds{100LL}}};
};

struct ConstraintFree
{
  State start;
  State target;
};

class sanity_checks_constraint_free
  : public sanity_checks_base, public testing::WithParamInterface<ConstraintFree>
{
};

TEST_P(sanity_checks_constraint_free, oneshot)
{
  const auto frame = "test_frame";
  const auto t = system_clock::now();
  auto p = GetParam();
  p.start.header.stamp = time_utils::to_message(t);
  p.start.header.frame_id = frame;
  p.target.header.stamp = time_utils::to_message(t);
  p.target.header.frame_id = frame;
  // Set environment
  environment_.add_ego_state(p.start);
  environment_.add_target_state(p.target);
  // Get context
  const auto ctx = environment_.context(frame, t);
  // Plan
  const auto & traj = planner_.plan(ctx);
  // Check
  using motion::motion_testing::dynamically_feasible;
  using motion::motion_testing::progresses_towards_target;
  EXPECT_FALSE(traj.points.empty());
  const auto tol = 0.05F;
  EXPECT_EQ(dynamically_feasible(traj, tol), traj.points.size());
  auto heading_tol = 0.006F;
  // Hack for weird heading cases
  if (std::fabs(motion::motion_common::to_angle(p.start.state.heading)) > 3.0F) {
    heading_tol = 0.0001F;
  }
  EXPECT_EQ(progresses_towards_target(traj, p.target.state, heading_tol), traj.points.size());
  //if (HasFailure()) {
    //planner_.debug_print(std::cout);
  //}
}

TEST_P(sanity_checks_constraint_free, simulation)
{
  const auto frame = "test_frame";
  auto t = system_clock::now();
  auto p = GetParam();
  auto start = p.start;
  start.header.stamp = time_utils::to_message(t);
  start.header.frame_id = frame;
  p.target.header.stamp = time_utils::to_message(t);
  p.target.header.frame_id = frame;
  // Set environment
  environment_.add_ego_state(start);
  environment_.add_target_state(p.target);
  for (auto idx = 0U; idx < 100U; ++idx) {
    // Get context
    const auto ctx = environment_.context(frame, t);
    // Plan
    const auto & traj = planner_.plan(ctx);
    // Check
    using motion::motion_testing::dynamically_feasible;
    using motion::motion_testing::progresses_towards_target;
    EXPECT_FALSE(traj.points.empty());
    const auto tol = 0.05F;
    EXPECT_EQ(dynamically_feasible(traj, tol), traj.points.size());
    auto heading_tol = 0.006F;
    // Hack for weird heading cases
    if (std::fabs(motion::motion_common::to_angle(p.start.state.heading)) > 3.0F) {
      heading_tol = 0.0001F;
    }
    EXPECT_EQ(progresses_towards_target(traj, p.target.state, heading_tol), traj.points.size());
    if (HasFailure()) {
      //planner_.debug_print(std::cout);
      break;
    }

    t += std::chrono::milliseconds{100LL};
    start.header.stamp = time_utils::to_message(t);
    start.state = traj.points.front();
    environment_.add_ego_state(start);
  }
}

INSTANTIATE_TEST_CASE_P(
  constraint_free,
  sanity_checks_constraint_free,
  testing::Values(
    // Simple constant velocity
    ConstraintFree{make_state(0.0F, 0.0F, 0.0F, 1.0F, 0.0F, 0.0F, system_clock::now()),
      make_state(10.0F, 0.0F, 0.0F, 1.0F, 0.0F, 0.0F, system_clock::now())},
    // Accelerating
    ConstraintFree{make_state(0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, system_clock::now()),
      make_state(30.0F, 0.0F, 0.0F, 10.0F, 0.0F, 0.0F, system_clock::now())},
    // Stopping
    ConstraintFree{make_state(0.0F, 0.0F, 0.0F, 10.0F, 0.0F, 0.0F, system_clock::now()),
      make_state(30.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, system_clock::now())},
    // Turning
    ConstraintFree{make_state(0.0F, 0.0F, 0.0F, 1.0F, 0.0F, 0.0F, system_clock::now()),
      make_state(10.0F, 10.0F, 1.0F, 1.0F, 0.0F, 0.0F, system_clock::now())},
    ConstraintFree{make_state(0.0F, 0.0F, 0.0F, 1.0F, 0.0F, 0.0F, system_clock::now()),
      make_state(10.0F, -10.0F, -1.0F, 1.0F, 0.0F, 0.0F, system_clock::now())},
    // Heading singularity
    ConstraintFree{make_state(0.0F, 0.0F, -3.13F, 1.0F, 0.0F, 0.0F, system_clock::now()),
      make_state(-10.0F, 0.0F, 3.13F, 1.0F, 0.0F, 0.0F, system_clock::now())},
    ConstraintFree{make_state(0.0F, 0.0F, 3.13F, 1.0F, 0.0F, 0.0F, system_clock::now()),
      make_state(-10.0F, 0.0F, -3.13F, 1.0F, 0.0F, 0.0F, system_clock::now())}
));
