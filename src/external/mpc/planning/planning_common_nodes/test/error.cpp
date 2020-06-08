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
#include <planning_common_nodes/planner_base_node.hpp>
#include <rclcpp/rclcpp.hpp>

#include <memory>

#include "publisher.hpp"

using motion::planning::planning_common_nodes::ContextSource;

struct PlanningTestError : public std::exception {};

class ErrorPlanner : public motion::planning::planning_common::PlannerBase
{
protected:
  const Trajectory & plan_impl(
    const motion::planning::planning_common::PlanningContext & context) override
  {
    (void)context;
    throw PlanningTestError{};
  }
};  // class ErrorPlanner

const auto ego_topic = "error_planner_test_state_topic";
const auto target_topic = "error_planner_test_target_topic";

class ErrorPlannerNode : public motion::planning::planning_common_nodes::PlannerBaseNode
{
public:
  ErrorPlannerNode()
  : PlannerBaseNode{
      "error_planner_test_node",
      "",
      "error_planner_test_trajectory_topic",
      ego_topic,
      target_topic,
      "error_planner_test_object_topic",
      "error_planner_test_boundary_topic",
      "error_planner_test_tf_topic",
      "error_planner_test_diagnostic_topic",
      ContextSource::TARGET,
      EnvironmentConfig{std::chrono::milliseconds{100LL}}
  }
  {
    set_planner(std::make_unique<ErrorPlanner>());
  }

  int error_count() const noexcept {return m_error_count;}

protected:
  void on_bad_compute(std::exception_ptr eptr) override
  {
    try {
      std::rethrow_exception(eptr);
    } catch (const PlanningTestError &) {
      ++m_error_count;
    } catch (...) {
      throw;
    }
  }

private:
  int m_error_count{0};
};  // class PlannerBaseNode

class error : public ::testing::Test
{
protected:
  void SetUp()
  {
    rclcpp::init(0, nullptr);
  }
  void TearDown()
  {
    (void)rclcpp::shutdown();
  }
};  // class error

TEST_F(error, basic)
{
  const auto pub = std::make_shared<TestPublisher>(
    "test_planner_base_node_error",
    PeriodCountTopic{
    std::chrono::milliseconds{50},
    10U,
    ego_topic
  },
    PeriodCountTopic{
    std::chrono::milliseconds{60},
    9U,
    target_topic
  }
  );
  const auto planner_ = std::make_shared<ErrorPlannerNode>();
  pub->match();
  rclcpp::executors::SingleThreadedExecutor exec;
  exec.add_node(planner_);
  exec.add_node(pub);
  while (!pub->done()) {
    EXPECT_NO_THROW(exec.spin_some(std::chrono::milliseconds(100LL)));
  }
  // spin one more time for good measure
  EXPECT_NO_THROW(exec.spin_some(std::chrono::milliseconds(100LL)));
  const auto error_count = planner_->error_count();
  EXPECT_GE(error_count + 1, 9);
  // TODO(c.ho) check error count against number of sent things
}
