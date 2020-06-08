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

#include <algorithm>
#include <memory>
#include <vector>

#include "publisher.hpp"

using motion::planning::planning_common_nodes::ContextSource;

class SourcePlanner : public motion::planning::planning_common::PlannerBase
{
protected:
  const Trajectory & plan_impl(
    const motion::planning::planning_common::PlanningContext & context) override
  {
    (void)context;
    m_ret.points.resize(1U);
    return m_ret;
  }

private:
  Trajectory m_ret{};
};  // class SourcePlanner

const auto traj_topic = "source_planner_test_trajectory_topic";
const auto ego_topic = "source_planner_test_state_topic";
const auto target_topic = "source_planner_test_target_topic";

class SourcePlannerNode : public motion::planning::planning_common_nodes::PlannerBaseNode
{
public:
  explicit SourcePlannerNode(ContextSource source)
  : PlannerBaseNode{
      "source_planner_test_node",
      "",
      traj_topic,
      ego_topic,
      target_topic,
      "source_planner_test_object_topic",
      "source_planner_test_boundary_topic",
      "source_planner_test_tf_topic",
      "source_planner_test_diagnostic_topic",
      source,
      EnvironmentConfig{std::chrono::milliseconds{100LL}}
  }
  {
    set_planner(std::make_unique<SourcePlanner>());
  }
};  // class PlannerBaseNode

class source : public ::testing::TestWithParam<ContextSource>
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

TEST_P(source, basic)
{
  const auto ego_count = 13U;
  const auto target_count = 9U;
  const auto pub = std::make_shared<TestPublisher>(
    "test_planner_base_node_error",
    PeriodCountTopic{
    std::chrono::milliseconds{40},
    ego_count,
    ego_topic
  },
    PeriodCountTopic{
    std::chrono::milliseconds{60},
    target_count,
    target_topic
  }
  );
  const auto listener =
    std::make_shared<rclcpp::Node>("planning_common_nodes_source_test_listener");
  const auto max_count = std::max(ego_count, target_count);
  std::vector<Trajectory> trajectories{};
  trajectories.reserve(max_count);
  const auto sub = listener->create_subscription<Trajectory>(traj_topic,
      rclcpp::QoS{max_count}.reliable(),
      [&trajectories](const Trajectory::SharedPtr msg) -> void {
        trajectories.push_back(*msg);
      });
  const auto source = GetParam();
  const auto planner_ = std::make_shared<SourcePlannerNode>(source);
  pub->match();
  (void)::motion::motion_testing_nodes::wait_for_matched(*sub, 1U);
  rclcpp::executors::SingleThreadedExecutor exec;
  exec.add_node(planner_);
  exec.add_node(pub);
  exec.add_node(listener);
  while (!pub->done()) {
    EXPECT_NO_THROW(exec.spin_some(std::chrono::milliseconds(100LL)));
  }
  // spin one more time for good measure
  EXPECT_NO_THROW(exec.spin_some(std::chrono::milliseconds(100LL)));
  const auto count = [source]() {
      switch (source) {
        case ContextSource::EGO:
          return ego_count;
        case ContextSource::TARGET:
          return target_count;
        default:
          throw std::runtime_error{"Not supported"};
      }
    } ();
  EXPECT_GE(trajectories.size() + 1U, count);
  for (const auto & traj : trajectories) {
    EXPECT_EQ(traj.points.size(), 1U);
  }
}

INSTANTIATE_TEST_CASE_P(
  foo,
  source,
  testing::Values(
    ContextSource::EGO,
    ContextSource::TARGET
    // ContextSource::OBJECT,
    // ContextSource::BOUNDARY
  )
);
