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
#ifndef MOCK_PLANNER_NODE__MOCK_PLANNER_NODE_HPP_
#define MOCK_PLANNER_NODE__MOCK_PLANNER_NODE_HPP_

#include <mock_planner_node/visibility_control.hpp>
#include <mock_planner/mock_planner.hpp>
#include <planning_common_nodes/planner_base_node.hpp>

#include <string>

namespace motion
{
namespace planning
{
namespace mock_planner_node
{
class MOCK_PLANNER_NODE_PUBLIC MockPlannerNode : public planning_common_nodes::PlannerBaseNode
{
public:
  /// Parameter file constructor
  MockPlannerNode(const std::string & name, const std::string & ns);
  /// Explicit constructor
  MockPlannerNode(
    const std::string & name,
    const std::string & ns,
    const std::string & trajectory_topic,
    const std::string & ego_topic,
    const std::string & target_topic,
    const std::string & object_topic,
    const std::string & boundary_topic,
    const std::string & tf_topic,
    const std::string & diagnostic_topic,
    planning_common_nodes::ContextSource source,
    const planning_common::EnvironmentConfig & cfg,
    const mock_planner::PlannerConfig & planner_cfg);
};  // class MockPlannerNode
}  // namespace mock_planner_node
}  // namespace planning
}  // namespace motion

#endif  // MOCK_PLANNER_NODE__MOCK_PLANNER_NODE_HPP_
