// Copyright 2020 Sandro Merkli, inspired by Christopher Ho's mpc code
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
#include "recordreplay_planner_node/recordreplay_planner_node.hpp"

#include <memory>
#include <string>
#include <utility>

namespace motion
{
namespace planning
{
namespace recordreplay_planner_node
{
RecordReplayPlannerNode::RecordReplayPlannerNode(const std::string & name, const std::string & ns)
: Node{name, ns}
{
  set_planner(std::make_unique<recordreplay_planner::RecordReplayPlanner>());
}
////////////////////////////////////////////////////////////////////////////////
RecordReplayPlannerNode::RecordReplayPlannerNode(
  const std::string & name,
  const std::string & ns,
  const std::string & some_topic,
  const std::string & tf_topic,
  const std::string & diagnostic_topic)
: Node{name, ns}
{
  set_planner(std::make_unique<recordreplay_planner::RecordReplayPlanner>());
}

void RecordReplayPlannerNode::set_planner(PlannerPtr && planner) noexcept
{
  m_planner = std::forward<PlannerPtr &&>(planner);
}
}  // namespace recordreplay_planner_node
}  // namespace planning
}  // namespace motion
