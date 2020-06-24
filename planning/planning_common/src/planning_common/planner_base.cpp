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
#include "planning_common/planner_base.hpp"
#include <motion_common/motion_common.hpp>

namespace motion
{
namespace planning
{
namespace planning_common
{
const Trajectory & PlannerBase::plan(const PlanningContext & context)
{
  const auto & ego = context.ego_state();
  const auto & target = context.target_state();
  const auto past_target = motion_common::is_past_point(ego.state, target.state);
  constexpr auto dot_threshold = 0.5F;   // cosine 60 degrees; hardcoded for now
  const auto aligned_target =
    motion_common::is_aligned(ego.state.heading, target.state.heading, dot_threshold);
  if (past_target && aligned_target) {
    m_empty_trajectory.header = context.ego_state().header;
    m_empty_trajectory.points.clear();  // just to be sure
    return m_empty_trajectory;
  }
  return plan_impl(context);
}
}  // namespace planning_common
}  // namespace planning
}  // namespace motion
