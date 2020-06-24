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

#ifndef PLANNING_COMMON__PLANNER_BASE_HPP_
#define PLANNING_COMMON__PLANNER_BASE_HPP_

#include <autoware_auto_msgs/msg/trajectory.hpp>
#include <planning_common/planning_context.hpp>
#include <planning_common/visibility_control.hpp>

namespace motion
{
namespace planning
{
namespace planning_common
{
/// \brief A bare base class for planning algorithms
class PLANNING_COMMON_PUBLIC PlannerBase
{
public:
  const Trajectory & plan(const PlanningContext & context);

  virtual ~PlannerBase() noexcept = default;

protected:
  virtual const Trajectory & plan_impl(const PlanningContext & context) = 0;

private:
  Trajectory m_empty_trajectory{};
};  // class PlannerBase
}  // namespace planning_common
}  // namespace planning
}  // namespace motion
#endif  // PLANNING_COMMON__PLANNER_BASE_HPP_
