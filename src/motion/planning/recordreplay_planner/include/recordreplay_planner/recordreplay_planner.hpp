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

#ifndef RECORDREPLAY_PLANNER__RECORDREPLAY_PLANNER_HPP_
#define RECORDREPLAY_PLANNER__RECORDREPLAY_PLANNER_HPP_

#include <recordreplay_planner/visibility_control.hpp>

#include <ostream>

namespace motion
{
namespace planning
{
namespace recordreplay_planner
{

/// \brief A class for recording trajectories and replaying them as plans
class RECORDREPLAY_PLANNER_PUBLIC RecordReplayPlanner
{
public:
  RecordReplayPlanner();
  void some_public_method();

protected:
  void some_protected_method();

private:
  RECORDREPLAY_PLANNER_LOCAL void some_private_method();
};  // class PlannerBase
}  // namespace recordreplay_planner
}  // namespace planning
}  // namespace motion
#endif  // RECORDREPLAY_PLANNER__RECORDREPLAY_PLANNER_HPP_
