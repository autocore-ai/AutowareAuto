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
#include <autoware_auto_msgs/msg/vehicle_kinematic_state.hpp>
#include <autoware_auto_msgs/msg/trajectory.hpp>

#include <ostream>
#include <vector>

namespace motion
{
namespace planning
{
namespace recordreplay_planner
{
using State = autoware_auto_msgs::msg::VehicleKinematicState;
using Trajectory = autoware_auto_msgs::msg::Trajectory;

/// \brief A class for recording trajectories and replaying them as plans
class RECORDREPLAY_PLANNER_PUBLIC RecordReplayPlanner
{
public:
  RecordReplayPlanner();
  // Clear the internal recording buffer
  void clear_record() noexcept;

  // Add a new state to the record
  void record_state(const State & state_to_record);

  // Replay trajectory from stored plan. 
  const Trajectory & plan();

  // Return the number of currently-recorded State messages
  const uint32_t get_record_length() noexcept;

private:
  // Obtain a trajectory from the internally-stored recording buffer
  RECORDREPLAY_PLANNER_LOCAL const Trajectory & from_record(const std_msgs::msg::Header & header);

  std::vector<State> m_record_buffer;
  Trajectory m_trajectory{};
};  // class PlannerBase
}  // namespace recordreplay_planner
}  // namespace planning
}  // namespace motion
#endif  // RECORDREPLAY_PLANNER__RECORDREPLAY_PLANNER_HPP_
