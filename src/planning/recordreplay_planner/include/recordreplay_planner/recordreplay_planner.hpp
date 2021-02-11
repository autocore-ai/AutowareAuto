// Copyright 2020 Embotech AG, Zurich, Switzerland, inspired by Christopher Ho's mpc code
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
#include <motion_common/config.hpp>
#include <common/types.hpp>

#include <deque>
#include <string>
#include <map>

using autoware::common::types::bool8_t;
using autoware::common::types::float64_t;

namespace motion
{
namespace planning
{
namespace recordreplay_planner
{
using State = autoware_auto_msgs::msg::VehicleKinematicState;
using autoware_auto_msgs::msg::Trajectory;
using Heading = decltype(decltype(State::state)::heading);

enum class RecordReplayState
{
  IDLE,
  RECORDING,
  REPLAYING
};  // enum class RecordReplayState

/// \brief A class for recording trajectories and replaying them as plans
class RECORDREPLAY_PLANNER_PUBLIC RecordReplayPlanner
{
public:
  RecordReplayPlanner();

  // Record and replay control
  bool8_t is_recording() const noexcept;
  bool8_t is_replaying() const noexcept;
  void start_recording() noexcept;
  void stop_recording() noexcept;
  void start_replaying() noexcept;
  void stop_replaying() noexcept;

  // Clear the internal recording buffer
  void clear_record() noexcept;

  /// \brief Add a new state to the record buffer
  /// \param[in] state_to_record A state to attempt to add to the recording buffer
  /// \return True if state was added to record buffer, False otherwise
  bool record_state(const State & state_to_record);

  // Replay trajectory from stored plan. The current state of the vehicle is given
  // and the trajectory will be chosen from the stored plan such that the starting
  // point of the trajectory is as close as possible to this current state.
  const Trajectory & plan(const State & current_state);

  // Return the number of currently-recorded State messages
  std::size_t get_record_length() const noexcept;

  // Heading weight configuration
  void set_heading_weight(float64_t heading_weight);
  float64_t get_heading_weight();

  void set_min_record_distance(float64_t min_record_distance);
  float64_t get_min_record_distance() const;

  // Writing/Loading buffered trajectory information to/from disk
  void writeTrajectoryBufferToFile(const std::string & record_path);
  void readTrajectoryBufferFromFile(const std::string & replay_path);

private:
  // Obtain a trajectory from the internally-stored recording buffer
  RECORDREPLAY_PLANNER_LOCAL const Trajectory & from_record(const State & current_state);
  RECORDREPLAY_PLANNER_LOCAL std::size_t get_closest_state(const State & current_state);

  // Weight of heading in computations of differences between states
  float64_t m_heading_weight = 0.1;
  float64_t m_min_record_distance = 0.0;

  std::size_t m_traj_start_idx{};
  std::size_t m_traj_end_idx{};
  std::deque<State> m_record_buffer;
  Trajectory m_trajectory{};
  RecordReplayState m_recordreplaystate{RecordReplayState::IDLE};
};  // class RecordReplayPlanner
}  // namespace recordreplay_planner
}  // namespace planning
}  // namespace motion
#endif  // RECORDREPLAY_PLANNER__RECORDREPLAY_PLANNER_HPP_
