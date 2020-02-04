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
#include "recordreplay_planner/recordreplay_planner.hpp"

#include <algorithm>

#include "time_utils/time_utils.hpp"


namespace motion
{
namespace planning
{
namespace recordreplay_planner
{

// These may do more in the future
bool RecordReplayPlanner::is_recording() const noexcept
{
  return m_recordreplaystate == RecordReplayState::RECORDING;
}

bool RecordReplayPlanner::is_replaying() const noexcept
{
  return m_recordreplaystate == RecordReplayState::REPLAYING;
}

void RecordReplayPlanner::start_recording() noexcept
{
  m_recordreplaystate = RecordReplayState::RECORDING;
}

void RecordReplayPlanner::stop_recording() noexcept
{
  m_recordreplaystate = RecordReplayState::IDLE;
}

void RecordReplayPlanner::start_replaying() noexcept
{
  m_recordreplaystate = RecordReplayState::REPLAYING;
}

void RecordReplayPlanner::stop_replaying() noexcept
{
  m_recordreplaystate = RecordReplayState::IDLE;
}

void RecordReplayPlanner::clear_record() noexcept
{
  m_record_buffer.clear();
}

uint32_t RecordReplayPlanner::get_record_length() const noexcept
{
  return m_record_buffer.size();
}

void RecordReplayPlanner::record_state(const State & state_to_record)
{
  m_record_buffer.push_back(state_to_record);
}

const Trajectory & RecordReplayPlanner::plan(const State & current_state)
{
  return from_record(current_state);
}


// TODO(s.me) this currently just creates a single trajectory from the entire
// record. This will not work for longer recordings and does not fit the receding
// horizon idea.
//
// Another issue is whether it has to be resampled in time or if the data rates are
// constant enough so that this is not an issue.
const Trajectory & RecordReplayPlanner::from_record(const State & current_state)
{
  auto & trajectory = m_trajectory;
  const auto record_length = get_record_length();

  // Find the closest point to the current state in the stored states buffer
  const auto distance_from_current_state =
    [&current_state](State & other_state) {
      auto s1 = current_state.state, s2 = other_state.state;
      return (s1.x - s2.x) * (s1.x - s2.x) + (s1.y - s2.y) * (s1.y - s2.y);
    };

  ssize_t minimum_idx = 0;
  double minimum_distance = distance_from_current_state(m_record_buffer[0]);
  for (auto k = 1; k < record_length; ++k) {
    const auto d = distance_from_current_state(m_record_buffer[k]);
    if (d < minimum_distance) {
      minimum_distance = d;
      minimum_idx = k;
    }
  }

  // Determine how long the published trajectory will be
  const auto publication_length =
    std::min(record_length - minimum_idx, static_cast<ssize_t>(trajectory.points.max_size()));

  // Assemble the trajectory as desired
  trajectory.points.resize(publication_length);
  trajectory.header = current_state.header;
  const auto t0 = time_utils::from_message(m_record_buffer[minimum_idx].header.stamp);
  for (std::size_t i = {}; i < publication_length; ++i) {
    // Make the time spacing of the points match the recorded timing
    trajectory.points[i] = m_record_buffer[minimum_idx + i].state;
    trajectory.points[i].time_from_start = time_utils::to_message(
      time_utils::from_message(m_record_buffer[minimum_idx + i].header.stamp) - t0);
  }

  return trajectory;
}

}  // namespace recordreplay_planner
}  // namespace planning
}  // namespace motion
