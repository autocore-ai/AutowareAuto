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

RecordReplayPlanner::RecordReplayPlanner()
{
  // Does nothing yet
}


// These may do more in the future
bool RecordReplayPlanner::is_recording() noexcept
{
  return m_recordreplaystate == RecordReplayState::RECORDING;
}

bool RecordReplayPlanner::is_replaying() noexcept
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

uint32_t RecordReplayPlanner::get_record_length() noexcept
{
  return m_record_buffer.size();
}

void RecordReplayPlanner::record_state(const State & state_to_record)
{
  m_record_buffer.push_back(state_to_record);
}

const Trajectory & RecordReplayPlanner::plan(const State & current_state)
{
  return from_record(current_state.header);
}


// TODO(s.me) this currently just creates a single trajectory from the entire
// record. This will not work for longer recordings and does not fit the receding
// horizon idea.
//
// Another issue is whether it has to be resampled in time or if the data rates are
// constant enough so that this is not an issue.
const Trajectory & RecordReplayPlanner::from_record(const std_msgs::msg::Header & header)
{
  auto & trajectory = m_trajectory;
  const auto record_length = get_record_length();

  // Determine how long the published trajectory will be
  auto publication_length =
    std::min(record_length, static_cast<uint32_t>(trajectory.points.max_size()));

  // Assemble the trajectory as desired
  trajectory.points.resize(publication_length);
  trajectory.header = header;
  auto t0 = time_utils::from_message(m_record_buffer[0].header.stamp);
  for (std::size_t i = {}; i < publication_length; ++i) {
    auto & point = m_trajectory.points[i];

    // Make the time spacing of the points match the recorded timing
    trajectory.points[i] = m_record_buffer[i].state;
    trajectory.points[i].time_from_start = time_utils::to_message(
      time_utils::from_message(m_record_buffer[i].header.stamp) - t0);
  }

  // Remove the first point from the trajectory buffer to emulate a receding horizon
  // publication of the trajectory. TODO(s.me) this is easy, but won't allow multiple
  // playbacks of the same recorded trajectory.
  if (publication_length > 0) {
    m_record_buffer.pop_front();
  }

  return trajectory;
}

}  // namespace recordreplay_planner
}  // namespace planning
}  // namespace motion
