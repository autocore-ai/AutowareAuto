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
#include "recordreplay_planner/recordreplay_planner.hpp"
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

void RecordReplayPlanner::clear_record() noexcept
{
  m_record_buffer.clear();
}

void RecordReplayPlanner::record_state(const State & state_to_record)
{
  m_record_buffer.push_back(state_to_record);
}

const uint32_t RecordReplayPlanner::get_record_length() noexcept
{
  return m_record_buffer.size();
}

// TODO(s.me) this currently just creates a single trajectory from the entire
// record. This will not work for longer recordings, we'll need to create the
// trajectories in a receding horizon way from the record.
//
// Another issue is whether it has to be resampled in time.
const Trajectory & RecordReplayPlanner::from_record(const std_msgs::msg::Header & header)
{
  auto & traj = m_trajectory;
  const auto record_length = get_record_length();
  traj.points.resize(record_length);  // TODO(s.me) this will fail if record_length > 100
  traj.header = header;
  auto t0 = time_utils::from_message(m_record_buffer[0].header.stamp);
  for (std::size_t i = {}; i < record_length; ++i) {
    auto & pt = m_trajectory.points[i];

    // Make the time spacing of the points as they were recorded
    pt.time_from_start = time_utils::to_message(
      time_utils::from_message(m_record_buffer[i].header.stamp) - t0);
    traj.points[i] = m_record_buffer[i].state;
  }

  return traj;
}

}  // namespace recordreplay_planner
}  // namespace planning
}  // namespace motion
