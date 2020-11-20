// Copyright 2020 The Autoware Foundation
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "behavior_planner/trajectory_manager.hpp"
#include <geometry/common_2d.hpp>
#include <motion_common/motion_common.hpp>

#include <algorithm>
#include <limits>

namespace autoware
{
namespace behavior_planner
{

using autoware::common::geometry::minus_2d;
using autoware::common::geometry::norm_2d;
using motion::motion_common::to_angle;

TrajectoryManager::TrajectoryManager(const PlannerConfig & config)
: m_config(config)
{
}

void TrajectoryManager::clear_trajectory()
{
  m_trajectory.points.clear();
  m_sub_trajectories.clear();
  m_selected_trajectory = 0;
}

void TrajectoryManager::set_trajectory(const Trajectory & trajectory)
{
  clear_trajectory();
  m_trajectory = trajectory;
  set_sub_trajectories();
}

void TrajectoryManager::set_sub_trajectories()
{
  const auto is_positive = [](const TrajectoryPoint & pt) {
      // using epsilon instead to ensure change in sign.
      return pt.longitudinal_velocity_mps > -std::numeric_limits<float32_t>::epsilon();
    };

  if (m_trajectory.points.empty()) {
    m_sub_trajectories.push_back(m_trajectory);
    return;
  }

  Trajectory sub_trajectory;
  sub_trajectory.header = m_trajectory.header;

  auto prev_sign = is_positive(m_trajectory.points.front());
  for (auto & pt : m_trajectory.points) {
    const auto sign = is_positive(pt);
    if (prev_sign != sign && !sub_trajectory.points.empty()) {
      m_sub_trajectories.push_back(sub_trajectory);
      sub_trajectory.points.clear();
    }
    sub_trajectory.points.push_back(pt);
    prev_sign = sign;
  }
  if (!sub_trajectory.points.empty()) {
    m_sub_trajectories.push_back(sub_trajectory);
  }
}

bool8_t TrajectoryManager::is_trajectory_ready()
{
  return !m_sub_trajectories.empty();
}

std::size_t TrajectoryManager::get_closest_state(
  const State & current_state,
  const Trajectory & trajectory)
{
  const auto distance_from_current_state =
    [this, &current_state](const TrajectoryPoint & other_state) {
      const auto s1 = current_state.state, s2 = other_state;
      const auto distance = norm_2d(minus_2d(s1, s2));
      const auto angle_diff = std::abs(to_angle(s1.heading - s2.heading));
      return distance + m_config.heading_weight * angle_diff;
    };

  const auto comparison_function =
    [&distance_from_current_state](const TrajectoryPoint & one, const TrajectoryPoint & two)
    {return distance_from_current_state(one) < distance_from_current_state(two);};

  const auto minimum_index_iterator =
    std::min_element(
    std::begin(trajectory.points), std::end(trajectory.points),
    comparison_function);
  auto minimum_idx = std::distance(std::begin(trajectory.points), minimum_index_iterator);

  return static_cast<std::size_t>(minimum_idx);
}

size_t TrajectoryManager::get_remaining_length(const State & state)
{
  // remaining length of current selected sub trajectory
  const auto & current_trajectory = m_sub_trajectories.at(m_selected_trajectory);
  const size_t closest_index = get_closest_state(state, current_trajectory);
  size_t remaining_length = current_trajectory.points.size() - closest_index;

  // remaining length including rest of sub trajectories
  for (size_t i = m_selected_trajectory + 1; i < m_sub_trajectories.size(); i++) {
    remaining_length += m_sub_trajectories.at(i).points.size();
  }

  return remaining_length;
}

Trajectory TrajectoryManager::get_trajectory(const State & state)
{
  // select new sub_trajectory when vehicle is at stop
  if (state.state.longitudinal_velocity_mps < m_config.stop_velocity_thresh) {
    const auto & last_point = m_sub_trajectories.at(m_selected_trajectory).points.back();
    const auto distance = norm_2d(minus_2d(last_point, state.state));

    // increment index to select new sub_trajectory if vehicle has arrived the end of sub_trajectory
    if (distance < m_config.goal_distance_thresh) {
      m_selected_trajectory++;
      m_selected_trajectory = std::min(m_selected_trajectory, m_sub_trajectories.size() - 1);
    }
  }

  // TODO(mitsudome-r) implement trajectory refine functions if needed to integrate with controller
  // output = crop_form_current_state(input);
  // output = set_time_from_start(output);
  // output = interpolation();
  return m_sub_trajectories.at(m_selected_trajectory);
}

}  // namespace behavior_planner
}  // namespace autoware
