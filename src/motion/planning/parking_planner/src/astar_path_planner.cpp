// Copyright 2020 Embotech AG, Zurich, Switzerland. All rights reserved.
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
#include <common/types.hpp>
#include <cmath>
#include <vector>
#include <utility>
#include <queue>
#include <unordered_map>
#include <algorithm>

#include "parking_planner/astar_path_planner.hpp"
#include "parking_planner/parking_planner_types.hpp"
#include "parking_planner/geometry.hpp"

namespace autoware
{

namespace motion
{

namespace planning
{

namespace parking_planner
{

using autoware::common::types::float64_t;

static std::vector<VehicleState<float64_t>> expand_state_longitudinal_with_heading(
  const VehicleState<float64_t> & from_state)
{
  const float64_t from_x = from_state.get_x();
  const float64_t from_y = from_state.get_y();
  const float64_t from_heading = from_state.get_heading();
  const float64_t delta_x = cos(from_heading) * parking_planner::DELTA_LONGITUDINAL;
  const float64_t delta_y = sin(from_heading) * parking_planner::DELTA_LONGITUDINAL;
  const float64_t heading_minus = remainder(from_heading - parking_planner::DELTA_HEADING,
      MY_PI * 2.0);
  const float64_t heading_plus = remainder(from_heading + parking_planner::DELTA_HEADING,
      MY_PI * 2.0);
  std::vector<VehicleState<float64_t>> to_states(6, from_state);
  to_states[0].set_x(from_x - delta_x);
  to_states[0].set_y(from_y - delta_y);
  to_states[0].set_heading(heading_minus);
  to_states[1].set_x(from_x - delta_x);
  to_states[1].set_y(from_y - delta_y);
  to_states[1].set_heading(from_heading);
  to_states[2].set_x(from_x - delta_x);
  to_states[2].set_y(from_y - delta_y);
  to_states[2].set_heading(heading_plus);
  to_states[3].set_x(from_x + delta_x);
  to_states[3].set_y(from_y + delta_y);
  to_states[3].set_heading(heading_minus);
  to_states[4].set_x(from_x + delta_x);
  to_states[4].set_y(from_y + delta_y);
  to_states[4].set_heading(from_heading);
  to_states[5].set_x(from_x + delta_x);
  to_states[5].set_y(from_y + delta_y);
  to_states[5].set_heading(heading_plus);
  return to_states;
}

static bool check_collision_bounding_box_vs_obstacles(
  const VehicleState<float64_t> & vehicle_state,
  const Polytope2D<float64_t> & bounding_box,
  const std::vector<Polytope2D<float64_t>> & obstacles)
{
  Polytope2D<float64_t> my_box = bounding_box;  // create a copy before shift-and-rotate
  my_box.rotate_and_shift(vehicle_state.get_heading(), Point2D<float64_t>(),
    Point2D<float64_t>(vehicle_state.get_x(), vehicle_state.get_y()));
  for (const auto & obst : obstacles) {
    if (my_box.intersects_with(obst)) {
      return true;
    }
  }
  return false;
}

static uint64_t map_state_on_discretized_grid(
  const VehicleState<float64_t> & state,
  const VehicleState<float64_t> & reference)
{
  static constexpr uint64_t num_position_steps = static_cast<uint64_t>(round(
      parking_planner::MAX_EXPLORATION_RADIUS / parking_planner::DELTA_LONGITUDINAL)) * 2 + 1;
  uint64_t x_quant =
    static_cast<uint64_t>(round(
      (state.get_x() - reference.get_x() + parking_planner::MAX_EXPLORATION_RADIUS) /
      parking_planner::DELTA_LONGITUDINAL));
  uint64_t y_quant =
    static_cast<uint64_t>(round(
      (state.get_y() - reference.get_y() + parking_planner::MAX_EXPLORATION_RADIUS) /
      parking_planner::DELTA_LONGITUDINAL));
  uint64_t h_quant =
    static_cast<uint64_t>(round((remainder(state.get_heading() - reference.get_heading(),
    parking_planner::MY_PI * 2.0) + parking_planner::MY_PI) / parking_planner::DELTA_HEADING));
  return h_quant * num_position_steps * num_position_steps + y_quant * num_position_steps + x_quant;
}

std::vector<VehicleState<float64_t>> AstarPathPlanner::plan_astar(
  const VehicleState<float64_t> & current_state,
  const VehicleState<float64_t> & goal_state,
  const Polytope2D<float64_t> & vehicle_bounding_box,
  const std::vector<Polytope2D<float64_t>> & obstacles) const
{
  // Prepare data structure definitions
  using StateTransition = std::pair<std::pair<VehicleState<float64_t>, VehicleState<float64_t>>,
      std::pair<uint64_t, uint64_t>>;
  using CostPair = std::pair<float64_t, float64_t>;
  using QueueElement = std::pair<CostPair, StateTransition>;

  auto my_compare_queue_element =
    [](const QueueElement & e1, const QueueElement & e2)
    {
      return e1.first > e2.first;
    };

  using OpenSet = std::priority_queue<QueueElement,
      std::vector<QueueElement>,
      decltype(my_compare_queue_element)>;

  using ClosedSet = std::unordered_map<uint64_t,
      StateTransition>;

  OpenSet open_set(my_compare_queue_element);
  ClosedSet closed_set;

  // Initialize data structures for the given problem data
  Point2D<float64_t> vect_current_to_goal = Point2D<float64_t>(current_state.get_x(),
      current_state.get_y()) -
    Point2D<float64_t>(goal_state.get_x(), goal_state.get_y());
  float64_t dist_current_to_goal = vect_current_to_goal.norm2();
  uint64_t current_discrete = map_state_on_discretized_grid(current_state, goal_state);
  uint64_t goal_discrete = map_state_on_discretized_grid(goal_state, goal_state);
  open_set.push({{dist_current_to_goal, 0},
      {{current_state, current_state},
        {current_discrete, current_discrete}}});

  // Run the main exploration loop
  size_t num_nodes_left = parking_planner::MAX_NUM_EXPLORATION_NODES;
  while (!open_set.empty() && (0 != num_nodes_left--)) {
    QueueElement top_element = open_set.top();
    open_set.pop();
    const float64_t & f_cost = top_element.first.second;
    const VehicleState<float64_t> & to_state = top_element.second.first.second;
    const uint64_t & to_discrete = top_element.second.second.second;

    if (!closed_set.count(to_discrete)) {
      closed_set[to_discrete] = top_element.second;
      if (to_discrete == goal_discrete) {
        break;
      }

      if (!check_collision_bounding_box_vs_obstacles(to_state, vehicle_bounding_box, obstacles)) {
        const std::vector<VehicleState<float64_t>> expanded_states =
          expand_state_longitudinal_with_heading(to_state);
        for (const VehicleState<float64_t> & next_state : expanded_states) {
          uint64_t next_discrete = map_state_on_discretized_grid(next_state, goal_state);
          if (!closed_set.count(next_discrete)) {
            float64_t next_f_cost = f_cost + parking_planner::DELTA_LONGITUDINAL;
            Point2D<float64_t> vect_to_goal = Point2D<float64_t>(
              next_state.get_x(), next_state.get_y()) -
              Point2D<float64_t>(goal_state.get_x(), goal_state.get_y());
            float64_t dist_to_goal = vect_to_goal.norm2();
            if (dist_to_goal < parking_planner::MAX_EXPLORATION_RADIUS) {
              float64_t next_g_cost = f_cost + dist_to_goal;
              open_set.push({{next_g_cost, next_f_cost},
                  {{to_state, next_state},
                    {to_discrete, next_discrete}}});
            }
          }
        }
      }
    }
  }

  // Done exploring, assemble the output path from the results
  std::vector<VehicleState<float64_t>> result;
  uint64_t destination_discrete = goal_discrete;
  while (0 != closed_set.count(destination_discrete) &&
    (destination_discrete != current_discrete))
  {
    result.emplace_back(closed_set[destination_discrete].first.first);
    destination_discrete = closed_set[destination_discrete].second.first;
  }
  result.push_back(current_state);
  std::reverse(result.begin(), result.end());
  return result;
}

}  // namespace parking_planner
}  // namespace planning
}  // namespace motion
}  // namespace autoware
