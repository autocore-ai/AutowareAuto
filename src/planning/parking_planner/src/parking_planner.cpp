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

// This file contains the "main entry point" of the parking planner in the form of
// a class "ParkingPlanner" and its main method, "plan" (at the end of the file).
#include <common/types.hpp>
#include <iostream>
#include <vector>
#include <limits>

#include "parking_planner/configuration.hpp"
#include "parking_planner/parking_planner.hpp"
#include "parking_planner/parking_planner_types.hpp"
#include "parking_planner/astar_path_planner.hpp"

namespace autoware
{

namespace motion
{

namespace planning
{

namespace parking_planner
{

using autoware::common::types::float64_t;

PlanningResult::PlanningResult(
  const Trajectory<float64_t> trajectory,
  const std::size_t nlp_iterations,
  const float64_t nlp_proc_time,
  const PlanningStatus status)
{
  m_trajectory = trajectory;
  m_nlp_iterations = nlp_iterations;
  m_nlp_proc_time = nlp_proc_time;
  m_status = status;
}


// ---- Initial trajectory guess computation that goes into NLP -----------------------------
static std::vector<VehicleState<float64_t>> resize_state_vector(
  const std::vector<VehicleState<float64_t>> & states_input,
  const std::size_t target_length)
{
  // Make the state trajectory as long as we need the output to be. This is done in a
  // somewhat naïve fashion, because it works well enough.

  // - If length matches, we just make an owned copy to make the interface the same
  //   as for the modified-length-versions
  const auto input_length = states_input.size();
  if (input_length == target_length) {
    return std::vector<VehicleState<float64_t>>(states_input);
  }

  // - Length does not match
  std::vector<VehicleState<float64_t>> resized_states{};
  if (input_length < target_length) {  // Grow the vector to the target length
    // Copy existing trajectory
    resized_states.insert(resized_states.end(), states_input.begin(), states_input.end() );

    // Repeat the last entry as many times as needed
    for (std::size_t k = {}; k < (target_length - input_length); k++) {
      resized_states.push_back(states_input.back() );
    }

  } else if (input_length > target_length) {  // Shrink the vector to the target length
    const std::size_t number_of_points_to_skip = input_length - target_length;

    // Pick a stride that's big enough to allow us to skip the necessary number of points
    std::size_t skip_length = 1;
    while (skip_length * target_length < input_length) {
      skip_length += 1;
    }

    // Skip points with the given skip length until we're close to the end
    auto read_iter = states_input.begin();
    std::size_t number_of_skipped_points = 0;
    for (; number_of_skipped_points < (number_of_points_to_skip - skip_length);
      number_of_skipped_points += skip_length)
    {
      resized_states.push_back(*read_iter);
      read_iter += static_cast<int64_t>(skip_length + 1);
    }

    // The final amount of points to skip is less than skip_length now
    read_iter += static_cast<int64_t>(number_of_points_to_skip - number_of_skipped_points);

    // Copy over the rest normally
    for (; read_iter != states_input.end(); read_iter++) {
      resized_states.push_back(*read_iter);
    }
  }

  // float64_t-check if the resizing worked properly
  if (resized_states.size() != target_length) {
    throw std::length_error("Resize code resized to the wrong length");
  }

  return resized_states;
}

Trajectory<float64_t> ParkingPlanner::create_trajectory_from_states(
  const std::vector<VehicleState<float64_t>> & states_input,
  const std::size_t desired_trajectory_length) const
{
  // Create a version of the states vector that is of the length we need
  const auto resized_states = resize_state_vector(states_input, desired_trajectory_length);

  // Splice those states into a Trajectory along with a guess of "the commands are just zero"
  Trajectory<float64_t> trajectory{};
  for (const auto & state : resized_states) {
    const auto input_guess = VehicleCommand<float64_t>{};
    trajectory.push_back(TrajectoryStep<float64_t>(input_guess, state));
  }

  return trajectory;
}


// ---- Main parking planner interface ------------------------------------------------------
ParkingPlanner::ParkingPlanner(
  const BicycleModelParameters<float64_t> & parameters,
  const NLPCostWeights<float64_t> & nlp_weights,
  const VehicleState<float64_t> & lower_state_bounds,
  const VehicleState<float64_t> & upper_state_bounds,
  const VehicleCommand<float64_t> & lower_command_bounds,
  const VehicleCommand<float64_t> & upper_command_bounds
)
: m_nlp_planner(nlp_weights, lower_state_bounds, upper_state_bounds,
    lower_command_bounds, upper_command_bounds), m_model_parameters(parameters)
{
  m_astar_planner = AstarPathPlanner();
}


PlanningResult ParkingPlanner::plan(
  const VehicleState<float64_t> & current_state,
  const VehicleState<float64_t> & goal_state,
  const std::vector<Polytope2D<float64_t>> & obstacles
) const
{
  // If the starting and target angles would be closer if we shift by 2*pi in either
  // direction, do that. This will help avoid the planner coming up with "loop" solutions.
  // TODO(s.me) this should be cleaned up after AVP
  const auto goal_heading = goal_state.get_heading();
  const auto current_heading = current_state.get_heading();
  const auto current_heading_difference = std::abs(goal_heading - current_heading);
  VehicleState<float64_t> adapted_goal_state = goal_state;
  const auto pi = 3.14159;
  if (std::abs(goal_heading - (2 * pi) - current_heading) < current_heading_difference) {
    adapted_goal_state.set_heading(goal_heading - (2 * pi) );
  } else if (std::abs(goal_heading + (2 * pi) - current_heading) < current_heading_difference) {
    adapted_goal_state.set_heading(goal_heading + (2 * pi) );
  }

  // Run discretized global A* planner
  const auto vehicle_bounding_box =
    BicycleModel<float64_t,
      float64_t>(m_model_parameters).compute_bounding_box(VehicleState<float64_t>{});
  const std::vector<VehicleState<float64_t>> astar_output =
    m_astar_planner.plan_astar(
    current_state,
    adapted_goal_state,
    vehicle_bounding_box,
    obstacles);

  // Translate A* planner output to a trajectory with commands
  const auto trajectory_guess = this->create_trajectory_from_states(astar_output, HORIZON_LENGTH);

  // Run NLP smoother, warm-started using the A* guess
  auto nlp_results = m_nlp_planner.plan_nlp(
    current_state, adapted_goal_state, trajectory_guess,
    obstacles, m_model_parameters);
  const auto smoothed_trajectory = nlp_results.m_trajectory;
  const std::size_t nlp_iterations =
    static_cast<std::size_t>(nlp_results.m_solve_info["iter_count"].to_int());
  const float64_t nlp_proc_time = nlp_results.m_solve_info["t_proc_total"].to_double();

  // Perform post-checking of trajectory for collisions and dynamics
  const auto checking_tolerance = 1e-4;
  const auto trajectory_ok = m_nlp_planner.check_trajectory(
    smoothed_trajectory, current_state,
    adapted_goal_state, obstacles, m_model_parameters, checking_tolerance);
  std::cout << "Trajectory ok is: " << trajectory_ok << std::endl;

  // Return final result
  if (trajectory_ok) {
    return PlanningResult(smoothed_trajectory, nlp_iterations, nlp_proc_time, PlanningStatus::OK);
  } else {
    return PlanningResult(
      smoothed_trajectory, nlp_iterations, nlp_proc_time,
      PlanningStatus::NLP_ERROR);
  }
}

}  // namespace parking_planner
}  // namespace planning
}  // namespace motion
}  // namespace autoware
