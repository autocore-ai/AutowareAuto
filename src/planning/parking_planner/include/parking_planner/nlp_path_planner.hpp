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

#ifndef PARKING_PLANNER__NLP_PATH_PLANNER_HPP_
#define PARKING_PLANNER__NLP_PATH_PLANNER_HPP_

#include <casadi/casadi.hpp>
#include <common/types.hpp>
#include <vector>

#include "visibility_control.hpp"
#include "parking_planner_types.hpp"
#include "nlp_adapters.hpp"
#include "bicycle_model.hpp"

namespace autoware
{

namespace motion
{

namespace planning
{

namespace parking_planner
{

using autoware::common::types::float64_t;
struct NLPResults
{
  // cppcheck-suppress syntaxError
  Trajectory<float64_t> m_trajectory;
  casadi::Dict m_solve_info;
};

class PARKING_PLANNER_PUBLIC NLPPathPlanner
{
public:
  /// \brief Create an NLP-based trajectory planner
  /// \param[in] cost_weights Cost function weight parameters
  /// \param[in] lower_state_bounds Lower bounds on the state variables, valid across the entire
  ///            horizon
  /// \param[in] upper_state_bounds Upper bounds on the state variables, valid across the entire
  ///            horizon
  /// \param[in] lower_command_bounds Lower bounds on the command variables, valid across the
  ///            entire horizon
  /// \param[in] upper_command_bounds Upper bounds on the command variables, valid across the
  ///            entire horizon
  NLPPathPlanner(
    const NLPCostWeights<float64_t> & cost_weights,
    const VehicleState<float64_t> & lower_state_bounds,
    const VehicleState<float64_t> & upper_state_bounds,
    const VehicleCommand<float64_t> & lower_command_bounds,
    const VehicleCommand<float64_t> & upper_command_bounds
  );

  /// \brief Plan a dynamically feasible, collision-free path from a given starting state to a
  ///        given end state.
  /// \param[in] current_state Starting vehicle state for the path planning
  /// \param[in] goal_state Desired final state for the path planning
  /// \param[in] initial_guess An initial guess for the trajectory. Does not need to be
  ///            dynamically feasible.
  /// \param[in] obstacles The obstacles to avoid
  /// \param[in] model_parameters Physical model parameters of the vehicle
  /// \return Planning results. The m_solve_info field can be investigated for solver errors, but
  ///         the trajectory should be post-checked separately for constraint satisfaction.
  NLPResults plan_nlp(
    const VehicleState<float64_t> & current_state,
    const VehicleState<float64_t> & goal_state,
    const Trajectory<float64_t> & initial_guess,
    const std::vector<Polytope2D<float64_t>> & obstacles,
    const BicycleModelParameters<float64_t> & model_parameters
  ) const;

  /// \brief Check a given trajectory for feasibility in terms of dynamics, variable bounds and
  ///        obstacle avoidance.
  /// \param[in] trajectory The trajectory to check
  /// \param[in] initial_state State that should be at the start of the trajectory
  /// \param[in] goal_state State that should be at the end of the trajectory
  /// \param[in] obstacles List of obstacles the trajectory should not collide with
  /// \param[in] model_parameters Physical model parameters of the vehicle
  /// \param[in] tolerance Numerical tolerance to use in the checks
  /// \return True if everything is OK.
  bool check_trajectory(
    const Trajectory<float64_t> & trajectory,
    const VehicleState<float64_t> & initial_state,
    const VehicleState<float64_t> & goal_state,
    const std::vector<Polytope2D<float64_t>> & obstacles,
    const BicycleModelParameters<float64_t> & model_parameters,
    const float64_t tolerance
  ) const;

private:
  NLPCostWeights<float64_t> m_cost_weights;
  VehicleState<float64_t> m_lower_state_bounds;
  VehicleState<float64_t> m_upper_state_bounds;
  VehicleCommand<float64_t> m_lower_command_bounds;
  VehicleCommand<float64_t> m_upper_command_bounds;
};


}  // namespace parking_planner
}  // namespace planning
}  // namespace motion
}  // namespace autoware

#endif  // PARKING_PLANNER__NLP_PATH_PLANNER_HPP_
