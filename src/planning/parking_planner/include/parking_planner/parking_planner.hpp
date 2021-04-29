// Copyright 2020 Embotech AG, Zurich, Switzerland. Arm Limited. All rights reserved.
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

// This file contains the main interface to the parking planner. In order to
// use the planner, one constructs an object of the ParkingPlanner class and
// then calls the "plan" method with the appropriate arguments.

#ifndef PARKING_PLANNER__PARKING_PLANNER_HPP_
#define PARKING_PLANNER__PARKING_PLANNER_HPP_

#include <autoware_auto_msgs/msg/had_map_route.hpp>
#include <autoware_auto_msgs/msg/trajectory.hpp>
#include <common/types.hpp>
#include <lanelet2_core/LaneletMap.h>
#include <vector>

#include "astar_path_planner.hpp"
#include "bicycle_model.hpp"
#include "nlp_path_planner.hpp"
#include "parking_planner_types.hpp"
#include "visibility_control.hpp"

namespace autoware
{
namespace motion
{
namespace planning
{
namespace parking_planner
{
using autoware::common::types::float64_t;
/// \brief Parking planner return codes
enum class PlanningStatus
{
  /// Everything went well
  OK,

  /// There was an error running the nonlinear solver
  NLP_ERROR
};

PARKING_PLANNER_PUBLIC std::vector<Polytope2D<float64_t>> convert_drivable_area_to_obstacles(
  const lanelet::Polygon3d & drivable_area);

PARKING_PLANNER_PUBLIC autoware_auto_msgs::msg::Trajectory
convert_parking_planner_to_autoware_trajectory(
  const Trajectory<float64_t> & parking_trajectory);

/// \brief Results of a parking planning call
class PARKING_PLANNER_PUBLIC PlanningResult
{
public:
  /// \brief Construct a planning result
  /// \param[in] trajectory The trajectory computed by the planner
  /// \param[in] nlp_iterations The number of iterations required by the solver
  /// \param[in] nlp_proc_time NLP process time, in seconds
  /// \param[in] status Status of the result
  PlanningResult(
    const Trajectory<float64_t> trajectory,
    const std::size_t nlp_iterations,
    const float64_t nlp_proc_time,
    const PlanningStatus status);

  const Trajectory<float64_t> & get_trajectory() const noexcept
  {
    return m_trajectory;
  }
  std::size_t get_nlp_iterations() const noexcept
  {
    return m_nlp_iterations;
  }
  float64_t get_nlp_proc_time() const noexcept
  {
    return m_nlp_proc_time;
  }
  PlanningStatus get_status() const noexcept
  {
    return m_status;
  }

private:
  /// Trajectory computed for this result
  Trajectory<float64_t> m_trajectory;

  /// Number of iterations required to find the solution of the NLP
  std::size_t m_nlp_iterations;

  /// Process time taken by the NLP
  float64_t m_nlp_proc_time;

  /// Status of the solution
  PlanningStatus m_status;
};  // class PlanningResult

/// \brief Parking motion planner
class PARKING_PLANNER_PUBLIC ParkingPlanner
{
public:
  /// \brief Create a parking motion planner object. This can be changed to be dependency-injection
  //         friendly once that becomes required.
  /// \param[in] parameters Parameters of the vehicle model to use
  /// \param[in] nlp_weights Cost function weight parameters to use in the non-linear optimization
  /// \param[in] lower_state_bounds Lower bounds on the states (applied throught the horizon)
  /// \param[in] upper_state_bounds Upper bounds on the states (applied throught the horizon)
  /// \param[in] lower_command_bounds Lower bounds on the commands (applied throught the horizon)
  /// \param[in] upper_command_bounds Upper bounds on the commands (applied throught the horizon)
  ParkingPlanner(
    const BicycleModelParameters<float64_t> & parameters,
    const NLPCostWeights<float64_t> & nlp_weights,
    const VehicleState<float64_t> & lower_state_bounds,
    const VehicleState<float64_t> & upper_state_bounds,
    const VehicleCommand<float64_t> & lower_command_bounds,
    const VehicleCommand<float64_t> & upper_command_bounds);

  /// \brief Plan a maneuver in a synchronous manner. This call blocks.
  /// \param[in] current_state State of the vehicle at the start of the maneuver
  /// \param[in] goal_state State the vehicle should be in at the end of the maneuver
  /// \param[in] obstacles List of static obstacles to avoid, in the form of polyhedra
  /// \return Result of the planning procedure
  PlanningResult plan(
    const VehicleState<float64_t> & current_state,
    const VehicleState<float64_t> & goal_state,
    const std::vector<Polytope2D<float64_t>> & obstacles) const;


  const BicycleModelParameters<float64_t> & get_parameters() const
  {
    return m_model_parameters;
  }

private:
  /// \brief Create a full Trajectory data structure from just a list of states. This is used
  ///        in the translation of the Astar output to an initial guess for the NLP. If the input
  ///        does not match the desired length, it is adapted to that length by simple means.
  //         The resulting trajectory is currently not dynamically feasible, since it is only
  //         used as an initial guess in the NLP solver and already useful enough as it is.
  /// \param[in] states_input vector of vehicle states that should be turned into a full Trajectory
  /// \param[in] desired_trajectory_length Length in number of steps that the output should have
  /// \return Full trajectory (states and commands), not necessarily dynamically feasible.
  Trajectory<float64_t>
  create_trajectory_from_states(
    const std::vector<VehicleState<float64_t>> & states_input,
    const std::size_t desired_trajectory_length) const;

  /// A* path planner acting as a global path finder
  AstarPathPlanner m_astar_planner;

  /// The non-linear path planner acting as a smoother
  NLPPathPlanner m_nlp_planner;

  /// Vehicle model parameters to use in the maneuver planning.
  BicycleModelParameters<float64_t> m_model_parameters;
};  // class ParkingPlanner

}  // namespace parking_planner
}  // namespace planning
}  // namespace motion
}  // namespace autoware

#endif  // PARKING_PLANNER__PARKING_PLANNER_HPP_
