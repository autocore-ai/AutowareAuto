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

// This file contains unit tests for the NLP path planner / smoother.

#include <gtest/gtest.h>
#include <common/types.hpp>
#include <vector>
#include "parking_planner/geometry.hpp"
#include "parking_planner/parking_planner_types.hpp"
#include "parking_planner/bicycle_model.hpp"
#include "parking_planner/nlp_path_planner.hpp"
#include "parking_planner/nlp_adapters.hpp"
#include "parking_planner/configuration.hpp"
#include "parking_planner/astar_path_planner.hpp"


using autoware::common::types::float64_t;
using Point2D = autoware::motion::planning::parking_planner::Point2D<float64_t>;
using Polytope2D = autoware::motion::planning::parking_planner::Polytope2D<float64_t>;
using VehicleState = autoware::motion::planning::parking_planner::VehicleState<float64_t>;
using VehicleCommand = autoware::motion::planning::parking_planner::VehicleCommand<float64_t>;
using BicycleModelParameters =
  autoware::motion::planning::parking_planner::BicycleModelParameters<float64_t>;
using BicycleModel =
  autoware::motion::planning::parking_planner::BicycleModel<float64_t, float64_t>;
using Trajectory = autoware::motion::planning::parking_planner::Trajectory<float64_t>;
using TrajectoryStep = autoware::motion::planning::parking_planner::TrajectoryStep<float64_t>;
using NLPCostWeights = autoware::motion::planning::parking_planner::NLPCostWeights<float64_t>;
using NLPPathPlanner = autoware::motion::planning::parking_planner::NLPPathPlanner;
using autoware::motion::planning::parking_planner::HORIZON_LENGTH;


static Trajectory create_dummy_initial_guess()
{
  Trajectory initial_guess{};
  for (std::size_t k = {}; k < HORIZON_LENGTH; k++) {
    initial_guess.push_back(
      TrajectoryStep(
        // Using 0.1 here because 0.0 is usually numerically a bad initial guess
        VehicleCommand(0.1, 0.1),
        VehicleState(0.1, 0.1, 0.1, 0.1, 0.1)));
  }

  return initial_guess;
}
TEST(BicycleModel, SimpleSolve) {
  const auto parameters = BicycleModelParameters(1.5, 1.5, 2, 0.5, 0.5);
  const NLPCostWeights weights(1.0, 1.0, 0.0);
  const VehicleState lower_state_bounds(-100, -100, -10, -2 * 3.14156, -0.52);
  const VehicleState upper_state_bounds(+100, +100, +10, +2 * 3.14156, +0.52);
  const VehicleCommand lower_command_bounds(-3.0, -50);
  const VehicleCommand upper_command_bounds(+3.0, +50);
  std::vector<Polytope2D> obstacles{};

  const auto start1 = VehicleState(5.0, 0.0, 0.0, 0.5, 0.0);
  const auto goal1 = VehicleState(-1.0, 1.0, 0.0, 0.0, 0.0);
  const auto initial_guess1 = create_dummy_initial_guess();
  const auto nlp_path_planner = NLPPathPlanner(
    weights,
    lower_state_bounds, upper_state_bounds,
    lower_command_bounds, upper_command_bounds);
  const auto results1 = nlp_path_planner.plan_nlp(
    start1, goal1, initial_guess1, obstacles,
    parameters);
  const auto trajectory1 = results1.m_trajectory;

  const auto start2 = VehicleState(3.0, 6.0, 0.0, 3.14, 0.0);
  const auto goal2 = VehicleState(-2.0, 9.0, 0.0, 1.57, 0.0);
  const auto initial_guess2 = create_dummy_initial_guess();
  const auto results2 = nlp_path_planner.plan_nlp(
    start2, goal2, initial_guess2, obstacles,
    parameters);
  const auto trajectory2 = results2.m_trajectory;
}
