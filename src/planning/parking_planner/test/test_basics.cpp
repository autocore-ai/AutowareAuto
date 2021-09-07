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

// This file contains some unit tests for rudimentary functionality of the
// bicycle model code and data structures

#include <gtest/gtest.h>
#include <common/types.hpp>
#include "parking_planner/geometry.hpp"
#include "parking_planner/parking_planner_types.hpp"
#include "parking_planner/bicycle_model.hpp"
#include "parking_planner/nlp_path_planner.hpp"
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
using autoware::motion::planning::parking_planner::HORIZON_LENGTH;


TEST(VehicleState, Serialization) {
  auto vstest = VehicleState(0.0, 1.1, 2.2, 3.3, 4.4);
  auto roundtrip = VehicleState::deserialize(vstest.serialize());

  EXPECT_EQ(vstest.get_x(), roundtrip.get_x());
  EXPECT_EQ(vstest.get_y(), roundtrip.get_y());
  EXPECT_EQ(vstest.get_velocity(), roundtrip.get_velocity());
  EXPECT_EQ(vstest.get_heading(), roundtrip.get_heading());
  EXPECT_EQ(vstest.get_steering(), roundtrip.get_steering());
}

TEST(VehicleCommand, Serialization) {
  auto vctest = VehicleCommand(0.0, 1.1);
  auto roundtrip = VehicleCommand::deserialize(vctest.serialize());

  EXPECT_EQ(vctest.get_steering_rate(), roundtrip.get_steering_rate());
  EXPECT_EQ(vctest.get_throttle(), roundtrip.get_throttle());
}


TEST(BicycleModel, ParametersGettersetters) {
  auto test = BicycleModelParameters(0, 1, 2, 3, 4);

  test.set_length_front(0.1);
  EXPECT_EQ(test.get_length_front(), 0.1);

  test.set_length_rear(0.2);
  EXPECT_EQ(test.get_length_rear(), 0.2);

  test.set_vehicle_width(0.3);
  EXPECT_EQ(test.get_vehicle_width(), 0.3);

  test.set_front_overhang(0.4);
  EXPECT_EQ(test.get_front_overhang(), 0.4);

  test.set_rear_overhang(0.5);
  EXPECT_EQ(test.get_rear_overhang(), 0.5);
}


TEST(BicycleModel, DynamicsBasic) {
  // Construct a test model
  auto parameters = BicycleModelParameters(1, 1, 1, 0.5, 0.5);
  auto model = BicycleModel(parameters);

  // Construct a test state
  auto vstest = VehicleState(0.0, 0.0, 0.0, 0.0, 0.0);

  // Construct a test input
  auto inputtest = VehicleCommand(0.0, 1.0);

  // Check that the derivatives make sense
  auto derivatives = model.dynamics(vstest, inputtest);

  // Perform some sanity checks on the derivatives
  EXPECT_GT(derivatives.get_velocity(), 0.0);
  EXPECT_EQ(derivatives.get_x(), 0.0);
  EXPECT_EQ(derivatives.get_y(), 0.0);
  EXPECT_EQ(derivatives.get_steering(), 0.0);
  EXPECT_EQ(derivatives.get_heading(), 0.0);
}
