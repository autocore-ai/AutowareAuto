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

// This file contains some unit tests for the A* path planner.

#include <gtest/gtest.h>
#include <common/types.hpp>
#include <vector>
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


TEST(AstarPathPlanner, DirectPathX) {
  static constexpr float64_t dist = 10.0;
  const VehicleState current_state(0.0, 0.0, 0.0, 0.0, 0.0);
  const VehicleState goal_state(dist, 0.0, 0.0, 0.0, 0.0);

  Polytope2D vehicle_bounding_vertices(
    std::vector<Point2D>({{1.0, 1.0}, {-1.0, 1.0}, {-1.0, -1.0}, {1.0, -1.0}}));

  const Polytope2D vehicle_bounding_box(vehicle_bounding_vertices);

  const auto planner = autoware::motion::planning::parking_planner::AstarPathPlanner();

  std::vector<VehicleState> vehicle_states =
    planner.plan_astar(current_state, goal_state, vehicle_bounding_box, {});

  EXPECT_EQ(
    vehicle_states.size(),
    static_cast<size_t>(dist /
    autoware::motion::planning::parking_planner::DELTA_LONGITUDINAL + 0.5) + 1);
}

TEST(AstarPathPlanner, DirectPathY) {
  static constexpr float64_t dist = 10.0;
  const VehicleState current_state(0.0, 0.0, 0.0, 3.14159 / 2, 0.0);
  const VehicleState goal_state(0.0, dist, 0.0, 3.14159 / 2, 0.0);

  Polytope2D vehicle_bounding_vertices(
    std::vector<Point2D>({{1.0, 1.0}, {-1.0, 1.0}, {-1.0, -1.0}, {1.0, -1.0}}));

  const Polytope2D vehicle_bounding_box(vehicle_bounding_vertices);

  const auto planner = autoware::motion::planning::parking_planner::AstarPathPlanner();

  std::vector<VehicleState> vehicle_states =
    planner.plan_astar(current_state, goal_state, vehicle_bounding_box, {});

  EXPECT_EQ(
    vehicle_states.size(),
    static_cast<size_t>(dist /
    autoware::motion::planning::parking_planner::DELTA_LONGITUDINAL + 0.5) + 1);
}

TEST(AstarPathPlanner, ObstaclePathX) {
  static constexpr float64_t dist = 10.0;
  const VehicleState current_state(0.0, 0.0, 0.0, 0.0, 0.0);
  const VehicleState goal_state(dist, 0.0, 0.0, 0.0, 0.0);

  // Set some parameters, then compute the bounding box from those
  const auto parameters = BicycleModelParameters(0.8, 0.8, 1.0, 0.1, 0.1);
  const auto model = BicycleModel(parameters);
  const auto vehicle_bounding_box = model.compute_bounding_box(VehicleState{});

  Polytope2D obstacle(
    std::vector<Point2D>(
      {{dist / 2 + 1.0, 1.0}, {dist / 2 - 1.0, 1.0}, {dist / 2 - 1.0, -1.0},
        {dist / 2 + 1.0, -1.0}}));

  std::vector<Polytope2D> obstacles;
  obstacles.push_back(obstacle);

  const auto planner = autoware::motion::planning::parking_planner::AstarPathPlanner();

  std::vector<VehicleState> vehicle_states =
    planner.plan_astar(current_state, goal_state, vehicle_bounding_box, obstacles);

  // the path is expected to be longer than the direct path
  EXPECT_GT(
    vehicle_states.size(),
    static_cast<size_t>(dist /
    autoware::motion::planning::parking_planner::DELTA_LONGITUDINAL + 0.5) + 1);

  // count number of collisions
  int32_t num_collisions = 0;
  for (auto s : vehicle_states) {
    for (const auto & obst : obstacles) {
      Polytope2D v(vehicle_bounding_box);
      v.rotate_and_shift(s.get_heading(), {0.0, 0.0}, {s.get_x(), s.get_y()});
      if (obst.intersects_with(v)) {
        num_collisions++;
      }
    }
  }

  EXPECT_EQ(num_collisions, 0);
}

TEST(AstarPathPlanner, ObstaclePathY) {
  static constexpr float64_t dist = 10.0;
  const VehicleState current_state(0.0, 0.0, 0.0, 0.0, 0.0);
  const VehicleState goal_state(0.0, dist, 0.0, 0.0, 0.0);

  // Set some parameters, then compute the bounding box from those
  const auto parameters = BicycleModelParameters(0.8, 0.8, 1.0, 0.1, 0.1);
  const auto model = BicycleModel(parameters);
  const auto vehicle_bounding_box = model.compute_bounding_box(VehicleState{});

  Polytope2D obstacle(
    std::vector<Point2D>(
      {{1.0, 1.0 + dist / 2}, {-1.0, 1.0 + dist / 2}, {-1.0, -1.0 + dist / 2},
        {1.0, -1.0 + dist / 2}}));

  std::vector<Polytope2D> obstacles;
  obstacles.push_back(obstacle);

  const auto planner = autoware::motion::planning::parking_planner::AstarPathPlanner();

  std::vector<VehicleState> vehicle_states =
    planner.plan_astar(current_state, goal_state, vehicle_bounding_box, obstacles);

  // the path is expected to be longer than the direct path
  EXPECT_GT(
    vehicle_states.size(),
    static_cast<size_t>(dist /
    autoware::motion::planning::parking_planner::DELTA_LONGITUDINAL + 0.5) + 1);

  // count number of collisions
  int32_t num_collisions = 0;
  for (auto s : vehicle_states) {
    for (const auto & obst : obstacles) {
      Polytope2D v(vehicle_bounding_box);
      v.rotate_and_shift(s.get_heading(), {0.0, 0.0}, {s.get_x(), s.get_y()});
      if (obst.intersects_with(v)) {
        num_collisions++;
      }
    }
  }

  EXPECT_EQ(num_collisions, 0);
}

TEST(AstarPathPlanner, ImmediateObstacle) {
  static constexpr float64_t dist = 5.0;
  const VehicleState current_state(-dist / 2, 0.0, 0.0, 0.0, 0.0);
  const VehicleState goal_state(dist / 2, 0.0, 0.0, 0.0, 0.0);

  // Set some parameters, then compute the bounding box from those
  const auto parameters = BicycleModelParameters(0.8, 0.8, 1.0, 0.1, 0.1);
  const auto model = BicycleModel(parameters);
  const auto vehicle_bounding_box = model.compute_bounding_box(VehicleState{});

  Polytope2D wall(
    std::vector<Point2D>(
      {{-dist / 2 + 2.0, 3.0}, {-dist / 2 + 1.0, 3.0}, {-dist / 2 + 1.0, -3.0},
        {-dist / 2 + 2.0, -3.0}}));

  std::vector<Polytope2D> obstacles({wall});

  const auto planner = autoware::motion::planning::parking_planner::AstarPathPlanner();

  std::vector<VehicleState> vehicle_states =
    planner.plan_astar(current_state, goal_state, vehicle_bounding_box, obstacles);

  // the path is expected to be longer than the direct path
  EXPECT_GT(
    vehicle_states.size(),
    static_cast<size_t>(dist /
    autoware::motion::planning::parking_planner::DELTA_LONGITUDINAL + 0.5) + 1);

  // count number of collisions
  int32_t num_collisions = 0;
  for (auto s : vehicle_states) {
    for (const auto & obst : obstacles) {
      Polytope2D v(vehicle_bounding_box);
      v.rotate_and_shift(s.get_heading(), {0.0, 0.0}, {s.get_x(), s.get_y()});
      if (obst.intersects_with(v)) {
        num_collisions++;
      }
    }
  }

  EXPECT_EQ(num_collisions, 0);
}

TEST(AstarPathPlanner, SidewaysParkingBroad) {
  static constexpr float64_t dist = 2.0;
  const VehicleState current_state(0.0, 0.0, 0.0, 0.0, 0.0);
  const VehicleState goal_state(0.0, dist, 0.0, 0.0, 0.0);

  // Set some parameters, then compute the bounding box from those
  const auto parameters = BicycleModelParameters(0.8, 0.8, 1.0, 0.1, 0.1);
  const auto model = BicycleModel(parameters);
  const auto vehicle_bounding_box = model.compute_bounding_box(VehicleState{});

  Polytope2D front_parked_car(
    std::vector<Point2D>(
      {{4.0, 1.0 + dist}, {2.0, 1.0 + dist}, {2.0, -1.0 + dist},
        {4.0, -1.0 + dist}}));

  Polytope2D back_parked_car(
    std::vector<Point2D>(
      {{-2.0, 1.0 + dist}, {-4.0, 1.0 + dist}, {-4.0, -1.0 + dist},
        {-2.0, -1.0 + dist}}));

  std::vector<Polytope2D> obstacles;
  obstacles.push_back(front_parked_car);
  obstacles.push_back(back_parked_car);

  const auto planner = autoware::motion::planning::parking_planner::AstarPathPlanner();

  std::vector<VehicleState> vehicle_states =
    planner.plan_astar(current_state, goal_state, vehicle_bounding_box, obstacles);

  // count number of collisions
  int32_t num_collisions = 0;
  for (auto s : vehicle_states) {
    for (const auto & obst : obstacles) {
      Polytope2D v(vehicle_bounding_box);
      v.rotate_and_shift(s.get_heading(), {0.0, 0.0}, {s.get_x(), s.get_y()});
      if (obst.intersects_with(v)) {
        num_collisions++;
      }
    }
  }

  EXPECT_EQ(num_collisions, 0);
}

TEST(AstarPathPlanner, SidewaysParkingNarrow) {
  static constexpr float64_t dist = 2.0;
  const VehicleState current_state(0.0, 0.0, 0.0, 0.0, 0.0);
  const VehicleState goal_state(0.0, dist, 0.0, 0.0, 0.0);

  // Set some parameters, then compute the bounding box from those
  const auto parameters = BicycleModelParameters(0.8, 0.8, 1.0, 0.1, 0.1);
  const auto model = BicycleModel(parameters);
  const auto vehicle_bounding_box = model.compute_bounding_box(VehicleState{});

  Polytope2D front_parked_car(
    std::vector<Point2D>(
      {{3.5, 1.0 + dist}, {1.5, 1.0 + dist}, {1.5, -1.0 + dist},
        {3.5, -1.0 + dist}}));

  Polytope2D back_parked_car(
    std::vector<Point2D>(
      {{-1.5, 1.0 + dist}, {-3.5, 1.0 + dist}, {-3.5, -1.0 + dist},
        {-1.5, -1.0 + dist}}));

  std::vector<Polytope2D> obstacles;
  obstacles.push_back(front_parked_car);
  obstacles.push_back(back_parked_car);

  const auto planner = autoware::motion::planning::parking_planner::AstarPathPlanner();

  std::vector<VehicleState> vehicle_states =
    planner.plan_astar(current_state, goal_state, vehicle_bounding_box, obstacles);

  // count number of collisions
  int32_t num_collisions = 0;
  for (auto s : vehicle_states) {
    for (const auto & obst : obstacles) {
      Polytope2D v(vehicle_bounding_box);
      v.rotate_and_shift(s.get_heading(), {0.0, 0.0}, {s.get_x(), s.get_y()});
      if (obst.intersects_with(v)) {
        num_collisions++;
      }
    }
  }

  EXPECT_EQ(num_collisions, 0);
}


TEST(AstarPathPlanner, ParallelShort) {
  const VehicleState current_state(0.0, 0.0, 0.0, 0, 0.0);
  const VehicleState goal_state(2.0, -1.5, 0.0, 0, 0.0);

  // Set some parameters, then compute the bounding box from those
  const auto parameters = BicycleModelParameters(0.8, 0.8, 1.0, 0.1, 0.1);
  const auto model = BicycleModel(parameters);
  const auto vehicle_bounding_box = model.compute_bounding_box(VehicleState{});

  const Polytope2D back_parked_car(
    std::vector<Point2D>({{0.5, -0.8}, {-2.0, -0.8}, {-2.0, -2.5}, {0.5, -2.5}}));

  const Polytope2D front_parked_car(
    std::vector<Point2D>({{10.0, -0.8}, {3.5, -0.8}, {3.5, -2.5}, {10, -2.5}}));

  const Polytope2D wall_box(
    std::vector<Point2D>({{10.0, -2.5}, {-2.0, -2.5}, {-2.0, -4.0}, {10.0, -4.0}}));

  const std::vector<Polytope2D> obstacles({back_parked_car, front_parked_car, wall_box});

  const auto planner = autoware::motion::planning::parking_planner::AstarPathPlanner();

  const std::vector<VehicleState> vehicle_states =
    planner.plan_astar(current_state, goal_state, vehicle_bounding_box, obstacles);

  // count number of collisions
  int32_t num_collisions = 0;
  for (auto s : vehicle_states) {
    for (const auto & obst : obstacles) {
      Polytope2D v(vehicle_bounding_box);
      v.rotate_and_shift(s.get_heading(), {0.0, 0.0}, {s.get_x(), s.get_y()});
      if (obst.intersects_with(v)) {
        num_collisions++;
      }
    }
  }

  EXPECT_EQ(num_collisions, 0);
}

TEST(AstarPathPlanner, ParallelLong) {
  const VehicleState current_state(0.0, 0.0, 0.0, 0, 0.0);
  const VehicleState goal_state(5.0, -1.5, 0.0, 0, 0.0);

  // Set some parameters, then compute the bounding box from those
  const auto parameters = BicycleModelParameters(0.8, 0.8, 1.0, 0.1, 0.1);
  const auto model = BicycleModel(parameters);
  const auto vehicle_bounding_box = model.compute_bounding_box(VehicleState{});

  const std::vector<Polytope2D> obstacles;  // no obstacles

  const auto planner = autoware::motion::planning::parking_planner::AstarPathPlanner();

  const std::vector<VehicleState> vehicle_states =
    planner.plan_astar(current_state, goal_state, vehicle_bounding_box, obstacles);

  // count number of collisions
  int32_t num_collisions = 0;
  for (auto s : vehicle_states) {
    for (const auto & obst : obstacles) {
      Polytope2D v(vehicle_bounding_box);
      v.rotate_and_shift(s.get_heading(), {0.0, 0.0}, {s.get_x(), s.get_y()});
      if (obst.intersects_with(v)) {
        num_collisions++;
      }
    }
  }

  EXPECT_EQ(num_collisions, 0);
}

TEST(AstarPathPlanner, ParallelBehind) {
  const VehicleState current_state(0.0, 0.0, 0.0, 0, 0.0);
  const VehicleState goal_state(-2.0, -1.5, 0.0, 0, 0.0);

  // Set some parameters, then compute the bounding box from those
  const auto parameters = BicycleModelParameters(0.8, 0.8, 1.0, 0.1, 0.1);
  const auto model = BicycleModel(parameters);
  const auto vehicle_bounding_box = model.compute_bounding_box(VehicleState{});

  const Polytope2D back_parked_car(
    std::vector<Point2D>({{-4.0, -0.8}, {-6.0, -0.8}, {-6.0, -2.5}, {-4.0, -2.5}}));

  const Polytope2D front_parked_car(
    std::vector<Point2D>({{10.0, -0.8}, {0.0, -0.8}, {0.0, -2.5}, {10.0, -2.5}}));

  const Polytope2D wall_box(
    std::vector<Point2D>({{10.0, -2.5}, {-5.0, -2.5}, {-5.0, -4.0}, {10.0, -4.0}}));

  const std::vector<Polytope2D> obstacles({back_parked_car, front_parked_car, wall_box});

  const auto planner = autoware::motion::planning::parking_planner::AstarPathPlanner();

  const std::vector<VehicleState> vehicle_states =
    planner.plan_astar(current_state, goal_state, vehicle_bounding_box, obstacles);

  // count number of collisions
  int32_t num_collisions = 0;
  for (auto s : vehicle_states) {
    for (const auto & obst : obstacles) {
      Polytope2D v(vehicle_bounding_box);
      v.rotate_and_shift(s.get_heading(), {0.0, 0.0}, {s.get_x(), s.get_y()});
      if (obst.intersects_with(v)) {
        num_collisions++;
      }
    }
  }

  EXPECT_EQ(num_collisions, 0);
}

TEST(AstarPathPlanner, InfeasibleSituationCurrentWalled) {
  const VehicleState current_state(0.0, 0.0, 0.0, 0, 0.0);
  const VehicleState goal_state(15.0, 0.0, 0.0, 0, 0.0);

  // Set some parameters, then compute the bounding box from those
  const auto parameters = BicycleModelParameters(0.8, 0.8, 1.0, 0.1, 0.1);
  const auto model = BicycleModel(parameters);
  const auto vehicle_bounding_box = model.compute_bounding_box(VehicleState{});

  const Polytope2D wall_box_upper(
    std::vector<Point2D>({{10.0, 10.0}, {-10.0, 10.0}, {-10.0, 9.0}, {10.0, 9.0}}));

  const Polytope2D wall_box_lower(
    std::vector<Point2D>({{10.0, -9.0}, {-10.0, -9.0}, {-10.0, -10.0}, {10.0, -10.0}}));

  const Polytope2D wall_box_right(
    std::vector<Point2D>({{10.0, 9.0}, {9.0, 9.0}, {9.0, -9.0}, {10.0, -9.0}}));

  const Polytope2D wall_box_left(
    std::vector<Point2D>({{-9.0, 9.0}, {-10.0, 9.0}, {-10.0, -9.0}, {-9.0, -9.0}}));

  const std::vector<Polytope2D> obstacles({wall_box_upper, wall_box_lower, wall_box_right,
      wall_box_left});

  const auto planner = autoware::motion::planning::parking_planner::AstarPathPlanner();

  const std::vector<VehicleState> vehicle_states =
    planner.plan_astar(current_state, goal_state, vehicle_bounding_box, obstacles);

  // count number of collisions
  int32_t num_collisions = 0;
  for (auto s : vehicle_states) {
    for (const auto & obst : obstacles) {
      Polytope2D v(vehicle_bounding_box);
      v.rotate_and_shift(s.get_heading(), {0.0, 0.0}, {s.get_x(), s.get_y()});
      if (obst.intersects_with(v)) {
        num_collisions++;
      }
    }
  }

  EXPECT_EQ(num_collisions, 0);
  EXPECT_EQ(vehicle_states.size(), 1U);
}
