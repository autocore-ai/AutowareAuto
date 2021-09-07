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

// This file contains tests for the geometry functionality.

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


TEST(Geometry, PointArithmetic)
{
  Point2D p1(0, 2);
  Point2D p2(3, 0);

  EXPECT_EQ(p1 + p2, Point2D(3, 2));  // noice
  EXPECT_EQ(p1 - p2, Point2D(-3, 2));  // noice
  EXPECT_EQ(p1 * 2, Point2D(0, 4));  // noice
  EXPECT_EQ(p1 / 2, Point2D(0, 1));  // noice
}

TEST(Geometry, PolyhedronBasics1)
{
  Point2D p1(0, 0);
  Point2D p2(1, 0);
  Point2D p3(1, 1);
  Point2D p4(0, 1);

  auto points = {p1, p2, p3, p4};

  Polytope2D P(points);

  for (const auto & p : points) {
    EXPECT_TRUE(P.contains_point(p) );
  }
  EXPECT_FALSE(P.contains_point(Point2D(2, 2) ) );
  EXPECT_FALSE(P.contains_point(Point2D(-1, -1) ) );
}

TEST(Geometry, PolyhedronBasics2)
{
  Point2D p1(0, 0);
  Point2D p2(2, 0);
  Point2D p3(0, 2);

  auto points = {p1, p2, p3};

  Polytope2D P(points);

  for (const auto & p : points) {
    EXPECT_TRUE(P.contains_point(p) );
  }
  EXPECT_TRUE(P.contains_point(Point2D(1, 1) ) );
  EXPECT_FALSE(P.contains_point(Point2D(1, 1.1) ) );
  EXPECT_FALSE(P.contains_point(Point2D(-1, -1) ) );
}

TEST(Geometry, PolyhedronBasics3)
{
  Point2D p1(-1, -1);
  Point2D p2(2, 2);
  Point2D p3(-1, 2);

  auto points = {p1, p2, p3};

  Polytope2D P(points);

  for (const auto & p : points) {
    EXPECT_TRUE(P.contains_point(p) );
  }
  EXPECT_TRUE(P.contains_point(Point2D(0, 0) ) );
  EXPECT_TRUE(P.contains_point(Point2D(-0.5, -0.5) ) );
  EXPECT_TRUE(P.contains_point(Point2D(1, 1) ) );
}

TEST(Geometry, PolyhedronIntersection1)
{
  const Polytope2D p1({{1.0, 1.0}, {-1.0, 1.0}, {-1.0, -1.0}, {1.0, -1.0}});
  const Polytope2D p2({{5.0, 5.0}, {3.0, 5.0}, {3.0, 3.0}, {5.0, 3.0}});

  EXPECT_FALSE(p1.intersects_with(p2));
}

TEST(Geometry, PolyhedronIntersection2)
{
  const Polytope2D p1({{1.0, 1.0}, {-1.0, 1.0}, {-1.0, -1.0}, {1.0, -1.0}});
  const Polytope2D p2({{2.0, 1.0}, {0.0, 1.0}, {0.0, -1.0}, {2.0, -1.0}});

  EXPECT_TRUE(p1.intersects_with(p2));
}
