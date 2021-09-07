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

#include <lane_planner/lane_planner.hpp>
#include <geometry/common_2d.hpp>
#include <memory>

#include "gtest/gtest.h"

using autoware_auto_msgs::msg::MapPrimitive;
using autoware_auto_msgs::msg::HADMapRoute;
using autoware_auto_msgs::msg::HADMapSegment;
using autoware_auto_msgs::msg::TrajectoryPoint;

using motion::motion_common::VehicleConfig;
using motion::planning::trajectory_smoother::TrajectorySmootherConfig;

using autoware::common::types::bool8_t;
using autoware::common::types::float32_t;
using autoware::common::types::float64_t;

// returns a map with a lane has given number of points(n_points)
// length of the lane will be n_points meters in y direction
lanelet::LaneletMapPtr getALaneletMapWithLaneId(
  const lanelet::Id & id, const float64_t velocity,
  const size_t n_points)
{
  lanelet::Points3d right_points, left_points, center_points;
  constexpr float64_t resolution = 1.0;
  for (size_t i = 0; i < n_points; i++) {
    const auto y = resolution * static_cast<float64_t>(i);
    left_points.push_back(lanelet::Point3d(lanelet::utils::getId(), -1, y, 0));
    right_points.push_back(lanelet::Point3d(lanelet::utils::getId(), 1, y, 0));
    center_points.push_back(lanelet::Point3d(lanelet::utils::getId(), 0, y, 0));
  }
  lanelet::LineString3d ls1(lanelet::utils::getId(), left_points);
  lanelet::LineString3d ls2(lanelet::utils::getId(), right_points);
  lanelet::LineString3d ls3(lanelet::utils::getId(), center_points);

  lanelet::Lanelet ll(id, ls1, ls2);
  ll.setCenterline(ls3);
  ll.setAttribute(lanelet::AttributeName::SpeedLimit, velocity);

  return lanelet::utils::createMap({ll});
}

HADMapRoute getARoute(const int64_t lane_id, const float32_t length)
{
  HADMapRoute had_map_route;
  had_map_route.start_point.position.x = 0;
  had_map_route.start_point.position.y = 0;
  had_map_route.goal_point.position.x = 0;
  had_map_route.goal_point.position.y = length;

  MapPrimitive primitive;
  primitive.id = lane_id;
  HADMapSegment segment;
  segment.preferred_primitive_id = primitive.id;
  had_map_route.segments.push_back(segment);
  had_map_route.segments.front().primitives.push_back(primitive);

  return had_map_route;
}

class LanePlannerTest : public ::testing::Test
{
public:
  LanePlannerTest()
  {
    const VehicleConfig vehicle_param{
      1.0F,  // cg_to_front_m:
      1.0F,  // cg_to_rear_m:
      0.1F,  // front_corner_stiffness:
      0.1F,  // rear_corner_stiffness:
      1500.0F,  // mass_kg:
      12.0F,  // yaw_inertia_kgm2:
      2.0F,  // width_m:
      0.5F,  // front_overhang_m:
      0.5F  // rear_overhang_m:
    };
    const TrajectorySmootherConfig config{
      1.0F,  // standard_deviation
      5  // kernel_size
    };
    const autoware::lane_planner::LanePlannerConfig planner_config{
      2.0F  // trajectory_resolution
    };
    m_planner_ptr = std::make_shared<autoware::lane_planner::LanePlanner>(
      vehicle_param, config,
      planner_config);
  }
  std::shared_ptr<autoware::lane_planner::LanePlanner> m_planner_ptr;
};

TEST(TestFunction, Distance2d)
{
  TrajectoryPoint pt1, pt2;
  pt1.x = 3.0F;
  pt1.y = 0.0F;

  pt2.x = 0.0F;
  pt2.y = 4.0F;

  // same point returns distance 0
  ASSERT_FLOAT_EQ(autoware::lane_planner::distance2d(pt1, pt1), 0.0F);

  ASSERT_FLOAT_EQ(autoware::lane_planner::distance2d(pt1, pt2), 5.0F);
}

TEST(TestFunction, Curvature)
{
  TrajectoryPoint pt1, pt2, pt3;
  pt1.x = 1.0F;
  pt1.y = 0.0F;

  pt2.x = 0.0F;
  pt2.y = 1.0F;

  pt3.x = -1.0F;
  pt3.y = 0.0F;

  // circle with radius = 1
  ASSERT_FLOAT_EQ(autoware::lane_planner::calculate_curvature(pt1, pt2, pt3), 1.0F);

  // opposite direction gives -1
  ASSERT_FLOAT_EQ(autoware::lane_planner::calculate_curvature(pt3, pt2, pt1), -1.0F);

  pt1 = autoware::common::geometry::times_2d(pt1, 2.0F);
  pt2 = autoware::common::geometry::times_2d(pt2, 2.0F);
  pt3 = autoware::common::geometry::times_2d(pt3, 2.0F);

  // doubling the radius results in half the curvature
  ASSERT_FLOAT_EQ(autoware::lane_planner::calculate_curvature(pt1, pt2, pt3), 0.5F);
}

TEST_F(LanePlannerTest, PlanSimpleTrajectory)
{
  // create map
  const auto lane_id = lanelet::utils::getId();
  constexpr float64_t velocity_mps = 1.0;
  constexpr size_t n_points = 5;
  const auto lanelet_map_ptr = getALaneletMapWithLaneId(lane_id, velocity_mps, n_points);

  // create route message
  const auto had_map_route = getARoute(lane_id, 5.0F);

  const auto trajectory = m_planner_ptr->plan_trajectory(had_map_route, lanelet_map_ptr);

  // return trajectory should not be empty
  ASSERT_FALSE(trajectory.points.empty());

  TrajectoryPoint trajectory_start_point;
  trajectory_start_point.x = static_cast<float32_t>(had_map_route.start_point.position.x);
  trajectory_start_point.y = static_cast<float32_t>(had_map_route.start_point.position.y);
  trajectory_start_point.heading = had_map_route.start_point.heading;

  // start point of trajectory should be same as start point
  const auto distance = autoware::lane_planner::distance2d(
    trajectory_start_point,
    trajectory.points.front());
  ASSERT_DOUBLE_EQ(distance, 0.0);
}

TEST_F(LanePlannerTest, PlanInvalidRoute)
{
  // create map
  const auto lane_id = lanelet::utils::getId();
  constexpr float64_t velocity_mps = 1.0;
  constexpr size_t n_points = 5;
  const auto lanelet_map_ptr = getALaneletMapWithLaneId(lane_id, velocity_mps, n_points);

  // create route message with invalid id
  const auto route = getARoute(lanelet::InvalId, 5.0F);

  const auto trajectory = m_planner_ptr->plan_trajectory(route, lanelet_map_ptr);

  // return trajectory should be empty if there is no valid lane
  ASSERT_TRUE(trajectory.points.empty());
}
