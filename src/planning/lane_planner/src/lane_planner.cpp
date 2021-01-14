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

#include "lane_planner/lane_planner.hpp"
#include <lanelet2_traffic_rules/TrafficRules.h>
#include <lanelet2_traffic_rules/TrafficRulesFactory.h>
#include <lanelet2_core/geometry/Lanelet.h>
#include <lanelet2_core/geometry/LineString.h>
#include <had_map_utils/had_map_utils.hpp>
#include <geometry/common_2d.hpp>
#include <limits>
#include <algorithm>

namespace autoware
{
namespace lane_planner
{

float32_t distance2d(const TrajectoryPoint & p1, const TrajectoryPoint & p2)
{
  const auto diff = autoware::common::geometry::minus_2d(p1, p2);
  return autoware::common::geometry::norm_2d(diff);
}

// calculate curvature by circle fitting to three points
float32_t calculate_curvature(
  const TrajectoryPoint & p1, const TrajectoryPoint & p2,
  const TrajectoryPoint & p3)
{
  const float32_t epsilon = std::numeric_limits<float32_t>::epsilon();
  float32_t den = std::max(
    distance2d(p1, p2) * distance2d(
      p2,
      p3) * distance2d(
      p3, p1), epsilon);
  const float32_t curvature =
    2.0F * ((p2.x - p1.x) * (p3.y - p1.y) - (p2.y - p1.y) * (p3.x - p1.x)) / den;
  return curvature;
}

size_t get_closest_lanelet(const lanelet::ConstLanelets & lanelets, const TrajectoryPoint & point)
{
  float64_t closest_distance = std::numeric_limits<float64_t>::max();
  size_t closest_index = 0;
  for (size_t i = 0; i < lanelets.size(); i++) {
    const auto & llt = lanelets.at(i);
    const auto & point2d = lanelet::Point2d(lanelet::InvalId, point.x, point.y).basicPoint2d();
    // TODO(mitsudome-r): change this implementation to remove dependency to boost
    const float64_t distance = lanelet::geometry::distanceToCenterline2d(llt, point2d);
    if (distance < closest_distance) {
      closest_distance = distance;
      closest_index = i;
    }
  }
  return closest_index;
}


LanePlanner::LanePlanner(
  const VehicleConfig & vehicle_param,
  const TrajectorySmootherConfig & config,
  const LanePlannerConfig & planner_config)
: m_vehicle_param(vehicle_param),
  m_planner_config(planner_config),
  m_trajectory_smoother(config)
{
}

autoware_auto_msgs::msg::TrajectoryPoint convertToTrajectoryPoint(
  const lanelet::ConstPoint3d & pt,
  const float32_t velocity)
{
  autoware_auto_msgs::msg::TrajectoryPoint trajectory_point;
  trajectory_point.x = static_cast<float32_t>(pt.x());
  trajectory_point.y = static_cast<float32_t>(pt.y());
  trajectory_point.longitudinal_velocity_mps = velocity;
  return trajectory_point;
}

lanelet::Point3d convertToLaneletPoint(
  const autoware_auto_msgs::msg::TrajectoryPoint & pt)
{
  return lanelet::Point3d(lanelet::InvalId, pt.x, pt.y, 0.0);
}

Trajectory LanePlanner::plan_trajectory(
  const autoware_auto_msgs::msg::Route & route,
  const lanelet::LaneletMapConstPtr & map)
{
  // generate trajectory. Only x, y, and velocity is filled in at this point
  auto trajectory_points = generate_base_trajectory(route, map);

  // calculate missing fields in trajectory
  set_angle(&trajectory_points);

  set_steering_angle(&trajectory_points);

  set_time_from_start(&trajectory_points);

  auto trajectory = create_trajectory_message(route.header, trajectory_points);

  modify_velocity(&trajectory);

  return trajectory;
}

void LanePlanner::modify_velocity(Trajectory * trajectory)
{
  if (trajectory->points.empty()) {
    return;
  }

  // always set zero velocity at the end of trajectory for safety.
  auto & last_pt = trajectory->points.back();
  last_pt.longitudinal_velocity_mps = 0.0F;
  last_pt.lateral_velocity_mps = 0.0F;
  last_pt.acceleration_mps2 = 0.0F;
  last_pt.heading_rate_rps = 0.0F;
  m_trajectory_smoother.Filter(*trajectory);
}

TrajectoryPoints LanePlanner::generate_base_trajectory(
  const Route & route,
  const LaneletMapConstPtr & map)
{
  using lanelet::utils::to2D;

  lanelet::ConstLanelets lanelets;
  for (const auto & primitive : route.primitives) {
    try {
      const auto lane = map->laneletLayer.get(primitive.id);
      lanelets.push_back(lane);
    } catch (const lanelet::NoSuchPrimitiveError & ex) {
      // stop adding lanelets if lane cannot be found. e.g. goal is outside of queried submap
      break;
    }
  }

  // return empty trajectory if there are no lanes
  if (lanelets.empty()) {
    return TrajectoryPoints();
  }

  const auto start_index = get_closest_lanelet(lanelets, route.start_point);

  TrajectoryPoints trajectory_points;

  // using Germany Location since it is default location for Lanelet2
  // TODO(mitsudome-r): create define default location for Autoware
  lanelet::traffic_rules::TrafficRulesPtr traffic_rules_ptr =
    lanelet::traffic_rules::TrafficRulesFactory::create(
    lanelet::Locations::Germany,
    lanelet::Participants::Vehicle);

  // set position and velocity
  trajectory_points.push_back(route.start_point);
  for (size_t i = start_index; i < lanelets.size(); i++) {
    const auto & lanelet = lanelets.at(i);
    const auto & centerline = autoware::common::had_map_utils::generateFineCenterline(
      lanelet,
      m_planner_config.trajectory_resolution);
    const auto speed_limit =
      static_cast<float32_t>(traffic_rules_ptr->speedLimit(lanelet).speedLimit.value());

    float64_t start_length = 0;
    if (i == start_index) {
      const auto start_point = convertToLaneletPoint(route.start_point);
      start_length =
        lanelet::geometry::toArcCoordinates(to2D(centerline), to2D(start_point)).length;
    }

    float64_t end_length = std::numeric_limits<float32_t>::max();
    if (i == lanelets.size() - 1) {
      const auto goal_point = convertToLaneletPoint(route.goal_point);
      end_length = lanelet::geometry::toArcCoordinates(to2D(centerline), to2D(goal_point)).length;
    }

    float64_t accumulated_length = 0;
    // skip first point to avoid inserting overlaps
    for (size_t j = 1; j < centerline.size(); j++) {
      const auto llt_prev_pt = centerline[j - 1];
      const auto llt_pt = centerline[j];
      accumulated_length += lanelet::geometry::distance2d(to2D(llt_prev_pt), to2D(llt_pt));
      if (accumulated_length < start_length) {continue;}
      if (accumulated_length > end_length) {break;}
      trajectory_points.push_back(convertToTrajectoryPoint(llt_pt, speed_limit));
    }
  }
  trajectory_points.push_back(route.goal_point);
  return trajectory_points;
}

void LanePlanner::set_angle(TrajectoryPoints * trajectory_points)
{
  for (size_t i = 0; i < trajectory_points->size(); i++) {
    float32_t angle = 0;
    auto & pt = trajectory_points->at(i);
    if (i + 1 < trajectory_points->size()) {
      const auto & next_pt = trajectory_points->at(i + 1);
      angle = std::atan2(
        next_pt.y - pt.y,
        next_pt.x - pt.x);
    } else if (i != 0) {
      const auto & prev_pt = trajectory_points->at(i - 1);
      angle = std::atan2(
        pt.y - prev_pt.y,
        pt.x - prev_pt.x);
    }
    // TODO(mitsudome-r): do faster computation of heading from diff_x, and diff_y
    pt.heading.real = std::cos(angle / 2.0F);
    pt.heading.imag = std::sin(angle / 2.0F);
  }
}

void LanePlanner::set_steering_angle(TrajectoryPoints * trajectory_points)
{
  if (trajectory_points->empty()) {
    return;
  }

  // set steering angle
  const auto wheel_base = m_vehicle_param.length_cg_front_axel() +
    m_vehicle_param.length_cg_rear_axel();
  for (size_t i = 1; i < trajectory_points->size() - 1; i++) {
    auto & pt = trajectory_points->at(i);
    const auto & prev_pt = trajectory_points->at(i - 1);
    const auto & next_pt = trajectory_points->at(i + 1);
    const auto curvature = calculate_curvature(prev_pt, pt, next_pt);
    pt.front_wheel_angle_rad = std::atan(wheel_base * curvature);
  }
}

void LanePlanner::set_time_from_start(TrajectoryPoints * trajectory_points)
{
  if (trajectory_points->empty()) {
    return;
  }

  // set time_from_start
  double accumulated_time = 0;
  trajectory_points->at(0).time_from_start.sec = 0;
  trajectory_points->at(0).time_from_start.nanosec = 0;
  for (size_t i = 1; i < trajectory_points->size(); i++) {
    auto & pt = trajectory_points->at(i);
    const auto & prev_pt = trajectory_points->at(i - 1);
    const double distance_x = pt.x - prev_pt.x;
    const double distance_y = pt.y - prev_pt.y;
    const double distance = std::sqrt(distance_x * distance_x + distance_y * distance_y);
    const double velocity = prev_pt.longitudinal_velocity_mps;
    accumulated_time += distance / std::max(velocity, 0.5);
    std::chrono::nanoseconds duration(static_cast<int64_t>(accumulated_time * 1e9));
    pt.time_from_start = time_utils::to_message(duration);
  }
}

Trajectory LanePlanner::create_trajectory_message(
  const std_msgs::msg::Header & header,
  const TrajectoryPoints & trajectory_points)
{
  Trajectory trajectory;
  trajectory.header = header;
  size_t trajectory_length =
    std::min(trajectory_points.size(), static_cast<size_t>(Trajectory::CAPACITY));
  trajectory.points.resize(trajectory_length);
  for (size_t i = 0; i < trajectory_length; i++) {
    trajectory.points.at(i) = trajectory_points.at(i);
  }
  return trajectory;
}

}  // namespace lane_planner
}  // namespace autoware
