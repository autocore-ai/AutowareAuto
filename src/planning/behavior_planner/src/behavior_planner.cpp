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

#include "behavior_planner/behavior_planner.hpp"
#include <lanelet2_core/geometry/LineString.h>
#include <geometry/common_2d.hpp>
#include <motion_common/motion_common.hpp>

#include <algorithm>
#include <limits>
#include <string>
#include <unordered_map>
#include <vector>

namespace autoware
{
namespace behavior_planner
{

using autoware::common::geometry::minus_2d;
using autoware::common::geometry::norm_2d;
using autoware::common::geometry::plus_2d;
using autoware::common::geometry::times_2d;
using autoware::common::geometry::closest_segment_point_2d;
using autoware::common::geometry::point_line_segment_distance_2d;

PlannerType get_planner_type_from_primitive(
  const MapPrimitive & map_primitive)
{
  static const std::unordered_map<std::string, PlannerType> primitive_to_planner_type({
        {"parking", PlannerType::PARKING},
        {"drivable_area", PlannerType::PARKING},
        {"lane", PlannerType::LANE}});

  const std::string & primitive_type = map_primitive.primitive_type;

  if (primitive_to_planner_type.find(primitive_type) != primitive_to_planner_type.end()) {
    return primitive_to_planner_type.at(primitive_type);
  } else {
    return PlannerType::UNKNOWN;
  }
}

autoware_auto_msgs::msg::TrajectoryPoint convert_to_trajectory_point(
  const lanelet::ConstPoint2d & pt)
{
  autoware_auto_msgs::msg::TrajectoryPoint trajectory_point;
  trajectory_point.x = static_cast<float32_t>(pt.x());
  trajectory_point.y = static_cast<float32_t>(pt.y());
  return trajectory_point;
}

TrajectoryPoint get_closest_point_on_lane(
  const TrajectoryPoint & point, const int64_t lane_id,
  const lanelet::LaneletMapPtr & lanelet_map_ptr, const float32_t offset)
{
  TrajectoryPoint closest_point_on_lane;

  const auto lanelet = lanelet_map_ptr->laneletLayer.get(lane_id);

  //  we do planning based on 2D plane
  const auto & centerline = lanelet::utils::to2D(lanelet.centerline());

  if (centerline.size() < 2) {
    std::cerr << "Could not get closest point on Lane due to invalid centerline of lane " <<
      lanelet.id() << std::endl;
    return closest_point_on_lane;
  }

  // first find the closest point on line and fine length along lane
  float32_t min_distance = std::numeric_limits<float32_t>::max();
  float32_t length_along_line = 0.0f, accumulated_length = 0.0f;
  for (size_t i = 1; i < centerline.size(); i++) {
    const auto prev_pt = convert_to_trajectory_point(centerline[i - 1]);
    const auto current_pt = convert_to_trajectory_point(centerline[i]);

    const auto distance = point_line_segment_distance_2d(prev_pt, current_pt, point);
    if (distance < min_distance) {
      min_distance = distance;
      const auto point_on_lane = closest_segment_point_2d(prev_pt, current_pt, point);
      length_along_line = accumulated_length + norm_2d(minus_2d(prev_pt, point_on_lane));
    }
    accumulated_length += norm_2d(minus_2d(prev_pt, current_pt));
  }

  // we find a point from line length with offset
  float32_t length_with_offset = length_along_line + offset;
  length_with_offset = std::max(length_with_offset, 0.0f);
  length_with_offset = std::min(length_with_offset, accumulated_length);

  accumulated_length = 0.0f;
  for (size_t i = 1; i < centerline.size(); i++) {
    const auto prev_pt = convert_to_trajectory_point(centerline[i - 1]);
    const auto current_pt = convert_to_trajectory_point(centerline[i]);
    const auto distance = norm_2d(minus_2d(prev_pt, current_pt));
    if (accumulated_length + distance >= length_with_offset) {
      const auto direction_vector = minus_2d(current_pt, prev_pt);
      const auto ratio = (length_with_offset - accumulated_length) / distance;
      closest_point_on_lane = plus_2d(prev_pt, times_2d(direction_vector, ratio));
      const float32_t angle = std::atan2(direction_vector.y, direction_vector.x);
      closest_point_on_lane.heading = motion::motion_common::from_angle(angle);
    }
    accumulated_length += distance;
  }

  return closest_point_on_lane;
}


BehaviorPlanner::BehaviorPlanner(const PlannerConfig & config)
:  m_current_subroute(0),
  m_config(config),
  m_trajectory_manager(config),
  m_is_trajectory_complete(false)
{
}

void BehaviorPlanner::clear_route()
{
  m_current_subroute = 0;
  m_subroutes.clear();
  m_trajectory_manager.clear_trajectory();
}

void BehaviorPlanner::set_route(const Route & route, const lanelet::LaneletMapPtr & lanelet_map_ptr)
{
  // create subroutes from given global route
  clear_route();

  // initialization for before loop
  RouteWithType subroute;
  subroute.route.start_point = route.start_point;
  auto prev_type = PlannerType::UNKNOWN;
  MapPrimitive prev_primitive;
  if (!route.primitives.empty()) {
    const auto & first_primitive = route.primitives.front();
    prev_type = get_planner_type_from_primitive(first_primitive);
    subroute.planner_type = prev_type;
    prev_primitive = first_primitive;
  }

  for (const auto & primitive : route.primitives) {
    const auto & type = get_planner_type_from_primitive(primitive);
    subroute.route.primitives.push_back(primitive);

    //  create subroute when Planner Type changes
    if (type != prev_type) {
      if (prev_type == PlannerType::PARKING && type == PlannerType::LANE) {
        // Determine parking direction and set offset direction accordingly
        float32_t route_offset = m_config.subroute_goal_offset_parking2lane;
        const auto closest_lane_point = get_closest_point_on_lane(
          subroute.route.start_point,
          primitive.id, lanelet_map_ptr, 0.0f);
        const auto parking_dir = get_parking_direction(
          subroute.route.start_point, closest_lane_point);

        if (parking_dir == ParkingDirection::HEAD_IN) {
          // Add extra distance for vehicle length
          route_offset = -(route_offset + 1.0f);
        }

        // create parking subroute
        // set goal to closest poin on lane from starting point
        subroute.route.goal_point = get_closest_point_on_lane(
          subroute.route.start_point,
          primitive.id, lanelet_map_ptr, route_offset);
        m_subroutes.push_back(subroute);

        // reinitialize for next subroute
        subroute.planner_type = type;
        subroute.route.primitives.clear();
        subroute.route.primitives.push_back(primitive);
        subroute.route.start_point = subroute.route.goal_point;
      }
      if (prev_type == PlannerType::LANE && type == PlannerType::PARKING) {
        // Determine parking direction and set offset direction accordingly
        float32_t route_offset = m_config.subroute_goal_offset_lane2parking;
        const auto closest_lane_point = get_closest_point_on_lane(
          route.goal_point, prev_primitive.id,
          lanelet_map_ptr, 0.0f);
        const auto parking_dir = get_parking_direction(
          route.goal_point, closest_lane_point);

        if (parking_dir == ParkingDirection::HEAD_IN) {
          // Add extra distance for vehicle length
          route_offset = -(route_offset + 1.0f);
        }

        // Currently, we assume that final goal is close to lane.
        subroute.route.goal_point = get_closest_point_on_lane(
          route.goal_point, prev_primitive.id,
          lanelet_map_ptr, route_offset);
        m_subroutes.push_back(subroute);

        // reinitialize for next subroute
        subroute.planner_type = type;
        subroute.route.primitives.clear();
        subroute.route.primitives.push_back(prev_primitive);
        subroute.route.primitives.push_back(primitive);
        subroute.route.start_point = subroute.route.goal_point;
      }
    }
    prev_type = type;
    prev_primitive = primitive;
  }

  // add final subroute
  // note that prev_type is actually type of final primitive after the loop is done
  if (prev_type == PlannerType::LANE) {
    subroute.route.goal_point = get_closest_point_on_lane(
      route.goal_point,
      prev_primitive.id, lanelet_map_ptr, 0.0f);
  } else {
    subroute.route.goal_point = route.goal_point;
  }
  m_subroutes.push_back(subroute);
}

bool8_t BehaviorPlanner::is_route_ready()
{
  return !m_subroutes.empty();
}

void BehaviorPlanner::set_next_subroute()
{
  m_current_subroute = std::min(m_current_subroute + 1, m_subroutes.size() - 1);
}

RouteWithType BehaviorPlanner::get_current_subroute(const State & ego_state)
{
  if (!is_route_ready()) {
    return RouteWithType();
  }
  auto updated_subroute = m_subroutes.at(m_current_subroute);
  updated_subroute.route.header = ego_state.header;
  if (updated_subroute.planner_type == PlannerType::LANE) {
    updated_subroute.route.start_point = ego_state.state;
  }
  return updated_subroute;
}

RouteWithType BehaviorPlanner::get_current_subroute()
{
  if (!is_route_ready()) {
    return RouteWithType();
  }
  return m_subroutes.at(m_current_subroute);
}

ParkingDirection BehaviorPlanner::get_parking_direction(
  const TrajectoryPoint & parking_point,
  const TrajectoryPoint & closest_lane_point)
{
  // Calculate angle from parking point to closest lane point
  const auto direction_vector = minus_2d(closest_lane_point, parking_point);
  const float32_t diff_angle = std::atan2(direction_vector.y, direction_vector.x);

  // Get heading angle of parking point
  const auto heading_angle = motion::motion_common::to_angle(parking_point.heading);

  // If absolute difference between angles is gt pi/2, the parking orientation is head-in
  if (std::fabs(diff_angle - heading_angle) > autoware::common::types::PI_2) {
    return ParkingDirection::HEAD_IN;
  } else {
    return ParkingDirection::TOE_IN;
  }
}

TrajectoryPoint BehaviorPlanner::get_sub_goal()
{
  return m_subroutes.at(m_current_subroute).route.goal_point;
}

PlannerType BehaviorPlanner::get_planner_type()
{
  if (m_subroutes.empty()) {
    return PlannerType::UNKNOWN;
  }
  return m_subroutes.at(m_current_subroute).planner_type;
}

bool8_t BehaviorPlanner::is_vehicle_stopped(const State & state)
{
  return std::abs(state.state.longitudinal_velocity_mps) <
         m_config.stop_velocity_thresh;
}

bool8_t BehaviorPlanner::has_arrived_goal(const State & state)
{
  if (!is_route_ready()) {
    return false;
  }

  const auto satisfy_velocity_condition = is_vehicle_stopped(state);

  const auto & route = m_subroutes.back().route;
  const auto distance = norm_2d(minus_2d(route.goal_point, state.state));
  const auto satsify_distance_condition = distance < m_config.goal_distance_thresh;

  return satisfy_velocity_condition && satsify_distance_condition;
}

bool8_t BehaviorPlanner::has_arrived_subroute_goal(const State & state)
{
  const auto satisfy_velocity_condition = is_vehicle_stopped(state);

  const auto & route = get_current_subroute().route;
  const auto distance = norm_2d(minus_2d(route.goal_point, state.state));
  const auto satsify_distance_condition = distance < m_config.goal_distance_thresh;

  return satisfy_velocity_condition && satsify_distance_condition;
}

bool8_t BehaviorPlanner::needs_new_trajectory(const State & ego_state)
{
  // we need trajectory if we don't have trajectory yet
  if (!is_trajectory_ready()) {
    return true;
  }

  // we need trajectory if trajectory does not reach to subroute goal
  const auto length = get_remaining_length(ego_state);
  const auto is_close_to_end = length < static_cast<size_t>(Trajectory::CAPACITY / 2);
  return is_close_to_end && !m_is_trajectory_complete;
}

bool8_t BehaviorPlanner::is_trajectory_ready()
{
  return m_trajectory_manager.is_trajectory_ready();
}
void BehaviorPlanner::set_trajectory(const Trajectory & trajectory)
{
  if (trajectory.points.empty()) {
    m_is_trajectory_complete = false;
    return;
  }

  m_trajectory_manager.set_trajectory(trajectory);

  const auto & last_point = trajectory.points.back();
  const auto & route = get_current_subroute().route;
  const auto distance = norm_2d(minus_2d(route.goal_point, last_point));
  m_is_trajectory_complete = distance < m_config.goal_distance_thresh;
}

Trajectory BehaviorPlanner::get_trajectory(const State & state)
{
  return m_trajectory_manager.get_trajectory(state);
}

size_t BehaviorPlanner::get_remaining_length(const State & state)
{
  return m_trajectory_manager.get_remaining_length(state);
}

uchar8_t BehaviorPlanner::get_desired_gear(const State & state)
{
  const auto trajectory = get_trajectory(state);
  for (const auto & pt : trajectory.points) {
    if (pt.longitudinal_velocity_mps > std::numeric_limits<float32_t>::epsilon()) {
      return VehicleStateCommand::GEAR_DRIVE;
    }
    if (pt.longitudinal_velocity_mps < -std::numeric_limits<float32_t>::epsilon()) {
      return VehicleStateCommand::GEAR_REVERSE;
    }
  }
  return VehicleStateCommand::GEAR_DRIVE;
}

std::vector<RouteWithType> BehaviorPlanner::get_subroutes()
{
  return m_subroutes;
}

}  // namespace behavior_planner
}  // namespace autoware
