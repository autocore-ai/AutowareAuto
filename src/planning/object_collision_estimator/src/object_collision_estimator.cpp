// Copyright 2020-2021 Arm Limited
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <geometry/intersection.hpp>
#include <motion_common/config.hpp>
#include <geometry_msgs/msg/point32.hpp>
#include <geometry/bounding_box/bounding_box_common.hpp>
#include <geometry/bounding_box/rotating_calipers.hpp>
#include <common/types.hpp>
#include <algorithm>
#include <list>
#include <vector>

#include "object_collision_estimator/object_collision_estimator.hpp"

namespace motion
{
namespace planning
{
namespace object_collision_estimator
{

using autoware::common::geometry::bounding_box::minimum_perimeter_bounding_box;
using autoware::common::geometry::get_normal;
using autoware::common::geometry::minus_2d;
using autoware::common::geometry::plus_2d;
using autoware::common::geometry::rotate_2d;
using autoware::common::geometry::times_2d;
using autoware::common::types::float32_t;
using motion::motion_common::to_angle;
using motion::planning::trajectory_smoother::TrajectorySmoother;
using geometry_msgs::msg::Point32;

/// \brief Convert a trajectory waypoint into a bounding box representing the volume occupied by the
///        ego vehicle while on this waypoint.
/// \param pt A waypoint of a trajectory
/// \param vehicle_param Ego vehicle parameter defining its dimensions
/// \param safety_factor A factor to inflate the size of the vehicle so to avoid getting too close
///                      to obstacles.
/// \return BoundingBox The box bounding the ego vehicle at the waypoint.
BoundingBox waypointToBox(
  const TrajectoryPoint & pt,
  const VehicleConfig & vehicle_param,
  const float32_t safety_factor)
{
  // Shorthands to keep the formulas sane
  float32_t lf = vehicle_param.length_cg_front_axel() + vehicle_param.front_overhang();
  float32_t lr = vehicle_param.length_cg_rear_axel() + vehicle_param.rear_overhang();
  float32_t wh = vehicle_param.width() * 0.5f;
  float32_t heading = to_angle(pt.heading);
  float32_t ch = std::cos(heading);
  float32_t sh = std::sin(heading);

  // inflate size of vehicle by safety factor
  lf *= safety_factor;
  lr *= safety_factor;
  wh *= safety_factor;

  // Create a list of corners for the vehicle
  std::list<Point32> vehicle_corners;

  {     // Front left
    auto p = Point32{};
    p.x = pt.x + (lf * ch) - (wh * sh);
    p.y = pt.y + (lf * sh) + (wh * ch);
    vehicle_corners.push_back(p);
  }
  {     // Front right
    auto p = Point32{};
    p.x = pt.x + (lf * ch) + (wh * sh);
    p.y = pt.y + (lf * sh) - (wh * ch);
    vehicle_corners.push_back(p);
  }
  {     // Rear right
    auto p = Point32{};
    p.x = pt.x - (lr * ch) + (wh * sh);
    p.y = pt.y - (lr * sh) - (wh * ch);
    vehicle_corners.push_back(p);
  }
  {     // Rear left
    auto p = Point32{};
    p.x = pt.x - (lr * ch) - (wh * sh);
    p.y = pt.y - (lr * sh) + (wh * ch);
    vehicle_corners.push_back(p);
  }

  return minimum_perimeter_bounding_box(vehicle_corners);
}

/// \brief Determine if a obstacle is too far away from a way point given a distance threshold
/// \param way_point a trajectory way point
/// \param obstacle_bbox a bounding box with 4 corners representing the obstacle
/// \param distance_threshold the minimum distance for a obstacle to be determined too far away
/// \return bool8_t Return true if the bounding box of the obstacle is at least distance_threshold
///         away from the way point.
bool8_t isTooFarAway(
  const TrajectoryPoint & way_point, const BoundingBox & obstacle_bbox,
  const float32_t distance_threshold)
{
  bool is_too_far_away{true};
  auto distance_threshold_squared = distance_threshold * distance_threshold;

  for (const auto & corner : obstacle_bbox.corners) {
    auto dx = corner.x - way_point.x;
    auto dy = corner.y - way_point.y;
    auto distance_squared = (dx * dx) + (dy * dy);

    if (distance_threshold_squared > distance_squared) {
      is_too_far_away = false;
      break;
    }
  }

  return is_too_far_away;
}

/// \brief Detect possible collision between a trajectory and a list of obstacle bounding boxes.
///        Return the index in the trajectory where the first collision happens.
/// \param trajectory Planned trajectory of ego vehicle.
/// \param obstacles Array of bounding boxes of detected obstacles.
/// \param vehicle_param Configuration regarding the dimensions of the ego vehicle
/// \param safety_factor A factor to inflate the size of the vehicle so to avoid getting too close
///                      to obstacles.
/// \param waypoint_bboxes A list of bounding boxes around each waypoint in the trajectory
/// \return int32_t The index into the trajectory points where the first collision happens. If no
///         collision is detected, -1 is returned.
int32_t detectCollision(
  const Trajectory & trajectory,
  const BoundingBoxArray & obstacles,
  const VehicleConfig & vehicle_param,
  const float32_t safety_factor,
  BoundingBoxArray & waypoint_bboxes)
{
  // find the dimension of the ego vehicle.
  const auto vehicle_length =
    vehicle_param.front_overhang() + vehicle_param.length_cg_front_axel() +
    vehicle_param.length_cg_rear_axel() + vehicle_param.rear_overhang();
  const auto vehicle_width = vehicle_param.width();
  const auto vehicle_diagonal = sqrtf(
    (vehicle_width * vehicle_width) + (vehicle_length * vehicle_length));

  // define a distance threshold to filter obstacles that are too far away to cause any collision.
  const float32_t distance_threshold{vehicle_diagonal * safety_factor};

  int32_t collision_index = -1;

  waypoint_bboxes.boxes.clear();
  for (std::size_t i = 0; i < trajectory.points.size(); ++i) {
    waypoint_bboxes.boxes.push_back(
      waypointToBox(trajectory.points[i], vehicle_param, safety_factor));
  }
  for (std::size_t i = 0; (i < trajectory.points.size()) && (collision_index == -1); ++i) {
    // calculate a bounding box given a trajectory point
    const auto & waypoint_bbox = waypoint_bboxes.boxes.at(i);

    // Check for collisions with all perceived obstacles
    for (const auto & obstacle_bbox : obstacles.boxes) {
      if (!isTooFarAway(
          trajectory.points[i], obstacle_bbox,
          distance_threshold) && autoware::common::geometry::intersect(
          waypoint_bbox.corners.begin(), waypoint_bbox.corners.end(),
          obstacle_bbox.corners.begin(), obstacle_bbox.corners.end()))
      {
        // Collision detected, set end index (non-inclusive), this will end outer loop immediately
        collision_index = static_cast<decltype(collision_index)>(i);
        break;
      }
    }
  }

  return collision_index;
}

/// \brief Returns the index that vehicle should stop when the object colliding index
///        and stop distance is given
/// \param trajectory Planned trajectory of ego vehicle.
/// \param collision_index Index of trajectory point that collides with and obstacle
/// \param stop_margin Distance between the control point of vehicle (CoG or base_link) and obstacle
/// \return int32_t The index into the trajectory points where vehicle should stop.
int32_t getStopIndex(
  const Trajectory & trajectory,
  const int32_t collision_index,
  const float32_t stop_margin) noexcept
{
  if (collision_index < 0) {
    return collision_index;
  }
  int32_t stop_index = collision_index;
  float32_t accumulated_distance = 0;

  for (int32_t i = collision_index; i >= 1; i--) {
    const auto & prev_pt = trajectory.points.at(static_cast<std::size_t>(i - 1));
    const auto & pt = trajectory.points.at(static_cast<std::size_t>(i));

    const auto dx = prev_pt.x - pt.x;
    const auto dy = prev_pt.y - pt.y;
    accumulated_distance += std::hypot(dx, dy);
    if (accumulated_distance >= stop_margin) {
      stop_index = i;
      break;
    }
  }
  return stop_index;
}

ObjectCollisionEstimator::ObjectCollisionEstimator(
  ObjectCollisionEstimatorConfig config,
  TrajectorySmoother smoother) noexcept
: m_config(config), m_smoother(smoother)
{
  // safety factor could not be smaller than 1
  if (m_config.safety_factor < 1.0f) {
    m_config.safety_factor = 1.0f;
  }
}

void ObjectCollisionEstimator::updatePlan(Trajectory & trajectory) noexcept
{
  // Collision detection
  auto collision_index = detectCollision(
    trajectory, m_obstacles, m_config.vehicle_config,
    m_config.safety_factor, m_trajectory_bboxes);

  auto trajectory_end_idx = getStopIndex(trajectory, collision_index, m_config.stop_margin);

  if (trajectory_end_idx >= 0) {
    // Cut trajectory short to just before the collision point
    trajectory.points.resize(static_cast<std::size_t>(trajectory_end_idx));

    if (trajectory_end_idx >= 1) {
      // Mark the last point along the trajectory as "stopping" by setting
      // accelerations and velocities to zero.
      const auto trajectory_last_idx = static_cast<std::size_t>(trajectory_end_idx - 1);

      trajectory.points[trajectory_last_idx].longitudinal_velocity_mps = 0.0;
      trajectory.points[trajectory_last_idx].acceleration_mps2 = 0.0;

      // smooth the velocity profile of the trajectory so that it is realistically
      // executable.
      m_smoother.Filter(trajectory);
    }
  }
}

std::vector<BoundingBox> ObjectCollisionEstimator::updateObstacles(
  const BoundingBoxArray & bounding_boxes) noexcept
{
  m_obstacles = bounding_boxes;

  std::vector<BoundingBox> modified_obstacles;
  for (auto & box : m_obstacles.boxes) {
    if (std::min(box.size.x, box.size.y) < m_config.min_obstacle_dimension_m) {
      Point32 heading;
      heading.x = box.orientation.w;
      heading.y = box.orientation.z;
      // Double the quaternion's angle to get the heading, which is a unit vector colinear to the
      // y-axis.
      rotate_2d(heading, heading.x, heading.y);

      // Compute base vectors of the new bounding box. Those vectors have the new desired length so
      // that the corners are scaled at an equal distance on each side.
      box.size.x = std::max(box.size.x, m_config.min_obstacle_dimension_m);
      box.size.y = std::max(box.size.y, m_config.min_obstacle_dimension_m);
      auto vect_x = times_2d(minus_2d(get_normal(heading)), box.size.x / 2);
      auto vect_y = times_2d(heading, box.size.y / 2);

      // Bottom left corner: -x-y
      box.corners[0] = plus_2d(box.centroid, minus_2d(plus_2d(vect_x, vect_y)));
      // Bottom right corner: x-y
      box.corners[1] = plus_2d(box.centroid, minus_2d(vect_x, vect_y));
      // Top right corner: x+y
      box.corners[2] = plus_2d(box.centroid, plus_2d(vect_x, vect_y));
      // Top left corner: -x+y
      box.corners[3] = plus_2d(box.centroid, minus_2d(vect_y, vect_x));

      modified_obstacles.push_back(box);
    }
  }

  return modified_obstacles;
}

}  // namespace object_collision_estimator
}  // namespace planning
}  // namespace motion
