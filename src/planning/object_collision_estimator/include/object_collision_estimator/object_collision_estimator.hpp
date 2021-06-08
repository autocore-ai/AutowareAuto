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

#ifndef OBJECT_COLLISION_ESTIMATOR__OBJECT_COLLISION_ESTIMATOR_HPP_
#define OBJECT_COLLISION_ESTIMATOR__OBJECT_COLLISION_ESTIMATOR_HPP_

#include <autoware_auto_msgs/msg/trajectory.hpp>
#include <autoware_auto_msgs/msg/trajectory_point.hpp>
#include <autoware_auto_msgs/msg/bounding_box_array.hpp>
#include <autoware_auto_msgs/msg/bounding_box.hpp>
#include <motion_common/config.hpp>
#include <trajectory_smoother/trajectory_smoother.hpp>
#include <common/types.hpp>
#include <algorithm>
#include <vector>
#include <cmath>

#include "object_collision_estimator/visibility_control.hpp"

namespace motion
{
namespace planning
{
namespace object_collision_estimator
{

using motion::motion_common::VehicleConfig;
using motion::planning::trajectory_smoother::TrajectorySmoother;
using autoware_auto_msgs::msg::Trajectory;
using autoware_auto_msgs::msg::TrajectoryPoint;
using autoware_auto_msgs::msg::BoundingBox;
using autoware_auto_msgs::msg::BoundingBoxArray;
using autoware::common::types::float32_t;
using autoware::common::types::PI;

typedef struct
{
  // configuration related to the vehicle dimensions
  VehicleConfig vehicle_config;

  // safety factor to artificially inflate the size of the vehicle to prevent ego vehicle getting
  // too close to any obstacles
  float32_t safety_factor;
  float32_t stop_margin;
  float32_t min_obstacle_dimension_m;
} ObjectCollisionEstimatorConfig;

/// \brief Given a trajectory and a list of obstacles, detect possible collision points between the
/// ego vehicle and obstacle along the trajectory. Modify the trajectory so that the vehicle stops
/// before the collision.
class OBJECT_COLLISION_ESTIMATOR_PUBLIC ObjectCollisionEstimator
{
public:
  /// \brief Construct a new Object Collision Estimator object
  /// \param[in] config configuration struct for the estimator
  /// \param[in] smoother trajectory smoother to be used during modification of trajectory
  explicit ObjectCollisionEstimator(
    ObjectCollisionEstimatorConfig config,
    TrajectorySmoother smoother) noexcept;

  /// \brief Update the list of obstacles
  /// \details The collision estimator stores a list of obstacle. Coordinates should be in the same
  ///          frame as the the trajectories. When this function is called, the old list is replaced
  ///          with the new list passed as the parameter.
  ///          Modify bounding boxes with edges smaller than min_obstacle_dimension_m by computing
  ///          new corners from their centroid and orientation values. The orientation and centroid
  ///          of the modified bounding boxes are left unchanged.
  ///          Assumptions:
  ///           * the points are in counterclockwise order
  ///           * box.size.x and box.size.y map to the length of the first and second edges
  ///             respectively
  /// \param[in] bounding_boxes A array of bounding boxes representing a list of obstacles
  /// \returns A vector of obstacles modified for being smaller than min_obstacle_dimension_m
  std::vector<BoundingBox> updateObstacles(
    const BoundingBoxArray & bounding_boxes) noexcept;

  /// \brief Perform collision detection given an trajectory
  /// \details the list of obstacles should be passed to the estimator with a prior call to
  ///          updateObstacles. When a collision is detected, the trajectory is modified in place
  ///          such that the velocity of the ego vehicle becomes 0 before the collision point.
  /// \param[inout] trajectory The intended trajectory of the ego vehicle. If a collision is
  ///               detected, this variable is modified in place.
  void updatePlan(Trajectory & trajectory) noexcept;

  /// \brief Get the latest bounding box of target trajectory
  /// \returns The latest bounding box of the target trajectory
  BoundingBoxArray getTrajectoryBoundingBox() const {return m_trajectory_bboxes;}

private:
  ObjectCollisionEstimatorConfig m_config;
  BoundingBoxArray m_obstacles{};
  BoundingBoxArray m_trajectory_bboxes{};
  TrajectorySmoother m_smoother;
};

}  // namespace object_collision_estimator
}  // namespace planning
}  // namespace motion

#endif  // OBJECT_COLLISION_ESTIMATOR__OBJECT_COLLISION_ESTIMATOR_HPP_
