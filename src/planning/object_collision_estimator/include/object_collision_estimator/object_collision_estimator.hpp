// Copyright 2020 Arm Limited
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
using autoware_auto_msgs::msg::Trajectory;
using autoware_auto_msgs::msg::TrajectoryPoint;
using autoware_auto_msgs::msg::BoundingBox;
using autoware_auto_msgs::msg::BoundingBoxArray;
using autoware::common::types::float32_t;
using autoware::common::types::PI;

typedef struct
{
  float32_t standard_deviation;  // standard deviation of the gaussian kernel
  uint32_t kernel_size;  // length of the gaussian kernel
} TrajectorySmootherConfig;

/// \brief Smooth over the trajectory by passing it through a gaussian filter
class OBJECT_COLLISION_ESTIMATOR_PUBLIC TrajectorySmoother
{
public:
  /// \brief Initialise the gaussian kernel in the constructor
  /// \param[in] config Configuration containing parameters for the kernel
  explicit TrajectorySmoother(TrajectorySmootherConfig config)
  {
    float32_t s = 2 * config.standard_deviation * config.standard_deviation;
    float32_t sum = 0.0;

    // Generate a gaussian filter kernel
    for (std::size_t i = 0; i < config.kernel_size; ++i) {
      int32_t x = static_cast<int32_t>(i - config.kernel_size / 2);
      float32_t value = std::exp(static_cast<float32_t>(-(x * x)) / s);
      m_kernel.push_back(value);
      sum += value;
    }

    // Normalize the kernel
    for (std::size_t i = 0; i < config.kernel_size; ++i) {
      m_kernel[i] /= sum;
    }
  }

  /// \brief Make the trajectory velocity smooth by passing it through a gaussian filter.
  /// \param[inout] trajectory The trajectory to be smoothed. This is modified in place.
  void Filter(Trajectory & trajectory)
  {
    if (trajectory.points.size() > 2) {
      // zero out velocity at a few points at the end of trajectory so that the post filter velocity
      // gradually ramp down to zero. The last point would have already been zeroed by the
      // estimator.
      std::size_t zero_run_length = std::min(trajectory.points.size() / 2, m_kernel.size() / 2);
      for (std::size_t i = trajectory.points.size() - 1 - zero_run_length;
        i < trajectory.points.size() - 1; ++i)
      {
        trajectory.points[i].longitudinal_velocity_mps = 0;
      }

      // avoid changing the start and end point of trajectory
      // use same padding for points beyond either end of the trajectory
      std::vector<float32_t> velocity_profile{};
      for (std::size_t i = 1; i < trajectory.points.size() - 1; ++i) {
        float32_t sum = 0;
        for (std::size_t j = 0; j < m_kernel.size(); ++j) {
          std::int32_t points_index = static_cast<int32_t>(i - (m_kernel.size() / 2) + j);
          if (points_index < 0) {
            points_index = 0;
          } else if (points_index >= static_cast<int32_t>(trajectory.points.size())) {
            points_index = static_cast<int32_t>(trajectory.points.size() - 1);
          }
          sum +=
            trajectory.points[static_cast<std::size_t>(points_index)].longitudinal_velocity_mps *
            m_kernel[j];
        }

        velocity_profile.push_back(sum);
      }

      // Apply the velocity profile
      for (std::size_t i = 1; i < trajectory.points.size() - 1; ++i) {
        trajectory.points[i].longitudinal_velocity_mps = velocity_profile[i - 1];
      }
    }
  }

private:
  std::vector<float32_t> m_kernel{};
};

typedef struct
{
  // configuration related to the vehicle dimensions
  VehicleConfig vehicle_config;

  // safety factor to artificially inflate the size of the vehicle to prevent ego vehicle getting
  // too close to any obstacles
  float32_t safety_factor;
  float32_t stop_margin;
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
  /// \param[in] bounding_boxes A array of bounding boxes representing a list of obstacles
  void updateObstacles(const BoundingBoxArray & bounding_boxes) noexcept;

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
