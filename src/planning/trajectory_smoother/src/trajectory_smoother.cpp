// Copyright 2020-2021 The Autoware Foundation
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

#include "trajectory_smoother/trajectory_smoother.hpp"

#include <algorithm>
#include <vector>
namespace motion
{
namespace planning
{
namespace trajectory_smoother
{

using autoware::common::types::float32_t;

TrajectorySmoother::TrajectorySmoother(TrajectorySmootherConfig config)
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

void TrajectorySmoother::Filter(Trajectory & trajectory)
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

}  // namespace trajectory_smoother
}  // namespace planning
}  // namespace motion
