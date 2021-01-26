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

/// \copyright Copyright 2020 The Autoware Foundation
/// \file
/// \brief This file defines the trajectory_smoother class.

#ifndef TRAJECTORY_SMOOTHER__TRAJECTORY_SMOOTHER_HPP_
#define TRAJECTORY_SMOOTHER__TRAJECTORY_SMOOTHER_HPP_

#include "autoware_auto_msgs/msg/trajectory.hpp"
#include <common/types.hpp>
#include <vector>
#include <cmath>

#include "trajectory_smoother/visibility_control.hpp"

namespace motion
{
namespace planning
{
namespace trajectory_smoother
{

using autoware_auto_msgs::msg::Trajectory;
using autoware::common::types::float32_t;

typedef struct
{
  float32_t standard_deviation;  // standard deviation of the gaussian kernel
  uint32_t kernel_size;  // length of the gaussian kernel
} TrajectorySmootherConfig;


/// \brief Smooth over the trajectory by passing it through a gaussian filter
class TRAJECTORY_SMOOTHER_PUBLIC TrajectorySmoother
{
public:
  /// \brief Initialise the gaussian kernel in the constructor
  /// \param[in] config Configuration containing parameters for the kernel
  explicit TrajectorySmoother(TrajectorySmootherConfig config);

  /// \brief Make the trajectory velocity smooth by passing it through a gaussian filter.
  /// \param[inout] trajectory The trajectory to be smoothed. This is modified in place.
  void Filter(Trajectory & trajectory);

private:
  std::vector<float32_t> m_kernel{};
};

}  // namespace trajectory_smoother
}  // namespace planning
}  // namespace motion

#endif  // TRAJECTORY_SMOOTHER__TRAJECTORY_SMOOTHER_HPP_
