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

#include "trajectory_spoofer/trajectory_spoofer.hpp"

#include <time_utils/time_utils.hpp>

#include <chrono>
#include <cmath>

namespace autoware
{
namespace trajectory_spoofer
{
Complex32 TrajectorySpoofer::to_2d_quaternion(float32_t yaw_angle)
{
  Complex32 heading;
  heading.real = std::cos(yaw_angle / 2.0);
  heading.imag = std::sin(yaw_angle / 2.0);
  return heading;
}

float32_t TrajectorySpoofer::to_yaw_angle(const Complex32 & quat_2d)
{
  // theta = atan2(2qxqw, 1-2(qw)^2)
  float32_t sin_y = 2.0 * quat_2d.real * quat_2d.imag;
  float32_t cos_y = 1.0 - 2.0 * quat_2d.imag * quat_2d.imag;
  return std::atan2(sin_y, cos_y);
}

Trajectory TrajectorySpoofer::spoof_straight_trajectory(
  const VehicleKinematicState & starting_point,
  int32_t num_of_points,
  float32_t length,
  bool velocity_ramp_on)
{
  Trajectory straight_trajectory;

  // Start from the current location
  TrajectoryPoint pt = starting_point.state;
  straight_trajectory.points.push_back(pt);

  const float32_t pos_delta = length / static_cast<float32_t>(num_of_points - 1);
  const auto time_delta = std::chrono::nanoseconds(
    static_cast<int64_t>(
      (pos_delta / starting_point.state.longitudinal_velocity_mps) * nano_in_sec));
  const auto yaw_angle = to_yaw_angle(starting_point.state.heading);

  for (int i = 1; i < num_of_points; ++i) {
    pt.time_from_start = time_utils::to_message(
      time_utils::from_message(pt.time_from_start) + time_delta);
    pt.x += std::cos(yaw_angle) * pos_delta;
    pt.y += std::sin(yaw_angle) * pos_delta;
    straight_trajectory.points.push_back(pt);
  }

  return straight_trajectory;
}

Trajectory TrajectorySpoofer::spoof_circular_trajectory(
  const VehicleKinematicState & starting_point,
  int32_t num_of_points,
  float32_t radius)
{
  Trajectory circular_trajectory;

  // Start from the current location
  TrajectoryPoint pt = starting_point.state;
  circular_trajectory.points.push_back(pt);

  // TODO(josh.whitley): Populate

  return circular_trajectory;
}

Trajectory TrajectorySpoofer::spoof_curved_trajectory(
  const VehicleKinematicState & starting_point,
  int32_t num_of_points,
  float32_t radius,
  float32_t length,
  CurveType mode,
  bool velocity_ramp_on)
{
  Trajectory curved_trajectory;

  // Start from the current location
  TrajectoryPoint pt = starting_point.state;
  curved_trajectory.points.push_back(pt);

  // TODO(josh.whitley): Populate

  return curved_trajectory;
}
}  // namespace trajectory_spoofer
}  // namespace autoware
