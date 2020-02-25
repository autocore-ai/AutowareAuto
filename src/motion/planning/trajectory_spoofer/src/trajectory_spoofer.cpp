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
std::chrono::nanoseconds TrajectorySpoofer::get_travel_time(float32_t dist, float32_t speed)
{
  auto travel_time_ns = (dist / speed) * NANO_IN_SEC;
  return std::chrono::nanoseconds(static_cast<int64_t>(travel_time_ns));
}

Complex32 TrajectorySpoofer::to_2d_quaternion(float64_t yaw_angle)
{
  Complex32 heading;
  heading.real = std::cos(yaw_angle / 2.0);
  heading.imag = std::sin(yaw_angle / 2.0);
  return heading;
}

float64_t TrajectorySpoofer::to_yaw_angle(const Complex32 & quat_2d)
{
  // theta = atan2(2qxqw, 1-2(qw)^2)
  const float64_t sin_y = 2.0 * quat_2d.real * quat_2d.imag;
  const float64_t cos_y = 1.0 - 2.0 * quat_2d.imag * quat_2d.imag;
  const float64_t rad_quad = std::atan2(sin_y, cos_y);

  if (rad_quad < 0) {
    return rad_quad + CIRC_RAD;
  } else {
    return rad_quad;
  }
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
  const auto time_delta = get_travel_time(
    pos_delta, starting_point.state.longitudinal_velocity_mps);
  const auto yaw_angle = to_yaw_angle(starting_point.state.heading);
  const auto start_time = time_utils::from_message(pt.time_from_start);

  for (int i = 1; i < num_of_points; ++i) {
    pt.time_from_start = time_utils::to_message(start_time + i * time_delta);
    pt.x = std::cos(yaw_angle) * i * pos_delta;
    pt.y = std::sin(yaw_angle) * i * pos_delta;
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

  const float64_t end_heading_rad = to_yaw_angle(pt.heading);
  // Number of segments = number of points - 1
  const float64_t seg_angle_rad = CIRC_RAD / static_cast<float32_t>(num_of_points - 1);
  const float64_t seg_len = 2.0 * radius * std::sin(seg_angle_rad / 2.0);
  float64_t angle_dist_remain;
  float64_t new_head_diff;
  float64_t new_head;

  for (int i = 1; i < num_of_points; ++i) {
    // TODO(josh.whitley): Y values are incorrect
    pt.x = pt.x + std::cos(to_yaw_angle(pt.heading)) * seg_len;
    pt.y = pt.y + std::sin(to_yaw_angle(pt.heading)) * seg_len;

    angle_dist_remain = CIRC_RAD - i * seg_angle_rad;

    if (angle_dist_remain < seg_angle_rad) {
      new_head_diff = std::abs(to_yaw_angle(pt.heading) - end_heading_rad - CIRC_RAD);
    } else {
      new_head_diff = angle_dist_remain / (num_of_points - i - 1);
    }

    new_head = to_yaw_angle(pt.heading) + new_head_diff;

    // Clip to 0 to 2pi
    if (new_head < 0) {
      new_head += CIRC_RAD;
    } else if (new_head >= CIRC_RAD) {
      new_head -= CIRC_RAD;
    }

    pt.heading = to_2d_quaternion(new_head);

    circular_trajectory.points.push_back(pt);

    // TODO(josh.whitley): Calculate adjusted:
    // 1. lateral_velocity_mps
    // 2. heading_rate_rps
    // 3. front_wheel_angle_rad
    // 4. rear_wheel_angle_rad
  }

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
