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

namespace autoware
{
namespace trajectory_spoofer
{
TrajectoryPoint TrajectorySpoofer::create_trajectory_point(
  float32_t x, float32_t y,
  float32_t yaw_angle_rad,
  float32_t longitudinal_velocit_mps,
  float32_t lateral_velocity_mps,
  float32_t acceleration_mps,
  float32_t heading_rate_rps,
  float32_t front_wheel_angle_rad,
  float32_t rear_wheel_angle_rad)
{
  TrajectoryPoint point;

  // TODO(josh.whitley): Populate

  return point;
}

Trajectory TrajectorySpoofer::spoof_straight_trajectory(
  VehicleKinematicState starting_point,
  int32_t num_of_points,
  float32_t length,
  bool velocity_ramp_on)
{
  Trajectory straight_trajectory;

  // TODO(josh.whitley): Populate

  return straight_trajectory;
}

Trajectory TrajectorySpoofer::spoof_circular_trajectory(
  VehicleKinematicState starting_point,
  int32_t num_of_points,
  float32_t radius)
{
  Trajectory circular_trajectory;

  // TODO(josh.whitley): Populate

  return circular_trajectory;
}

Trajectory TrajectorySpoofer::spoof_curved_trajectory(
  VehicleKinematicState starting_point,
  int32_t num_of_points,
  float32_t radius,
  float32_t length,
  CurveType mode,
  bool velocity_ramp_on)
{
  Trajectory curved_trajectory;

  // TODO(josh.whitley): Populate

  return curved_trajectory;
}
}  // namespace trajectory_spoofer
}  // namespace autoware
