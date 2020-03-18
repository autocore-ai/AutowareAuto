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
/// \brief This file defines the TrajectorySpoofer class.

#ifndef TRAJECTORY_SPOOFER__TRAJECTORY_SPOOFER_HPP_
#define TRAJECTORY_SPOOFER__TRAJECTORY_SPOOFER_HPP_

#include <trajectory_spoofer/visibility_control.hpp>

#include <autoware_auto_msgs/msg/complex32.hpp>
#include <autoware_auto_msgs/msg/trajectory.hpp>
#include <autoware_auto_msgs/msg/trajectory_point.hpp>
#include <autoware_auto_msgs/msg/vehicle_kinematic_state.hpp>
#include <builtin_interfaces/msg/duration.hpp>
#include <common/types.hpp>
#include <rclcpp/duration.hpp>
#include <rclcpp/rclcpp.hpp>

#include <chrono>
#include <cmath>

namespace autoware
{
namespace trajectory_spoofer
{
using Complex32 = autoware_auto_msgs::msg::Complex32;
using DurationMsg = builtin_interfaces::msg::Duration;
using Trajectory = autoware_auto_msgs::msg::Trajectory;
using TrajectoryPoint = autoware_auto_msgs::msg::TrajectoryPoint;
using VehicleKinematicState = autoware_auto_msgs::msg::VehicleKinematicState;

using autoware::common::types::bool8_t;
using autoware::common::types::float32_t;
using autoware::common::types::float64_t;
using autoware::common::types::TAU;

constexpr float64_t NANO_IN_SEC = 1000000000.0L;

class TrajectorySpoofer
{
  enum class CurveType
  {
    RIGHT_TURN = 0,
    LEFT_TURN = 1
  };

private:
  std::chrono::nanoseconds get_travel_time(float32_t dist, float32_t speed);
  Trajectory init_trajectory(const VehicleKinematicState & starting_point);

  float32_t target_speed_;

public:
  TrajectorySpoofer();
  explicit TrajectorySpoofer(float32_t target_speed);

  static Complex32 to_2d_quaternion(float64_t yaw_angle);
  static float64_t to_yaw_angle(const Complex32 & quat_2d);

  float32_t get_target_speed();
  void set_target_speed(float32_t target_speed);

  Trajectory spoof_straight_trajectory(
    const VehicleKinematicState & starting_point,
    int32_t num_of_points,
    float32_t length,
    bool8_t speed_ramp_on = false);

  Trajectory spoof_circular_trajectory(
    const VehicleKinematicState & starting_point,
    int32_t num_of_points,
    float32_t radius,
    bool8_t speed_ramp_on = false);

  Trajectory spoof_curved_trajectory(
    const VehicleKinematicState & starting_point,
    int32_t num_of_points,
    float32_t radius,
    float32_t length,
    CurveType mode = CurveType::RIGHT_TURN,
    bool8_t speed_ramp_on = false);
};
}  // namespace trajectory_spoofer
}  // namespace autoware

#endif  // TRAJECTORY_SPOOFER__TRAJECTORY_SPOOFER_HPP_
