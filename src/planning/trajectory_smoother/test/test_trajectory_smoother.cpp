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

#include <motion_testing/motion_testing.hpp>
#include <cmath>

#include "gtest/gtest.h"
#include "trajectory_smoother/trajectory_smoother.hpp"

#define DT_MS 100

using motion::motion_testing::constant_velocity_trajectory;
using motion::motion_testing::constant_acceleration_trajectory;
using Trajectory = autoware_auto_msgs::msg::Trajectory;
using TrajectorySmoother = motion::planning::trajectory_smoother::TrajectorySmoother;
using TrajectorySmootherConfig = motion::planning::trajectory_smoother::TrajectorySmootherConfig;

// Introduce random noise in the velocity values.
void introduce_noise(Trajectory * trajectory, float range)
{
  std::mt19937 gen(0);
  std::normal_distribution<float> dis(0, range / 5);
  for (auto & point : trajectory->points) {
    point.longitudinal_velocity_mps += dis(gen);
  }
}

void dbg_print_traj(const Trajectory & trajectory)
{
  ASSERT_GE(trajectory.points.size(), 3U);
  const float dt = static_cast<float>(DT_MS) / 1000;
  float v0 = (trajectory.points.cbegin() + 1)->longitudinal_velocity_mps;
  float a0 = (v0 - trajectory.points.cbegin()->longitudinal_velocity_mps) / dt;
  std::cerr << trajectory.points.front().longitudinal_velocity_mps << ", 0, 0" << std::endl;
  std::cerr << v0 << ", " << a0 << ", 0" << std::endl;
  for (auto it = trajectory.points.cbegin() + 2; it != trajectory.points.cend(); it++) {
    float v1 = it->longitudinal_velocity_mps;
    float a1 = (v1 - v0) / dt;
    float jerk = (a1 - a0) / dt;
    v0 = v1;
    a0 = a1;
    std::cerr << v1 << ", " << a1 << ", " << jerk << std::endl;
  }
}

/// \brief Assert that a trajectory stays inside given limits in terms of acceleration and jerk.
/// \param[in] trajectory The trajectory
/// \param[in] max_acc Absolute value of the maximum allowed acceleration, [m/s^2]
/// \param[in] max_jerk Absolute value of the maximum allowed jerk: the derivative of the
/// acceleration, [m/s^3]
void assert_trajectory(const Trajectory & trajectory, float max_acc, float max_jerk)
{
  ASSERT_GE(max_acc, 0.0F);
  ASSERT_GE(max_jerk, 0.0F);
  ASSERT_GE(trajectory.points.size(), 3U);
  const float dt = static_cast<float>(DT_MS) / 1000;
  float v0 = (trajectory.points.cbegin() + 1)->longitudinal_velocity_mps;
  float a0 = (v0 - trajectory.points.cbegin()->longitudinal_velocity_mps) / dt;
  for (auto it = trajectory.points.cbegin() + 2; it != trajectory.points.cend(); it++) {
    float v1 = it->longitudinal_velocity_mps;
    float a1 = (v1 - v0) / dt;
    EXPECT_LE(std::abs(a1), max_acc);
    float jerk = (a1 - a0) / dt;
    // Ignore the first jerk value.
    // The trajectory smoother keeps the velocity value of the first trajectory point, which creates
    // an outstanding value here.
    if (it != trajectory.points.cbegin() + 2) {
      EXPECT_LE(std::abs(jerk), max_jerk);
    }
    v0 = v1;
    a0 = a1;
  }
}

/// \brief Assert that a trajectory stays inside some limits in terms of acceleration and jerk.
/// The trajectory is expected to come to a stop in a given amount of time. This function computes
/// theoretical maximum values before asserting that the trajectory respects them.
/// \param[in] trajectory The trajectory
/// \param[in] velocity The velocity of the vehicle when the vehicle starts braking, [m/s]
/// \param[in] stop_time The time given for the vehicle to come to a stop, [s]
void assert_trajectory_stop(const Trajectory & trajectory, float velocity, float stop_time)
{
  ASSERT_GE(velocity, 0.0F);
  ASSERT_GT(stop_time, 0.0F);
  // Theoretical lower bound of the maximum acceleration and jerk, based on a step-function jerk
  // with value -max_jerk from 0 to stop_time/2 and max_jerk from stop_time/2 to stop_time. The
  // actual acceleration and jerk will be higher, but should be somewhat proportional to those
  // values.
  const float lower_bound_max_acc = velocity / stop_time;
  const float lower_bound_max_jerk = 4 * velocity / (stop_time * stop_time);
  // The safety factor is a magic number tying the lower bound to an acceptable upper bound.
  const float safety_factor = 2.5;
  const float max_acc = safety_factor * lower_bound_max_acc;
  const float max_jerk = safety_factor * lower_bound_max_jerk;
  assert_trajectory(trajectory, max_acc, max_jerk);
}

// Constant speed of 10mps. Last velocity point at 0mps.
TEST(TrajectorySmoother, Constant) {
  const std::chrono::milliseconds dt(DT_MS);
  auto trajectory = constant_velocity_trajectory(
    0, 0, 1, 10,
    std::chrono::duration_cast<std::chrono::nanoseconds>(dt));
  trajectory.points.resize(100);
  // Zero out last point
  trajectory.points.back().longitudinal_velocity_mps = 0.0F;

  // Generate TrajectorySmoother
  const uint32_t kernel_size = 25;
  const TrajectorySmootherConfig config{5.0, kernel_size};
  TrajectorySmoother smoother(config);

  // Send through smoother
  smoother.Filter(trajectory);

  assert_trajectory_stop(trajectory, 10, static_cast<float>(kernel_size) * DT_MS / 1000);
}

// Constant speed of 10mps. Random noise added. Last velocity point at 0mps.
TEST(TrajectorySmoother, ConstantWithNoise) {
  const std::chrono::milliseconds dt(DT_MS);
  auto trajectory = constant_velocity_trajectory(
    0, 0, 1, 10,
    std::chrono::duration_cast<std::chrono::nanoseconds>(dt));
  trajectory.points.resize(100);
  introduce_noise(&trajectory, 10);
  // Zero out last point
  trajectory.points.back().longitudinal_velocity_mps = 0.0F;

  // Generate TrajectorySmoother
  const uint32_t kernel_size = 25;
  const TrajectorySmootherConfig config{5.0, kernel_size};
  TrajectorySmoother smoother(config);

  // Send through smoother
  smoother.Filter(trajectory);

  assert_trajectory_stop(trajectory, 10, static_cast<float>(kernel_size) * DT_MS / 1000);
}

// Constant acceleration, from 10mps to 20mps. Last velocity point at 0mps.
TEST(TrajectorySmoother, Acceleration) {
  const std::chrono::milliseconds dt(DT_MS);
  auto trajectory = constant_acceleration_trajectory(
    0, 0, 1, 10, 1,
    std::chrono::duration_cast<std::chrono::nanoseconds>(dt));
  trajectory.points.resize(100);
  // Zero out last point
  trajectory.points.back().longitudinal_velocity_mps = 0.0F;

  // Generate TrajectorySmoother
  const uint32_t kernel_size = 25;
  const TrajectorySmootherConfig config{5.0, kernel_size};
  TrajectorySmoother smoother(config);

  // Send through smoother
  smoother.Filter(trajectory);

  assert_trajectory_stop(trajectory, 20, static_cast<float>(kernel_size) * DT_MS / 1000);
}

// Constant acceleration, from 10mps to 20mps in 3.3s. Followed by a constant deceleration to 0mps
// in 6.7s.
TEST(TrajectorySmoother, AccelerationDeceleration) {
  constexpr float T = 10;   // Total time of the trajectory, [s]
  constexpr float v0 = 10;  // Initial velocity, [m/s]
  constexpr float a = 3;    // Absolute value of both the acceleration and deceleration, [m/s^2]
  constexpr float ratio = 0.5 - v0 / (2 * T * a);
  static_assert(ratio > 0 && ratio < 1, "ratio outside of (0, 1) range");
  const int points = T * (1000 / DT_MS);
  const int p0 = static_cast<int>(std::round(points * ratio));
  const std::chrono::milliseconds dt(DT_MS);
  auto trajectory = constant_acceleration_trajectory(
    0, 0, 1, 10, 3,
    std::chrono::duration_cast<std::chrono::nanoseconds>(dt));
  trajectory.points.resize(p0);
  auto trajectory_decel = constant_acceleration_trajectory(
    0, 0, 1, 20, -3,
    std::chrono::duration_cast<std::chrono::nanoseconds>(dt));
  trajectory_decel.points.resize(points - p0);
  for (auto & point : trajectory_decel.points) {
    trajectory.points.push_back(point);
  }
  // Zero out last point
  trajectory.points.back().longitudinal_velocity_mps = 0.0F;

  // Generate TrajectorySmoother
  const uint32_t kernel_size = 25;
  const TrajectorySmootherConfig config{5.0, kernel_size};
  TrajectorySmoother smoother(config);

  // Send through smoother
  smoother.Filter(trajectory);

  // The stopping part of this test is expected to have a slightly larger magnitude than the change
  // in acceleration, hence the use of assert_trajectory_stop.
  assert_trajectory_stop(trajectory, 7, static_cast<float>(kernel_size) * DT_MS / 1000);
}

// Constant deceleration, from 10mps to 0mps in 3.3s. Then stay at 0mps over the remaining 6.7s.
TEST(TrajectorySmoother, DecelerationHalt) {
  const std::chrono::milliseconds dt(DT_MS);
  auto trajectory = constant_acceleration_trajectory(
    0, 0, 1, 10, -3,
    std::chrono::duration_cast<std::chrono::nanoseconds>(dt));
  trajectory.points.resize(100);
  for (auto & point : trajectory.points) {
    if (point.longitudinal_velocity_mps < 0) {
      point.longitudinal_velocity_mps = 0;
    }
  }

  // Generate TrajectorySmoother
  const uint32_t kernel_size = 25;
  const TrajectorySmootherConfig config{5.0, kernel_size};
  TrajectorySmoother smoother(config);

  // Send through smoother
  smoother.Filter(trajectory);

  // Constants based on Gaussian filter's performance
  assert_trajectory(trajectory, 4, 3);
}
