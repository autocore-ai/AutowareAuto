// Copyright 2019 Apex.AI, Inc.
// Co-developed by Tier IV, Inc. and Apex.AI, Inc.
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

#include <gtest/gtest.h>
#include <autoware_auto_msgs/msg/trajectory.hpp>
#include <motion_common/motion_common.hpp>
#include <motion_model/catr_model.hpp>
#include <osrf_testing_tools_cpp/memory_tools/memory_tools.hpp>
#include <time_utils/time_utils.hpp>

#include <string>

#include "pure_pursuit/pure_pursuit.hpp"

using autoware::motion::control::pure_pursuit::Config;
using autoware::motion::control::pure_pursuit::PurePursuit;
using autoware_auto_msgs::msg::Trajectory;
using autoware_auto_msgs::msg::TrajectoryPoint;
using TrajectoryPointStamped = autoware_auto_msgs::msg::VehicleKinematicState;
using autoware::motion::control::pure_pursuit::ControllerDiagnostic;
using ::motion::motion_common::from_angle;
using autoware_auto_msgs::msg::VehicleControlCommand;
using autoware::motion::motion_model::CatrModel;
using autoware::motion::motion_model::CatrState;
using Eigen::Matrix;

constexpr auto PI = 3.14159F;

static const float TOL = 1.0E-5F;

using float32_t = float;

/// Hold some basic configuration parameters
class PurePursuitTest : public ::testing::Test
{
public:
  PurePursuitTest()
  {
    size = 100U;
  }

protected:
  Trajectory traj;
  uint32_t size;
  TrajectoryPointStamped current_pose;
  VehicleControlCommand command;
};

void create_traj(Trajectory & traj, uint32_t size, float32_t heading = PI / 4.0F)
{
  const float32_t slope = 1.0F;
  const float32_t offset = 1.0F;
  traj.points.resize(size);
  traj.header.frame_id = "traj";
  traj.header.stamp = time_utils::to_message(std::chrono::system_clock::now());
  const auto q_heading = from_angle(heading);
  for (uint32_t idx = 0U; idx < size; ++idx) {
    traj.points[idx].time_from_start.sec = static_cast<int32_t>(idx) / 10;
    traj.points[idx].time_from_start.nanosec = (idx % 10U) * 1'000'000U;
    traj.points[idx].x = (static_cast<float32_t>(idx) * slope) + offset;
    traj.points[idx].y = (static_cast<float32_t>(idx) * slope) + offset;
    traj.points[idx].longitudinal_velocity_mps = (static_cast<float32_t>(idx) + offset);
    traj.points[idx].heading = q_heading;
  }
}

void create_reverse_traj(Trajectory & traj, uint32_t size, float32_t heading = -(3.0F * PI) / 4.0F)
{
  const float32_t slope = 1.0F;
  const float32_t offset = 1.0F;
  traj.points.resize(size);
  traj.header.frame_id = "traj";
  traj.header.stamp = time_utils::to_message(std::chrono::system_clock::now());
  const auto q_heading = from_angle(heading);
  for (uint32_t idx = 0U; idx < size; ++idx) {
    traj.points[idx].time_from_start.sec = static_cast<int32_t>(idx) / 10;
    traj.points[idx].time_from_start.nanosec = (idx % 10U) * 1'000'000U;
    traj.points[idx].x = -((static_cast<float32_t>(idx) * slope) + offset);
    traj.points[idx].y = -((static_cast<float32_t>(idx) * slope) + offset);
    traj.points[idx].longitudinal_velocity_mps = -(static_cast<float32_t>(idx) + offset);
    traj.points[idx].heading = q_heading;
  }
}

void create_current_pose(
  TrajectoryPointStamped & current_stamp,
  float32_t x,
  float32_t y,
  float32_t heading,
  float32_t velocity,
  float32_t acceleration,
  float32_t heading_rate,
  std::string frame_id)
{
  current_stamp.state.x = x;
  current_stamp.state.y = y;
  current_stamp.state.heading = from_angle(heading);
  current_stamp.state.longitudinal_velocity_mps = velocity;
  current_stamp.state.acceleration_mps2 = acceleration;
  current_stamp.state.heading_rate_rps = heading_rate;
  current_stamp.header.frame_id = "traj";  // frame_id;
  current_stamp.header.stamp = time_utils::to_message(std::chrono::system_clock::now());
}


TEST_F(PurePursuitTest, config)
{
  EXPECT_THROW(Config(0.0F, 1.0F, 0.2F, true, true, 2.0F, 0.1F, 2.0F), std::domain_error);
  EXPECT_THROW(Config(-1.0F, 1.0F, 0.2F, true, true, 2.0F, 0.1F, 2.0F), std::domain_error);
  EXPECT_THROW(Config(10.0F, -1.0F, 0.2F, true, true, 2.0F, 0.1F, 2.0F), std::domain_error);
  EXPECT_THROW(Config(10.0F, 1.0F, -1.0F, true, true, 2.0F, 0.1F, 2.0F), std::domain_error);
  EXPECT_THROW(Config(10.0F, 1.0F, 0.2F, true, true, -1.0F, 0.1F, 2.0F), std::domain_error);
  EXPECT_THROW(Config(10.0F, 1.0F, 0.2F, true, true, 1.0F, -0.1F, 2.0F), std::domain_error);
  EXPECT_THROW(Config(10.0F, 1.0F, 0.2F, true, true, 1.0F, 0.1F, -2.0F), std::domain_error);
}

TEST_F(PurePursuitTest, simple)
{
  const Config cfg(0.5F, 100.0F, 0.2F, false, false, 2.0F, 0.1F, 2.0F);
  PurePursuit controller(cfg);
  const float32_t dist_front_rear_wheels = cfg.get_distance_front_rear_wheel();

  create_traj(traj, size);
  create_current_pose(current_pose, 0.0F, 0.0F, 0.0F, 1.0F, 0.0F, 0.0F, "pose1");

  EXPECT_NO_MEMORY_OPERATIONS_BEGIN();
  controller.set_trajectory(traj);
  command = controller.compute_command(current_pose);
  const Trajectory & traj_result = controller.get_reference_trajectory();

  EXPECT_FLOAT_EQ(command.long_accel_mps2, 0.0F);
  EXPECT_FLOAT_EQ(command.front_wheel_angle_rad, atanf(1.0F * dist_front_rear_wheels));

  create_current_pose(current_pose, 1.5F, 1.0F, 0.0F, 5.0F, 0.0F, 0.0F, "pose2");

  command = controller.compute_command(current_pose);

  EXPECT_FLOAT_EQ(command.long_accel_mps2, -21.0F / (pow(1.25F, 0.5F) * 2.0F));
  EXPECT_FLOAT_EQ(command.front_wheel_angle_rad, atanf(1.6F * dist_front_rear_wheels));  // 1.25/2.0

  create_current_pose(current_pose, 0.5F, 2.0F, 0.0F, 5.0F, 0.0F, 0.0F, "pose3");

  controller.set_trajectory(traj);
  command = controller.compute_command(current_pose);

  EXPECT_FLOAT_EQ(command.long_accel_mps2, -24.0F / (pow(1.25F, 0.5F) * 2.0F));
  EXPECT_FLOAT_EQ(command.front_wheel_angle_rad, -atanf(1.6F * dist_front_rear_wheels));

  EXPECT_NO_MEMORY_OPERATIONS_END();
}

TEST_F(PurePursuitTest, reverse)
{
  const Config cfg(0.5F, 100.0F, 0.2F, false, false, 2.0F, 0.1F, 2.0F);
  PurePursuit controller(cfg);
  const float32_t dist_front_rear_wheels = cfg.get_distance_front_rear_wheel();

  create_reverse_traj(traj, size);
  create_current_pose(current_pose, 0.0F, 0.0F, 0.0F, -1.0F, 0.0F, 0.0F, "pose1");

  EXPECT_NO_MEMORY_OPERATIONS_BEGIN();
  controller.set_trajectory(traj);
  command = controller.compute_command(current_pose);

  EXPECT_FLOAT_EQ(command.long_accel_mps2, 0.0F);
  EXPECT_FLOAT_EQ(command.front_wheel_angle_rad, -atanf(1.0F * dist_front_rear_wheels));

  create_current_pose(current_pose, -1.5F, -1.0F, 0.0F, -5.0F, 0.0F, 0.0F, "pose2");

  command = controller.compute_command(current_pose);

  EXPECT_FLOAT_EQ(command.long_accel_mps2, 21.0F / (pow(1.25F, 0.5F) * 2.0F));
  // 2.0/1.25
  EXPECT_FLOAT_EQ(command.front_wheel_angle_rad, -atanf(1.6F * dist_front_rear_wheels));

  // heading error: -(7.0 / 4.0) * PI -> (1.0 / 4.0) * PI
  create_reverse_traj(traj, size);
  create_current_pose(
    current_pose, 0.0, 0.0F, PI, 1.0F, 0.0F, 0.0F, "pose2");
  controller.set_trajectory(traj);
  command = controller.compute_command(current_pose);

  traj.points[1].heading = from_angle((3.0 * PI) / 4.0F);
  create_current_pose(current_pose, -1.5F, -1.5F, PI, 1.0F, 0.0F, 0.0F, "pose2");
  controller.set_trajectory(traj);
  command = controller.compute_command(current_pose);

  EXPECT_NO_MEMORY_OPERATIONS_END();
}

TEST_F(PurePursuitTest, interpolation)
{
  const Config cfg(0.4F, 100.0F, 0.2F, true, false, 2.0F, 0.1F, 2.0F);
  PurePursuit controller(cfg);
  const float32_t dist_front_rear_wheels = cfg.get_distance_front_rear_wheel();

  create_traj(traj, size);
  create_current_pose(current_pose, 0.0F, 0.0F, 0.0F, 1.0F, 0.0F, 0.0F, "pose1");

  EXPECT_NO_MEMORY_OPERATIONS_BEGIN();
  controller.set_trajectory(traj);
  command = controller.compute_command(current_pose);

  EXPECT_FLOAT_EQ(command.long_accel_mps2, 0.0F);
  EXPECT_FLOAT_EQ(command.front_wheel_angle_rad,
    atanf(dist_front_rear_wheels / (0.4F / sqrtf(2.0F))));

  TrajectoryPointStamped current_pose;
  create_current_pose(
    current_pose, 1.5F, 1.0F, PI / 4.0F, 10.0F, 0.0F, 0.0F, "pose2");

  command = controller.compute_command(current_pose);

  create_current_pose(current_pose, 2.0F, 4.0F, 0.0F, 5.0F * powf(2.0F, 0.5F), 0.0F, 0.0F, "pose3");

  command = controller.compute_command(current_pose);

  EXPECT_FLOAT_EQ(command.long_accel_mps2, -41.0F / (pow(2.0F, 0.5F) * 2.0F));
  EXPECT_FLOAT_EQ(command.front_wheel_angle_rad, -atanf(1.0F * dist_front_rear_wheels));
  EXPECT_NO_MEMORY_OPERATIONS_END();
}
