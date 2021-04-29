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
/// \brief This file defines the lane_planner class.

#ifndef LANE_PLANNER__LANE_PLANNER_HPP_
#define LANE_PLANNER__LANE_PLANNER_HPP_

#include <lane_planner/visibility_control.hpp>

#include <autoware_auto_msgs/msg/vehicle_kinematic_state.hpp>
#include <autoware_auto_msgs/msg/trajectory.hpp>
#include <autoware_auto_msgs/msg/had_map_route.hpp>
#include <common/types.hpp>
#include <motion_common/config.hpp>

#include <trajectory_smoother/trajectory_smoother.hpp>

#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_core/primitives/Point.h>

#include <iostream>
#include <vector>

namespace autoware
{
/// \brief TODO(ryohsuke.mitsudome): Document namespaces!
namespace lane_planner
{
using motion::motion_common::VehicleConfig;
using motion::motion_common::to_angle;

using TrajectoryPoint = autoware_auto_msgs::msg::TrajectoryPoint;
using TrajectoryPoints = std::vector<TrajectoryPoint>;
using State = autoware_auto_msgs::msg::VehicleKinematicState;
using autoware_auto_msgs::msg::HADMapRoute;
using autoware_auto_msgs::msg::Trajectory;
using Heading = decltype(decltype(State::state)::heading);

using motion::planning::trajectory_smoother::TrajectorySmootherConfig;
using motion::planning::trajectory_smoother::TrajectorySmoother;

using autoware::common::types::bool8_t;
using autoware::common::types::float32_t;
using autoware::common::types::float64_t;

using lanelet::LaneletMapConstPtr;

// utility functions
LANE_PLANNER_PUBLIC float32_t distance2d(const TrajectoryPoint & p1, const TrajectoryPoint & p2);

// calculate curvature by circle fitting to three points
LANE_PLANNER_PUBLIC float32_t calculate_curvature(
  const TrajectoryPoint & p1, const TrajectoryPoint & p2,
  const TrajectoryPoint & p3);

struct LANE_PLANNER_PUBLIC LanePlannerConfig
{
  float32_t trajectory_resolution;
};

/// \brief A class for recording trajectories and replaying them as plans
class LANE_PLANNER_PUBLIC LanePlanner
{
public:
  explicit LanePlanner(
    const VehicleConfig & vehicle_param,
    const TrajectorySmootherConfig & config,
    const LanePlannerConfig & planner_config);

  Trajectory plan_trajectory(
    const autoware_auto_msgs::msg::HADMapRoute & had_map_route,
    const LaneletMapConstPtr & map);

private:
  VehicleConfig m_vehicle_param;
  LanePlannerConfig m_planner_config;

  TrajectorySmoother m_trajectory_smoother;

  // trajectory planning sub functions
  TrajectoryPoints generate_base_trajectory(
    const HADMapRoute & had_map_route,
    const LaneletMapConstPtr & map);
  void set_angle(TrajectoryPoints * trajectory_points);
  void set_steering_angle(TrajectoryPoints * trajectory_points);
  void set_time_from_start(TrajectoryPoints * trajectory_points);
  void modify_velocity(Trajectory * trajectory);

  Trajectory create_trajectory_message(
    const std_msgs::msg::Header & header,
    const TrajectoryPoints & trajectory_points);
};  // class LanePlanner

}  // namespace lane_planner
}  // namespace autoware

#endif  // LANE_PLANNER__LANE_PLANNER_HPP_
