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
/// \brief This file defines the behavior_planner class.

#ifndef BEHAVIOR_PLANNER__BEHAVIOR_PLANNER_HPP_
#define BEHAVIOR_PLANNER__BEHAVIOR_PLANNER_HPP_

#include <behavior_planner/visibility_control.hpp>
#include <behavior_planner/trajectory_manager.hpp>

// Autoware packages
#include <common/types.hpp>
#include <autoware_auto_msgs/msg/route.hpp>
#include <autoware_auto_msgs/msg/trajectory.hpp>
#include <autoware_auto_msgs/msg/vehicle_kinematic_state.hpp>

// lanelet headers
#include <lanelet2_core/LaneletMap.h>

// others
#include <iostream>
#include <vector>

namespace autoware
{
/// \brief TODO(ryohsuke.mitsudome): Document namespaces!
namespace behavior_planner
{

// TODO(mitsudome-r): extract this into common library to be used
// both in behavior planner and global planner
namespace PrimitiveType
{
constexpr const char ParkingSpot[] = "parking_spot";
constexpr const char DrivableArea[] = "drivable_area";
constexpr const char Lane[] = "lane";
}  // namespace PrimitiveType


using autoware_auto_msgs::msg::Route;
using autoware_auto_msgs::msg::MapPrimitive;
using autoware_auto_msgs::msg::TrajectoryPoint;
using State = autoware_auto_msgs::msg::VehicleKinematicState;

using autoware::common::types::bool8_t;
using autoware::common::types::float32_t;

enum PlannerType
{
  LANE,
  PARKING,
  UNKNOWN
};

struct RouteWithType
{
  Route route;
  PlannerType planner_type;
};

/// \class BehaviorPlanner
/// \brief Class that contains core functions of the behavior planner.
class BEHAVIOR_PLANNER_PUBLIC BehaviorPlanner
{
public:
  explicit BehaviorPlanner(const PlannerConfig & config);

  void set_route(const Route & route, const lanelet::LaneletMapPtr & lanelet_map_ptr);
  void set_next_subroute(const State & ego_state);

  bool8_t is_route_ready();

  bool8_t needs_new_trajectory(const State & state);
  TrajectoryPoint get_sub_goal();
  bool8_t has_arrived_subroute_goal(const State & state);

  bool8_t is_vehicle_stopped(const State & state);

  RouteWithType get_current_subroute(const State & ego_state);
  PlannerType get_planner_type();

  // relay to trajectory_manager
  bool8_t is_trajectory_ready();
  void set_trajectory(const Trajectory & trajectory);
  Trajectory get_trajectory(const State & state);
  size_t get_remaining_length(const State & state);

private:
  RouteWithType get_current_subroute();
  std::vector<RouteWithType> m_subroutes;
  std::size_t m_current_subroute;

  // parameters
  PlannerConfig m_config;

  // TODO(mitsudome-r): remove the manager once splitting of trajectory gets unnecessary
  TrajectoryManager m_trajectory_manager;

  bool8_t m_is_trajectory_complete;

  std::size_t get_closest_subroute(const State & ego_state);
};
}  // namespace behavior_planner
}  // namespace autoware

#endif  // BEHAVIOR_PLANNER__BEHAVIOR_PLANNER_HPP_
