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
/// \brief This file defines the lane_planner_node class.

#ifndef LANE_PLANNER_NODE__LANE_PLANNER_NODE_HPP_
#define LANE_PLANNER_NODE__LANE_PLANNER_NODE_HPP_

#include <lane_planner_node/visibility_control.hpp>

#include <lane_planner/lane_planner.hpp>
#include <trajectory_planner_node_base/trajectory_planner_node_base.hpp>

#include <string>
#include <memory>

namespace autoware
{
namespace lane_planner_node
{

using trajectory_planner_node_base::TrajectoryPlannerNodeBase;
using motion::motion_common::VehicleConfig;
using motion::motion_common::Real;
using autoware::common::types::float32_t;
using autoware::common::types::float64_t;

using motion::planning::object_collision_estimator::TrajectorySmootherConfig;

using autoware_auto_msgs::srv::HADMapService;
using autoware_auto_msgs::msg::Route;
using autoware_auto_msgs::msg::Trajectory;

/// \class LanePlannerNode
/// \brief ROS 2 Node for hello world.
class LANE_PLANNER_PUBLIC LanePlannerNode : public TrajectoryPlannerNodeBase
{
public:
  /// \brief default constructor, starts driver
  /// \param[in] options Node options as rclcpp::NodeOptions
  /// \throw runtime error if failed to start threads or configure driver
  explicit LanePlannerNode(const rclcpp::NodeOptions & options);

private:
  std::unique_ptr<lane_planner::LanePlanner> m_planner;

  /// \brief creates map request from given route.
  //         Implementer should request for relevent map objects for their planner
  HADMapService::Request create_map_request(const Route & route) override;

  /// \brief do trajectory plannig for given route.
  //         It should return the trajectory and status (SUCCESS, FAIL)
  Trajectory plan_trajectory(
    const Route & route,
    const lanelet::LaneletMapPtr & lanelet_map_ptr) override;
};


}  // namespace lane_planner_node
}  // namespace autoware

#endif  // LANE_PLANNER_NODE__LANE_PLANNER_NODE_HPP_
