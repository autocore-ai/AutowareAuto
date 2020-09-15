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
/// \brief This file defines the trajectory_planner_node_base_node class.

#ifndef TRAJECTORY_PLANNER_NODE_BASE__TRAJECTORY_PLANNER_NODE_BASE_HPP_
#define TRAJECTORY_PLANNER_NODE_BASE__TRAJECTORY_PLANNER_NODE_BASE_HPP_

#include <trajectory_planner_node_base/visibility_control.hpp>

// ROS
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

// Autoware Package
#include <autoware_auto_msgs/srv/had_map_service.hpp>
#include <autoware_auto_msgs/action/plan_trajectory.hpp>
#include <autoware_auto_msgs/msg/route.hpp>
#include <common/types.hpp>

// external libraries
#include <lanelet2_core/LaneletMap.h>
#include <memory>
#include <string>

namespace autoware
{
namespace trajectory_planner_node_base
{

using autoware::common::types::bool8_t;

using HADMapService = autoware_auto_msgs::srv::HADMapService;
using Route = autoware_auto_msgs::msg::Route;
using Trajectory = autoware_auto_msgs::msg::Trajectory;

enum class PlannerState
{
  IDLE,
  PLANNING
};  // enum class PlannerState

/// \class TrajectoryPlannerNodeBase
/// \brief ROS 2 Wrapper Node for Trajectory Planner.
class TRAJECTORY_PLANNER_NODE_BASE_PUBLIC TrajectoryPlannerNodeBase : public rclcpp::Node
{
public:
  /// \brief default constructor, starts planner
  /// \param[in] node_name name of the ROS node
  /// \param[in] options node options for rclcpp Node
  explicit TrajectoryPlannerNodeBase(
    const std::string & node_name,
    const rclcpp::NodeOptions & options);

protected:
  /// \brief creates map request from given route. Implementer should request
  //         for relevent map objects for their planner
  virtual HADMapService::Request create_map_request(const Route & route) = 0;

  /// \brief do trajectory plannig for given route. It should return the trajectory
  //         and status (SUCCESS, FAIL)
  virtual Trajectory plan_trajectory(
    const Route & route,
    const lanelet::LaneletMapPtr & lanelet_map_ptr) = 0;

private:
  using PlanTrajectoryAction = autoware_auto_msgs::action::PlanTrajectory;
  using GoalHandle = rclcpp_action::ServerGoalHandle<PlanTrajectoryAction>;

  // ROS Interface
  rclcpp_action::Server<PlanTrajectoryAction>::SharedPtr m_planner_server;
  rclcpp::Client<HADMapService>::SharedPtr m_map_client;

  // callback
  TRAJECTORY_PLANNER_NODE_BASE_LOCAL rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID & uuid,
    const std::shared_ptr<const PlanTrajectoryAction::Goal> goal);
  TRAJECTORY_PLANNER_NODE_BASE_LOCAL rclcpp_action::CancelResponse handle_cancel(
    const std::shared_ptr<GoalHandle> goal_handle);
  TRAJECTORY_PLANNER_NODE_BASE_LOCAL void handle_accepted(
    const std::shared_ptr<GoalHandle> goal_handle);
  void map_response(rclcpp::Client<HADMapService>::SharedFuture future);

  // \brief Validation of trajectory
  bool8_t is_trajectory_valid(const Trajectory & trajectory);

  std::shared_ptr<GoalHandle> m_goal_handle{nullptr};

  PlannerState m_planner_state;
  bool8_t is_planning();
  void start_planning();
  void stop_planning();
};

}  // namespace trajectory_planner_node_base
}  // namespace autoware

#endif  // TRAJECTORY_PLANNER_NODE_BASE__TRAJECTORY_PLANNER_NODE_BASE_HPP_
