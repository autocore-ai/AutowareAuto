// Copyright 2020 Embotech AG, Zurich, Switzerland, inspired by Christopher Ho's mpc code
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
#ifndef PARKING_PLANNER_NODE__PARKING_PLANNER_NODE_HPP_
#define PARKING_PLANNER_NODE__PARKING_PLANNER_NODE_HPP_

#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <parking_planner_node/visibility_control.hpp>
#include <parking_planner/parking_planner.hpp>

#include <trajectory_planner_node_base/trajectory_planner_node_base.hpp>
#include <autoware_auto_msgs/srv/had_map_service.hpp>
#include <autoware_auto_msgs/action/plan_trajectory.hpp>
#include <autoware_auto_msgs/msg/vehicle_kinematic_state.hpp>
#include <autoware_auto_msgs/msg/trajectory.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <lanelet2_core/LaneletMap.h>
#include <motion_common/motion_common.hpp>
#include <motion_common/config.hpp>
#include <common/types.hpp>

#include <rclcpp_action/rclcpp_action.hpp>
#include <rclcpp/rclcpp.hpp>

#include <string>
#include <memory>
#include <thread>
#include <future>

using autoware::common::types::float64_t;
using motion::motion_common::VehicleConfig;
using motion::motion_common::Real;

namespace autoware
{
namespace motion
{
namespace planning
{
namespace parking_planner_node
{
using PlannerPtr = std::unique_ptr<autoware::motion::planning::parking_planner::ParkingPlanner>;
using HADMapService = autoware_auto_msgs::srv::HADMapService;
using Route = autoware_auto_msgs::msg::Route;
using State = autoware_auto_msgs::msg::VehicleKinematicState;
using autoware_auto_msgs::msg::Trajectory;
using ParkerNLPCostWeights = autoware::motion::planning::parking_planner::NLPCostWeights<float64_t>;
using ParkerVehicleState = autoware::motion::planning::parking_planner::VehicleState<float64_t>;
using ParkerVehicleCommand = autoware::motion::planning::parking_planner::VehicleCommand<float64_t>;
using ParkingPlanner = autoware::motion::planning::parking_planner::ParkingPlanner;
using autoware_auto_msgs::msg::TrajectoryPoint;

class PARKING_PLANNER_NODE_PUBLIC ParkingPlannerNode : public
  autoware::trajectory_planner_node_base::TrajectoryPlannerNodeBase
{
public:
  explicit ParkingPlannerNode(const rclcpp::NodeOptions & options);

protected:
  HADMapService::Request create_map_request(const Route & route);

  Trajectory plan_trajectory(
    const Route & route,
    const lanelet::LaneletMapPtr & lanelet_map_ptr);

  PlannerPtr m_planner{nullptr};

private:
  PARKING_PLANNER_NODE_LOCAL void init(
    const VehicleConfig & vehicle_param,
    const ParkerNLPCostWeights & optimization_weights,
    const ParkerVehicleState & lower_state_bounds,
    const ParkerVehicleState & upper_state_bounds,
    const ParkerVehicleCommand & lower_command_bounds,
    const ParkerVehicleCommand & upper_command_bounds
  );
};  // class parkingPlannerNode
}  // namespace parking_planner_node
}  // namespace planning
}  // namespace motion
}  // namespace autoware

#endif  // PARKING_PLANNER_NODE__PARKING_PLANNER_NODE_HPP_
