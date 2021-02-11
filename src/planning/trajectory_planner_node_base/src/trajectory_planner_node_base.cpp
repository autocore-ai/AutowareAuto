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

#include "trajectory_planner_node_base/trajectory_planner_node_base.hpp"
#include <had_map_utils/had_map_conversion.hpp>

//lint -e537 NOLINT  // cpplint vs pclint
#include <string>
#include <chrono>
#include <memory>

namespace autoware
{
namespace trajectory_planner_node_base
{

TrajectoryPlannerNodeBase::TrajectoryPlannerNodeBase(
  const std::string & node_name,
  const std::string & action_server_name,
  const rclcpp::NodeOptions & node_options)
: Node{node_name, node_options},
  m_planner_state{PlannerState::IDLE}
{
  using namespace std::literals::chrono_literals;

  // Setup Map Service
  m_map_client = this->create_client<HADMapService>("HAD_Map_Service");

  while (!m_map_client->wait_for_service(1s)) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(get_logger(), "Interrupted while waiting for map server.");
      rclcpp::shutdown();
      return;
    }
    RCLCPP_INFO(get_logger(), "Waiting for map service...");
  }

  m_planner_server = rclcpp_action::create_server<PlanTrajectoryAction>(
    this->get_node_base_interface(),
    this->get_node_clock_interface(),
    this->get_node_logging_interface(),
    this->get_node_waitables_interface(),
    action_server_name,
    [this](auto uuid, auto goal) {return this->handle_goal(uuid, goal);},
    [this](auto goal_handle) {return this->handle_cancel(goal_handle);},
    [this](auto goal_handle) {return this->handle_accepted(goal_handle);});
}

bool8_t TrajectoryPlannerNodeBase::is_trajectory_valid(const Trajectory & trajectory)
{
  if (trajectory.points.empty()) {
    return false;
  }

  // currently we only check minimal validation
  // TODO(mitudome-r): add more validation in the future.
  // e.g. resolution of trajectory, kinematic feasibility, ...

  return true;
}
bool8_t TrajectoryPlannerNodeBase::is_planning()
{
  return m_planner_state == PlannerState::PLANNING;
}
void TrajectoryPlannerNodeBase::start_planning()
{
  m_planner_state = PlannerState::PLANNING;
}
void TrajectoryPlannerNodeBase::stop_planning()
{
  m_planner_state = PlannerState::IDLE;
}

rclcpp_action::GoalResponse TrajectoryPlannerNodeBase::handle_goal(
  const rclcpp_action::GoalUUID & uuid,
  const std::shared_ptr<const PlanTrajectoryAction::Goal> goal)
{
  (void)goal;
  (void)uuid;
  if (is_planning()) {
    // Can't start replaying if we already are
    RCLCPP_ERROR(this->get_logger(), "Planner is already running. Rejecting new goal.");
    return rclcpp_action::GoalResponse::REJECT;
  }
  RCLCPP_INFO(this->get_logger(), "received new goal");
  start_planning();
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse TrajectoryPlannerNodeBase::handle_cancel(
  const std::shared_ptr<GoalHandle> goal_handle)
{
  (void)goal_handle;
  if (is_planning()) {
    RCLCPP_INFO(this->get_logger(), "Cancel replaying");
    auto result = std::make_shared<PlanTrajectoryAction::Result>();
    result->result = PlanTrajectoryAction::Result::FAIL;
    m_goal_handle->canceled(result);
    stop_planning();
  }

  return rclcpp_action::CancelResponse::ACCEPT;
}

void TrajectoryPlannerNodeBase::handle_accepted(
  const std::shared_ptr<GoalHandle> goal_handle)
{
  // Store the goal handle in order to send result in map_response callback.
  m_goal_handle = goal_handle;

  auto map_request = std::make_shared<HADMapService::Request>();
  *map_request = create_map_request(m_goal_handle->get_goal()->sub_route);

  // TODO(mitsudome-r): If synchronized service request is available,
  // replace it with synchronized implementation
  auto result =
    m_map_client->async_send_request(
    map_request,
    std::bind(&TrajectoryPlannerNodeBase::map_response, this, std::placeholders::_1));
}

void TrajectoryPlannerNodeBase::map_response(rclcpp::Client<HADMapService>::SharedFuture future)
{
  auto lanelet_map_ptr = std::make_shared<lanelet::LaneletMap>();
  autoware::common::had_map_utils::fromBinaryMsg(future.get()->map, lanelet_map_ptr);

  RCLCPP_INFO(get_logger(), "Start planning");
  const auto & trajectory = plan_trajectory(m_goal_handle->get_goal()->sub_route, lanelet_map_ptr);
  RCLCPP_INFO(get_logger(), "Finished planning");

  if (is_trajectory_valid(trajectory)) {
    auto result = std::make_shared<PlanTrajectoryAction::Result>();
    result->result = PlanTrajectoryAction::Result::SUCCESS;
    result->trajectory = trajectory;
    m_goal_handle->succeed(result);
    RCLCPP_INFO(get_logger(), "Sent planned trajectory with %d points", trajectory.points.size());
  } else {
    RCLCPP_INFO(get_logger(), "Aborting planning due to invalid trajectory");
    auto result = std::make_shared<PlanTrajectoryAction::Result>();
    result->result = PlanTrajectoryAction::Result::FAIL;
    m_goal_handle->abort(result);
  }
  stop_planning();
}

}  // namespace trajectory_planner_node_base
}  // namespace autoware
