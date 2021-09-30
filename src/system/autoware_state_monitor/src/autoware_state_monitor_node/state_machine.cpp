// Copyright 2021 The Autoware Foundation
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
//
// Co-developed by Tier IV, Inc. and Robotec.AI sp. z o.o.

#include "autoware_state_monitor/state_machine.hpp"

#include <cmath>

namespace autoware
{
namespace state_monitor
{

double distance2d(
  const geometry_msgs::msg::Point & p1,
  const geometry_msgs::msg::Point & p2)
{
  return std::hypot(p1.x - p2.x, p1.y - p2.y);
}

StateMachine::StateMachine(const StateMachineParams & state_param)
: state_param_(state_param)
{
}

bool StateMachine::isNearGoal(
  const geometry_msgs::msg::Pose & current_pose,
  const autoware_auto_msgs::msg::RoutePoint & goal_pose,
  const double th_dist) const
{
  return distance2d(current_pose.position, goal_pose.position) < th_dist;
}

bool StateMachine::isStopped(
  const OdometryBuffer & odometry_buffer,
  const double stopped_velocity_threshold_mps) const
{
  for (const auto & odometry : odometry_buffer) {
    if (std::abs(static_cast<double>(odometry->velocity_mps)) > stopped_velocity_threshold_mps) {
      return false;
    }
  }
  return true;
}

bool StateMachine::isVehicleInitialized() const
{
  return true;
}

bool StateMachine::isRouteReceived() const
{
  return state_input_.route != executing_route_;
}

bool StateMachine::isPlanningCompleted() const
{
  return true;
}

bool StateMachine::isAutonomousMode() const
{
  using autoware_auto_msgs::msg::VehicleStateReport;
  if (!state_input_.vehicle_state_report) {
    return false;
  }

  // TODO(mdrwiega) Enable mode checking after fix in lgsvl interface
  // if (state_input_.vehicle_state_report->mode != VehicleStateReport::MODE_AUTONOMOUS) {
  //   return false;
  // }

  return true;
}

bool StateMachine::isEngaged() const
{
  if (!state_input_.engage) {
    return false;
  }

  if (state_input_.engage->engage != 1) {
    return false;
  }

  return true;
}

bool StateMachine::isOverridden() const
{
  return !isEngaged() || !isAutonomousMode();
}

bool StateMachine::hasArrivedGoal() const
{
  if (!state_input_.current_pose || !state_input_.goal_pose) {
    return false;
  }

  const auto & current_pose = state_input_.current_pose->pose;
  const auto & goal_pose = *state_input_.goal_pose;
  const auto is_near_goal = isNearGoal(
    current_pose, goal_pose, state_param_.arrived_distance_threshold);

  const auto is_stopped = isStopped(
    state_input_.odometry_buffer, state_param_.stopped_velocity_threshold_mps);

  if (is_near_goal && is_stopped) {
    return true;
  }

  return false;
}

bool StateMachine::isFinalizing() const
{
  return state_input_.is_finalizing;
}

State StateMachine::getCurrentState() const
{
  return autoware_state_;
}

State StateMachine::updateState(const StateInput & state_input)
{
  state_input_ = state_input;

  autoware_state_ = judgeAutowareState();

  return autoware_state_;
}

State StateMachine::judgeAutowareState() const
{
  using autoware_auto_msgs::msg::AutowareState;

  if (isFinalizing()) {
    return AutowareState::FINALIZING;
  }

  switch (autoware_state_) {
    case AutowareState::INITIALIZING: {
        if (isVehicleInitialized()) {
          if (!flags_.waiting_after_initializing) {
            flags_.waiting_after_initializing = true;
            times_.initializing_completed = state_input_.current_time;
            break;
          }

          // Wait for initialization complete
          const auto time_from_initializing =
            state_input_.current_time - times_.initializing_completed;
          if (time_from_initializing.seconds() >= state_param_.wait_time_after_initializing) {
            flags_.waiting_after_initializing = false;
            return AutowareState::WAITING_FOR_ROUTE;
          }
        }
        break;
      }

    case AutowareState::WAITING_FOR_ROUTE: {
        if (isRouteReceived()) {
          return AutowareState::PLANNING;
        }
        break;
      }

    case AutowareState::PLANNING: {
        executing_route_ = state_input_.route;
        if (isPlanningCompleted()) {
          if (!flags_.waiting_after_planning) {
            flags_.waiting_after_planning = true;
            times_.planning_completed = state_input_.current_time;
            break;
          }

          // Wait after planning completed
          const auto time_from_planning = state_input_.current_time - times_.planning_completed;
          if (time_from_planning.seconds() >= state_param_.wait_time_after_planning) {
            flags_.waiting_after_planning = false;
            return AutowareState::WAITING_FOR_ENGAGE;
          }
        }
        break;
      }

    case AutowareState::WAITING_FOR_ENGAGE: {
        if (isRouteReceived()) {
          return AutowareState::PLANNING;
        }

        if (isEngaged() && isAutonomousMode()) {
          return AutowareState::DRIVING;
        }

        if (hasArrivedGoal()) {
          times_.arrived_goal = state_input_.current_time;
          return AutowareState::ARRIVED_GOAL;
        }
        break;
      }

    case AutowareState::DRIVING: {
        if (isRouteReceived()) {
          return AutowareState::PLANNING;
        }

        if (isOverridden()) {
          return AutowareState::WAITING_FOR_ENGAGE;
        }

        if (hasArrivedGoal()) {
          times_.arrived_goal = state_input_.current_time;
          return AutowareState::ARRIVED_GOAL;
        }
        break;
      }

    case AutowareState::ARRIVED_GOAL: {
        const auto time_from_arrived_goal = state_input_.current_time - times_.arrived_goal;
        if (time_from_arrived_goal.seconds() >= state_param_.wait_time_after_arrived_goal) {
          return AutowareState::WAITING_FOR_ROUTE;
        }
        break;
      }

    case AutowareState::FINALIZING: {
        break;
      }

    default:
      throw std::runtime_error("Invalid state: " + std::to_string(autoware_state_));
  }

  // continue previous state when break
  return autoware_state_;
}

}  // namespace state_monitor
}  // namespace autoware
