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

#ifndef AUTOWARE_STATE_MONITOR__STATE_MACHINE_HPP_
#define AUTOWARE_STATE_MONITOR__STATE_MACHINE_HPP_

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "rclcpp/time.hpp"

#include "autoware_auto_msgs/msg/autoware_state.hpp"
#include "autoware_auto_msgs/msg/engage.hpp"
#include "autoware_auto_msgs/msg/had_map_route.hpp"
#include "autoware_auto_msgs/msg/vehicle_state_report.hpp"

#include "autoware_state_monitor/state.hpp"
#include "autoware_state_monitor/odometry_buffer.hpp"
#include "autoware_state_monitor/visibility_control.hpp"

namespace autoware
{
namespace state_monitor
{

/// \brief Input state of the state machine.
struct StateInput
{
  /// Current system time.
  rclcpp::Time current_time;
  /// Current pose of the vehicle.
  geometry_msgs::msg::PoseStamped::ConstSharedPtr current_pose;
  /// Goal pose of the vehicle.
  autoware_auto_msgs::msg::RoutePoint::ConstSharedPtr goal_pose;
  /// Determines if the vehicle should be engaged or disengaged.
  autoware_auto_msgs::msg::Engage::ConstSharedPtr engage;
  /// Report about the current state of the vehicle.
  autoware_auto_msgs::msg::VehicleStateReport::ConstSharedPtr vehicle_state_report;
  /// Planned global route.
  autoware_auto_msgs::msg::HADMapRoute::ConstSharedPtr route;
  /// Buffer that stores odometry messages.
  OdometryBuffer odometry_buffer;
  /// Determines if the system should be finalized.
  bool is_finalizing = false;
};

/// \brief Parameters used by the state machine
struct StateMachineParams
{
  /// Distance threshold between a current position and a goal position.
  double arrived_distance_threshold;
  /// Length of the odometry buffer used in checking if vehicle is stopped.
  double stopped_time_threshold;
  /// Velocity threshold for determining if vehicle is stopped.
  double stopped_velocity_threshold_mps;
  /// Delay after initialization and before transition to a next state.
  double wait_time_after_initializing;
  /// Delay after planning and before transition to a next state.
  double wait_time_after_planning;
  /// Delay after arrived goal and before transition to a next state.
  double wait_time_after_arrived_goal;
};

/// \brief State machine for determining a state of the Autoware system.
class AUTOWARE_STATE_MONITOR_PUBLIC StateMachine
{
public:
  /// \brief Construct the state machine.
  /// \param state_param The set of parameters used by state machine.
  explicit StateMachine(const StateMachineParams & state_param);

  /// \brief Get the current state of the state machine.
  /// \return The current state.
  State getCurrentState() const;

  /// \brief Update the state machine.
  /// \param state_input Input values used during the determination of the new state.
  /// \return A state after the state machine update.
  State updateState(const StateInput & state_input);

private:
  struct Times
  {
    rclcpp::Time arrived_goal;
    rclcpp::Time initializing_completed;
    rclcpp::Time planning_completed;
  };

  struct Flags
  {
    bool waiting_after_initializing = false;
    bool waiting_after_planning = false;
  };

  bool isVehicleInitialized() const;
  bool isRouteReceived() const;
  bool isPlanningCompleted() const;
  bool isAutonomousMode() const;
  bool isEngaged() const;
  bool isOverridden() const;
  bool hasArrivedGoal() const;
  bool isFinalizing() const;

  bool isNearGoal(
    const geometry_msgs::msg::Pose & current_pose,
    const autoware_auto_msgs::msg::RoutePoint & goal_pose,
    const double th_dist) const;

  bool isStopped(
    const OdometryBuffer & odometry_buffer,
    const double stopped_velocity_threshold_mps) const;

  State judgeAutowareState() const;

  State autoware_state_ = autoware_auto_msgs::msg::AutowareState::INITIALIZING;
  StateInput state_input_;
  const StateMachineParams state_param_;

  mutable Times times_;
  mutable Flags flags_;
  mutable autoware_auto_msgs::msg::HADMapRoute::ConstSharedPtr executing_route_ = nullptr;
};

}  // namespace state_monitor
}  // namespace autoware

#endif  // AUTOWARE_STATE_MONITOR__STATE_MACHINE_HPP_
