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
#ifndef RECORDREPLAY_PLANNER_NODES__RECORDREPLAY_PLANNER_NODE_HPP_
#define RECORDREPLAY_PLANNER_NODES__RECORDREPLAY_PLANNER_NODE_HPP_

#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <recordreplay_planner_nodes/visibility_control.hpp>
#include <recordreplay_planner/recordreplay_planner.hpp>
#include <recordreplay_planner_actions/action/record_trajectory.hpp>
#include <recordreplay_planner_actions/action/replay_trajectory.hpp>

#include <autoware_auto_msgs/msg/trajectory.hpp>
#include <autoware_auto_msgs/msg/vehicle_kinematic_state.hpp>
#include <autoware_auto_msgs/srv/modify_trajectory.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <motion_common/motion_common.hpp>
#include <motion_common/config.hpp>
#include <common/types.hpp>

#include <rclcpp_action/rclcpp_action.hpp>
#include <rclcpp/rclcpp.hpp>

#include <string>
#include <memory>

using autoware::common::types::float64_t;

namespace motion
{
namespace planning
{
namespace recordreplay_planner_nodes
{
using PlannerPtr = std::unique_ptr<motion::planning::recordreplay_planner::RecordReplayPlanner>;
using autoware_auto_msgs::msg::Trajectory;
using autoware_auto_msgs::srv::ModifyTrajectory;
using recordreplay_planner_actions::action::RecordTrajectory;
using recordreplay_planner_actions::action::ReplayTrajectory;
using State = autoware_auto_msgs::msg::VehicleKinematicState;
using Transform = geometry_msgs::msg::TransformStamped;
using motion::motion_common::Real;

class RECORDREPLAY_PLANNER_NODES_PUBLIC RecordReplayPlannerNode : public rclcpp::Node
{
public:
  using GoalHandleRecordTrajectory = rclcpp_action::ServerGoalHandle<RecordTrajectory>;
  using GoalHandleReplayTrajectory = rclcpp_action::ServerGoalHandle<ReplayTrajectory>;

  /// Parameter file constructor
  explicit RecordReplayPlannerNode(const rclcpp::NodeOptions & node_options);

protected:
  rclcpp_action::Server<RecordTrajectory>::SharedPtr m_recordserver;
  rclcpp_action::Server<ReplayTrajectory>::SharedPtr m_replayserver;
  // May be nullptr if disabled
  rclcpp::Client<ModifyTrajectory>::SharedPtr m_modify_trajectory_client;
  std::shared_ptr<GoalHandleRecordTrajectory> m_recordgoalhandle;
  std::shared_ptr<GoalHandleReplayTrajectory> m_replaygoalhandle;

  rclcpp::Subscription<State>::SharedPtr m_ego_sub{};
  rclcpp::Publisher<Trajectory>::SharedPtr m_trajectory_pub{};
  PlannerPtr m_planner{nullptr};

private:
  RECORDREPLAY_PLANNER_NODES_LOCAL void on_ego(const State::SharedPtr & msg);

  RECORDREPLAY_PLANNER_NODES_LOCAL void modify_trajectory_response(
    rclcpp::Client<ModifyTrajectory>::SharedFuture future);

  // TODO(s.me) there does not seem to be a RecordTrajectory::SharedPtr? Also
  // the return types need to be changed to the rclcpp_action types once the package
  // is available.
  RECORDREPLAY_PLANNER_NODES_LOCAL rclcpp_action::GoalResponse record_handle_goal(
    const rclcpp_action::GoalUUID & uuid,
    const std::shared_ptr<const RecordTrajectory::Goal> goal);
  RECORDREPLAY_PLANNER_NODES_LOCAL rclcpp_action::CancelResponse record_handle_cancel(
    const std::shared_ptr<GoalHandleRecordTrajectory> goal_handle);
  RECORDREPLAY_PLANNER_NODES_LOCAL void record_handle_accepted(
    const std::shared_ptr<GoalHandleRecordTrajectory> goal_handle);

  RECORDREPLAY_PLANNER_NODES_LOCAL rclcpp_action::GoalResponse replay_handle_goal(
    const rclcpp_action::GoalUUID & uuid,
    const std::shared_ptr<const ReplayTrajectory::Goal> goal);
  RECORDREPLAY_PLANNER_NODES_LOCAL rclcpp_action::CancelResponse replay_handle_cancel(
    const std::shared_ptr<GoalHandleReplayTrajectory> goal_handle);
  RECORDREPLAY_PLANNER_NODES_LOCAL void replay_handle_accepted(
    const std::shared_ptr<GoalHandleReplayTrajectory> goal_handle);

  std::string m_odom_frame_id{};
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  bool m_enable_object_collision_estimator = false;
};  // class RecordReplayPlannerNode
}  // namespace recordreplay_planner_nodes
}  // namespace planning
}  // namespace motion

#endif  // RECORDREPLAY_PLANNER_NODES__RECORDREPLAY_PLANNER_NODE_HPP_
