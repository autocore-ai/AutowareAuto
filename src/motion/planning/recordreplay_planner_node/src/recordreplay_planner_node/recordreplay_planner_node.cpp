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
#include "recordreplay_planner_node/recordreplay_planner_node.hpp"

#include <memory>
#include <string>
#include <utility>

namespace motion
{
namespace planning
{
namespace recordreplay_planner_node
{
RecordReplayPlannerNode::RecordReplayPlannerNode(const std::string & name, const std::string & ns)
: Node{name, ns}
{
  // TODO(s.me) get topics from parameters
  const std::string tf_topic = "some_invalid_tf";
  const std::string ego_topic = "some_invalid_ego";
  const std::string trajectory_topic = "some_invalid_trajectory";
  init(ego_topic, tf_topic, trajectory_topic);
}
////////////////////////////////////////////////////////////////////////////////
RecordReplayPlannerNode::RecordReplayPlannerNode(
  const std::string & name,
  const std::string & ns,
  const std::string & ego_topic,
  const std::string & tf_topic,
  const std::string & trajectory_topic)
: Node{name, ns}
{
  init(ego_topic, tf_topic, trajectory_topic);
}

void RecordReplayPlannerNode::init(
  const std::string & ego_topic,
  const std::string & tf_topic,
  const std::string & trajectory_topic)
{
  using rclcpp::QoS;
  using std::placeholders::_1;
  using std::placeholders::_2;

  // Set up action for control of recording and replaying
  m_recordserver = rclcpp_action::create_server<RecordTrajectory>(
    this->get_node_base_interface(),
    this->get_node_clock_interface(),
    this->get_node_logging_interface(),
    this->get_node_waitables_interface(),
    "recordtrajectory",
    std::bind(&RecordReplayPlannerNode::record_handle_goal, this, _1, _2),
    std::bind(&RecordReplayPlannerNode::record_handle_cancel, this, _1),
    std::bind(&RecordReplayPlannerNode::record_handle_accepted, this, _1));

  m_replayserver = rclcpp_action::create_server<ReplayTrajectory>(
    this->get_node_base_interface(),
    this->get_node_clock_interface(),
    this->get_node_logging_interface(),
    this->get_node_waitables_interface(),
    "replaytrajectory",
    std::bind(&RecordReplayPlannerNode::replay_handle_goal, this, _1, _2),
    std::bind(&RecordReplayPlannerNode::replay_handle_cancel, this, _1),
    std::bind(&RecordReplayPlannerNode::replay_handle_accepted, this, _1));

  // Set up subscribers for the actual recording
  using SubAllocT = rclcpp::SubscriptionOptionsWithAllocator<std::allocator<void>>;
  m_tf_sub = create_subscription<TFMessage>(tf_topic, QoS{10}.transient_local(),
      [this](const TFMessage::SharedPtr msg) {on_tf(msg);}, SubAllocT{});

  m_ego_sub = create_subscription<State>(ego_topic, QoS{10}.transient_local(),
      [this](const State::SharedPtr msg) {on_ego(msg);}, SubAllocT{});

  // Set up publishers
  using PubAllocT = rclcpp::PublisherOptionsWithAllocator<std::allocator<void>>;
  m_trajectory_pub =
    create_publisher<Trajectory>(trajectory_topic, QoS{10}.transient_local(), PubAllocT{});

  // Create and set a planner object that we'll talk to
  set_planner(std::make_unique<recordreplay_planner::RecordReplayPlanner>());
}


void RecordReplayPlannerNode::on_ego(const State::SharedPtr & msg)
{
  if (m_planner->is_recording()) {
    m_planner->record_state(*msg);  // TODO(s.me) check if this is OK
  }

  if (m_planner->is_replaying()) {
    const auto & traj = m_planner->plan(*msg);
    m_trajectory_pub->publish(traj);
  }
}

void RecordReplayPlannerNode::on_tf(const TFMessage::SharedPtr & msg)
{
  // TODO(s.me) Implement something here?
  (void)msg;
}

rclcpp_action::GoalResponse RecordReplayPlannerNode::record_handle_goal(
  const rclcpp_action::GoalUUID & uuid,
  const std::shared_ptr<const RecordTrajectory::Goal> goal)
{
  (void)goal;
  (void)uuid;
  if (m_planner->is_recording()) {
    // Can't start recording if we already are
    return rclcpp_action::GoalResponse::REJECT;
  }

  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse RecordReplayPlannerNode::record_handle_cancel(
  const std::shared_ptr<GoalHandleRecordTrajectory> goal_handle)
{
  (void)goal_handle;
  if (m_planner->is_recording()) {
    m_planner->stop_recording();
  }

  return rclcpp_action::CancelResponse::ACCEPT;
}

void RecordReplayPlannerNode::record_handle_accepted(
  const std::shared_ptr<GoalHandleRecordTrajectory> goal_handle)
{
  (void)goal_handle;
  // TODO(s.me) what to do here? Is this where we actually trigger something?
  m_planner->start_recording();
}

rclcpp_action::GoalResponse RecordReplayPlannerNode::replay_handle_goal(
  const rclcpp_action::GoalUUID & uuid,
  const std::shared_ptr<const ReplayTrajectory::Goal> goal)
{
  (void)goal;
  (void)uuid;
  if (m_planner->is_replaying()) {
    // Can't start replaying if we already are
    return rclcpp_action::GoalResponse::REJECT;
  }

  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse RecordReplayPlannerNode::replay_handle_cancel(
  const std::shared_ptr<GoalHandleReplayTrajectory> goal_handle)
{
  (void)goal_handle;
  if (m_planner->is_replaying()) {
    m_planner->stop_replaying();
  }

  return rclcpp_action::CancelResponse::ACCEPT;
}

void RecordReplayPlannerNode::replay_handle_accepted(
  const std::shared_ptr<GoalHandleReplayTrajectory> goal_handle)
{
  (void)goal_handle;
  // TODO(s.me) what to do here? Is this where we actually trigger something?
  m_planner->start_replaying();
}

void RecordReplayPlannerNode::set_planner(PlannerPtr && planner) noexcept
{
  m_planner = std::forward<PlannerPtr &&>(planner);
}
}  // namespace recordreplay_planner_node
}  // namespace planning
}  // namespace motion
