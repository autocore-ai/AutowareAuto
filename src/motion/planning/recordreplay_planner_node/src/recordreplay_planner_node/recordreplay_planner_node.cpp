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


#include <common/types.hpp>
#include <recordreplay_planner_node/recordreplay_planner_node.hpp>
#include <autoware_auto_tf2/tf2_autoware_auto_msgs.hpp>
#include <memory>
#include <string>
#include <utility>

using autoware::common::types::float32_t;
using autoware::common::types::float64_t;

namespace motion
{
namespace planning
{
namespace recordreplay_planner_node
{
RecordReplayPlannerNode::RecordReplayPlannerNode(const std::string & name, const std::string & ns)
: Node{name, ns}
{
  const auto ego_topic = "vehicle_state";
  const auto trajectory_topic = "planned_trajectory";
  const auto bounding_boxes_topic = "obstacle_bounding_boxes";
  const auto heading_weight =
    static_cast<float64_t>(declare_parameter("heading_weight").get<float32_t>());
  const auto min_record_distance =
    static_cast<float64_t>(declare_parameter("min_record_distance").get<float32_t>());

  const VehicleConfig vehicle_param{
    static_cast<Real>(declare_parameter("vehicle.cg_to_front_m").get<float32_t>()),
    static_cast<Real>(declare_parameter("vehicle.cg_to_rear_m").get<float32_t>()),
    static_cast<Real>(declare_parameter("vehicle.front_corner_stiffness").get<float32_t>()),
    static_cast<Real>(declare_parameter("vehicle.rear_corner_stiffness").get<float32_t>()),
    static_cast<Real>(declare_parameter("vehicle.mass_kg").get<float32_t>()),
    static_cast<Real>(declare_parameter("vehicle.yaw_inertia_kgm2").get<float32_t>()),
    static_cast<Real>(declare_parameter("vehicle.width_m").get<float32_t>()),
    static_cast<Real>(declare_parameter("vehicle.front_overhang_m").get<float32_t>()),
    static_cast<Real>(declare_parameter("vehicle.rear_overhang_m").get<float32_t>())
  };
  init(ego_topic, trajectory_topic, bounding_boxes_topic, vehicle_param, heading_weight,
    min_record_distance);
}
////////////////////////////////////////////////////////////////////////////////
RecordReplayPlannerNode::RecordReplayPlannerNode(
  const std::string & name,
  const std::string & ns,
  const std::string & ego_topic,
  const std::string & trajectory_topic,
  const std::string & bounding_boxes_topic,
  const VehicleConfig & vehicle_param,
  const float64_t heading_weight,
  const float64_t min_record_distance)
: Node{name, ns}
{
  init(ego_topic, trajectory_topic, bounding_boxes_topic, vehicle_param, heading_weight,
    min_record_distance);
}

void RecordReplayPlannerNode::init(
  const std::string & ego_topic,
  const std::string & trajectory_topic,
  const std::string & bounding_boxes_topic,
  const VehicleConfig & vehicle_param,
  const float64_t heading_weight,
  const float64_t min_record_distance
)
{
  using rclcpp::QoS;

  // Setup Tf Buffer with listener
  rclcpp::Clock::SharedPtr clock = std::make_shared<rclcpp::Clock>(RCL_ROS_TIME);
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(clock);
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  // Set up action for control of recording and replaying
  m_recordserver = rclcpp_action::create_server<RecordTrajectory>(
    this->get_node_base_interface(),
    this->get_node_clock_interface(),
    this->get_node_logging_interface(),
    this->get_node_waitables_interface(),
    "recordtrajectory",
    [this](auto uuid, auto goal) {return this->record_handle_goal(uuid, goal);},
    [this](auto goal_handle) {return this->record_handle_cancel(goal_handle);},
    [this](auto goal_handle) {return this->record_handle_accepted(goal_handle);});

  m_replayserver = rclcpp_action::create_server<ReplayTrajectory>(
    this->get_node_base_interface(),
    this->get_node_clock_interface(),
    this->get_node_logging_interface(),
    this->get_node_waitables_interface(),
    "replaytrajectory",
    [this](auto uuid, auto goal) {return this->replay_handle_goal(uuid, goal);},
    [this](auto goal_handle) {return this->replay_handle_cancel(goal_handle);},
    [this](auto goal_handle) {return this->replay_handle_accepted(goal_handle);});

  // Set up subscribers for the actual recording
  using SubAllocT = rclcpp::SubscriptionOptionsWithAllocator<std::allocator<void>>;
  m_ego_sub = create_subscription<State>(ego_topic, QoS{10},
      [this](const State::SharedPtr msg) {on_ego(msg);}, SubAllocT{});

  using SubAllocT = rclcpp::SubscriptionOptionsWithAllocator<std::allocator<void>>;
  m_boundingbox_sub = create_subscription<BoundingBoxArray>(bounding_boxes_topic,
      QoS{10},
      [this](const BoundingBoxArray::SharedPtr msg) {on_bounding_box(msg);}, SubAllocT{});

  // Set up publishers
  using PubAllocT = rclcpp::PublisherOptionsWithAllocator<std::allocator<void>>;
  m_trajectory_pub =
    create_publisher<Trajectory>(trajectory_topic, QoS{10}, PubAllocT{});

  m_trajectory_boundingbox_pub = create_publisher<BoundingBoxArray>("debug/trajectory_boxes",
      QoS{10});
  m_collison_boundingbox_pub = create_publisher<BoundingBoxArray>("debug/collison_boxes", QoS{10});
  m_transformed_boundingbox_pub = create_publisher<BoundingBoxArray>(
    "debug/transformed_obstacle_boxes", QoS{10});

  // Create and set a planner object that we'll talk to
  m_planner = std::make_unique<recordreplay_planner::RecordReplayPlanner>(vehicle_param);
  m_planner->set_heading_weight(heading_weight);
  m_planner->set_min_record_distance(min_record_distance);
}


void RecordReplayPlannerNode::on_ego(const State::SharedPtr & msg)
{
  if (m_odom_frame_id.empty()) {
    m_odom_frame_id = msg->header.frame_id;
  }

  if (m_planner->is_recording()) {
    RCLCPP_INFO_ONCE(this->get_logger(), "Recording ego position");
    m_planner->record_state(*msg);

    // Publish recording feedback information
    auto feedback_msg = std::make_shared<RecordTrajectory::Feedback>();
    feedback_msg->current_length = static_cast<int32_t>(m_planner->get_record_length());
    m_recordgoalhandle->publish_feedback(feedback_msg);
  }

  if (m_planner->is_replaying()) {
    RCLCPP_INFO_ONCE(this->get_logger(), "Replaying recorded ego postion as trajectory");
    const auto & traj = m_planner->plan(*msg);
    m_trajectory_pub->publish(traj);

    if (m_trajectory_boundingbox_pub->get_subscription_count() > 0U) {
      m_trajectory_boundingbox_pub->publish(m_planner->get_traj_boxes());
    }
    if (m_collison_boundingbox_pub->get_subscription_count() > 0U) {
      m_collison_boundingbox_pub->publish(m_planner->get_collision_boxes());
    }

    // Publish replaying feedback information
    auto feedback_msg = std::make_shared<ReplayTrajectory::Feedback>();
    feedback_msg->remaining_length = static_cast<int32_t>(traj.points.size());
    m_replaygoalhandle->publish_feedback(feedback_msg);
  }
}

void RecordReplayPlannerNode::on_bounding_box(const BoundingBoxArray::SharedPtr & msg)
{
  // Update most recent bounding box internally
  if (msg->header.frame_id == m_odom_frame_id) {
    m_planner->update_bounding_boxes(*msg);
  } else {
    tf2::Duration timeout = tf2::durationFromSec(0.2);
    if (tf_buffer_->canTransform(m_odom_frame_id, msg->header.frame_id,
      tf2_ros::fromMsg(msg->header.stamp), timeout) )
    {
      auto msg_tansformed = tf_buffer_->transform(*msg, m_odom_frame_id, timeout);
      if (m_transformed_boundingbox_pub->get_subscription_count() > 0U) {
        m_transformed_boundingbox_pub->publish(msg_tansformed);
      }
      m_planner->update_bounding_boxes(msg_tansformed);
    } else {
      RCLCPP_WARN(
        this->get_logger(), "on_bounding_box cannot transform %s to %s",
        msg->header.frame_id.c_str(), m_odom_frame_id.c_str());
    }
  }
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
  if (m_planner->is_recording()) {
    RCLCPP_INFO(this->get_logger(), "Cancel recording");
    m_planner->stop_recording();

    std::string record_path = goal_handle->get_goal()->record_path;

    // If a path is specified
    if (record_path.length() > 0) {
      // Write trajectory to file
      m_planner->writeTrajectoryBufferToFile(
        goal_handle->get_goal()->record_path);
    }
  }

  return rclcpp_action::CancelResponse::ACCEPT;
}

void RecordReplayPlannerNode::record_handle_accepted(
  const std::shared_ptr<GoalHandleRecordTrajectory> goal_handle)
{
  // Store the goal handle otherwise the action gets canceled immediately
  m_recordgoalhandle = goal_handle;
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
    RCLCPP_INFO(this->get_logger(), "Cancel replaying");
    m_planner->stop_replaying();
  }

  return rclcpp_action::CancelResponse::ACCEPT;
}

void RecordReplayPlannerNode::replay_handle_accepted(
  const std::shared_ptr<GoalHandleReplayTrajectory> goal_handle)
{
  // Store the goal handle otherwise the action gets canceled immediately
  m_replaygoalhandle = goal_handle;

  std::string replay_path = goal_handle->get_goal()->replay_path;

  // If a path is specified
  if (replay_path.length() > 0) {
    // Read trajectory from file
    m_planner->readTrajectoryBufferFromFile(
      goal_handle->get_goal()->replay_path);
  }

  // Publish loaded states as replaying feedback information
  auto feedback_msg = std::make_shared<ReplayTrajectory::Feedback>();
  auto & remaining_length = feedback_msg->remaining_length;
  remaining_length = static_cast<int32_t>(m_planner->get_record_length());
  m_replaygoalhandle->publish_feedback(feedback_msg);

  // Start the replaying process
  m_planner->start_replaying();
}
}  // namespace recordreplay_planner_node
}  // namespace planning
}  // namespace motion
