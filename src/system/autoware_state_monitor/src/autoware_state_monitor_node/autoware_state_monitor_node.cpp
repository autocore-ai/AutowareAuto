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

#include "autoware_state_monitor/autoware_state_monitor_node.hpp"

#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

namespace autoware
{
namespace state_monitor
{

AutowareStateMonitorNode::AutowareStateMonitorNode(const rclcpp::NodeOptions & node_options)
: Node("autoware_state_monitor", node_options),
  tf_buffer_(this->get_clock()),
  tf_listener_(tf_buffer_)
{
  using std::placeholders::_1;
  using std::placeholders::_2;
  using std::placeholders::_3;

  // Parameters
  update_rate_ = this->declare_parameter("update_rate", 10.0);
  local_frame_ = this->declare_parameter("local_frame", "base_link");
  global_frame_ = this->declare_parameter("global_frame", "map");

  // Parameters for StateMachine
  state_param_.arrived_distance_threshold =
    this->declare_parameter("arrived_distance_threshold", 1.0);
  state_param_.stopped_time_threshold =
    this->declare_parameter("stopped_time_threshold", 1.0);
  state_param_.stopped_velocity_threshold_mps =
    this->declare_parameter("stopped_velocity_threshold_mps", 0.01);
  state_param_.wait_time_after_initializing =
    this->declare_parameter("wait_time_after_initializing", 1.0);
  state_param_.wait_time_after_planning =
    this->declare_parameter("wait_time_after_planning", 1.0);
  state_param_.wait_time_after_arrived_goal =
    this->declare_parameter("wait_time_after_arrived_goal", 2.0);

  // State Machine
  state_machine_ = std::make_shared<StateMachine>(state_param_);

  // Odometry Updater
  odometry_updater_ = std::make_shared<OdometryUpdater>(
    state_input_.odometry_buffer, state_param_.stopped_time_threshold);

  // Callback Groups
  callback_group_subscribers_ = this->create_callback_group(
    rclcpp::CallbackGroupType::MutuallyExclusive);
  callback_group_services_ = this->create_callback_group(
    rclcpp::CallbackGroupType::MutuallyExclusive);
  auto subscriber_option = rclcpp::SubscriptionOptions();
  subscriber_option.callback_group = callback_group_subscribers_;

  // Subscriber
  sub_engage_ = this->create_subscription<Engage>(
    "input/engage", 1,
    std::bind(&AutowareStateMonitorNode::onAutowareEngage, this, _1), subscriber_option);
  sub_vehicle_state_report_ =
    this->create_subscription<VehicleStateReport>(
    "input/vehicle_state_report", 1,
    std::bind(&AutowareStateMonitorNode::onVehicleStateReport, this, _1), subscriber_option);
  sub_route_ = this->create_subscription<HADMapRoute>(
    "input/route", 1,
    std::bind(&AutowareStateMonitorNode::onRoute, this, _1), subscriber_option);
  sub_odometry_ = this->create_subscription<VehicleOdometry>(
    "input/odometry", 100,
    std::bind(&AutowareStateMonitorNode::onVehicleOdometry, this, _1), subscriber_option);

  // Service
  srv_shutdown_ = this->create_service<std_srvs::srv::Trigger>(
    "service/shutdown",
    std::bind(&AutowareStateMonitorNode::onShutdownService, this, _1, _2, _3),
    rmw_qos_profile_services_default, callback_group_services_);

  // Publisher
  pub_autoware_state_ =
    this->create_publisher<autoware_auto_msgs::msg::AutowareState>("output/autoware_state", 1);

  // Timer
  auto timer_callback = std::bind(&AutowareStateMonitorNode::onTimer, this);
  auto period = std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::duration<double>(1.0 / update_rate_));

  timer_ = std::make_shared<rclcpp::GenericTimer<decltype(timer_callback)>>(
    this->get_clock(), period, std::move(timer_callback),
    this->get_node_base_interface()->get_context());
  this->get_node_timers_interface()->add_timer(timer_, callback_group_subscribers_);
}

void AutowareStateMonitorNode::onAutowareEngage(const Engage::ConstSharedPtr msg)
{
  state_input_.engage = msg;
}

void AutowareStateMonitorNode::onVehicleStateReport(
  const VehicleStateReport::ConstSharedPtr msg)
{
  state_input_.vehicle_state_report = msg;
}

void AutowareStateMonitorNode::onRoute(const HADMapRoute::ConstSharedPtr msg)
{
  using RoutePoint = autoware_auto_msgs::msg::RoutePoint;

  state_input_.route = msg;

  // Get goal pose
  auto point = std::make_shared<RoutePoint>();
  *point = msg->goal_point;
  state_input_.goal_pose = RoutePoint::ConstSharedPtr(point);
}

void AutowareStateMonitorNode::onVehicleOdometry(
  const VehicleOdometry::ConstSharedPtr msg)
{
  odometry_updater_->update(msg);
}

bool AutowareStateMonitorNode::onShutdownService(
  const std::shared_ptr<rmw_request_id_t> request_header,
  const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
  const std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
  (void)request_header;
  (void)request;
  state_input_.is_finalizing = true;

  const auto t_start = this->get_clock()->now();
  constexpr double timeout = 3.0;

  while (rclcpp::ok()) {
    if (state_machine_->getCurrentState() == autoware_auto_msgs::msg::AutowareState::FINALIZING) {
      response->success = true;
      response->message = "Shutdown Autoware.";
      return true;
    }

    if ((this->get_clock()->now() - t_start).seconds() > timeout) {
      response->success = false;
      response->message = "Shutdown timeout.";
      return true;
    }

    rclcpp::Rate(10.0).sleep();
  }

  response->success = false;
  response->message = "Shutdown failure.";
  return true;
}

void AutowareStateMonitorNode::onTimer()
{
  const auto state = updateState();
  publishAutowareState(state);
}

State AutowareStateMonitorNode::updateState()
{
  state_input_.current_pose = getCurrentPose(tf_buffer_);
  state_input_.current_time = this->now();

  const auto prev_autoware_state = state_machine_->getCurrentState();
  const auto autoware_state = state_machine_->updateState(state_input_);

  if (autoware_state != prev_autoware_state) {
    RCLCPP_INFO(
      this->get_logger(), "state changed: %s -> %s",
      toString(prev_autoware_state).c_str(),
      toString(autoware_state).c_str());
  }

  return autoware_state;
}

void AutowareStateMonitorNode::publishAutowareState(const State & state)
{
  autoware_auto_msgs::msg::AutowareState autoware_state_msg;
  autoware_state_msg.stamp = get_clock()->now();
  autoware_state_msg.state = state;
  pub_autoware_state_->publish(autoware_state_msg);
}

geometry_msgs::msg::PoseStamped::SharedPtr AutowareStateMonitorNode::getCurrentPose(
  const tf2_ros::Buffer & tf_buffer) const
{
  geometry_msgs::msg::TransformStamped tf_current_pose;

  try {
    tf_current_pose = tf_buffer.lookupTransform(
      global_frame_, local_frame_, tf2::TimePointZero);
  } catch (tf2::TransformException & ex) {
    return nullptr;
  }

  auto pose = std::make_shared<geometry_msgs::msg::PoseStamped>();
  pose->header = tf_current_pose.header;
  pose->pose.orientation = tf_current_pose.transform.rotation;
  pose->pose.position.x = tf_current_pose.transform.translation.x;
  pose->pose.position.y = tf_current_pose.transform.translation.y;
  pose->pose.position.z = tf_current_pose.transform.translation.z;

  return pose;
}

}  // namespace state_monitor
}  // namespace autoware

#include "rclcpp_components/register_node_macro.hpp"  // NOLINT
RCLCPP_COMPONENTS_REGISTER_NODE(
  autoware::state_monitor::AutowareStateMonitorNode)
