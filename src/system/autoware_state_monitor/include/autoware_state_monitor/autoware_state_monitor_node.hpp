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

#ifndef AUTOWARE_STATE_MONITOR__AUTOWARE_STATE_MONITOR_NODE_HPP_
#define AUTOWARE_STATE_MONITOR__AUTOWARE_STATE_MONITOR_NODE_HPP_

#include <memory>
#include <string>

// ROS
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

// Autoware
#include "autoware_auto_msgs/msg/autoware_state.hpp"
#include "autoware_auto_msgs/msg/engage.hpp"
#include "autoware_auto_msgs/msg/had_map_route.hpp"
#include "autoware_auto_msgs/msg/vehicle_odometry.hpp"
#include "autoware_auto_msgs/msg/vehicle_state_report.hpp"

// Local
#include "autoware_state_monitor/state.hpp"
#include "autoware_state_monitor/odometry_updater.hpp"
#include "autoware_state_monitor/state_machine.hpp"
#include "autoware_state_monitor/visibility_control.hpp"

namespace autoware
{
namespace state_monitor
{

/// \brief A node for monitoring the state of Autoware system
class AUTOWARE_STATE_MONITOR_PUBLIC AutowareStateMonitorNode : public rclcpp::Node
{
public:
  /// \brief Constructor
  /// \param[in] node_options Node options
  explicit AutowareStateMonitorNode(const rclcpp::NodeOptions & node_options);

private:
  using VehicleStateReport = autoware_auto_msgs::msg::VehicleStateReport;
  using VehicleOdometry = autoware_auto_msgs::msg::VehicleOdometry;
  using HADMapRoute = autoware_auto_msgs::msg::HADMapRoute;
  using Engage = autoware_auto_msgs::msg::Engage;

  // Parameters
  double update_rate_;
  /// Local (child) frame used during the vehicle pose estimation
  std::string local_frame_;
  /// Global (parent) frame used during the vehicle pose estimation
  std::string global_frame_;

  // TF
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  // CallbackGroups
  rclcpp::CallbackGroup::SharedPtr callback_group_subscribers_;
  rclcpp::CallbackGroup::SharedPtr callback_group_services_;

  // Subscribers
  rclcpp::Subscription<Engage>::SharedPtr sub_engage_;
  rclcpp::Subscription<VehicleStateReport>::SharedPtr sub_vehicle_state_report_;
  rclcpp::Subscription<HADMapRoute>::SharedPtr sub_route_;
  rclcpp::Subscription<VehicleOdometry>::SharedPtr sub_odometry_;

  void onAutowareEngage(const Engage::ConstSharedPtr msg);
  void onVehicleStateReport(const VehicleStateReport::ConstSharedPtr msg);
  void onRoute(const HADMapRoute::ConstSharedPtr msg);
  void onVehicleOdometry(const VehicleOdometry::ConstSharedPtr msg);

  // Service
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr srv_shutdown_;

  bool onShutdownService(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response);

  // Publisher
  rclcpp::Publisher<autoware_auto_msgs::msg::AutowareState>::SharedPtr pub_autoware_state_;

  // Timer
  void onTimer();
  rclcpp::TimerBase::SharedPtr timer_;

  // State Machine
  State updateState();
  void publishAutowareState(const State & state);
  std::shared_ptr<StateMachine> state_machine_;
  StateInput state_input_;
  StateMachineParams state_param_;

  geometry_msgs::msg::PoseStamped::SharedPtr getCurrentPose(
    const tf2_ros::Buffer & tf_buffer) const;

  std::shared_ptr<OdometryUpdater> odometry_updater_;
};

}  // namespace state_monitor
}  // namespace autoware

#endif  // AUTOWARE_STATE_MONITOR__AUTOWARE_STATE_MONITOR_NODE_HPP_
