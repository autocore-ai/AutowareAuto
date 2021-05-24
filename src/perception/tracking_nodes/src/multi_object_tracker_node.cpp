// Copyright 2021 The Autoware Foundation
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

#include "tracking_nodes/multi_object_tracker_node.hpp"

#include <rclcpp_components/register_node_macro.hpp>

#include <cstddef>
#include <cstdint>
#include <functional>
#include <memory>
#include <string>
#include <utility>

namespace autoware
{
namespace tracking_nodes
{

using autoware::common::types::float32_t;
using autoware::common::types::float64_t;
using autoware::perception::tracking::MultiObjectTracker;
using autoware::perception::tracking::MultiObjectTrackerOptions;
using autoware::perception::tracking::TrackerUpdateResult;
using autoware::perception::tracking::TrackerUpdateStatus;
using autoware_auto_msgs::msg::DetectedObjects;
using autoware_auto_msgs::msg::TrackedObjects;
using nav_msgs::msg::Odometry;
using std::placeholders::_1;
using std::placeholders::_2;

namespace
{
MultiObjectTracker init_tracker(rclcpp::Node & node)
{
  const float32_t max_distance =
    static_cast<float32_t>(node.declare_parameter(
      "association_max_distance").get<float64_t>());
  const float32_t max_area_ratio =
    static_cast<float32_t>(node.declare_parameter(
      "association_max_area_ratio").get<float64_t>());
  const float32_t default_variance =
    static_cast<float32_t>(node.declare_parameter(
      "ekf_default_variance").get<float64_t>());
  const float32_t noise_variance =
    static_cast<float32_t>(node.declare_parameter(
      "ekf_noise_variance").get<float64_t>());
  const std::chrono::nanoseconds pruning_time_threshold =
    std::chrono::milliseconds(
    node.declare_parameter(
      "pruning_time_threshold_ms").get<int64_t>());
  const std::size_t pruning_ticks_threshold =
    static_cast<std::size_t>(node.declare_parameter(
      "pruning_ticks_threshold").get<int64_t>());
  MultiObjectTrackerOptions options{{max_distance, max_area_ratio}, default_variance,
    noise_variance, pruning_time_threshold, pruning_ticks_threshold};
  return MultiObjectTracker{options};
}

std::string status_to_string(TrackerUpdateStatus status)
{
  // Use a switch statement without default since it warns when not all cases are handled.
  switch (status) {
    case TrackerUpdateStatus::Ok: return "Ok";
    case TrackerUpdateStatus::WentBackInTime: return "WentBackInTime";
    case TrackerUpdateStatus::DetectionFrameMismatch: return "DetectionFrameMismatch";
    case TrackerUpdateStatus::TrackerFrameMismatch: return "TrackerFrameMismatch";
    case TrackerUpdateStatus::FrameNotGravityAligned: return "FrameNotGravityAligned";
    case TrackerUpdateStatus::InvalidShape: return "InvalidShape";
    case TrackerUpdateStatus::EmptyDetection: return "EmptyDetection";
  }
  return "Invalid status";
}

}  // namespace

MultiObjectTrackerNode::MultiObjectTrackerNode(const rclcpp::NodeOptions & options)
:  Node("multi_object_tracker_node", options),
  m_tracker(init_tracker(*this))
{
  const size_t history_depth = static_cast<size_t>(this->declare_parameter("history_depth", 20));
  m_pub = this->create_publisher<TrackedObjects>("tracked_objects", history_depth);
  m_objects_sub.subscribe(
    this, "detected_objects",
    rclcpp::QoS(history_depth).get_rmw_qos_profile());
  m_odom_sub.subscribe(this, "odometry", rclcpp::QoS(history_depth).get_rmw_qos_profile());

  m_sync =
    std::make_shared<message_filters::TimeSynchronizer<DetectedObjects, Odometry>>(
    m_objects_sub, m_odom_sub, history_depth);
  m_sync->registerCallback(std::bind(&MultiObjectTrackerNode::process, this, _1, _2));
}

void MultiObjectTrackerNode::process(
  const DetectedObjects::ConstSharedPtr & objs,
  const Odometry::ConstSharedPtr & odom)
{
  TrackerUpdateResult result = m_tracker.update(*objs, *odom);
  if (result.status == TrackerUpdateStatus::Ok) {
    // The tracker returns its result in a unique_ptr, so the more efficient publish(unique_ptr<T>)
    // overload can be used.
    m_pub->publish(std::move(result.objects));
  } else {
    RCLCPP_WARN(
      get_logger(), "Tracker update for detection at time %d.%d failed. Reason: %s",
      objs->header.stamp.sec, objs->header.stamp.nanosec,
      status_to_string(result.status).c_str());
  }
}

}  // namespace tracking_nodes
}  // namespace autoware

// This acts as an entry point, allowing the component to be
// discoverable when its library is being loaded into a running process
RCLCPP_COMPONENTS_REGISTER_NODE(autoware::tracking_nodes::MultiObjectTrackerNode)
