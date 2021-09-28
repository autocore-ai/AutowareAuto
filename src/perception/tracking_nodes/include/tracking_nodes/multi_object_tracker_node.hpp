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
//
// Co-developed by Tier IV, Inc. and Apex.AI, Inc.

/// \copyright Copyright 2021 The Autoware Foundation
/// \file
/// \brief This file defines the tracking_nodes_node class.

#ifndef TRACKING_NODES__MULTI_OBJECT_TRACKER_NODE_HPP_
#define TRACKING_NODES__MULTI_OBJECT_TRACKER_NODE_HPP_

#include <autoware_auto_msgs/msg/detected_objects.hpp>
#include <autoware_auto_msgs/msg/tracked_objects.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <message_filters/cache.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/time_synchronizer.h>
#include <mpark_variant_vendor/variant.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2/buffer_core.h>
#include <tf2_msgs/msg/tf_message.hpp>
#include <tf2_ros/transform_listener.h>
#include <tracking/multi_object_tracker.hpp>
#include <tracking_nodes/visibility_control.hpp>

#include <memory>
#include <string>

namespace autoware
{
namespace tracking_nodes
{

/// \class MultiObjectTrackerNode
/// \brief ROS 2 Node for tracking. Subscribes to DetectedObjects and Odometry or
///        PoseWithCovairanceStamped (depends on use_ndt param) and produces TrackedObjects
class TRACKING_NODES_PUBLIC MultiObjectTrackerNode : public rclcpp::Node
{
  using DetectedObjects = autoware_auto_msgs::msg::DetectedObjects;
  using ClassifiedRoiArray = autoware_auto_msgs::msg::ClassifiedRoiArray;
  using OdometryMsg = nav_msgs::msg::Odometry;
  using PoseMsg = geometry_msgs::msg::PoseWithCovarianceStamped;
  using OdomCache = message_filters::Cache<OdometryMsg>;

public:
  /// \brief Constructor
  explicit MultiObjectTrackerNode(const rclcpp::NodeOptions & options);

private:
  void TRACKING_NODES_LOCAL odometry_callback(const OdometryMsg::ConstSharedPtr msg);
  void TRACKING_NODES_LOCAL pose_callback(const PoseMsg::ConstSharedPtr msg);
  void TRACKING_NODES_LOCAL detected_objects_callback(const DetectedObjects::ConstSharedPtr msg);
  void TRACKING_NODES_LOCAL classified_roi_callback(const ClassifiedRoiArray::ConstSharedPtr msg);

  common::types::bool8_t m_use_ndt{true};
  common::types::bool8_t m_use_vision{true};
  std::size_t m_history_depth{0UL};

  tf2::BufferCore m_tf_buffer;
  tf2_ros::TransformListener m_tf_listener;

  /// The actual tracker implementation.
  autoware::perception::tracking::MultiObjectTracker m_tracker;

  rclcpp::Subscription<PoseMsg>::SharedPtr m_pose_subscription{};
  rclcpp::Subscription<OdometryMsg>::SharedPtr m_odom_subscription{};
  rclcpp::Subscription<DetectedObjects>::SharedPtr m_detected_objects_subscription{};
  rclcpp::Subscription<ClassifiedRoiArray>::SharedPtr m_vision_subcription{};

  /// A cache that stores the odometry messages.
  std::unique_ptr<OdomCache> m_odom_cache{};

  /// A Publisher for tracked objects.
  rclcpp::Publisher<autoware_auto_msgs::msg::TrackedObjects>::SharedPtr m_track_publisher{};
  /// A publisher for all the leftover objects.
  rclcpp::Publisher<autoware_auto_msgs::msg::DetectedObjects>::SharedPtr m_leftover_publisher{};

  // Visualization variables & functions
  void maybe_visualize(
    const perception::tracking::DetectedObjectsUpdateResult & result,
    DetectedObjects all_objects);

  bool8_t m_visualize_track_creation = false;

  rclcpp::Publisher<DetectedObjects>::SharedPtr m_track_creating_clusters_pub;
};

}  // namespace tracking_nodes
}  // namespace autoware

#endif  // TRACKING_NODES__MULTI_OBJECT_TRACKER_NODE_HPP_
