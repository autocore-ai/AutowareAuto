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
  using Odometry = nav_msgs::msg::Odometry;

  using OdomSubscriber = message_filters::Subscriber<nav_msgs::msg::Odometry>;
  using PoseSubscriber = message_filters::Subscriber<geometry_msgs::msg
      ::PoseWithCovarianceStamped>;

  using OdomCache = message_filters::Cache<nav_msgs::msg::Odometry>;
  using PoseCache = message_filters::Cache<geometry_msgs::msg::PoseWithCovarianceStamped>;

public:
  /// \brief Constructor
  explicit MultiObjectTrackerNode(const rclcpp::NodeOptions & options);

  /// Callback for matching detections + odom messages.
  /// This unusual signature is mandated by message_filters.
  void process(
    const DetectedObjects::ConstSharedPtr & objs,
    const Odometry::ConstSharedPtr & odom);
  /// Callback for matching vision detections + odom messages.
  /// This unusual signature is mandated by message_filters.
  void process(
    const ClassifiedRoiArray::ConstSharedPtr & rois,
    const Odometry::ConstSharedPtr & odom);

  // Forward declare structs that handle variant. Declaration outside the class to better handle
  // formatting with uncrustify
  struct ProcessLidar;
  struct ProcessVision;

private:
  bool8_t m_use_vision = true;

  tf2::BufferCore m_tf_buffer;
  tf2_ros::TransformListener m_tf_listener;

  /// The actual tracker implementation.
  autoware::perception::tracking::MultiObjectTracker m_tracker;
  size_t m_history_depth = 0U;
  bool8_t m_use_ndt = true;

  /// Subscription to detection messages.
  rclcpp::Subscription<DetectedObjects>::SharedPtr m_lidar_clusters_sub;
  std::experimental::optional<rclcpp::Subscription<ClassifiedRoiArray>::SharedPtr>
  m_maybe_vision_sub;

  mpark::variant<PoseSubscriber, OdomSubscriber> m_pose_or_odom_sub;
  mpark::variant<std::shared_ptr<OdomCache>, std::shared_ptr<PoseCache>> m_pose_or_odom_cache;

  /// A Publisher for tracked objects.
  rclcpp::Publisher<autoware_auto_msgs::msg::TrackedObjects>::SharedPtr m_track_publisher;
  /// A publisher for all the leftover objects.
  rclcpp::Publisher<autoware_auto_msgs::msg::DetectedObjects>::SharedPtr m_leftover_publisher;
};

/// Struct to call the process function with correct arguments for the different types of cache
/// variant
struct MultiObjectTrackerNode::ProcessLidar
{
  ProcessLidar(MultiObjectTrackerNode * tracker_ptr, DetectedObjects::ConstSharedPtr msg);
  void operator()(const std::shared_ptr<OdomCache> & cache_ptr);
  void operator()(const std::shared_ptr<PoseCache> & cache_ptr);

private:
  DetectedObjects::ConstSharedPtr m_msg;
  MultiObjectTrackerNode * m_node_ptr;
  rclcpp::Time m_left_interval;
  rclcpp::Time m_right_interval;
};

/// Struct to call the process function with correct arguments for the different types of cache
/// variant
struct MultiObjectTrackerNode::ProcessVision
{
  ProcessVision(MultiObjectTrackerNode * tracker_ptr, ClassifiedRoiArray::ConstSharedPtr msg);
  void operator()(const std::shared_ptr<OdomCache> & cache_ptr);
  void operator()(const std::shared_ptr<PoseCache> & cache_ptr);

private:
  ClassifiedRoiArray::ConstSharedPtr m_msg;
  MultiObjectTrackerNode * m_node_ptr;
  rclcpp::Time m_left_interval;
  rclcpp::Time m_right_interval;
};

}  // namespace tracking_nodes
}  // namespace autoware

#endif  // TRACKING_NODES__MULTI_OBJECT_TRACKER_NODE_HPP_
