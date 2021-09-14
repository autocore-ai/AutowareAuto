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
/// \brief This file defines the ground_truth_visualizer_node class.

#ifndef CLUSTER_PROJECTION_NODE__CLUSTER_PROJECTION_NODE_HPP_
#define CLUSTER_PROJECTION_NODE__CLUSTER_PROJECTION_NODE_HPP_

#include <autoware_auto_msgs/msg/classified_roi_array.hpp>
#include <autoware_auto_msgs/msg/detected_objects.hpp>
#include <cluster_projection_node/visibility_control.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tracking/projection.hpp>
#include <tf2/buffer_core.h>
#include <tf2_ros/transform_listener.h>

#include <memory>
#include <string>

namespace autoware
{
namespace cluster_projection_node
{

/// Class receives object clusters, projects them and publisheas the projections as a roi array
class CLUSTER_PROJECTION_NODE_PUBLIC ClusterProjectionNode : public rclcpp::Node
{
public:
  explicit ClusterProjectionNode(const rclcpp::NodeOptions & options);

  /// Project and publish clusters
  void cluster_callback(
    autoware_auto_msgs::msg::DetectedObjects::ConstSharedPtr objects_msg);

private:
  rclcpp::Subscription<autoware_auto_msgs::msg::DetectedObjects>::SharedPtr m_clusters_sub;
  rclcpp::Publisher<autoware_auto_msgs::msg::ClassifiedRoiArray>::SharedPtr m_projection_pub;
  autoware::perception::tracking::CameraModel m_camera_model;
  tf2::BufferCore m_buffer;
  tf2_ros::TransformListener m_tf_listener;
  std::string m_camera_frame;
};
}  // namespace cluster_projection_node
}  // namespace autoware

#endif  // CLUSTER_PROJECTION_NODE__CLUSTER_PROJECTION_NODE_HPP_
