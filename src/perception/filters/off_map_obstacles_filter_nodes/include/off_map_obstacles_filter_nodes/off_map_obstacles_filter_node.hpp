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
/// \brief This file defines the off_map_obstacles_filter_nodes_node class.

#ifndef OFF_MAP_OBSTACLES_FILTER_NODES__OFF_MAP_OBSTACLES_FILTER_NODE_HPP_
#define OFF_MAP_OBSTACLES_FILTER_NODES__OFF_MAP_OBSTACLES_FILTER_NODE_HPP_

#include <memory>
#include <string>

#include "autoware_auto_msgs/msg/bounding_box_array.hpp"
#include "autoware_auto_msgs/srv/had_map_service.hpp"
#include "off_map_obstacles_filter_nodes/visibility_control.hpp"
#include "off_map_obstacles_filter/off_map_obstacles_filter.hpp"
#include "common/types.hpp"

#include "lanelet2_core/LaneletMap.h"
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "visualization_msgs/msg/marker_array.hpp"

namespace autoware
{
// Namespace for the node that filters off-map obstacles
namespace off_map_obstacles_filter_nodes
{

using float64_t = autoware::common::types::float64_t;

using OffMapObstaclesFilter = autoware::off_map_obstacles_filter::OffMapObstaclesFilter;
/// \class OffMapObstaclesFilterNode
/// \brief ROS 2 Node that removes obstacles that are off the map. See the design doc for more.
class OFF_MAP_OBSTACLES_FILTER_NODES_PUBLIC OffMapObstaclesFilterNode : public rclcpp::Node
{
public:
  /// \brief Constructor.
  /// \param options Node options.
  explicit OffMapObstaclesFilterNode(const rclcpp::NodeOptions & options);

  /// \brief Callback for the client call
  /// \param msg The BoundingBoxArray message containing obstacles
  void map_response(const rclcpp::Client<autoware_auto_msgs::srv::HADMapService>::SharedFuture msg);

  /// \brief The main callback of this node
  /// \param msg The BoundingBoxArray message containing obstacles
  void process_bounding_boxes(const autoware_auto_msgs::msg::BoundingBoxArray::SharedPtr msg) const;

private:
  /// The actual filter implementation – this will be nullptr before the map has arrived.
  std::unique_ptr<OffMapObstaclesFilter> m_filter;
  /// Input
  const rclcpp::Subscription<autoware_auto_msgs::msg::BoundingBoxArray>::SharedPtr
    m_sub_ptr;
  /// Output
  const rclcpp::Publisher<autoware_auto_msgs::msg::BoundingBoxArray>::SharedPtr m_pub_ptr;
  /// Client for getting the map. Only used once.
  const rclcpp::Client<autoware_auto_msgs::srv::HADMapService>::SharedPtr m_map_client_ptr;
  /// Debugging publisher
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr m_marker_pub_ptr;
  /// TF buffer
  tf2_ros::Buffer m_tf2_buffer;
  /// TF listener
  tf2_ros::TransformListener m_tf2_listener;
  /// Stores the overlap before the filter has been constructed
  float64_t m_overlap_threshold{1.0};  // Placeholder value
};
}  // namespace off_map_obstacles_filter_nodes
}  // namespace autoware

#endif  // OFF_MAP_OBSTACLES_FILTER_NODES__OFF_MAP_OBSTACLES_FILTER_NODE_HPP_
