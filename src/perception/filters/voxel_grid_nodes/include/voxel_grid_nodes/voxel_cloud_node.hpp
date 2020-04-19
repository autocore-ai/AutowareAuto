// Copyright 2017-2019 Apex.AI, Inc.
// Co-developed by Tier IV, Inc. and Apex.AI, Inc.
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

/// \file
/// \brief This file defines the algorithmic interface for applying voxel grid downsampling to a
///        PointBlock message
#ifndef VOXEL_GRID_NODES__VOXEL_CLOUD_NODE_HPP_
#define VOXEL_GRID_NODES__VOXEL_CLOUD_NODE_HPP_

#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <voxel_grid_nodes/algorithm/voxel_cloud_base.hpp>
#include <rclcpp/rclcpp.hpp>
#include <lifecycle_msgs/msg/transition.hpp>
#include <lifecycle_msgs/msg/state.hpp>
#include <common/types.hpp>
#include <memory>
#include <string>

using autoware::common::types::bool8_t;

namespace autoware
{
namespace perception
{
namespace filters
{
/// \brief Objects that tie voxel_grid classes to Apex.OS and interprocess communication
namespace voxel_grid_nodes
{
rclcpp::QoS parse_qos(
  const rclcpp::ParameterValue & durability_param,
  const rclcpp::ParameterValue & depth_param, const rclcpp::QoS & default_qos = rclcpp::QoS(10));

/// \brief Boilerplate node that subscribes to point clouds and publishes a downsampled version
class VOXEL_GRID_NODES_PUBLIC VoxelCloudNode : public rclcpp_lifecycle::LifecycleNode
{
public:
  /// \brief Parameter constructor
  /// \param[in] node_name Name of the node, controls which parameter set from the file is matched
  /// \param[in] node_namespace Name of the node's namespace, controls which parameters are used
  VoxelCloudNode(
    const std::string & node_name,
    const std::string & node_namespace = "");

  /// \brief Explicit constructor
  /// \param[in] node_name Name of the node
  /// \param[in] sub_topic Name of input topic
  /// \param[in] pub_topic Name of downsampled output topic
  /// \param[in] cfg Configuration object for VoxelGrid
  /// \param[in] is_approximate Whether the internal voxel grid is approximate or not (centroid)
  /// \param sub_qos QoS profile for the subscription. By default, set to depth of 10.
  /// \param pub_qos QoS profile for the publisher. By default, set to depth of 10.
  VoxelCloudNode(
    const std::string & node_name,
    const std::string & sub_topic,
    const std::string & pub_topic,
    const voxel_grid::Config & cfg,
    const bool8_t is_approximate,
    const rclcpp::QoS sub_qos = rclcpp::QoS(10),
    const rclcpp::QoS pub_qos = rclcpp::QoS(10));

  /// \brief Core run loop
  void callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);

private:
  /// \brief Initialize state transition callbacks and voxel grid
  /// \param[in] cfg Configuration object for voxel grid
  /// \param[in] is_approximate whether to instantiate an approximate or centroid voxel grid
  void VOXEL_GRID_NODES_LOCAL init(const voxel_grid::Config & cfg, const bool8_t is_approximate);

  using Message = sensor_msgs::msg::PointCloud2;

  using Transition = lifecycle_msgs::msg::Transition;

  const rclcpp::Subscription<Message>::SharedPtr m_sub_ptr;
  const std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<Message>> m_pub_ptr;
  std::unique_ptr<algorithm::VoxelCloudBase> m_voxelgrid_ptr;
  bool8_t m_has_failed;
};  // VoxelCloudNode
}  // namespace voxel_grid_nodes
}  // namespace filters
}  // namespace perception
}  // namespace autoware

#endif  // VOXEL_GRID_NODES__VOXEL_CLOUD_NODE_HPP_
