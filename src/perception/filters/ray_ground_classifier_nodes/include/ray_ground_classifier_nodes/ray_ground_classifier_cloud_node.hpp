// Copyright 2017-2018 Apex.AI, Inc.
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
/// \brief This file defines a ROS 2 node that contains a VLP16 driver and a ground filter and
///        publishes three different point cloud topics

#ifndef RAY_GROUND_CLASSIFIER_NODES__RAY_GROUND_CLASSIFIER_CLOUD_NODE_HPP_
#define RAY_GROUND_CLASSIFIER_NODES__RAY_GROUND_CLASSIFIER_CLOUD_NODE_HPP_

#include <common/types.hpp>
#include <lifecycle_msgs/msg/state.hpp>
#include <lifecycle_msgs/msg/transition.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <ray_ground_classifier/ray_ground_classifier.hpp>
#include <ray_ground_classifier/ray_aggregator.hpp>
#include <lidar_utils/point_cloud_utils.hpp>

#include <memory>
#include <string>

#include "ray_ground_classifier_nodes/visibility_control.hpp"

using autoware::common::types::bool8_t;
using autoware::common::types::char8_t;

namespace autoware
{
namespace perception
{
namespace filters
{
namespace ray_ground_classifier_nodes
{

using sensor_msgs::msg::PointCloud2;

/// \brief A node that takes in unstructured point clouds and partitions them into ground and
///        nonground points
class RAY_GROUND_CLASSIFIER_PUBLIC RayGroundClassifierCloudNode
  : public rclcpp_lifecycle::LifecycleNode
{
public:
  /// \param[in] node_name Name of this node
  /// \param[in] node_namespace Name of this node's namespace
  /// \throw std::runtime_error if configuration fails
  RayGroundClassifierCloudNode(
    const std::string & node_name,
    const std::string & node_namespace = "");

  /// \brief Explicit constructor
  /// \param[in] node_name Name of this node
  /// \param[in] raw_topic Name of input topic of points directly from drivers
  /// \param[in] ground_topic Name of output topic of ground points
  /// \param[in] nonground_topic Name of output topic of nonground points
  /// \param[in] frame_id Name of coordinate frame id for outgoing clouds
  /// \param[in] timeout Timeout for waitset, i.e. max period between messages on raw_topic
  /// \param[in] pcl_size Number of points to preallocate in pointcloud messages
  /// \param[in] cfg Configuration class for ground classifier
  /// \param[in] agg_cfg Configuration class for ray aggregator
  /// \throw std::runtime_error if configuration fails
  RayGroundClassifierCloudNode(
    const std::string & node_name,
    const std::string & raw_topic,
    const std::string & ground_topic,
    const std::string & nonground_topic,
    const std::string & frame_id,
    const std::chrono::nanoseconds & timeout,
    const std::size_t pcl_size,
    const ray_ground_classifier::Config & cfg,
    const ray_ground_classifier::RayAggregator::Config & agg_cfg);

private:
  /// \brief Activates publishers
  /// \return Success, failure, or error key
  RAY_GROUND_CLASSIFIER_NODES_LOCAL
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_activate_internal(const rclcpp_lifecycle::State &);
  /// \brief Deactivates publishers
  /// \return Success, failure, or error key
  RAY_GROUND_CLASSIFIER_NODES_LOCAL
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_deactivate_internal(const rclcpp_lifecycle::State &);
  /// \brief Registers on_activate() and on_deactivate() callbacks, allocates messages,
  ///        for use in constructor
  RAY_GROUND_CLASSIFIER_NODES_LOCAL void register_callbacks_preallocate();
  /// \brief Resets state of ray aggregator and messages
  RAY_GROUND_CLASSIFIER_NODES_LOCAL void reset();
  // Algorithmic core
  ray_ground_classifier::RayGroundClassifier m_classifier;
  ray_ground_classifier::RayAggregator m_aggregator;
  // preallocated message
  PointCloud2 m_ground_msg;
  PointCloud2 m_nonground_msg;
  const std::size_t m_pcl_size;
  const std::string m_frame_id;
  // Basic stateful stuff, will get refactored after we have a proper state machine implementation
  bool8_t m_has_failed;
  // publishers and subscribers
  const std::chrono::nanoseconds m_timeout;
  const rclcpp::Subscription<PointCloud2>::SharedPtr m_raw_sub_ptr;
  const std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<PointCloud2>> m_ground_pub_ptr;
  const std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<PointCloud2>> m_nonground_pub_ptr;
  /// \brief Read samples from the subscription
  void callback(const PointCloud2::SharedPtr msg);
  uint32_t m_ground_pc_idx;
  autoware::common::lidar_utils::PointCloudIts m_ground_pc_its;
  uint32_t m_nonground_pc_idx;
  autoware::common::lidar_utils::PointCloudIts m_nonground_pc_its;
};  // class RayGroundFilterDriverNode
}  // namespace ray_ground_classifier_nodes
}  // namespace filters
}  // namespace perception
}  // namespace autoware
#endif  // RAY_GROUND_CLASSIFIER_NODES__RAY_GROUND_CLASSIFIER_CLOUD_NODE_HPP_
