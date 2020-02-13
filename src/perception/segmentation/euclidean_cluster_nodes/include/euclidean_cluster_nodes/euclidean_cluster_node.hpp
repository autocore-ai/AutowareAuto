// Copyright 2019 Apex.AI, Inc.
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
/// \brief This file defines the euclidean cluster algorithm for object detection

#ifndef EUCLIDEAN_CLUSTER_NODES__EUCLIDEAN_CLUSTER_NODE_HPP_
#define EUCLIDEAN_CLUSTER_NODES__EUCLIDEAN_CLUSTER_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <autoware_auto_msgs/msg/point_clusters.hpp>
#include <euclidean_cluster_nodes/visibility_control.hpp>
#include <autoware_auto_msgs/msg/bounding_box_array.hpp>
#include <euclidean_cluster_nodes/details/common.hpp>
#include <euclidean_cluster/euclidean_cluster.hpp>
#include <voxel_grid_nodes/algorithm/voxel_cloud_approximate.hpp>
#include <memory>
#include <string>

namespace autoware
{
namespace perception
{
namespace segmentation
{
/// \brief Main instantiation of algorithms in object detection stack
namespace euclidean_cluster_nodes
{
using float32_t = float;
using bool8_t = bool;
using Clusters = euclidean_cluster::Clusters;
/// \brief Combined object detection node, primarily does clustering, can also do in-place
///        downsampling and bounding box formation
class EUCLIDEAN_CLUSTER_NODES_PUBLIC EuclideanClusterNode : public rclcpp::Node
{
  using PointCloud2 = sensor_msgs::msg::PointCloud2;
  using VoxelAlgorithm = filters::voxel_grid_nodes::algorithm::VoxelCloudApproximate;

public:
  /// \brief Constructor, no parameters are provided for launch parameter compatibility
  /// \param[in] node_name Name of the node
  /// \param[in] node_namespace Name of the node's namespace
  EuclideanClusterNode(
    const std::string & node_name,
    const std::string & node_namespace = "");

  /// \brief Explicit parameter constructor
  /// \param[in] node_name Name of the node
  /// \param[in] node_namespace Namespace of the node
  /// \param[in] cloud_topic Name of the input point cloud topic
  /// \param[in] cluster_topic Name of the raw output topic, may be empty if publishing is not
  ///                          desired
  /// \param[in] box_topic Name of the processed output topic, may be empty if computing bounding
  ///                      boxes and publishing is not desired
  /// \param[in] cls_cfg Configuration class for clustering algorithm
  /// \param[in] hash_cfg Configuration class for clustering algorithm's underlying data structure
  /// \param[in] use_lfit Whether to use lfit algorithm for bounding box computation, if false,
  ///                     the faster, less semantically accurate, eigenbox algorithm is used
  /// \param[in] use_z Whether to use the z component of the input data. If bounding boxes are also
  ///                  computed, will properly populate the z field of centroids and size
  /// \param[in] voxel_cfg_ptr Configuration class for an approximate voxel grid. If provided,
  ///                          inputs will be downsampled before clustering
  EuclideanClusterNode(
    const std::string & node_name,
    const std::string & node_namespace,
    const std::string & cloud_topic,
    const std::string & cluster_topic,
    const std::string & box_topic,
    const euclidean_cluster::Config & cls_cfg,
    const euclidean_cluster::HashConfig & hash_cfg,
    const bool8_t use_lfit = true,
    const bool8_t use_z = false,
    const std::unique_ptr<filters::voxel_grid::Config> voxel_cfg_ptr = nullptr);

private:
  /// \brief Main callback function
  void EUCLIDEAN_CLUSTER_NODES_LOCAL handle(const PointCloud2::SharedPtr msg_ptr);
  /// \brief Initialization function
  void EUCLIDEAN_CLUSTER_NODES_LOCAL init(const euclidean_cluster::Config & cfg);
  /// \brief Insert directly into clustering algorithm
  void EUCLIDEAN_CLUSTER_NODES_LOCAL insert_plain(const PointCloud2 & cloud);
  /// \brief Pass points through a voxel grid before inserting into clustering algorithm
  void EUCLIDEAN_CLUSTER_NODES_LOCAL insert_voxel(const PointCloud2 & cloud);
  /// \brief Dispatch to appropriate insertion method
  void EUCLIDEAN_CLUSTER_NODES_LOCAL insert(const PointCloud2 & cloud);
  /// \brief Updates cluster meta-information, and publishes
  void EUCLIDEAN_CLUSTER_NODES_LOCAL publish_clusters(
    Clusters & clusters,
    const std_msgs::msg::Header & header);
  /// \brief Dispatch to publishing and/or bounding box computation based on configuration
  void EUCLIDEAN_CLUSTER_NODES_LOCAL handle_clusters(
    Clusters & clusters,
    const std_msgs::msg::Header & header);

  // pub/sub
  const rclcpp::Subscription<PointCloud2>::SharedPtr m_cloud_sub_ptr;
  const rclcpp::Publisher<Clusters>::SharedPtr m_cluster_pub_ptr;
  const rclcpp::Publisher<BoundingBoxArray>::SharedPtr m_box_pub_ptr;
  // algorithms
  euclidean_cluster::EuclideanCluster m_cluster_alg;
  Clusters m_clusters;
  BoundingBoxArray m_boxes;
  std::unique_ptr<VoxelAlgorithm> m_voxel_ptr;
  const bool8_t m_use_lfit;
  const bool8_t m_use_z;
};  // class EuclideanClusterNode
}  // namespace euclidean_cluster_nodes
}  // namespace segmentation
}  // namespace perception
}  // namespace autoware
#endif  // EUCLIDEAN_CLUSTER_NODES__EUCLIDEAN_CLUSTER_NODE_HPP_
