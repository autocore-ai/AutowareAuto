// Copyright 2020-2021 Arm Ltd., TierIV
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

#ifndef APOLLO_LIDAR_SEGMENTATION_NODES__APOLLO_LIDAR_SEGMENTATION_NODE_HPP_
#define APOLLO_LIDAR_SEGMENTATION_NODES__APOLLO_LIDAR_SEGMENTATION_NODE_HPP_

#include <rclcpp/node.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/subscription.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <apollo_lidar_segmentation/apollo_lidar_segmentation.hpp>
#include <apollo_lidar_segmentation_nodes/visibility_control.hpp>

#include <memory>

namespace autoware
{
namespace perception
{
namespace segmentation
{
namespace apollo_lidar_segmentation_nodes
{
/// \brief Object detection node based on neural network inference.
class APOLLO_LIDAR_SEGMENTATION_NODES_PUBLIC ApolloLidarSegmentationNode : public rclcpp::Node
{
private:
  const rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr m_cloud_sub_ptr;
  const rclcpp::Publisher<autoware_auto_msgs::msg::BoundingBoxArray>::SharedPtr m_box_pub_ptr;
  const rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr m_marker_pub_ptr;
  const std::shared_ptr<apollo_lidar_segmentation::ApolloLidarSegmentation> m_detector_ptr;
  /// \brief Main callback function.
  void APOLLO_LIDAR_SEGMENTATION_NODES_LOCAL pointCloudCallback(
    const sensor_msgs::msg::PointCloud2::SharedPtr & msg);

public:
  /// \brief Constructor
  /// \param options Additional options to control creation of the node.
  explicit ApolloLidarSegmentationNode(const rclcpp::NodeOptions & options);
};
}  // namespace apollo_lidar_segmentation_nodes
}  // namespace segmentation
}  // namespace perception
}  // namespace autoware
#endif  // APOLLO_LIDAR_SEGMENTATION_NODES__APOLLO_LIDAR_SEGMENTATION_NODE_HPP_
