// Copyright 2019-2021 the Autoware Foundation
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

#include <common/types.hpp>
#include <point_cloud_fusion_nodes/point_cloud_fusion_node.hpp>
#include <point_cloud_msg_wrapper/point_cloud_msg_wrapper.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>

#include <algorithm>
#include <memory>
#include <string>
#include <vector>

using autoware::common::types::bool8_t;
using autoware::common::types::float32_t;

namespace autoware
{
namespace perception
{
namespace filters
{
namespace point_cloud_fusion_nodes
{

PointCloudFusionNode::PointCloudFusionNode(
  const rclcpp::NodeOptions & node_options)
: Node("point_cloud_fusion_nodes", node_options),
  m_cloud_publisher(create_publisher<PointCloudMsgT>("output_topic", rclcpp::QoS(10))),
  m_input_topics(static_cast<std::size_t>(declare_parameter("number_of_sources").get<int>())),
  m_output_frame_id(declare_parameter("output_frame_id").get<std::string>()),
  m_cloud_capacity(static_cast<uint32_t>(declare_parameter("cloud_size").get<int>()))
{
  for (size_t i = 0; i < m_input_topics.size(); ++i) {
    m_input_topics[i] = "input_topic" + std::to_string(i + 1);
  }
  init();
}

void PointCloudFusionNode::init()
{
  m_core = std::make_unique<point_cloud_fusion::PointCloudFusion>(
    m_cloud_capacity,
    m_input_topics.size());

  using autoware::common::types::PointXYZI;
  point_cloud_msg_wrapper::PointCloud2Modifier<PointXYZI>{
    m_cloud_concatenated, m_output_frame_id}.reserve(m_cloud_capacity);

  if (m_input_topics.size() > 8 || m_input_topics.size() < 2) {
    throw std::domain_error(
            "Number of sources for point cloud fusion must be between 2 and 8."
            " Found: " + std::to_string(m_input_topics.size()));
  }

  for (size_t i = 0; i < 8; ++i) {
    if (i < m_input_topics.size()) {
      m_cloud_subscribers[i] = std::make_unique<message_filters::Subscriber<PointCloudMsgT>>(
        this, m_input_topics[i]);
    } else {
      m_cloud_subscribers[i] = std::make_unique<message_filters::Subscriber<PointCloudMsgT>>(
        this, m_input_topics[0]);
    }
  }
  m_cloud_synchronizer = std::make_unique<message_filters::Synchronizer<SyncPolicyT>>(
    SyncPolicyT(10), *m_cloud_subscribers[0], *m_cloud_subscribers[1], *m_cloud_subscribers[2],
    *m_cloud_subscribers[3], *m_cloud_subscribers[4], *m_cloud_subscribers[5],
    *m_cloud_subscribers[6], *m_cloud_subscribers[7]);

  m_cloud_synchronizer->registerCallback(
    std::bind(
      &PointCloudFusionNode::pointcloud_callback, this, std::placeholders::_1,
      std::placeholders::_2, std::placeholders::_3, std::placeholders::_4, std::placeholders::_5,
      std::placeholders::_6, std::placeholders::_7, std::placeholders::_8));
}

std::chrono::nanoseconds PointCloudFusionNode::convert_msg_time(builtin_interfaces::msg::Time stamp)
{
  return std::chrono::seconds(stamp.sec) + std::chrono::nanoseconds(stamp.nanosec);
}

void
PointCloudFusionNode::pointcloud_callback(
  const PointCloudMsgT::ConstSharedPtr & msg1, const PointCloudMsgT::ConstSharedPtr & msg2,
  const PointCloudMsgT::ConstSharedPtr & msg3, const PointCloudMsgT::ConstSharedPtr & msg4,
  const PointCloudMsgT::ConstSharedPtr & msg5, const PointCloudMsgT::ConstSharedPtr & msg6,
  const PointCloudMsgT::ConstSharedPtr & msg7, const PointCloudMsgT::ConstSharedPtr & msg8)
{
  std::array<PointCloudMsgT::ConstSharedPtr, 8> msgs{msg1, msg2, msg3, msg4, msg5, msg6, msg7,
    msg8};

  // reset pointcloud before using
  using autoware::common::types::PointXYZI;
  point_cloud_msg_wrapper::PointCloud2Modifier<PointXYZI> modifier{m_cloud_concatenated};
  modifier.clear();
  modifier.reserve(m_cloud_capacity);

  auto latest_stamp = msgs[0]->header.stamp;
  auto total_size = 0U;

  // Get the latest time stamp of the point clouds and find the total size after concatenation
  for (uint32_t msg_idx = 0; msg_idx < m_input_topics.size(); ++msg_idx) {
    const auto & stamp = msgs[msg_idx]->header.stamp;
    if (convert_msg_time(stamp) > convert_msg_time(latest_stamp)) {
      latest_stamp = stamp;
    }
    total_size += msgs[msg_idx]->width;
  }

  if (total_size > m_cloud_capacity) {
    RCLCPP_WARN(
      get_logger(), "pointclouds that are trying to be fused exceed the cloud capacity. "
      "The exceeded clouds will be ignored.");
  }

  // Go through all the messages and fuse them.
  uint32_t fused_cloud_size = 0;
  try {
    fused_cloud_size = m_core->fuse_pc_msgs(msgs, m_cloud_concatenated);
  } catch (point_cloud_fusion::PointCloudFusion::Error fuse_error) {
    if (fuse_error == point_cloud_fusion::PointCloudFusion::Error::TOO_LARGE) {
      RCLCPP_WARN(get_logger(), "Pointcloud is too large to be fused and will be ignored.");
    } else if (fuse_error == point_cloud_fusion::PointCloudFusion::Error::INSERT_FAILED) {
      RCLCPP_ERROR(get_logger(), "Points could not be added correctly to the fused cloud.");
    } else {
      RCLCPP_ERROR(get_logger(), "Unknown error.");
    }
  }

  if (fused_cloud_size > 0) {
    // Resize and publish.
    modifier.resize(fused_cloud_size);

    m_cloud_concatenated.header.stamp = latest_stamp;
    m_cloud_publisher->publish(m_cloud_concatenated);
  }
}
}  // namespace point_cloud_fusion_nodes
}  // namespace filters
}  // namespace perception
}  // namespace autoware

RCLCPP_COMPONENTS_REGISTER_NODE(
  autoware::perception::filters::point_cloud_fusion_nodes::PointCloudFusionNode)
