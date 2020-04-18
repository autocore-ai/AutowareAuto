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

#include <tf2_ros/buffer_interface.h>  // TODO(esteve): Workaround for https://github.com/ros2/geometry2/pull/126
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>
#include <point_cloud_fusion/point_cloud_fusion.hpp>
#include <point_cloud_fusion/point_cloud_transform.hpp>
#include <common/types.hpp>
#include <memory>
#include <vector>
#include <algorithm>
#include <string>

using autoware::common::types::bool8_t;
using autoware::common::types::float32_t;

namespace autoware
{
namespace perception
{
namespace filters
{
namespace point_cloud_fusion
{

PointCloudFusionNode::PointCloudFusionNode(
  const std::string & node_name,
  const std::string & node_namespace)
: Node(node_name, node_namespace),
  m_cloud_publisher(create_publisher<PointCloudMsgT>(declare_parameter("output_topic")
    .get<std::string>(), rclcpp::QoS(10))),
  m_tf_listener(m_tf_buffer, std::shared_ptr<rclcpp::Node>(this, [](auto) {}), false),
  m_input_topics(declare_parameter("input_topics").get<std::vector<std::string>>()),
  m_output_frame_id(declare_parameter("output_frame_id").get<std::string>()),
  m_cloud_capacity(declare_parameter("cloud_size").get<uint32_t>())
{
  init();
}

PointCloudFusionNode::PointCloudFusionNode(
  const std::string & node_name,
  const std::string & node_namespace,
  const std::string & output_topic,
  const std::vector<std::string> & input_topics,
  const std::string & output_frame_id,
  const uint32_t cloud_size)
: Node(node_name, node_namespace),
  m_cloud_publisher(create_publisher<PointCloudMsgT>(output_topic, rclcpp::QoS(10))),
  m_tf_listener(m_tf_buffer, std::shared_ptr<rclcpp::Node>(this, [](auto) {}), false),
  m_input_topics(input_topics),
  m_output_frame_id(output_frame_id),
  m_cloud_capacity(cloud_size)
{
  init();
}


void PointCloudFusionNode::init()
{
  common::lidar_utils::init_pcl_msg(m_cloud_concatenated, m_output_frame_id,
    m_cloud_capacity);

  if (m_input_topics.size() > 8 || m_input_topics.size() < 2) {
    throw std::domain_error("Number of sources for point cloud fusion must be between 2 and 8."
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
    std::bind(&PointCloudFusionNode::pointcloud_callback, this, std::placeholders::_1,
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

  uint32_t pc_concat_idx = 0;
  // reset pointcloud before using
  common::lidar_utils::reset_pcl_msg(m_cloud_concatenated, m_cloud_capacity,
    pc_concat_idx);

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
    RCLCPP_WARN(get_logger(), "pointclouds that are trying to be fused exceed the cloud capacity. "
      "The exceeded clouds will be ignored.");
  }

  // Go through all the messages and fuse them.
  auto fused_cloud_size = fuse_pc_msgs(msgs);

  if (fused_cloud_size > 0) {
    // Resize and publish.
    common::lidar_utils::resize_pcl_msg(m_cloud_concatenated, fused_cloud_size);

    m_cloud_concatenated.header.stamp = latest_stamp;
    m_cloud_publisher->publish(m_cloud_concatenated);
  }
}

uint32_t PointCloudFusionNode::fuse_pc_msgs(
  const std::array<PointCloudMsgT::ConstSharedPtr, 8> & msgs)
{
  uint32_t pc_concat_idx = 0;

  for (size_t i = 0; i < m_input_topics.size(); ++i) {
    try {
      bool8_t concat_success;
      auto tf_lookup_time = std::chrono::time_point<std::chrono::system_clock,
          std::chrono::nanoseconds>(convert_msg_time(msgs[i]->header.stamp));

      // Get the transform between the msg and the output frame.
      auto transform = m_tf_buffer.lookupTransform(m_output_frame_id, msgs[i]->header.frame_id,
          tf_lookup_time);

      if (is_identity(transform)) {
        // No need to transform points since they are in the same frame as the fused cloud
        concat_success = concatenate_pointcloud(*msgs[i], m_cloud_concatenated, pc_concat_idx);
      } else {
        // Transform the points, then add them to the fused cloud
        PointCloudT transformed_pc;
        tf2::doTransform(*msgs[i], transformed_pc, transform);
        concat_success = concatenate_pointcloud(transformed_pc, m_cloud_concatenated,
            pc_concat_idx);
      }
      if (!concat_success) {
        // Cloud could not be added to the m_cloud_concatenated since the size limit is reached.
        // No point in trying the remaining clouds.
        RCLCPP_WARN(get_logger(), "Reached the capacity of the fused cloud, ignoring the "
          "remaining cloud messages and publishing.");
        break;
      }
    } catch (tf2::TransformException & ex) {
      // A tf2 lookup error must've occured. We can still try the next pointcloud.
      RCLCPP_ERROR(this->get_logger(), ex.what());
    }
  }
  return pc_concat_idx;
}

bool8_t PointCloudFusionNode::concatenate_pointcloud(
  const sensor_msgs::msg::PointCloud2 & pc_in,
  sensor_msgs::msg::PointCloud2 & pc_out,
  uint32_t & concat_idx) const
{
  auto ret = true;
  if ((pc_in.width + concat_idx) > m_cloud_capacity) {
    RCLCPP_WARN(
      this->get_logger(),
      "Pointcloud is too large to be fused and will be ignored.");
    ret = false;
  } else {
    sensor_msgs::PointCloud2ConstIterator<float32_t> x_it_in(pc_in, "x");
    sensor_msgs::PointCloud2ConstIterator<float32_t> y_it_in(pc_in, "y");
    sensor_msgs::PointCloud2ConstIterator<float32_t> z_it_in(pc_in, "z");
    sensor_msgs::PointCloud2ConstIterator<float32_t> intensity_it_in(pc_in, "intensity");

    while (x_it_in != x_it_in.end() &&
      y_it_in != y_it_in.end() &&
      z_it_in != z_it_in.end() &&
      intensity_it_in != intensity_it_in.end())
    {
      common::types::PointXYZIF pt;
      pt.x = *x_it_in;
      pt.y = *y_it_in;
      pt.z = *z_it_in;
      pt.intensity = *intensity_it_in;

      if (common::lidar_utils::add_point_to_cloud(pc_out, pt, concat_idx)) {
        ++x_it_in;
        ++y_it_in;
        ++z_it_in;
        ++intensity_it_in;
      } else {
        // Somehow the point could be inserted to the concatenated cloud. Something regarding
        // the cloud sizes must be off.
        RCLCPP_ERROR(this->get_logger(), "Points could not be added correctly to the fused cloud");
        ret = false;
        break;
      }
    }
  }
  return ret;
}
}  // namespace point_cloud_fusion
}  // namespace filters
}  // namespace perception
}  // namespace autoware
