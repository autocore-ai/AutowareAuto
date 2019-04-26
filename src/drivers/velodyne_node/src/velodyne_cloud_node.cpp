// Copyright 2018 Apex.AI, Inc.
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

#include <string>
#include <chrono>
#include <vector>

#include "lidar_utils/lidar_types.hpp"
#include "lidar_utils/point_cloud_utils.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include "velodyne_node/velodyne_cloud_node.hpp"

namespace autoware
{
namespace drivers
{
namespace velodyne_node
{
VelodyneCloudNode::VelodyneCloudNode(
  const std::string & node_name,
  const std::string & topic,
  const std::string & ip,
  const uint16_t port,
  const std::string & frame_id,
  const std::size_t cloud_size,
  const velodyne_driver::Vlp16Translator::Config & config)
: UdpDriverNode<velodyne_driver::Vlp16Translator::Packet, sensor_msgs::msg::PointCloud2>(
    node_name,
    topic,
    ip,
    port),
  m_translator(config),
  m_published_cloud(false),
  m_remainder_start_idx(0U),
  m_point_cloud_idx(0),
  m_frame_id(frame_id),
  m_cloud_size(cloud_size)
{
  m_point_block.reserve(autoware::drivers::velodyne_driver::Vlp16Translator::POINT_BLOCK_CAPACITY);
  // If your preallocated cloud size is too small, the node really won't operate well at all
  if (static_cast<uint32_t>(m_point_block.capacity()) >= cloud_size) {
    throw std::runtime_error("VelodyneCloudNode: cloud_size must be > PointBlock::CAPACITY");
  }
}

////////////////////////////////////////////////////////////////////////////////
VelodyneCloudNode::VelodyneCloudNode(
  const std::string & node_name,
  const std::string & node_namespace,
  const std::string & param_file)
: UdpDriverNode(node_name, node_namespace, param_file),
  m_translator(
      {
        static_cast<float>(get_parameter("rpm").as_int()),
        velodyne_driver::make_point(
          static_cast<float>(get_parameter("translation.dx_m").as_double()),
          static_cast<float>(get_parameter("translation.dy_m").as_double()),
          static_cast<float>(get_parameter("translation.dz_m").as_double())
        ),
        velodyne_driver::make_point(
          static_cast<float>(get_parameter("rotation.roll_rad").as_double()),
          static_cast<float>(get_parameter("rotation.pitch_rad").as_double()),
          static_cast<float>(get_parameter("rotation.yaw_rad").as_double())
        ),
        static_cast<float>(get_parameter("filter.min_radius_m").as_double()),
        static_cast<float>(get_parameter("filter.max_radius_m").as_double()),
        static_cast<float>(get_parameter("filter.min_angle_deg").as_double()),
        static_cast<float>(get_parameter("filter.max_angle_deg").as_double())
      }),
  m_published_cloud(false),
  m_remainder_start_idx(0U),
  m_point_cloud_idx(0),
  m_frame_id(get_parameter("frame_id").as_string().c_str()),
  m_cloud_size(static_cast<std::size_t>(get_parameter("cloud_size").as_int()))
{
  m_point_block.reserve(autoware::drivers::velodyne_driver::Vlp16Translator::POINT_BLOCK_CAPACITY);
}
////////////////////////////////////////////////////////////////////////////////
void VelodyneCloudNode::init_output(sensor_msgs::msg::PointCloud2 & output)
{
  autoware::common::lidar_utils::init_pcl_msg(output, m_frame_id.c_str(), m_cloud_size);
}

////////////////////////////////////////////////////////////////////////////////
bool VelodyneCloudNode::convert(
  const velodyne_driver::Vlp16Translator::Packet & pkt,
  sensor_msgs::msg::PointCloud2 & output)
{
  sensor_msgs::PointCloud2Modifier pc_modifier(output);
  // This handles the case when the below loop exited due to containing extra points
  if (m_published_cloud) {
    // reset the pointcloud
    pc_modifier.clear();
    m_point_cloud_idx = 0;
    // resize back to the capacity for building a new pointcloud
    pc_modifier.resize(m_cloud_size);

    // deserialize remainder into pointcloud
    m_published_cloud = false;
    for (uint32_t idx = m_remainder_start_idx; idx < m_point_block.size(); ++idx) {
      const autoware::common::lidar_utils::PointXYZIF & pt = m_point_block[idx];
      (void)add_point_to_cloud(output, pt, m_point_cloud_idx);
      // Here I am ignoring the return value, because this operation should never fail.
      // In the constructor I ensure that cloud_size > PointBlock::CAPACITY. This means
      // I am guaranteed to fit at least one whole PointBlock into my PointCloud2.
      // Because just above this for loop, I reset the capacity of the pcl message,
      // I am guaranteed to have capacity for the remainder of a point block.
    }
  }
  m_translator.convert(pkt, m_point_block);
  for (uint32_t idx = 0U; idx < m_point_block.size(); ++idx) {
    const autoware::common::lidar_utils::PointXYZIF & pt = m_point_block[idx];
    if (static_cast<uint16_t>(autoware::common::lidar_utils::PointXYZIF::END_OF_SCAN_ID) != pt.id) {
      if (!add_point_to_cloud(output, pt, m_point_cloud_idx)) {
        m_published_cloud = true;
        m_remainder_start_idx = idx;
      }
    } else {
      m_published_cloud = true;
      m_remainder_start_idx = idx;
      break;
    }
  }
  if (m_published_cloud) {
    // resize pointcloud down to its actual size
    pc_modifier.resize(static_cast<size_t>(m_point_cloud_idx));
    output.row_step = output.width * output.point_step;
    output.header.stamp = this->now();
  }

  return m_published_cloud;
}

////////////////////////////////////////////////////////////////////////////////
bool VelodyneCloudNode::get_output_remainder(sensor_msgs::msg::PointCloud2 & output)
{
  // The assumption checked in the constructor is that the PointCloud size is bigger than
  // the PointBlocks, which can fully contain a packet. The use case of this method is in case
  // PacketT > OutputT, which is not the case here.
  (void)output;
  return false;
}

}  // namespace velodyne_node
}  // namespace drivers
}  // namespace autoware
