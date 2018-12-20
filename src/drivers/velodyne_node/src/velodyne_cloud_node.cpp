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
  init_pcl_msg(output, m_frame_id.c_str(), m_cloud_size);
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
      const velodyne_driver::PointXYZIF & pt = m_point_block[idx];
      (void)add_point_to_cloud(output, pt);
      // Here I am ignoring the return value, because this operation should never fail.
      // In the constructor I ensure that cloud_size > PointBlock::CAPACITY. This means
      // I am guaranteed to fit at least one whole PointBlock into my PointCloud2.
      // Because just above this for loop, I reset the capacity of the pcl message,
      // I am guaranteed to have capacity for the remainder of a point block.
    }
  }
  m_translator.convert(pkt, m_point_block);
  for (uint32_t idx = 0U; idx < m_point_block.size(); ++idx) {
    const velodyne_driver::PointXYZIF & pt = m_point_block[idx];
    if (static_cast<uint16_t>(velodyne_driver::PointXYZIF::END_OF_SCAN_ID) != pt.id) {
      if (!add_point_to_cloud(output, pt)) {
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

////////////////////////////////////////////////////////////////////////////////
void VelodyneCloudNode::init_pcl_msg(
  sensor_msgs::msg::PointCloud2 & msg,
  const std::string & frame_id,
  const std::size_t size)
{
  msg.height = 1U;
  msg.is_bigendian = false;
  msg.is_dense = false;
  msg.header.frame_id = frame_id;
  // set the fields
  sensor_msgs::PointCloud2Modifier modifier(msg);
  modifier.setPointCloud2Fields(4U, "x", 1U, sensor_msgs::msg::PointField::FLOAT32,
    "y", 1U, sensor_msgs::msg::PointField::FLOAT32,
    "z", 1U, sensor_msgs::msg::PointField::FLOAT32,
    "intensity", 1U, sensor_msgs::msg::PointField::FLOAT32);
  // allocate memory
  modifier.resize(size);
}

////////////////////////////////////////////////////////////////////////////////

bool VelodyneCloudNode::add_point_to_cloud(
  sensor_msgs::msg::PointCloud2 & cloud,
  const velodyne_driver::PointXYZIF & pt)
{
  bool ret = false;

  sensor_msgs::PointCloud2Iterator<float> x_it(cloud, "x");
  sensor_msgs::PointCloud2Iterator<float> y_it(cloud, "y");
  sensor_msgs::PointCloud2Iterator<float> z_it(cloud, "z");
  sensor_msgs::PointCloud2Iterator<float> intensity_it(cloud, "intensity");

  x_it += m_point_cloud_idx;
  y_it += m_point_cloud_idx;
  z_it += m_point_cloud_idx;
  intensity_it += m_point_cloud_idx;

  // Actual size is 20 due to padding. This check is to make sure that when we do a insert of
  // 16 bytes, we will not stride past the bounds of the structure.
  static_assert(
    sizeof(velodyne_driver::PointXYZIF) >= ((4U * sizeof(float)) + sizeof(uint16_t)),
    "PointXYZIF is not expected size: ");

  if (x_it != x_it.end() &&
    y_it != y_it.end() &&
    z_it != z_it.end() &&
    intensity_it != intensity_it.end())
  {
    // add the point data
    *x_it = pt.x;
    *y_it = pt.y;
    *z_it = pt.z;
    *intensity_it = pt.intensity;

    // increment the index to keep track of the pointcloud's size
    m_point_cloud_idx++;
    ret = true;
  }
  return ret;
}
}  // namespace velodyne_node
}  // namespace drivers
}  // namespace autoware
