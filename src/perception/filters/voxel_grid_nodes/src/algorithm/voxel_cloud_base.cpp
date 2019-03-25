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

#include <voxel_grid_nodes/algorithm/voxel_cloud_base.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <string>

namespace autoware
{
namespace perception
{
namespace filters
{
namespace voxel_grid_nodes
{
namespace algorithm
{
////////////////////////////////////////////////////////////////////////////////

VoxelCloudBase::~VoxelCloudBase()
{
  // No base members
}

void VoxelCloudBase::init_pcl_msg(
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

bool VoxelCloudBase::add_point_to_cloud(
  sensor_msgs::msg::PointCloud2 & cloud,
  const PointXYZIF & pt)
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
    sizeof(PointXYZIF) >= ((4U * sizeof(float)) + sizeof(uint16_t)),
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
void VoxelCloudBase::reset_cloud_idx()
{
  m_point_cloud_idx = 0;
}

}  // namespace algorithm
}  // namespace voxel_grid_nodes
}  // namespace filters
}  // namespace perception
}  // namespace autoware
