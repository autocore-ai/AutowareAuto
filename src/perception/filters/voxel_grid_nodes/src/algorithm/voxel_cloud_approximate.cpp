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

#include <cstring>

#include "lidar_utils/point_cloud_utils.hpp"
#include "voxel_grid_nodes/algorithm/voxel_cloud_approximate.hpp"

using autoware::common::lidar_utils::add_point_to_cloud;

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
VoxelCloudApproximate::VoxelCloudApproximate(const voxel_grid::Config & cfg)
: VoxelCloudBase(),
  m_cloud(),
  m_grid(cfg)
{
  // frame id is arbitrary, not the responsibility of this component
  autoware::common::lidar_utils::init_pcl_msg(m_cloud, "base_link", cfg.get_capacity());
}

////////////////////////////////////////////////////////////////////////////////
void VoxelCloudApproximate::insert(
  const sensor_msgs::msg::PointCloud2 & msg, std::size_t point_step)
{
  m_cloud.header = msg.header;
  for (std::size_t idx = 0U; idx < msg.data.size(); idx += point_step) {
    PointXYZIF pt;
    //lint -e{925, 9110} Need to convert pointers and use bit for external API NOLINT
    (void)memmove(
      static_cast<void *>(&pt.x),
      static_cast<const void *>(&msg.data[idx]),
      point_step);
    m_grid.insert(pt);
  }
  // TODO(c.ho) overlay?
}

////////////////////////////////////////////////////////////////////////////////
const sensor_msgs::msg::PointCloud2 & VoxelCloudApproximate::get()
{
  // resetting the index for the pointcloud iterators
  autoware::common::lidar_utils::reset_pcl_msg(m_cloud, m_grid.capacity(), m_point_cloud_idx);

  for (const auto & it : m_grid) {
    const auto & pt = it.second.get();
    (void)add_point_to_cloud(m_cloud, pt, m_point_cloud_idx);
    // Don't need to check if cloud can't fit since it has the same capacity as the grid
    // insert will throw if the grid is at capacity
  }
  m_grid.clear();
  autoware::common::lidar_utils::resize_pcl_msg(m_cloud, m_point_cloud_idx);

  return m_cloud;
}
}  // namespace algorithm
}  // namespace voxel_grid_nodes
}  // namespace filters
}  // namespace perception
}  // namespace autoware
