// Copyright 2020 the Autoware Foundation
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

#include <point_cloud_mapping/map.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <lidar_utils/point_cloud_utils.hpp>
#include <pcl/io/pcd_io.h>
#include <algorithm>
#include <string>

namespace autoware
{
namespace mapping
{
namespace point_cloud_mapping
{
PlainPointCloudMap::PlainPointCloudMap(std::size_t capacity, const std::string & frame)
: m_capacity(capacity),
  m_frame_id{frame}
{
  m_cloud.reserve(capacity);
}

MapUpdateSummary PlainPointCloudMap::try_add_observation(
  const PlainPointCloudMap::Cloud & observation,
  const geometry_msgs::msg::PoseWithCovarianceStamped &)
{
  // pose parameter is not used as the input cloud is already expected to be on the map frame.
  if (observation.header.frame_id != m_frame_id) {
    throw std::runtime_error("pointcloud map and the updates should be on the same frame");
  }

  const auto check_not_end = [](const auto & its) {
      return std::all_of(its.cbegin(), its.cend(), [](const auto & it) {return it != it.end();});
    };

  MapUpdateSummary ret{MapUpdateType::NO_CHANGE, 0U};

  std::array<CloudConstIt, NUM_FIELDS> obs_its = {
    CloudConstIt(observation, "x"),
    CloudConstIt(observation, "y"),
    CloudConstIt(observation, "z"),
    CloudConstIt(observation, "intensity")
  };

  ret.update_type = m_cloud.empty() ? MapUpdateType::NEW : MapUpdateType::UPDATE;

  auto obs_idx = 0U;
  for (; check_not_end(obs_its); ++obs_idx) {
    if (m_cloud.size() >= m_capacity) {
      if (obs_idx == 0U) {
        ret.update_type = MapUpdateType::NO_CHANGE;
      } else {
        ret.update_type = MapUpdateType::PARTIAL_UPDATE;
      }
      break;
    }

    // Transfer the point from the observation to the map and advance the iterators.
    pcl::PointXYZI pt;
    pt.x = *obs_its[0];
    pt.y = *obs_its[1];
    pt.z = *obs_its[2];
    pt.intensity = *obs_its[3];
    m_cloud.push_back(pt);
    // Advance the iterators.
    std::for_each(obs_its.begin(), obs_its.end(), [](auto & it) {++it;});
  }

  if (m_cloud.size() > m_capacity) {
    throw std::logic_error("PlainPointCloudMap: Number of points in the map exceed"
            "the capacity. This shouldn't happen, something must be wrong.");
  }

  ret.num_added_pts = obs_idx;
  return ret;
}

void PlainPointCloudMap::write(const std::string & file_name_prefix) const
{
  pcl::io::savePCDFile(file_name_prefix + ".pcd", m_cloud);
}


std::size_t PlainPointCloudMap::size() const noexcept
{
  return m_cloud.size();
}

std::size_t PlainPointCloudMap::capacity() const noexcept
{
  return m_capacity;
}

void PlainPointCloudMap::clear()
{
  // reset indices and pc iterators.
  m_cloud.clear();
}

/////////////////////////////////////////

VoxelMap::VoxelMap(
  const perception::filters::voxel_grid::Config & grid_config,
  const std::string & frame)
: m_grid_config{grid_config}, m_frame_id{frame} {}

MapUpdateSummary VoxelMap::try_add_observation(
  const Cloud & observation,
  const geometry_msgs::msg::PoseWithCovarianceStamped &)
{
  if (observation.header.frame_id != m_frame_id) {
    throw std::runtime_error("pointcloud map and the updates should be on the same frame");
  }

  const auto check_not_end = [](const auto & its) {
      return std::all_of(its.cbegin(), its.cend(), [](const auto & it) {return it != it.end();});
    };

  MapUpdateSummary ret{MapUpdateType::NO_CHANGE, 0U};

  std::array<CloudConstIt, NUM_FIELDS> obs_its = {
    CloudConstIt(observation, "x"),
    CloudConstIt(observation, "y"),
    CloudConstIt(observation, "z"),
    CloudConstIt(observation, "intensity")
  };

  ret.update_type = m_grid.empty() ? MapUpdateType::NEW : MapUpdateType::UPDATE;
  auto obs_idx = 0U;
  for (; check_not_end(obs_its); ++obs_idx) {
    common::types::PointXYZIF pt{*obs_its[0], *obs_its[1], *obs_its[2], *obs_its[3]};
    const auto pt_key = m_grid_config.index(pt);
    if (m_grid.size() >= capacity() &&
      m_grid.find(pt_key) == m_grid.end())
    {
      if (obs_idx == 0U) {
        ret.update_type = MapUpdateType::NO_CHANGE;
      } else {
        ret.update_type = MapUpdateType::PARTIAL_UPDATE;
      }
      break;
    }

    m_grid[pt_key].add_observation(pt);

    // Advance the iterators.
    std::for_each(obs_its.begin(), obs_its.end(), [](auto & it) {++it;});
  }
  ret.num_added_pts = obs_idx;
  return ret;
}

void VoxelMap::write(const std::string & file_name_prefix) const
{
  // TODO(yunus.caliskan) Remove dynamic allocations.
  // pcl cloud is constructed here and the map is copied to it for sake
  // of simplicity to be able to use the pcl library's pcd writer.
  pcl::PointCloud<pcl::PointXYZI> cloud;
  cloud.reserve(m_grid.size());
  for (const auto & vx : m_grid) {
    pcl::PointXYZI pt;
    const auto & vx_pt = vx.second.get();
    pt.x = vx_pt.x;
    pt.y = vx_pt.y;
    pt.z = vx_pt.z;
    pt.intensity = vx_pt.intensity;
    cloud.push_back(pt);
  }
  pcl::io::savePCDFile(file_name_prefix + ".pcd", cloud);
}

void VoxelMap::clear()
{
  m_grid.clear();
}
std::size_t VoxelMap::size() const noexcept
{
  return m_grid.size();
}

std::size_t VoxelMap::capacity() const noexcept
{
  return m_grid_config.get_capacity();
}
}  // namespace point_cloud_mapping
}  // namespace mapping
}  // namespace autoware
