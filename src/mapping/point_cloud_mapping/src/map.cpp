// Copyright 2020 Apex.AI, Inc.
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

#include <point_cloud_mapping/map.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <lidar_utils/point_cloud_utils.hpp>
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
  m_frame_id{frame},
  m_pc_modifier{m_cloud}
{
  common::lidar_utils::init_pcl_msg(m_cloud, frame, capacity);
  // iterators must be created after pc is initialized
  m_cloud_its.insert(m_cloud_its.end(), {
          CloudIt(m_cloud, "x"),
          CloudIt(m_cloud, "y"),
          CloudIt(m_cloud, "z"),
          CloudIt(m_cloud, "intensity")
        });
}

MapUpdateSummary PlainPointCloudMap::try_add_observation(
  const PlainPointCloudMap::Cloud & observation,
  const geometry_msgs::msg::PoseWithCovarianceStamped &)
{
  if (observation.header.frame_id != m_frame_id) {
    throw std::runtime_error("pointcloud map and the updates should be on the same frame");
  }

  const auto check_not_end = [](const auto & its) {
      return std::all_of(its.cbegin(), its.cend(), [](const auto & it) {return it != it.end();});
    };

  if (m_pc_modifier.size() < m_capacity) {
    m_pc_modifier.resize(m_capacity);
  }

  MapUpdateSummary ret{MapUpdateType::NO_CHANGE, 0U};

  std::array<CloudConstIt, NUM_FIELDS> obs_its = {
    CloudConstIt(observation, "x"),
    CloudConstIt(observation, "y"),
    CloudConstIt(observation, "z"),
    CloudConstIt(observation, "intensity")
  };

  ret.update_type = (m_map_size > 0U) ? MapUpdateType::UPDATE : MapUpdateType::NEW;

  // TODO(yunus.caliskan): appending clouds should probably be a common function (#102)
  auto obs_idx = 0U;
  for (; check_not_end(obs_its); ++obs_idx) {
    if (!check_not_end(m_cloud_its)) {
      if (obs_idx == 0U) {
        ret.update_type = MapUpdateType::NO_CHANGE;
      } else {
        ret.update_type = MapUpdateType::PARTIAL_UPDATE;
      }
      break;
    }

    // Transfer the point from the observation to the map and advance the iterators.
    for (auto idx = 0U; idx < NUM_FIELDS; ++idx) {
      *m_cloud_its[idx] = *obs_its[idx];
      ++m_cloud_its[idx];
      ++obs_its[idx];
    }
    ++m_map_size;
  }


  if (m_map_size > m_capacity) {
    throw std::logic_error("PlainPointCloudMap: Number of points in the map exceed"
            "the capacity. This shouldn't happen, something must be wrong.");
  }

  ret.num_added_pts = obs_idx;
  return ret;
}

const PlainPointCloudMap::Cloud & PlainPointCloudMap::get()
{
  m_pc_modifier.resize(m_map_size);
  return m_cloud;
}

bool PlainPointCloudMap::full()
{
  return m_map_size == m_capacity;
}

void PlainPointCloudMap::clear()
{
  // reset indices and pc iterators.
  m_map_size = 0U;
  m_cloud_its = {
    CloudIt(m_cloud, "x"),
    CloudIt(m_cloud, "y"),
    CloudIt(m_cloud, "z"),
    CloudIt(m_cloud, "intensity")
  };
  return m_pc_modifier.clear();
}


}  // namespace point_cloud_mapping
}  // namespace mapping
}  // namespace autoware
