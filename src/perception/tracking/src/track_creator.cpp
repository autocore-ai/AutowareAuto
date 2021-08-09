// Copyright 2021 Apex.AI, Inc.
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
//
// Co-developed by Tier IV, Inc. and Apex.AI, Inc.

#include <tracking/track_creator.hpp>

#include <vector>

namespace autoware
{
namespace perception
{
namespace tracking
{

TrackCreator::TrackCreator(const TrackCreatorConfig & config)
: m_config(config) {}

void TrackCreator::add_unassigned_lidar_clusters(
  const autoware_auto_msgs::msg::DetectedObjects & clusters,
  const AssociatorResult & associator_result)
{
  m_lidar_clusters.objects.clear();
  m_lidar_clusters.header = clusters.header;
  for (const auto & unassigned_idx : associator_result.unassigned_detection_indices) {
    m_lidar_clusters.objects.push_back(clusters.objects[unassigned_idx]);
  }
}

std::vector<TrackedObject> TrackCreator::create_tracks_from_all_clusters()
{
  std::vector<TrackedObject> retval;

  for (const auto & cluster : m_lidar_clusters.objects) {
    retval.emplace_back(
      TrackedObject(cluster, m_config.default_variance, m_config.noise_variance));
  }

  return retval;
}

std::vector<TrackedObject> TrackCreator::create_tracks()
{
  switch (m_config.policy) {
    case TrackCreationPolicy::LidarClusterOnly:
      return create_tracks_from_all_clusters();
    default:
      throw std::runtime_error("Policy not implemented/wrong");
  }
}

}  // namespace tracking
}  // namespace perception
}  // namespace autoware
