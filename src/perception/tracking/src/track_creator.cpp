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

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <time_utils/time_utils.hpp>
#include <tracking/track_creator.hpp>

#include <functional>
#include <memory>
#include <set>
#include <vector>

namespace autoware
{
namespace perception
{
namespace tracking
{
CreationPolicyBase::CreationPolicyBase(
  const float64_t default_variance,
  const float64_t noise_variance,
  const tf2::BufferCore & tf_buffer)
: m_default_variance{default_variance}, m_noise_variance{noise_variance}, m_tf_buffer{tf_buffer} {}

autoware_auto_msgs::msg::DetectedObjects CreationPolicyBase::populate_unassigned_lidar_detections(
  const autoware_auto_msgs::msg::DetectedObjects & clusters,
  const AssociatorResult & associator_result)
{
  autoware_auto_msgs::msg::DetectedObjects retval;
  retval.header = clusters.header;
  for (const auto & unassigned_idx : associator_result.unassigned_detection_indices) {
    retval.objects.push_back(clusters.objects[unassigned_idx]);
  }
  return retval;
}

LidarOnlyPolicy::LidarOnlyPolicy(
  const float64_t default_variance, const float64_t noise_variance,
  const tf2::BufferCore & tf_buffer)
: CreationPolicyBase(default_variance, noise_variance, tf_buffer) {}

void LidarOnlyPolicy::add_objects(
  const autoware_auto_msgs::msg::DetectedObjects & clusters,
  const AssociatorResult & associator_result)
{
  m_lidar_clusters = populate_unassigned_lidar_detections(clusters, associator_result);
}

TrackCreationResult LidarOnlyPolicy::create()
{
  TrackCreationResult retval;
  for (const auto & cluster : m_lidar_clusters.objects) {
    retval.tracks.emplace_back(
      TrackedObject(cluster, m_default_variance, m_noise_variance));
  }
  return retval;
}

constexpr uint32_t LidarClusterIfVisionPolicy::kVisionCacheSize;

LidarClusterIfVisionPolicy::LidarClusterIfVisionPolicy(
  const VisionPolicyConfig & cfg, const float64_t default_variance,
  const float64_t noise_variance, const tf2::BufferCore & tf_buffer)
: CreationPolicyBase{default_variance, noise_variance, tf_buffer},
  m_cfg{cfg},
  m_associator{cfg.associator_cfg, tf_buffer}
{
}

void LidarClusterIfVisionPolicy::add_objects(
  const autoware_auto_msgs::msg::DetectedObjects & clusters,
  const AssociatorResult & associator_result)
{
  m_lidar_clusters = populate_unassigned_lidar_detections(clusters, associator_result);
}

void LidarClusterIfVisionPolicy::add_objects(
  const autoware_auto_msgs::msg::ClassifiedRoiArray & vision_rois,
  const AssociatorResult & associator_result)
{
  autoware_auto_msgs::msg::ClassifiedRoiArray::SharedPtr vision_rois_msg =
    std::make_shared<autoware_auto_msgs::msg::ClassifiedRoiArray>();
  vision_rois_msg->header = vision_rois.header;
  for (const auto & unassigned_idx : associator_result.unassigned_detection_indices) {
    vision_rois_msg->rois.push_back(vision_rois.rois[unassigned_idx]);
  }

  const auto & frame_id = vision_rois_msg->header.frame_id;
  auto matching_vision_cache = m_vision_cache_map.find(frame_id);
  if (matching_vision_cache == m_vision_cache_map.end()) {
    const auto emplace_status = m_vision_cache_map.emplace(frame_id, kVisionCacheSize);
    matching_vision_cache = emplace_status.first;
  }
  matching_vision_cache->second.add(vision_rois_msg);
}

void LidarClusterIfVisionPolicy::create_using_cache(
  const VisionCache & vision_cache,
  TrackCreationResult & creator_ret)
{
  // For foxy time has to be initialized explicitly with sec, nanosec constructor to use the
  // correct clock source when querying message_filters::cache.
  // Refer: https://github.com/ros2/message_filters/issues/32
  const rclcpp::Time t{m_lidar_clusters.header.stamp.sec,
    m_lidar_clusters.header.stamp.nanosec};
  const auto before = t - m_cfg.max_vision_lidar_timestamp_diff;
  const auto after = t + m_cfg.max_vision_lidar_timestamp_diff;
  const auto vision_msg_matches = vision_cache.getInterval(before, after);

  if (vision_msg_matches.empty()) {
    std::cerr << "No matching vision msgs for creating tracks" << std::endl;
    return;
  }

  const auto & vision_msg = *vision_msg_matches.back();
  const auto association_result = m_associator.assign(vision_msg, m_lidar_clusters);

  if (!creator_ret.maybe_roi_stamps) {
    creator_ret.maybe_roi_stamps = std::vector<builtin_interfaces::msg::Time>{};
  }
  creator_ret.maybe_roi_stamps->push_back(vision_msg.header.stamp);

  std::set<size_t, std::greater<>> lidar_idx_to_erase;

  for (size_t cluster_idx = 0U; cluster_idx < m_lidar_clusters.objects.size();
    cluster_idx++)
  {
    if (association_result.track_assignments[cluster_idx] != AssociatorResult::UNASSIGNED) {
      lidar_idx_to_erase.insert(cluster_idx);
      // TrackedObject constructor uses the classification field in the DetectedObject to
      // initialize track class. So assign the class from the associated ROI to the cluster.
      m_lidar_clusters.objects[cluster_idx].classification = vision_msg.rois[association_result
          .track_assignments[cluster_idx]].classifications;
      creator_ret.tracks.emplace_back(
        TrackedObject(
          m_lidar_clusters.objects[cluster_idx],
          m_default_variance, m_noise_variance));
    }
  }

  // Erase lidar clusters that are associated to a vision roi
  for (const auto idx : lidar_idx_to_erase) {
    m_lidar_clusters.objects.erase(m_lidar_clusters.objects.begin() + static_cast<int32_t>(idx));
  }
}

TrackCreationResult LidarClusterIfVisionPolicy::create()
{
  TrackCreationResult retval;
  for (const auto & frame_cache : m_vision_cache_map) {
    create_using_cache(frame_cache.second, retval);
  }
  retval.detections_leftover = m_lidar_clusters;
  return retval;
}

TrackCreator::TrackCreator(const TrackCreatorConfig & config, const tf2::BufferCore & tf_buffer)
{
  switch (config.policy) {
    case TrackCreationPolicy::LidarClusterOnly:
      m_policy_object = std::make_unique<LidarOnlyPolicy>(
        config.default_variance,
        config.noise_variance, tf_buffer);
      break;
    case TrackCreationPolicy::LidarClusterIfVision:
      if (config.vision_policy_config == std::experimental::nullopt) {
        throw std::runtime_error(
                "Vision policy config needs to be initialized to use "
                "LidarClusterIfVision track creation policy");
      }
      m_policy_object = std::make_unique<LidarClusterIfVisionPolicy>(
        config.vision_policy_config.value(),
        config.default_variance,
        config.noise_variance, tf_buffer);
      break;
    default:
      throw std::runtime_error("Track creation policy does not exist / not implemented");
  }
}

}  // namespace tracking
}  // namespace perception
}  // namespace autoware
