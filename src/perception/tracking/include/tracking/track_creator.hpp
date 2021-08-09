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

#ifndef TRACKING__TRACK_CREATOR_HPP_
#define TRACKING__TRACK_CREATOR_HPP_

#include <autoware_auto_msgs/msg/detected_objects.hpp>
#include <common/types.hpp>
#include <tracking/tracked_object.hpp>
#include <tracking/tracker_types.hpp>
#include <tracking/visibility_control.hpp>

#include <vector>

namespace autoware
{
namespace perception
{
namespace tracking
{

/// \brief Struct to hold all the configuration parameters for the track creation module
struct TRACKING_PUBLIC TrackCreatorConfig
{
  using float32_t = common::types::float32_t;
  /// Policy to be used to create new tracks
  TrackCreationPolicy policy = TrackCreationPolicy::LidarClusterOnly;
  /// When initializing a new track, this value is used for the variance when none is provided by
  /// the detection.
  float32_t default_variance = -1.0F;  // Invalid, to make sure it is set.
  /// The magnitude of the noise in the Kalman filter.
  float32_t noise_variance = -1.0F;  // Invalid, to make sure it is set.
};

/// \brief Class to create new tracks based on a predefined policy and unassociated detections
class TRACKING_PUBLIC TrackCreator
{
public:
  using float32_t = common::types::float32_t;
  /// \brief Constructor
  /// \param config parameters for track creation
  explicit TrackCreator(const TrackCreatorConfig & config);
  /// \brief Keep tracks of all unassigned lidar clusters. Clears the previously passed clusters
  /// \param clusters DetectedObjects msg output by the lidar clustering algorithm
  /// \param associator_result Struct containing indices of clusters that do not have track
  ///                          association
  void add_unassigned_lidar_clusters(
    const autoware_auto_msgs::msg::DetectedObjects & clusters,
    const AssociatorResult & associator_result);
  /// \brief Create new tracks based on the policy and unassociated detections. Call the
  ///        appropriate add_unassigned_* functions before calling this.
  /// \return vector of newly created TrackedObject objects
  std::vector<TrackedObject> create_tracks();

private:
  std::vector<TrackedObject> create_tracks_from_all_clusters();

  TrackCreatorConfig m_config;
  autoware_auto_msgs::msg::DetectedObjects m_lidar_clusters;
};

}  // namespace tracking
}  // namespace perception
}  // namespace autoware

#endif   // TRACKING__TRACK_CREATOR_HPP_
