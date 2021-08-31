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

#ifndef TRACKING__TRACKER_TYPES_HPP_
#define TRACKING__TRACKER_TYPES_HPP_

#include <Eigen/Core>
#include <tracking/visibility_control.hpp>

#include <limits>
#include <unordered_set>
#include <vector>

namespace autoware
{
namespace perception
{
namespace tracking
{

/// \brief Maximum number of tracks possible in every timestep
constexpr uint16_t MAX_NUM_TRACKS = 256U;

/// \brief Number of dimensions needed to represent object position for tracking (x and y)
constexpr uint16_t NUM_OBJ_POSE_DIM = 2U;

/// \brief Struct to store results after the assignment is done
struct TRACKING_PUBLIC AssociatorResult
{
  static constexpr std::size_t UNASSIGNED = std::numeric_limits<std::size_t>::max();
  /// \brief This vector stores the detection index associated with each track idx.
  ///        So, it should have Associator::m_num_tracks elements with each element having a value
  ///        between 0 to Association::m_num_detections or AssociatorResult::UNASSIGNED.
  std::vector<std::size_t> track_assignments;
  /// \brief Indices of detections that are not associated to any tracks
  std::unordered_set<std::size_t> unassigned_detection_indices;
  /// \brief Indices of tracks that are not associated to any detections
  std::unordered_set<std::size_t> unassigned_track_indices;
  /// \brief Indicates if there were errors in the data during association
  bool had_errors;
};

enum class TrackCreationPolicy
{
  /// Create tracks from every unassociated lidar cluster
  LidarClusterOnly,
  /// Create tracks from unassociated lidar clusters only if they have associated vision detections
  LidarClusterIfVision
};

}  // namespace tracking
}  // namespace perception
}  // namespace autoware

#endif  // TRACKING__TRACKER_TYPES_HPP_
