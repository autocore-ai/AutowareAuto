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
#ifndef TRACKING__DATA_ASSOCIATION_HPP_
#define TRACKING__DATA_ASSOCIATION_HPP_

#include <tracking/visibility_control.hpp>

#include <autoware_auto_msgs/msg/tracked_dynamic_object_array.hpp>
#include <autoware_auto_msgs/msg/detected_dynamic_object_array.hpp>
#include <hungarian_assigner/hungarian_assigner.hpp>
#include <tracking/tracker_types.hpp>

#include <experimental/optional>
#include <limits>
#include <map>
#include <vector>

namespace autoware
{
namespace perception
{
namespace tracking
{

using assigner_idx_t = autoware::fusion::hungarian_assigner::index_t;

/// \brief Class to create configuration parameters for data association
class TRACKING_PUBLIC DataAssociationConfig
{
public:
  /// \brief Constructor
  /// \param max_distance Max distance between a track and detection beyond which they wont be
  ///                     considered for association with each other
  /// \param max_area_ratio Max ratio between a track and detection area beyond which they wont be
  ///                       considered for association with each other
  DataAssociationConfig(const float max_distance, const float max_area_ratio);

  inline float get_max_distance_squared() const {return m_max_distance_squared;}

  inline float get_max_area_ratio() const {return m_max_area_ratio;}

  inline float get_max_area_ratio_inv() const {return m_max_area_ratio_inv;}

private:
  float m_max_distance;
  float m_max_distance_squared;
  float m_max_area_ratio;
  float m_max_area_ratio_inv;
};

/// \brief Struct to store results after the assignment is done
struct TRACKING_PUBLIC AssociatorResult
{
  static constexpr std::size_t UNASSIGNED = std::numeric_limits<std::size_t>::max();
  /// \brief This vector stores the detection index associated with each track idx.
  ///        So, it should have Associator::m_num_tracks elements with each element having a value
  ///        between 0 to Association::m_num_detections or AssociatorResult::UNASSIGNED.
  std::vector<std::size_t> track_assignments;
  /// \brief Indices of detections that are not associated to any tracks
  std::vector<std::size_t> unassigned_detection_indices;
  /// \brief Indices of tracks that are not associated to any detections
  std::vector<std::size_t> unassigned_track_indices;
};


/// \brief Class to perform data association between existing tracks and new detections using
///        mahalanobis distance and hungarian assigner
class TRACKING_PUBLIC Associator
{
public:
  /// \brief Constructor
  /// \param association_cfg Config object containing parameters to be used
  explicit Associator(const DataAssociationConfig & association_cfg);

  /// \brief Run assigner for the given list of detections and tracks
  /// \param detections List of detections
  /// \param tracks List of tracks
  /// \return Returns Associator result struct
  AssociatorResult assign(
    const autoware_auto_msgs::msg::DetectedDynamicObjectArray & detections, const
    autoware_auto_msgs::msg::TrackedDynamicObjectArray & tracks);

private:
  /// \brief Reset internal states of the associator
  void reset();

  /// \brief Loop through all detections and tracks and set weights between them in the assigner
  void compute_weights(
    const autoware_auto_msgs::msg::DetectedDynamicObjectArray & detections,
    const autoware_auto_msgs::msg::TrackedDynamicObjectArray & tracks);

  /// \brief Check if the given track and detection are similar enough to compute weight
  bool consider_associating(
    const autoware_auto_msgs::msg::DetectedDynamicObject & detection, const
    autoware_auto_msgs::msg::TrackedDynamicObject & track) const;

  /// Set weight in the assigner (Has to determine which idx is row and which is column)
  void set_weight(const float weight, const size_t det_idx, const size_t track_idx);

  /// \brief Extract result from the assigner and populate the AssociatorResult container
  AssociatorResult extract_result() const;

  DataAssociationConfig m_association_cfg;
  autoware::fusion::hungarian_assigner::hungarian_assigner_c<MAX_NUM_TRACKS> m_assigner;
  // Hungarian assigner expects a fat matrix (not tall). Bool tracks if tracks are rows or cols
  bool m_are_tracks_rows;
  size_t m_num_tracks;
  size_t m_num_detections;
};


}  // namespace tracking
}  // namespace perception
}  // namespace autoware

#endif  // TRACKING__DATA_ASSOCIATION_HPP_
