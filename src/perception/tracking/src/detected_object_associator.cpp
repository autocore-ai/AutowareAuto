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

#include <tracking/detected_object_associator.hpp>

#include <common/types.hpp>
#include <geometry/common_2d.hpp>
#include <helper_functions/mahalanobis_distance.hpp>

#include <algorithm>
#include <limits>
#include <vector>

namespace autoware
{
namespace perception
{
namespace tracking
{

using autoware::common::state_vector::variable::X;
using autoware::common::state_vector::variable::Y;
using autoware::common::types::float32_t;

constexpr std::size_t AssociatorResult::UNASSIGNED;

DataAssociationConfig::DataAssociationConfig(
  const float32_t max_distance,
  const float32_t max_area_ratio,
  const bool consider_edge_for_big_detections)
: m_max_distance(max_distance), m_max_distance_squared(max_distance * max_distance),
  m_max_area_ratio(max_area_ratio), m_max_area_ratio_inv(1.F / max_area_ratio),
  m_consider_edge_for_big_detections(consider_edge_for_big_detections) {}

DetectedObjectAssociator::DetectedObjectAssociator(const DataAssociationConfig & association_cfg)
: m_association_cfg(association_cfg) {}

AssociatorResult DetectedObjectAssociator::assign(
  const autoware_auto_msgs::msg::DetectedObjects & detections,
  const TrackedObjects & tracks)
{
  if (tracks.frame_id != detections.header.frame_id) {
    throw std::runtime_error(
            "Cannot associate tracks with detections - they are in different frames");
  }
  reset();
  m_num_detections = detections.objects.size();
  m_num_tracks = tracks.objects.size();
  m_are_tracks_rows = (m_num_tracks <= m_num_detections);
  if (m_are_tracks_rows) {
    m_assigner.set_size(
      static_cast<assigner_idx_t>(tracks.objects.size()),
      static_cast<assigner_idx_t>(detections.objects.size()));
  } else {
    m_assigner.set_size(
      static_cast<assigner_idx_t>(detections.objects.size()),
      static_cast<assigner_idx_t>(tracks.objects.size()));
  }
  compute_weights(detections, tracks);
  // TODO(gowtham.ranganathan): Revisit this after #979 since till then assigner will always
  //  return true
  (void)m_assigner.assign();

  return extract_result();
}

void DetectedObjectAssociator::reset()
{
  m_assigner.reset();

  m_num_tracks = 0U;
  m_num_detections = 0U;

  m_had_errors = false;
}

void DetectedObjectAssociator::compute_weights(
  const autoware_auto_msgs::msg::DetectedObjects & detections,
  const TrackedObjects & tracks)
{
  for (size_t det_idx = 0U; det_idx < detections.objects.size(); ++det_idx) {
    for (size_t track_idx = 0U; track_idx < tracks.objects.size(); ++track_idx) {
      const auto & track = tracks.objects[track_idx];
      const auto & detection = detections.objects[det_idx];

      try {
        if (consider_associating(detection, track)) {
          Eigen::Matrix<float32_t, NUM_OBJ_POSE_DIM, 1> sample;
          sample(0, 0) = static_cast<float32_t>(detection.kinematics.centroid_position.x);
          sample(1, 0) = static_cast<float32_t>(detection.kinematics.centroid_position.y);

          Eigen::Matrix<float32_t, NUM_OBJ_POSE_DIM,
            1> mean{track.centroid().cast<float32_t>()};

          Eigen::Matrix<float32_t, NUM_OBJ_POSE_DIM,
            NUM_OBJ_POSE_DIM> cov = track.position_covariance().cast<float32_t>();

          const auto dist = autoware::common::helper_functions::calculate_mahalanobis_distance(
            sample, mean, cov);

          set_weight(dist, det_idx, track_idx);
        }
      } catch (const std::runtime_error & e) {
        m_had_errors = true;
      } catch (const std::domain_error & e) {
        m_had_errors = true;
      }
    }
  }
}

bool DetectedObjectAssociator::consider_associating(
  const autoware_auto_msgs::msg::DetectedObject & detection,
  const TrackedObject & track) const
{
  const auto get_shortest_edge_size_squared = [&]() -> float32_t {
      float32_t retval = std::numeric_limits<float32_t>::max();
      for (auto current = detection.shape.polygon.points.begin();
        current != detection.shape.polygon.points.end(); ++current)
      {
        auto next = common::geometry::details::circular_next(
          detection.shape.polygon.points.begin(), detection.shape.polygon.points.end(), current);
        retval = std::min(retval, common::geometry::squared_distance_2d(*current, *next));
      }
      return retval;
    };

  const float32_t det_area = common::geometry::area_checked_2d(
    detection.shape.polygon.points.begin(), detection.shape.polygon.points.end());

  // TODO(gowtham.ranganathan): Add support for articulated objects
  const float32_t track_area = common::geometry::area_checked_2d(
    track.shape().polygon.points.begin(), track.shape().polygon.points.end());
  static constexpr float32_t kAreaEps = 1e-3F;

  if (common::helper_functions::comparisons::abs_eq_zero(det_area, kAreaEps) ||
    common::helper_functions::comparisons::abs_eq_zero(track_area, kAreaEps))
  {
    throw std::runtime_error("Detection or track area is zero");
  }

  const float32_t area_ratio = det_area / track_area;

  geometry_msgs::msg::Point track_centroid{};
  track_centroid.x = track.centroid().x();
  track_centroid.y = track.centroid().y();

  const auto compute_distance_threshold = [&get_shortest_edge_size_squared, this]() -> float32_t {
      if (m_association_cfg.consider_edge_for_big_detections()) {
        return std::max(
          m_association_cfg.get_max_distance_squared(),
          get_shortest_edge_size_squared());
      } else {
        return m_association_cfg.get_max_distance_squared();
      }
    };

  if (common::geometry::squared_distance_2d(
      detection.kinematics.centroid_position,
      track_centroid) > compute_distance_threshold())
  {
    return false;
  }

  if (area_ratio < m_association_cfg.get_max_area_ratio()) {
    if (area_ratio > m_association_cfg.get_max_area_ratio_inv()) {
      return true;
    }
  }
  return false;
}

void DetectedObjectAssociator::set_weight(
  const float32_t weight,
  const size_t det_idx, const size_t track_idx)
{
  if (m_are_tracks_rows) {
    m_assigner.set_weight(
      weight, static_cast<assigner_idx_t>(track_idx),
      static_cast<assigner_idx_t>(det_idx));
  } else {
    m_assigner.set_weight(
      weight, static_cast<assigner_idx_t>(det_idx),
      static_cast<assigner_idx_t>(track_idx));
  }
}

AssociatorResult DetectedObjectAssociator::extract_result() const
{
  AssociatorResult ret;
  ret.track_assignments.resize(m_num_tracks);
  std::fill(
    ret.track_assignments.begin(), ret.track_assignments.end(),
    AssociatorResult::UNASSIGNED);

  if (m_are_tracks_rows) {
    std::vector<common::types::bool8_t> detections_assigned(m_num_detections, false);
    for (size_t track_idx = 0U; track_idx < m_num_tracks; track_idx++) {
      const auto det_idx =
        static_cast<size_t>(m_assigner.get_assignment(static_cast<assigner_idx_t>(track_idx)));
      if (det_idx != Assigner::UNASSIGNED) {
        ret.track_assignments[track_idx] = det_idx;
        detections_assigned[det_idx] = true;
      } else {
        ret.unassigned_track_indices.insert(track_idx);
      }
    }

    for (size_t i = 0U; i < detections_assigned.size(); ++i) {
      if (!detections_assigned[i]) {
        ret.unassigned_detection_indices.insert(i);
      }
    }
  } else {
    std::vector<common::types::bool8_t> tracks_assigned(m_num_tracks, false);
    for (size_t det_idx = 0U; det_idx < m_num_detections; det_idx++) {
      const auto track_idx =
        static_cast<size_t>(m_assigner.get_assignment(static_cast<assigner_idx_t>(det_idx)));
      if (track_idx != Assigner::UNASSIGNED) {
        ret.track_assignments[track_idx] = det_idx;
        tracks_assigned[track_idx] = true;
      } else {
        ret.unassigned_detection_indices.insert(det_idx);
      }
    }

    for (size_t i = 0U; i < tracks_assigned.size(); ++i) {
      if (!tracks_assigned[i]) {
        ret.unassigned_track_indices.insert(i);
      }
    }
  }

  return ret;
}

}  // namespace tracking
}  // namespace perception
}  // namespace autoware
