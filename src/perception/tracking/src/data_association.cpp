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

#include <tracking/data_association.hpp>

#include <common/types.hpp>
#include <geometry/common_2d.hpp>
#include <helper_functions/mahalanobis_distance.hpp>

#include <limits>
#include <vector>

namespace autoware
{
namespace perception
{
namespace tracking
{

constexpr std::size_t AssociatorResult::UNASSIGNED;

DataAssociationConfig::DataAssociationConfig(
  const float max_distance,
  const float max_area_ratio)
: m_max_distance(max_distance), m_max_distance_squared(max_distance * max_distance),
  m_max_area_ratio(max_area_ratio), m_max_area_ratio_inv(1.F / max_area_ratio) {}

Associator::Associator(const DataAssociationConfig & association_cfg)
: m_association_cfg(association_cfg) {}

AssociatorResult Associator::assign(
  const autoware_auto_msgs::msg::DetectedObjects & detections,
  const autoware_auto_msgs::msg::TrackedObjects & tracks)
{
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

void Associator::reset()
{
  m_assigner.reset();

  m_num_tracks = 0U;
  m_num_detections = 0U;
}

void Associator::compute_weights(
  const autoware_auto_msgs::msg::DetectedObjects & detections,
  const autoware_auto_msgs::msg::TrackedObjects & tracks)
{
  for (size_t det_idx = 0U; det_idx < detections.objects.size(); ++det_idx) {
    for (size_t track_idx = 0U; track_idx < tracks.objects.size(); ++track_idx) {
      const auto & track = tracks.objects[track_idx];
      const auto & detection = detections.objects[det_idx];

      if (consider_associating(detection, track)) {
        Eigen::Matrix<float, NUM_OBJ_POSE_DIM, 1> sample;
        sample(0, 0) = static_cast<float>(detection.kinematics.pose.pose.position.x);
        sample(1, 0) = static_cast<float>(detection.kinematics.pose.pose.position.y);

        Eigen::Matrix<float, NUM_OBJ_POSE_DIM, 1> mean(NUM_OBJ_POSE_DIM, 1U);
        mean(0, 0) = static_cast<float>(track.kinematics.pose.pose.position.x);
        mean(1, 0) = static_cast<float>(track.kinematics.pose.pose.position.y);

        Eigen::Matrix<float, NUM_OBJ_POSE_DIM,
          NUM_OBJ_POSE_DIM> cov = Eigen::Map<const Eigen::Matrix<double, 6, 6, Eigen::RowMajor>>(
          tracks.objects[track_idx]
          .kinematics.pose.covariance.data()).cast<float>().topLeftCorner<2, 2>();

        const auto dist = autoware::common::helper_functions::calculate_mahalanobis_distance(
          sample, mean, cov);

        set_weight(dist, det_idx, track_idx);
      }
    }
  }
}

bool Associator::consider_associating(
  const autoware_auto_msgs::msg::DetectedObject & detection,
  const autoware_auto_msgs::msg::TrackedObject & track) const
{
  const auto squared_distance_2d = [](const geometry_msgs::msg::Pose & p1, const
      geometry_msgs::msg::Pose & p2) -> float {
      return static_cast<float>((
               (p1.position.x - p2.position.x) *
               (p1.position.x - p2.position.x)) + (
               (p1.position.y - p2.position.y) *
               (p1.position.y - p2.position.y)));
    };

  const float det_area = common::geometry::area_checked_2d(
    detection.shape.polygon.points.begin(), detection.shape.polygon.points.end());
  // TODO(gowtham.ranganathan): Add support for articulated objects
  const float track_area = common::geometry::area_checked_2d(
    track.shape[0].polygon.points.begin(), track.shape[0].polygon.points.end());
  static constexpr float kAreaEps = 1e-3F;

  if (common::helper_functions::comparisons::abs_eq_zero(det_area, kAreaEps) ||
    common::helper_functions::comparisons::abs_eq_zero(track_area, kAreaEps))
  {
    throw std::runtime_error("Detection or track area is zero");
  }

  const float area_ratio = det_area / track_area;

  if (squared_distance_2d(detection.kinematics.pose.pose, track.kinematics.pose.pose) >
    m_association_cfg.get_max_distance_squared())
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

void Associator::set_weight(const float weight, const size_t det_idx, const size_t track_idx)
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

AssociatorResult Associator::extract_result() const
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
      if (det_idx != m_assigner.UNASSIGNED) {
        ret.track_assignments[track_idx] = det_idx;
        detections_assigned[det_idx] = true;
      } else {
        ret.unassigned_track_indices.push_back(track_idx);
      }
    }

    for (size_t i = 0U; i < detections_assigned.size(); ++i) {
      if (!detections_assigned[i]) {
        ret.unassigned_detection_indices.push_back(i);
      }
    }
  } else {
    std::vector<common::types::bool8_t> tracks_assigned(m_num_tracks, false);
    for (size_t det_idx = 0U; det_idx < m_num_detections; det_idx++) {
      const auto track_idx =
        static_cast<size_t>(m_assigner.get_assignment(static_cast<assigner_idx_t>(det_idx)));
      if (track_idx != AssociatorResult::UNASSIGNED) {
        ret.track_assignments[track_idx] = det_idx;
        tracks_assigned[track_idx] = true;
      } else {
        ret.unassigned_detection_indices.push_back(det_idx);
      }
    }

    for (size_t i = 0U; i < tracks_assigned.size(); ++i) {
      if (!tracks_assigned[i]) {
        ret.unassigned_track_indices.push_back(i);
      }
    }
  }

  return ret;
}

}  // namespace tracking
}  // namespace perception
}  // namespace autoware
