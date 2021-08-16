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

#include <common/types.hpp>
#include <tracking/detected_object_associator.hpp>
#include <tracking/greedy_roi_associator.hpp>
#include <algorithm>
#include <unordered_set>
#include <vector>

namespace autoware
{
namespace perception
{
namespace tracking
{
using autoware::common::types::float32_t;

GreedyRoiAssociator::GreedyRoiAssociator(
  const CameraIntrinsics & intrinsics,
  const geometry_msgs::msg::Transform & tf_camera_from_ego,
  const float32_t iou_threshold)
: m_camera{intrinsics, tf_camera_from_ego}, m_iou_threshold{iou_threshold}
{
}

AssociatorResult GreedyRoiAssociator::assign(
  const autoware_auto_msgs::msg::ClassifiedRoiArray & rois,
  const std::vector<TrackedObject> & tracks) const
{
  AssociatorResult result;
  result.track_assignments.resize(tracks.size());
  std::fill(
    result.track_assignments.begin(), result.track_assignments.end(),
    AssociatorResult::UNASSIGNED);

  std::unordered_set<std::size_t> unassigned_detection_indices;
  {
    std::size_t counter = 0U;
    std::generate_n(
      std::inserter(unassigned_detection_indices, unassigned_detection_indices.begin()),
      rois.rois.size(), [&counter]() {return counter++;});
  }

  for (auto track_idx = 0U; track_idx < tracks.size(); ++track_idx) {
    const auto & track = tracks[track_idx];
    const auto & maybe_projection = m_camera.project(track.shape());

    // There is no projection or the projection is collinear
    if (!maybe_projection) {
      result.unassigned_track_indices.push_back(track_idx);
      continue;
    }
    const auto detection_idx =
      match_detection(maybe_projection.value(), unassigned_detection_indices, rois);

    // There's no ROI assignment fit for the projection
    if (detection_idx == AssociatorResult::UNASSIGNED) {
      result.unassigned_track_indices.push_back(track_idx);
      continue;
    }
    // The track can be projected on the image and has a matching ROI, so they are associated
    result.track_assignments[track_idx] = detection_idx;
    unassigned_detection_indices.erase(detection_idx);
  }

  std::copy(
    unassigned_detection_indices.begin(), unassigned_detection_indices.end(),
    std::back_inserter(result.unassigned_detection_indices));

  return result;
}

std::size_t GreedyRoiAssociator::match_detection(
  const Projection & projection,
  const std::unordered_set<std::size_t> & available_roi_indices,
  const autoware_auto_msgs::msg::ClassifiedRoiArray & rois) const
{
  auto max_score = 0.0F;
  std::size_t max_score_idx = AssociatorResult::UNASSIGNED;
  for (const auto idx : available_roi_indices) {
    const auto score = m_iou_func(projection.shape, rois.rois[idx].polygon.points);
    max_score = std::max(score, max_score);
    max_score_idx = idx;
  }
  return max_score > m_iou_threshold ? max_score_idx : AssociatorResult::UNASSIGNED;
}
}  // namespace tracking
}  // namespace perception
}  // namespace autoware
