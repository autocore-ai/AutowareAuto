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
  AssociatorResult result = create_and_init_result(rois.rois.size(), tracks.size());

  for (auto track_idx = 0U; track_idx < tracks.size(); ++track_idx) {
    const auto matched_detection_idx = project_and_match_detection(
      tracks[track_idx].shape(), result.unassigned_detection_indices, rois);

    handle_matching_output(matched_detection_idx, track_idx, result);
  }

  return result;
}

AssociatorResult GreedyRoiAssociator::assign(
  const autoware_auto_msgs::msg::ClassifiedRoiArray & rois,
  const autoware_auto_msgs::msg::DetectedObjects & objects) const
{
  AssociatorResult result = create_and_init_result(rois.rois.size(), objects.objects.size());

  for (auto object_idx = 0U; object_idx < objects.objects.size(); ++object_idx) {
    auto detection_idx = project_and_match_detection(
      objects.objects[object_idx].shape, result.unassigned_detection_indices, rois);

    handle_matching_output(detection_idx, object_idx, result);
  }

  return result;
}

AssociatorResult GreedyRoiAssociator::create_and_init_result(
  const std::size_t rois_size,
  const std::size_t objects_size) const
{
  AssociatorResult result;
  result.track_assignments.resize(objects_size);
  std::fill(
    result.track_assignments.begin(), result.track_assignments.end(),
    AssociatorResult::UNASSIGNED);

  std::size_t counter = 0U;
  std::generate_n(
    std::inserter(
      result.unassigned_detection_indices, result.unassigned_detection_indices.begin()),
    rois_size, [&counter]() {return counter++;});
  return result;
}

std::size_t GreedyRoiAssociator::project_and_match_detection(
  const autoware_auto_msgs::msg::Shape & object_shape,
  const std::unordered_set<std::size_t> & available_roi_indices,
  const autoware_auto_msgs::msg::ClassifiedRoiArray & rois) const
{
  const auto & maybe_projection = m_camera.project(object_shape);

  // There is no projection or the projection is collinear
  if (!maybe_projection) {
    return AssociatorResult::UNASSIGNED;
  }

  auto max_score = 0.0F;
  std::size_t max_score_idx = AssociatorResult::UNASSIGNED;
  for (const auto idx : available_roi_indices) {
    const auto score = m_iou_func(maybe_projection.value().shape, rois.rois[idx].polygon.points);
    max_score = std::max(score, max_score);
    max_score_idx = idx;
  }
  return max_score > m_iou_threshold ? max_score_idx : AssociatorResult::UNASSIGNED;
}

void GreedyRoiAssociator::handle_matching_output(
  const std::size_t matched_detection_idx,
  const std::size_t object_idx,
  AssociatorResult & result) const
{
  // There's no ROI assignment fit for the projection
  if (matched_detection_idx == AssociatorResult::UNASSIGNED) {
    result.unassigned_track_indices.insert(object_idx);
    return;
  }
  // The track can be projected on the image and has a matching ROI, so they are associated
  result.track_assignments[object_idx] = matched_detection_idx;
  result.unassigned_detection_indices.erase(matched_detection_idx);
}
}  // namespace tracking
}  // namespace perception
}  // namespace autoware
