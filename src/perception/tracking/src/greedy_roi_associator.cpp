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
#include <geometry/bounding_box/bounding_box_common.hpp>
#include <time_utils/time_utils.hpp>
#include <tracking/detected_object_associator.hpp>
#include <tracking/greedy_roi_associator.hpp>

#include <algorithm>
#include <unordered_set>
#include <string>
#include <vector>

namespace autoware
{
namespace perception
{
namespace tracking
{
using autoware::common::types::float32_t;

const std::chrono::milliseconds GreedyRoiAssociator::kTfTooOld{100};

namespace
{
AssociatorResult create_and_init_result(
  const std::size_t rois_size,
  const std::size_t objects_size)
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

// Uses the given matched_detection_idx to assign to appropriate containers in result
void handle_matching_output(
  const std::size_t matched_detection_idx,
  const std::size_t object_idx,
  AssociatorResult & result)
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
}  // namespace

GreedyRoiAssociator::GreedyRoiAssociator(
  const GreedyRoiAssociatorConfig & config,
  const tf2::BufferCore & tf_buffer)
: m_camera{config.intrinsics}, m_iou_threshold{config.iou_threshold}, m_tf_buffer{tf_buffer} {}

AssociatorResult GreedyRoiAssociator::assign(
  const autoware_auto_msgs::msg::ClassifiedRoiArray & rois,
  const TrackedObjects & tracks) const
{
  AssociatorResult result = create_and_init_result(rois.rois.size(), tracks.objects.size());
  geometry_msgs::msg::TransformStamped tf_roi_from_track;
  try {
    tf_roi_from_track = lookup_transform_handler(
      rois.header.frame_id, tracks.frame_id, time_utils::from_message(rois.header.stamp));
  } catch (const std::exception & e) {
    std::cerr << "Transform lookup failed with exception: " << e.what() << std::endl;
    return result;
  }
  const details::ShapeTransformer transformer{tf_roi_from_track.transform};
  for (auto track_idx = 0U; track_idx < tracks.objects.size(); ++track_idx) {
    const auto matched_detection_idx = project_and_match_detection(
      transformer(
        tracks.objects[track_idx].shape(),
        geometry_msgs::msg::Point{}.set__x(
          tracks.objects[track_idx].centroid().x()).set__y(
          tracks.objects[track_idx].centroid().y()),
        tracks.objects[track_idx].orientation()), result
      .unassigned_detection_indices, rois);

    handle_matching_output(matched_detection_idx, track_idx, result);
  }

  return result;
}

AssociatorResult GreedyRoiAssociator::assign(
  const autoware_auto_msgs::msg::ClassifiedRoiArray & rois,
  const autoware_auto_msgs::msg::DetectedObjects & objects) const
{
  AssociatorResult result = create_and_init_result(rois.rois.size(), objects.objects.size());
  geometry_msgs::msg::TransformStamped tf_roi_from_detection;
  try {
    tf_roi_from_detection = lookup_transform_handler(
      rois.header.frame_id, objects.header.frame_id,
      time_utils::from_message(rois.header.stamp));
  } catch (const std::exception & e) {
    std::cerr << "Transform lookup failed with exception: " << e.what() << std::endl;
    return result;
  }
  const details::ShapeTransformer transformer{tf_roi_from_detection.transform};

  for (auto object_idx = 0U; object_idx < objects.objects.size(); ++object_idx) {
    auto & object = objects.objects[object_idx];
    auto detection_idx = project_and_match_detection(
      transformer(object.shape, object.kinematics.centroid_position, object.kinematics.orientation),
      result.unassigned_detection_indices, rois);

    handle_matching_output(detection_idx, object_idx, result);
  }

  return result;
}

geometry_msgs::msg::TransformStamped GreedyRoiAssociator::lookup_transform_handler(
  const std::string & target_frame,
  const std::string & source_frame,
  const tf2::TimePoint & stamp) const
{
  geometry_msgs::msg::TransformStamped retval;
  try {
    retval = m_tf_buffer.lookupTransform(target_frame, source_frame, stamp);
  } catch (const tf2::ExtrapolationException & e) {
    retval = m_tf_buffer.lookupTransform(target_frame, source_frame, tf2::TimePointZero);
    const auto stamp_diff = stamp - time_utils::from_message(retval.header.stamp);
    if (stamp_diff > kTfTooOld) {
      std::cerr << "Using tf that is " << std::chrono::duration_cast<std::chrono::milliseconds>(
        stamp_diff).count() << "ms old" << std::endl;
    }
  }
  return retval;
}

std::size_t GreedyRoiAssociator::project_and_match_detection(
  const std::vector<geometry_msgs::msg::Point32> & object_shape_in_camera_frame,
  const std::unordered_set<std::size_t> & available_roi_indices,
  const autoware_auto_msgs::msg::ClassifiedRoiArray & rois) const
{
  const auto & maybe_projection = m_camera.project(object_shape_in_camera_frame);

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

namespace details
{
ShapeTransformer::ShapeTransformer(const geometry_msgs::msg::Transform & tf)
: m_transformer{tf}
{
}

std::vector<geometry_msgs::msg::Point32> ShapeTransformer::operator()(
  const autoware_auto_msgs::msg::Shape & shape, const geometry_msgs::msg::Point & centroid,
  const geometry_msgs::msg::Quaternion & orientation) const
{
  std::vector<Point32> result;
  result.reserve(2U * shape.polygon.points.size());

  const auto corners = common::geometry::bounding_box::details::get_transformed_corners(
    shape, centroid, orientation);

  for (const auto & pt : corners) {
    Point32 pt_transformed{};
    // Transform the vertices on the bottom face
    m_transformer.transform(pt, pt_transformed);
    result.emplace_back(pt_transformed);

    // Transform the vertices on the top face
    m_transformer.transform(Point32{pt}.set__z(pt.z + shape.height), pt_transformed);
    result.emplace_back(pt_transformed);
  }

  return result;
}
}  // namespace details
}  // namespace tracking
}  // namespace perception
}  // namespace autoware
