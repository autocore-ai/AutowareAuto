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

#include <gtest/gtest.h>

#include <tracking/detected_object_associator.hpp>

#include <vector>

using TrackedObjects = autoware_auto_msgs::msg::TrackedObjects;
using TrackedObject = autoware_auto_msgs::msg::TrackedObject;

using DetectedObjects = autoware_auto_msgs::msg::DetectedObjects;
using DetectedObject = autoware_auto_msgs::msg::DetectedObject;

namespace tracking = autoware::perception::tracking;

namespace
{
constexpr auto kTrackerFrame = "odom";
}  // namespace

class AssociationTester : public testing::Test
{
protected:
  AssociationTester()
  : m_association_cfg(10.0F, 2.0F, true),
    m_associator(m_association_cfg)
  {
    // just set x and y fields of the covariance since only that is used in the associator
    m_some_covariance[0] = 0.5;
    m_some_covariance[1] = -0.09;
    m_some_covariance[3] = 1.0;
    m_some_covariance[4] = 0.09;
  }

  // Square will be centered on origin since vertices are not used for any check except area
  autoware_auto_msgs::msg::Shape create_square(float area)
  {
    autoware_auto_msgs::msg::Shape shape;
    const float side_length = std::sqrt(area);

    shape.polygon.points.push_back(
      geometry_msgs::msg::Point32{}.set__x(0.0F).set__y(0.0F)
      .set__z(0.0F));
    shape.polygon.points.push_back(
      geometry_msgs::msg::Point32{}.set__x(side_length).set__y(0.0F)
      .set__z(0.0F));
    shape.polygon.points.push_back(
      geometry_msgs::msg::Point32{}.set__x(side_length).set__y(side_length)
      .set__z(0.0F));
    shape.polygon.points.push_back(
      geometry_msgs::msg::Point32{}.set__x(0.0F).set__y(side_length)
      .set__z(0.0F));
    return shape;
  }

  std::array<double, 9> m_some_covariance;
  tracking::DataAssociationConfig m_association_cfg;
  tracking::DetectedObjectAssociator m_associator;
};


// Two objects, one track. Track has huge y variance and small x variance.
// Object 1 is shorter distance away on x, same y. Object2 is longer distance away on Y,
// same x. Associator should associate track with object2
TEST_F(AssociationTester, Basic)
{
  TrackedObjects tracks_msg;
  std::vector<tracking::TrackedObject> tracked_object_vec{};
  DetectedObject track1_obj;
  track1_obj.shape = create_square(4.0F);
  track1_obj.kinematics.centroid_position.x = 2.0;
  track1_obj.kinematics.centroid_position.y = 2.0;
  track1_obj.kinematics.position_covariance[0] = 0.5;
  track1_obj.kinematics.position_covariance[1] = -0.09;
  track1_obj.kinematics.position_covariance[3] = -0.09;
  track1_obj.kinematics.position_covariance[4] = 10.43;
  track1_obj.kinematics.has_position_covariance = true;

  tracked_object_vec.emplace_back(track1_obj, 0.0, 0.0);

  DetectedObjects objects_msg;
  DetectedObject obj1;
  obj1.shape = create_square(4.0F);
  obj1.kinematics.centroid_position.x = 2.5;
  obj1.kinematics.centroid_position.y = 2.0;
  obj1.kinematics.position_covariance[0] = 0.5;
  obj1.kinematics.position_covariance[1] = -0.09;
  obj1.kinematics.position_covariance[3] = -0.09;
  obj1.kinematics.position_covariance[4] = 10.43;
  objects_msg.objects.push_back(obj1);

  DetectedObject obj2;
  obj2 = obj1;
  obj2.kinematics.centroid_position.x = 2.0;
  obj2.kinematics.centroid_position.y = 3.0;
  objects_msg.objects.push_back(obj2);
  objects_msg.header.frame_id = kTrackerFrame;  // Set to the same frame as the tracker.

  tracking::TrackedObjects tracks{tracked_object_vec, kTrackerFrame};
  const auto ret = m_associator.assign(objects_msg, tracks);
  EXPECT_EQ(ret.track_assignments[0U], 1U);
  EXPECT_EQ(ret.unassigned_detection_indices.size(), 1U);
  EXPECT_TRUE(ret.unassigned_detection_indices.find(0U) != ret.unassigned_detection_indices.end());
}

// 10 tracks, 5 detections. Make sure 5 tracks are unassigned
TEST_F(AssociationTester, MoreTracksLessObjects)
{
  const auto num_tracks = 10U;
  auto num_associated_dets = 0U;

  TrackedObjects tracks_msg;
  std::vector<tracking::TrackedObject> tracked_object_vec{};
  DetectedObjects detections_msg;

  for (size_t i = 0U; i < num_tracks; ++i) {
    DetectedObject current_track;
    const auto current_shape = create_square(4.0F);
    current_track.shape = current_shape;
    current_track.kinematics.centroid_position.x = 2.0 * static_cast<double>(i + 1U);
    current_track.kinematics.centroid_position.y = 2.0 * static_cast<double>(i + 1U);
    current_track.kinematics.position_covariance = m_some_covariance;
    current_track.kinematics.has_position_covariance = true;
    tracked_object_vec.emplace_back(current_track, 0.0, 0.0);

    //  Create detections that can be associated with tracks
    if (i % 2 == 0) {
      ++num_associated_dets;
      DetectedObject current_detection;
      current_detection.shape = current_shape;
      // Move detections a bit to test out distance calculation logic as well
      current_detection.kinematics.centroid_position.x =
        current_track.kinematics.centroid_position.x + 0.6;
      current_detection.kinematics.centroid_position.y =
        current_track.kinematics.centroid_position.y + 0.8;
      current_detection.kinematics.position_covariance = m_some_covariance;

      detections_msg.objects.push_back(current_detection);
    }
  }
  detections_msg.header.frame_id = kTrackerFrame;  // Set to the same frame as the tracker.

  tracking::TrackedObjects tracks{tracked_object_vec, kTrackerFrame};
  const auto ret = m_associator.assign(detections_msg, tracks);

  EXPECT_EQ(ret.unassigned_track_indices.size(), num_tracks - num_associated_dets);
  for (size_t i = 0U; i < ret.unassigned_track_indices.size(); ++i) {
    EXPECT_TRUE(
      ret.unassigned_track_indices.find((i * 2U) + 1U) != ret.unassigned_track_indices.end());
  }
}

// 5 tracks and 5 detections. 3 of those detections have areas much smaller/bigger than the
// corresponding tracks. This should cause gating to fail and result in no association for
// 3 tracks.
TEST_F(AssociationTester, AreaGatingFails)
{
  const auto num_tracks = 5U;
  auto num_unassociated_dets = 0U;

  TrackedObjects tracks_msg;
  std::vector<tracking::TrackedObject> tracked_object_vec{};
  DetectedObjects detections_msg;

  // toggle to set some detections to bigger size and some to smaller size
  bool toggle = true;

  for (size_t i = 0U; i < num_tracks; ++i) {
    DetectedObject current_track;
    const auto current_shape = create_square(4.0F);
    current_track.shape = current_shape;
    current_track.kinematics.centroid_position.x = 2.0 * static_cast<double>(i + 1U);
    current_track.kinematics.centroid_position.y = 2.0 * static_cast<double>(i + 1U);
    current_track.kinematics.position_covariance = m_some_covariance;
    current_track.kinematics.has_position_covariance = true;
    tracked_object_vec.emplace_back(current_track, 0.0, 0.0);

    DetectedObject current_detection;
    if (i % 2 == 0) {
      // Create detections that cannot be associated with tracks
      ++num_unassociated_dets;
      if (toggle) {
        current_detection.shape = create_square(12.F);
        toggle = false;
      } else {
        current_detection.shape = create_square(0.5F);
        toggle = true;
      }
    } else {
      current_detection.shape = current_shape;
    }
    // Exact same position as track
    current_detection.kinematics.centroid_position = current_track.kinematics.centroid_position;
    current_detection.kinematics.position_covariance = m_some_covariance;

    detections_msg.objects.push_back(current_detection);
  }
  detections_msg.header.frame_id = kTrackerFrame;  // Set to the same frame as the tracker.

  tracking::TrackedObjects tracks{tracked_object_vec, kTrackerFrame};
  const auto ret = m_associator.assign(detections_msg, tracks);

  // Verify unassigned tracks
  ASSERT_EQ(ret.unassigned_track_indices.size(), num_unassociated_dets);
  for (size_t i = 0U, track_idx = 0U; i < ret.unassigned_track_indices.size();
    ++i, track_idx += 2)
  {
    EXPECT_TRUE(ret.unassigned_track_indices.find(track_idx) != ret.unassigned_track_indices.end());
  }

  // Verify unassigned detections
  ASSERT_EQ(ret.unassigned_detection_indices.size(), num_unassociated_dets);
  for (size_t i = 0U, det_idx = 0U; i < ret.unassigned_detection_indices.size();
    ++i, det_idx += 2)
  {
    EXPECT_TRUE(
      ret.unassigned_detection_indices.find(det_idx) != ret.unassigned_detection_indices.end());
  }

  // Verify assignments
  for (size_t track_idx = 0U; track_idx < num_tracks; track_idx++) {
    if (track_idx % 2 == 0) {
      EXPECT_EQ(ret.track_assignments[track_idx], tracking::AssociatorResult::UNASSIGNED);

    } else {
      EXPECT_EQ(ret.track_assignments[track_idx], track_idx);
    }
  }
}
