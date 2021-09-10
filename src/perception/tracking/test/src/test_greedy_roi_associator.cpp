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

#include "tracking/test_utils.hpp"
#include <gtest/gtest.h>
#include <tracking/greedy_roi_associator.hpp>
#include <tracking/projection.hpp>
#include <vector>

using autoware::perception::tracking::TrackedObject;
using autoware::perception::tracking::TrackedObjects;

using autoware_auto_msgs::msg::DetectedObjects;
using autoware_auto_msgs::msg::DetectedObject;
using autoware_auto_msgs::msg::Shape;
using autoware::perception::tracking::Projection;
using autoware::perception::tracking::CameraModel;
using autoware::perception::tracking::CameraIntrinsics;
using autoware_auto_msgs::msg::ClassifiedRoi;
using autoware_auto_msgs::msg::ClassifiedRoiArray;
using geometry_msgs::msg::Point32;

namespace tracking = autoware::perception::tracking;
using RoiAssociator = tracking::GreedyRoiAssociator;
using AssociatorResult = tracking::AssociatorResult;

namespace
{
constexpr autoware::common::types::bool8_t kIsStatic = true;
constexpr auto kTrackerFrame = "odom";
constexpr auto kCameraFrame = "camera";
constexpr auto kDummyTfAuthority = "test_authority";

geometry_msgs::msg::TransformStamped create_identity_transform(
  const std_msgs::msg::Header::_frame_id_type & frame_id,
  const std_msgs::msg::Header::_frame_id_type & child_frame_id,
  const std_msgs::msg::Header::_stamp_type & stamp) noexcept
{
  geometry_msgs::msg::Transform identity{};
  identity.rotation.w = 1.0;

  geometry_msgs::msg::TransformStamped tf;
  tf.header.frame_id = frame_id;
  tf.child_frame_id = child_frame_id;
  tf.header.stamp = stamp;
  tf.transform = identity;
  return tf;
}

}  // namespace

template<typename ObjectsT>
class TestRoiAssociation : public testing::Test
{
public:
  TestRoiAssociation()
  : intrinsics{CameraIntrinsics{500U, 500U, 5.0F, 5.0F}},
    camera{intrinsics},
    associator{{intrinsics, 0.1F}, tf_buffer} {init_frame_id();}

  void init_frame_id();

  void add_object(
    const Point32 & base_face_origin,
    float32_t half_width, float32_t half_length, float32_t shape_height);

  Shape get_ith_shape(const size_t i);

  CameraIntrinsics intrinsics;
  CameraModel camera;
  RoiAssociator associator;
  ObjectsT objects;
  tf2::BufferCore tf_buffer;
};

template<>
void TestRoiAssociation<TrackedObjects>::init_frame_id()
{
  objects.frame_id = kTrackerFrame;
}

template<>
void TestRoiAssociation<DetectedObjects>::init_frame_id()
{
  objects.header.frame_id = kTrackerFrame;
}

template<>
void TestRoiAssociation<TrackedObjects>::add_object(
  const Point32 & base_face_origin,
  float32_t half_width,
  float32_t half_length,
  float32_t shape_height)
{
  DetectedObject object;
  object.shape = make_rectangular_shape(base_face_origin, half_width, half_length, shape_height);
  objects.objects.emplace_back(object, 0.0, 0.0);
}

template<>
void TestRoiAssociation<DetectedObjects>::add_object(
  const Point32 & base_face_origin,
  float32_t half_width, float32_t half_length,
  float32_t shape_height)
{
  DetectedObject object;
  object.shape = make_rectangular_shape(base_face_origin, half_width, half_length, shape_height);
  objects.objects.push_back(object);
}

template<>
Shape TestRoiAssociation<TrackedObjects>::get_ith_shape(const size_t i)
{
  EXPECT_TRUE(objects.objects.size() > i);
  return objects.objects[i].shape();
}

template<>
Shape TestRoiAssociation<DetectedObjects>::get_ith_shape(const size_t i)
{
  EXPECT_TRUE(objects.objects.size() > i);
  return objects.objects[i].shape;
}

using MyTypes = ::testing::Types<TrackedObjects, DetectedObjects>;
// NOTE: This is the older version due to 1.8.0 of GTest. v1.8.1 uses TYPED_TEST_SUITE
// cppcheck-suppress syntaxError
TYPED_TEST_CASE(TestRoiAssociation, MyTypes, );

TYPED_TEST(TestRoiAssociation, CorrectAssociation) {
  ClassifiedRoiArray rois;
  rois.header.frame_id = kCameraFrame;

  const auto tf = create_identity_transform(
    rois.header.frame_id, kTrackerFrame, rois.header.stamp);
  this->tf_buffer.setTransform(tf, kDummyTfAuthority, kIsStatic);

  this->add_object(make_pt(10.0F, 10.0F, 10), 5.0F, 5.0F, 2.0F);
  const auto projection = this->camera.project(expand_shape_to_vector(this->get_ith_shape(0U)));
  ASSERT_TRUE(projection);
  rois.rois.push_back(projection_to_roi(projection.value()));
  const auto result = this->associator.assign(rois, this->objects);
  ASSERT_EQ(result.track_assignments.front(), 0U);
  ASSERT_EQ(result.unassigned_detection_indices.size(), 0U);
  ASSERT_EQ(result.unassigned_track_indices.size(), 0U);
}

TYPED_TEST(TestRoiAssociation, OutOfImageTest) {
  ClassifiedRoiArray rois;
  rois.header.frame_id = kCameraFrame;

  const auto tf = create_identity_transform(
    rois.header.frame_id, kTrackerFrame, rois.header.stamp);
  this->tf_buffer.setTransform(tf, kDummyTfAuthority, kIsStatic);

  // Create a track behind the camera
  this->add_object(make_pt(10.0F, 10.0F, -10), 5.0F, 5.0F, 2.0F);
  const auto projection = this->camera.project(expand_shape_to_vector(this->get_ith_shape(0U)));
  ASSERT_FALSE(projection);
  const auto result = this->associator.assign(rois, this->objects);
  ASSERT_EQ(result.track_assignments.front(), AssociatorResult::UNASSIGNED);
  ASSERT_EQ(result.unassigned_detection_indices.size(), 0U);
  ASSERT_TRUE(result.unassigned_track_indices.find(0U) != result.unassigned_track_indices.end());
}

TYPED_TEST(TestRoiAssociation, NonAssociatedTrackRoi) {
  ClassifiedRoiArray rois;
  rois.header.frame_id = kCameraFrame;

  const auto tf = create_identity_transform(
    rois.header.frame_id, kTrackerFrame, rois.header.stamp);
  this->tf_buffer.setTransform(tf, kDummyTfAuthority, kIsStatic);

  this->add_object(make_pt(10.0F, 10.0F, 10), 5.0F, 5.0F, 2.0F);

  // Use the phantom track to create a detection that is not associated with the correct track.
  TrackedObjects phantom_tracks;
  phantom_tracks.frame_id = kTrackerFrame;
  {
    DetectedObject tmp_obj;
    tmp_obj.shape = make_rectangular_shape(make_pt(-50.0F, -20.0F, 100), 2.0F, 5.0F, 25.0F);
    phantom_tracks.objects.push_back(TrackedObject{tmp_obj, 0.0, 0.0});
  }

  const auto projection = this->camera.project(
    expand_shape_to_vector(phantom_tracks.objects[0].shape()));
  ASSERT_TRUE(projection);
  rois.rois.push_back(projection_to_roi(projection.value()));
  const auto result = this->associator.assign(rois, this->objects);
  ASSERT_EQ(result.track_assignments.front(), AssociatorResult::UNASSIGNED);
  ASSERT_TRUE(
    result.unassigned_detection_indices.find(0U) != result.unassigned_detection_indices.end());
  ASSERT_TRUE(result.unassigned_track_indices.find(0U) != result.unassigned_track_indices.end());
}

/// \brief This test creates a series of tracks and corresponding detection ROIs.
/// The tracks have the following layout: [FN, FN, FN, TP, TP, TP, TP] where the false negatives
/// don't have corresponding ROI detections.
/// Likewise the ROIs have the following layout:  [TP, TP, TP, TP, FP, FP, FP] where false
/// positives don't have corresponding tracks.
TYPED_TEST(TestRoiAssociation, CombinedAssociationTest) {
  constexpr auto num_captured_tracks = 4U;
  constexpr auto num_noncaptured_tracks = 3U;
  constexpr auto num_nonassociated_rois = 3U;
  TrackedObjects phantom_tracks;  // Only used to create false-positive ROIs
  phantom_tracks.frame_id = kTrackerFrame;
  ClassifiedRoiArray rois;
  rois.header.frame_id = kCameraFrame;

  const auto tf = create_identity_transform(
    rois.header.frame_id, kTrackerFrame, rois.header.stamp);
  this->tf_buffer.setTransform(tf, kDummyTfAuthority, kIsStatic);

  // Objects not to be captured by the camera
  this->add_object(make_pt(10.0F, 10.0F, -10), 5.0F, 5.0F, 2.0F);
  this->add_object(make_pt(1e6F, 10.0F, 50), 15.0F, 25.0F, 10.0F);
  this->add_object(make_pt(50.0F, 1e6F, 100), 2.0F, 5.0F, 25.0F);

  // Objects to be captured by the camera (All have positive X,Y coordinates)
  this->add_object(make_pt(10.0F, 10.0F, 10), 5.0F, 5.0F, 2.0F);
  this->add_object(make_pt(20.0F, 10.0F, 50), 15.0F, 25.0F, 10.0F);
  this->add_object(make_pt(50.0F, 20.0F, 100), 2.0F, 5.0F, 25.0F);
  this->add_object(make_pt(105.0F, 5.0F, 100), 1.0F, 1.0F, 5.0F);

  // Push the correct projections to the roi array
  for (auto i = 0U; i < num_captured_tracks + num_noncaptured_tracks; ++i) {
    const auto projection = this->camera.project(expand_shape_to_vector(this->get_ith_shape(i)));
    if (i < num_noncaptured_tracks) {
      ASSERT_FALSE(projection);
    } else {
      ASSERT_TRUE(projection);
      rois.rois.push_back(projection_to_roi(projection.value()));
    }
  }

  // Create some phantom objects to create some false-positive ROIs (All have negative X,Y
  // coordinates so their projections don't get associated with the correct tracks)
  {
    DetectedObject tmp_obj;
    tmp_obj.shape = make_rectangular_shape(make_pt(-10.0F, -10.0F, 10), 5.0F, 5.0F, 2.0F);
    phantom_tracks.objects.push_back(TrackedObject{tmp_obj, 0.0, 0.0});
    tmp_obj.shape = make_rectangular_shape(make_pt(-20.0F, -10.0F, 50), 15.0F, 25.0F, 10.0F);
    phantom_tracks.objects.push_back(TrackedObject{tmp_obj, 0.0, 0.0});
    tmp_obj.shape = make_rectangular_shape(make_pt(-50.0F, -20.0F, 100), 2.0F, 5.0F, 25.0F);
    phantom_tracks.objects.push_back(TrackedObject{tmp_obj, 0.0, 0.0});
  }
  ASSERT_EQ(phantom_tracks.objects.size(), num_nonassociated_rois);

  // Push the false positive projections to the roi array
  for (const auto & phantom_track : phantom_tracks.objects) {
    const auto maybe_projection =
      this->camera.project(expand_shape_to_vector(phantom_track.shape()));
    ASSERT_TRUE(maybe_projection);
    rois.rois.push_back(projection_to_roi(maybe_projection.value()));
  }

  auto result = this->associator.assign(rois, this->objects);

  for (auto i = 0U; i < result.track_assignments.size(); ++i) {
    if (i < num_noncaptured_tracks) {
      EXPECT_EQ(result.track_assignments[i], AssociatorResult::UNASSIGNED);
    } else {
      // After the first bundle of non-captured tracks, the rest of the tracks are perfectly
      // aligned with the true positive ROIs.
      const auto corresponding_roi_index = i - num_noncaptured_tracks;
      EXPECT_EQ(result.track_assignments[i], corresponding_roi_index);
    }
  }
  EXPECT_EQ(result.unassigned_track_indices.size(), num_noncaptured_tracks);
  EXPECT_EQ(result.unassigned_detection_indices.size(), num_nonassociated_rois);

  for (auto i = 0U; i < result.unassigned_track_indices.size(); ++i) {
    EXPECT_TRUE(result.unassigned_track_indices.find(i) != result.unassigned_track_indices.end());
  }

  for (auto i = 0U; i < result.unassigned_detection_indices.size(); ++i) {
    // Unassigned rois reside at the end of the array, after the assigned rois
    EXPECT_TRUE(
      result.unassigned_detection_indices.find(i + num_captured_tracks) != result
      .unassigned_detection_indices.end());
  }
}
