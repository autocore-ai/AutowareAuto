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
#include <tracking/greedy_roi_associator.hpp>
#include <tracking/projection.hpp>
#include <vector>
#include "tracking/test_utils.hpp"

using TrackedObject = autoware::perception::tracking::TrackedObject;
using TrackedObjects = std::vector<TrackedObject>;

using DetectedObjects = autoware_auto_msgs::msg::DetectedObjects;
using DetectedObject = autoware_auto_msgs::msg::DetectedObject;
using Shape = autoware_auto_msgs::msg::Shape;
using Projection = autoware::perception::tracking::Projection;
using CameraModel = autoware::perception::tracking::CameraModel;
using CameraIntrinsics = autoware::perception::tracking::CameraIntrinsics;
using ClassifiedRoi = autoware_auto_msgs::msg::ClassifiedRoi;
using ClassifiedRoiArray = autoware_auto_msgs::msg::ClassifiedRoiArray;
namespace tracking = autoware::perception::tracking;
using RoiAssociator = tracking::GreedyRoiAssociator;
using AssociatorResult = tracking::AssociatorResult;
using Point32 = geometry_msgs::msg::Point32;

Point32 make_pt(float32_t x, float32_t y, float32_t z)
{
  return Point32{}.set__x(x).set__y(y).set__z(z);
}

Shape make_rectangular_shape(
  const Point32 & base_face_origin,
  float32_t half_width, float32_t half_length, float32_t shape_height)
{
  Shape retval;
  retval.polygon.points.push_back(
    make_pt(
      (base_face_origin.x + half_width),
      (base_face_origin.y + half_length), (base_face_origin.z)));
  retval.polygon.points.push_back(
    make_pt(
      (base_face_origin.x + half_width),
      (base_face_origin.y - half_length), (base_face_origin.z)));
  retval.polygon.points.push_back(
    make_pt(
      (base_face_origin.x - half_width),
      (base_face_origin.y + half_length), (base_face_origin.z)));
  retval.polygon.points.push_back(
    make_pt(
      (base_face_origin.x - half_width),
      (base_face_origin.y - half_length), (base_face_origin.z)));
  retval.height = shape_height;

  return retval;
}

ClassifiedRoi projection_to_roi(const Projection & projection)
{
  ClassifiedRoi roi;
  for (const auto & pt : projection.shape) {
    roi.polygon.points.push_back(Point32{}.set__x(pt.x).set__y(pt.y));
  }
  return roi;
}

geometry_msgs::msg::Transform make_identity()
{
  geometry_msgs::msg::Transform identity{};
  identity.rotation.set__w(1.0);
  return identity;
}

template<typename ObjType>
class TestRoiAssociation : public testing::Test
{
public:
  TestRoiAssociation()
  : intrinsics{CameraIntrinsics{500U, 500U, 5.0F, 5.0F}},
    camera{intrinsics},
    associator{{intrinsics, 0.1F}} {}

  void add_object(
    const Point32 & base_face_origin,
    float32_t half_width, float32_t half_length, float32_t shape_height);

  Shape get_ith_shape(const size_t i);

  CameraIntrinsics intrinsics;
  CameraModel camera;
  RoiAssociator associator;
  ObjType objects;
};

template<>
void TestRoiAssociation<TrackedObjects>::add_object(
  const Point32 & base_face_origin,
  float32_t half_width,
  float32_t half_length,
  float32_t shape_height)
{
  DetectedObject object;
  object.shape = make_rectangular_shape(base_face_origin, half_width, half_length, shape_height);
  objects.emplace_back(object, 0.0, 0.0);
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
  assert(objects.size() > i);
  return objects[i].shape();
}

template<>
Shape TestRoiAssociation<DetectedObjects>::get_ith_shape(const size_t i)
{
  assert(objects.objects.size() > i);
  return objects.objects[i].shape;
}

using MyTypes = ::testing::Types<TrackedObjects, DetectedObjects>;
// NOTE: This is the older version due to 1.8.0 of GTest. v1.8.1 uses TYPED_TEST_SUITE
// cppcheck-suppress syntaxError
TYPED_TEST_CASE(TestRoiAssociation, MyTypes, );

TYPED_TEST(TestRoiAssociation, correct_association) {
  ClassifiedRoiArray rois;

  this->add_object(make_pt(10.0F, 10.0F, 10), 5.0F, 5.0F, 2.0F);
  const auto projection = this->camera.project(expand_shape_to_vector(this->get_ith_shape(0U)));
  ASSERT_TRUE(projection);
  rois.rois.push_back(projection_to_roi(projection.value()));
  const auto result = this->associator.assign(rois, this->objects, make_identity());
  ASSERT_EQ(result.track_assignments.front(), 0U);
  ASSERT_EQ(result.unassigned_detection_indices.size(), 0U);
  ASSERT_EQ(result.unassigned_track_indices.size(), 0U);
}

TYPED_TEST(TestRoiAssociation, out_of_image_test) {
  ClassifiedRoiArray rois;

  // Create a track behind the camera
  this->add_object(make_pt(10.0F, 10.0F, -10), 5.0F, 5.0F, 2.0F);
  const auto projection = this->camera.project(expand_shape_to_vector(this->get_ith_shape(0U)));
  ASSERT_FALSE(projection);
  const auto result = this->associator.assign(rois, this->objects, make_identity());
  ASSERT_EQ(result.track_assignments.front(), AssociatorResult::UNASSIGNED);
  ASSERT_EQ(result.unassigned_detection_indices.size(), 0U);
  ASSERT_TRUE(result.unassigned_track_indices.find(0U) != result.unassigned_track_indices.end());
}

TYPED_TEST(TestRoiAssociation, non_associated_track_roi) {
  ClassifiedRoiArray rois;

  this->add_object(make_pt(10.0F, 10.0F, 10), 5.0F, 5.0F, 2.0F);

  // Use the phantom track to create a detection that is not associated with the correct track.
  TrackedObjects phantom_tracks;
  {
    DetectedObject tmp_obj;
    tmp_obj.shape = make_rectangular_shape(make_pt(-50.0F, -20.0F, 100), 2.0F, 5.0F, 25.0F);
    phantom_tracks.push_back(TrackedObject{tmp_obj, 0.0, 0.0});
  }

  const auto projection = this->camera.project(expand_shape_to_vector(phantom_tracks[0].shape()));
  ASSERT_TRUE(projection);
  rois.rois.push_back(projection_to_roi(projection.value()));
  const auto result = this->associator.assign(rois, this->objects, make_identity());
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
TYPED_TEST(TestRoiAssociation, combined_association_test) {
  constexpr auto num_captured_tracks = 4U;
  constexpr auto num_noncaptured_tracks = 3U;
  constexpr auto num_nonassociated_rois = 3U;
  TrackedObjects phantom_tracks;  // Only used to create false-positive ROIs
  ClassifiedRoiArray rois;

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
    phantom_tracks.push_back(TrackedObject{tmp_obj, 0.0, 0.0});
    tmp_obj.shape = make_rectangular_shape(make_pt(-20.0F, -10.0F, 50), 15.0F, 25.0F, 10.0F);
    phantom_tracks.push_back(TrackedObject{tmp_obj, 0.0, 0.0});
    tmp_obj.shape = make_rectangular_shape(make_pt(-50.0F, -20.0F, 100), 2.0F, 5.0F, 25.0F);
    phantom_tracks.push_back(TrackedObject{tmp_obj, 0.0, 0.0});
  }
  ASSERT_EQ(phantom_tracks.size(), num_nonassociated_rois);

  // Push the false positive projections to the roi array
  for (const auto & phantom_track : phantom_tracks) {
    const auto maybe_projection =
      this->camera.project(expand_shape_to_vector(phantom_track.shape()));
    ASSERT_TRUE(maybe_projection);
    rois.rois.push_back(projection_to_roi(maybe_projection.value()));
  }

  auto result = this->associator.assign(rois, this->objects, make_identity());

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
