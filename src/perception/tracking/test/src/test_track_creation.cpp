// Copyright 2021 The Autoware Foundation
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//
// Co-developed by Tier IV, Inc. and Apex.AI, Inc.

#include <utility>
#include <vector>

#include "gtest/gtest.h"
#include "time_utils/time_utils.hpp"
#include "tracking/test_utils.hpp"
#include "tracking/track_creator.hpp"
#include "tracking/tracked_object.hpp"

using AssociatorResult = autoware::perception::tracking::AssociatorResult;
using CameraModel = autoware::perception::tracking::CameraModel;
using CameraIntrinsics = autoware::perception::tracking::CameraIntrinsics;
using ClassifiedRoi = autoware_auto_msgs::msg::ClassifiedRoi;
using ClassifiedRoiArray = autoware_auto_msgs::msg::ClassifiedRoiArray;
using CreationPolicies = autoware::perception::tracking::TrackCreationPolicy;
using DetectedObject = autoware_auto_msgs::msg::DetectedObject;
using DetectedObjects = autoware_auto_msgs::msg::DetectedObjects;
using TrackCreator = autoware::perception::tracking::TrackCreator;
using TrackedObject = autoware::perception::tracking::TrackedObject;
using VisionPolicyConfig = autoware::perception::tracking::VisionPolicyConfig;

namespace
{
constexpr autoware::common::types::bool8_t kIsStatic = true;

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

class TestTrackCreator : public testing::Test
{
public:
  TestTrackCreator()
  : image_width{500U},
    image_heigth{500U},
    intrinsics{CameraIntrinsics{image_width, image_heigth, 5.0F, 5.0F,
        static_cast<float32_t>(image_width) / 2.0F, static_cast<float32_t>(image_heigth) / 2.0F}},
    vision_policy_cfg{{intrinsics, 0.1F}, std::chrono::milliseconds{20}},
    camera{intrinsics}
  {
    const auto get_roi_from_detection = [this](const DetectedObject & obj) -> ClassifiedRoi
      {
        const auto maybe_projection = this->camera.project(expand_shape_to_vector(obj.shape));
        assert(maybe_projection);
        return projection_to_roi(maybe_projection.value());
      };
    // Construct object roi pairs
    // First pair
    {
      DetectedObject obj1;
      obj1.shape = make_rectangular_shape(make_pt(10.0F, 10.0F, 10), 5.0F, 5.0F, 2.0F);
      ClassifiedRoi roi1;
      roi1 = get_roi_from_detection(obj1);
      object_roi_pairs.emplace_back(std::make_pair(obj1, roi1));

      // Second pair
      DetectedObject obj2;
      obj2.shape = make_rectangular_shape(make_pt(20.0F, 10.0F, 50), 15.0F, 25.0F, 10.0F);
      ClassifiedRoi roi2;
      roi2 = get_roi_from_detection(obj2);
      object_roi_pairs.emplace_back(std::make_pair(obj2, roi2));

      // Third pair
      DetectedObject obj3;
      obj3.shape = make_rectangular_shape(make_pt(50.0F, 20.0F, 100), 2.0F, 5.0F, 25.0F);
      ClassifiedRoi roi3;
      roi3 = get_roi_from_detection(obj3);
      object_roi_pairs.emplace_back(std::make_pair(obj3, roi3));
    }

    // Construct unmatched objects and rois
    {
      DetectedObject obj1;
      obj1.shape = make_rectangular_shape(make_pt(-10.0F, -10.0F, 10), 5.0F, 5.0F, 2.0F);
      ClassifiedRoi roi1;
      roi1 = get_roi_from_detection(obj1);
      unmatched_objects.emplace_back(obj1);
      unmatched_rois.emplace_back(roi1);

      DetectedObject obj2;
      obj2.shape = make_rectangular_shape(make_pt(-20.0F, -10.0F, 50), 15.0F, 25.0F, 10.0F);
      ClassifiedRoi roi2;
      roi2 = get_roi_from_detection(obj2);
      unmatched_objects.emplace_back(obj2);
      unmatched_rois.emplace_back(roi2);
    }
  }

  std::size_t image_width;
  std::size_t image_heigth;
  CameraIntrinsics intrinsics;
  VisionPolicyConfig vision_policy_cfg;
  CameraModel camera;
  std::vector<std::pair<DetectedObject, ClassifiedRoi>> object_roi_pairs;
  std::vector<DetectedObject> unmatched_objects;
  std::vector<ClassifiedRoi> unmatched_rois;
  tf2::BufferCore tf_buffer;
};

TEST(TrackCreatorTest, TestLidarOnly)
{
  tf2::BufferCore tf_buffer;
  TrackCreator creator{{CreationPolicies::LidarClusterOnly, 1.0, 1.0}, tf_buffer};
  DetectedObject obj;
  DetectedObjects objs;
  const int num_objects = 10;
  for (int i = 0; i < num_objects; ++i) {
    objs.objects.push_back(obj);
  }

  AssociatorResult result;
  result.unassigned_detection_indices = {0, 2, 4};

  creator.add_objects(objs, result);

  EXPECT_EQ(creator.create_tracks().tracks.size(), 3U);
  EXPECT_EQ(creator.create_tracks().detections_leftover.objects.size(), 0U);
}

// Test lidar and vision with two matches between them
TEST_F(TestTrackCreator, TestLidarIfVision2NewTracks)
{
  TrackCreator creator{
    {CreationPolicies::LidarClusterIfVision, 1.0, 1.0, this->vision_policy_cfg}, tf_buffer};
  auto now_time = time_utils::to_message(
    std::chrono::system_clock::time_point{std::chrono::system_clock::now()});

  // Add lidar
  DetectedObject lidar_detection;
  DetectedObjects lidar_detections;
  lidar_detections.header.stamp = now_time;
  lidar_detections.header.frame_id = "base_link";
  const int num_objects = 10;
  for (int i = 0; i < num_objects; ++i) {
    lidar_detections.objects.push_back(lidar_detection);
  }
  lidar_detections.objects[0] = this->object_roi_pairs[0].first;
  lidar_detections.objects[2] = this->object_roi_pairs[1].first;
  lidar_detections.objects[4] = this->unmatched_objects[1];
  AssociatorResult lidar_track_assignment;
  lidar_track_assignment.unassigned_detection_indices = {0, 2, 4};
  creator.add_objects(lidar_detections, lidar_track_assignment);

  // Add vision
  ClassifiedRoi vision_detection;
  ClassifiedRoiArray vision_detections;
  vision_detections.header.stamp = time_utils::to_message(
    time_utils::from_message(now_time) + std::chrono::milliseconds(15));
  vision_detections.header.frame_id = "camera";
  const int num_vision_detections = 6;
  for (int i = 0; i < num_vision_detections; ++i) {
    vision_detections.rois.push_back(vision_detection);
  }
  AssociatorResult vision_track_assignment;
  vision_track_assignment.unassigned_detection_indices = {1, 3, 5};
  vision_detections.rois[1] = this->object_roi_pairs[1].second;
  vision_detections.rois[3] = this->object_roi_pairs[0].second;
  vision_detections.rois[5] = this->unmatched_rois[0];
  creator.add_objects(vision_detections, vision_track_assignment);

  const auto tf = create_identity_transform(
    vision_detections.header.frame_id,
    lidar_detections.header.frame_id,
    vision_detections.header.stamp);
  tf_buffer.setTransform(tf, "test_authority", kIsStatic);

  // Test
  const auto ret = creator.create_tracks();
  EXPECT_EQ(ret.tracks.size(), 2U);
  EXPECT_TRUE(
    std::find_if(
      ret.tracks.begin(), ret.tracks.end(), [this](const TrackedObject & t) {
        return t.shape() == this->object_roi_pairs[0].first.shape;
      }) != ret.tracks.end());
  EXPECT_TRUE(
    std::find_if(
      ret.tracks.begin(), ret.tracks.end(), [this](const TrackedObject & t) {
        return t.shape() == this->object_roi_pairs[1].first.shape;
      }) != ret.tracks.end());

  EXPECT_EQ(ret.detections_leftover.objects.size(), 1U);
  EXPECT_EQ(ret.detections_leftover.objects[0U], lidar_detections.objects[4]);
  EXPECT_EQ(ret.detections_leftover.objects[0U].shape, this->unmatched_objects[1].shape);
}

// Test lidar and vision but no match between them
TEST_F(TestTrackCreator, TestLidarIfVisionNoNewTrack)
{
  TrackCreator creator{
    {CreationPolicies::LidarClusterIfVision, 1.0, 1.0, this->vision_policy_cfg}, tf_buffer};
  auto now_time = time_utils::to_message(
    std::chrono::system_clock::time_point{std::chrono::system_clock::now()});

  // Add lidar
  DetectedObject lidar_detection;
  DetectedObjects lidar_detections;
  lidar_detections.header.frame_id = "base_link";
  lidar_detections.header.stamp = now_time;
  const int num_objects = 10;
  for (int i = 0; i < num_objects; ++i) {
    lidar_detection.shape.height = i;  // use index as height to differentiate between detections
    lidar_detections.objects.push_back(lidar_detection);
  }
  AssociatorResult lidar_track_assignment;
  lidar_track_assignment.unassigned_detection_indices = {0, 2, 4};
  lidar_detections.objects[0] = this->object_roi_pairs[0].first;
  lidar_detections.objects[2] = this->object_roi_pairs[1].first;
  lidar_detections.objects[4] = this->object_roi_pairs[2].first;
  creator.add_objects(lidar_detections, lidar_track_assignment);

  // Add vision
  ClassifiedRoi vision_detection;
  ClassifiedRoiArray vision_detections;
  vision_detections.header.stamp = time_utils::to_message(
    time_utils::from_message(now_time) - std::chrono::milliseconds(15));
  vision_detections.header.frame_id = "camera";
  const int num_vision_detections = 5;
  for (int i = 0; i < num_vision_detections; ++i) {
    vision_detections.rois.push_back(vision_detection);
  }
  AssociatorResult vision_track_assignment;
  vision_track_assignment.unassigned_detection_indices = {1, 3};
  vision_detections.rois[1] = this->unmatched_rois[0];
  vision_detections.rois[3] = this->unmatched_rois[1];

  // Add vision that are older than the previous one
  ClassifiedRoiArray vision_detections_old1;
  vision_detections_old1.header.stamp = time_utils::to_message(
    time_utils::from_message(now_time) - std::chrono::milliseconds(17));
  vision_detections.header.frame_id = "camera";

  // Add vision that is out of range
  ClassifiedRoiArray vision_detections_old2;
  vision_detections_old1.header.stamp = time_utils::to_message(
    time_utils::from_message(now_time) - std::chrono::milliseconds(27));
  vision_detections.header.frame_id = "camera";

  creator.add_objects(vision_detections_old2, AssociatorResult{});
  creator.add_objects(vision_detections_old1, AssociatorResult{});
  creator.add_objects(vision_detections, vision_track_assignment);

  const auto tf = create_identity_transform(
    vision_detections.header.frame_id,
    lidar_detections.header.frame_id,
    vision_detections.header.stamp);
  tf_buffer.setTransform(tf, "test_authority", kIsStatic);

  // Test
  const auto ret = creator.create_tracks();
  EXPECT_EQ(ret.tracks.size(), 0U);
  EXPECT_EQ(ret.detections_leftover.objects.size(), 3U);
}

// No vision message within time range
TEST_F(TestTrackCreator, TestLidarIfVisionOutOfTimeRange)
{
  TrackCreator creator{
    {CreationPolicies::LidarClusterIfVision, 1.0, 1.0, this->vision_policy_cfg}, tf_buffer};
  auto now_time = time_utils::to_message(
    std::chrono::system_clock::time_point{std::chrono::system_clock::now()});

  // Add lidar
  DetectedObject lidar_detection;
  DetectedObjects lidar_detections;
  lidar_detections.header.frame_id = "base_link";
  lidar_detections.header.stamp = now_time;
  const int num_objects = 10;
  for (int i = 0; i < num_objects; ++i) {
    lidar_detection.shape.height = i;  // use index as height to differentiate between detections
    lidar_detections.objects.push_back(lidar_detection);
  }
  AssociatorResult lidar_track_assignment;
  lidar_track_assignment.unassigned_detection_indices = {0, 2, 4};
  creator.add_objects(lidar_detections, lidar_track_assignment);

  // Add vision
  ClassifiedRoi vision_detection;
  ClassifiedRoiArray vision_detections;
  vision_detections.header.stamp = time_utils::to_message(
    time_utils::from_message(now_time) + std::chrono::milliseconds(555));
  vision_detections.header.frame_id = "camera";
  const int num_vision_detections = 5;
  for (int i = 0; i < num_vision_detections; ++i) {
    vision_detections.rois.push_back(vision_detection);
  }
  AssociatorResult vision_track_assignment;
  vision_track_assignment.unassigned_detection_indices = {1, 3};
  creator.add_objects(vision_detections, vision_track_assignment);

  const auto tf = create_identity_transform(
    vision_detections.header.frame_id,
    lidar_detections.header.frame_id,
    vision_detections.header.stamp);
  tf_buffer.setTransform(tf, "test_authority", kIsStatic);

  // Test
  const auto ret = creator.create_tracks();
  EXPECT_EQ(ret.tracks.size(), 0U);
}
