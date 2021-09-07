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

#ifndef TEST_BOUNDING_BOX_COMPUTATION_HPP_
#define TEST_BOUNDING_BOX_COMPUTATION_HPP_

#include <euclidean_cluster/euclidean_cluster.hpp>

#include <vector>

#include "gtest/gtest.h"

using Clusters = autoware_auto_msgs::msg::PointClusters;
using BoundingBox = autoware_auto_msgs::msg::BoundingBox;
using BoundingBoxArray = autoware_auto_msgs::msg::BoundingBoxArray;
using DetectedObjects = autoware_auto_msgs::msg::DetectedObjects;
using DetectedObject = autoware_auto_msgs::msg::DetectedObject;
using Pt = autoware_auto_msgs::msg::PointXYZIF;

using autoware::perception::segmentation::euclidean_cluster::details::compute_bounding_boxes;
using autoware::perception::segmentation::euclidean_cluster::details::BboxMethod;
using autoware::perception::segmentation::euclidean_cluster::details::convert_to_detected_objects;

class BoundingBoxComputationTest : public ::testing::Test
{
protected:
  Pt make_pt(const float x, const float y, const float z = 0.0F)
  {
    Pt ret;
    ret.x = x;
    ret.y = y;
    ret.z = z;
    return ret;
  }

  Clusters make_clusters(std::vector<std::vector<Pt>> points_list)
  {
    Clusters ret;
    size_t boundary_idx = 0U;
    for (const auto & list : points_list) {
      boundary_idx += list.size();
      for (const auto & pt : list) {
        ret.points.push_back(pt);
      }
      ret.cluster_boundary.push_back(boundary_idx);
    }
    return ret;
  }

  void test_corners(
    const BoundingBox & box, const std::vector<Pt> & expect,
    const float TOL = 1.0E-6F)
  {
    for (uint32_t idx = 0U; idx < 4U; ++idx) {
      bool found = false;
      for (auto & p : expect) {
        if (fabsf(p.x - box.corners[idx].x) < TOL && fabsf(p.y - box.corners[idx].y) < TOL) {
          found = true;
          break;
        }
      }
      ASSERT_TRUE(found) << idx << ": " << box.corners[idx].x << ", " << box.corners[idx].y;
    }
  }

  void test_corners(
    const DetectedObject & object, const std::vector<Pt> & expect,
    const float TOL = 1.0E-6F)
  {
    ASSERT_EQ(object.shape.polygon.points.size(), 4U);
    for (uint32_t idx = 0U; idx < 4U; ++idx) {
      const auto & actual = object.shape.polygon.points[idx];
      bool found = false;
      for (auto & p : expect) {
        if (fabsf(p.x - actual.x) < TOL && fabsf(p.y - actual.y) < TOL && fabsf(p.z - actual.z) <
          TOL)
        {
          found = true;
          break;
        }
      }
      ASSERT_TRUE(found) << idx << ": " << actual.x << ", " << actual.y;
    }
  }

  void test_object_msg(
    const DetectedObject & object, const std::vector<Pt> & corners, const float
    TOL = 1.0E-6F)
  {
    test_corners(object, corners, TOL);
    EXPECT_EQ(object.existence_probability, 1.0F);
    ASSERT_EQ(object.classification.size(), 1U);
    EXPECT_EQ(
      object.classification[0U].classification,
      autoware_auto_msgs::msg::ObjectClassification::UNKNOWN);
    EXPECT_EQ(object.classification[0U].probability, 1.0F);
  }

  std::vector<Pt> pt_vector{make_pt(0.F, 0.F), make_pt(0.F, 1.F),
    make_pt(-1.F, 2.F), make_pt(0.F, 3.F),
    make_pt(0.F, 4.F), make_pt(1.F, 0.F), make_pt(2.F, -1.F),
    make_pt(3.F, 0.F), make_pt(4.F, 0.F)};
  std::vector<Pt> lfit_expected_corners{make_pt(4, -1), make_pt(-1, 4),
    make_pt(4, 4), make_pt(-1, -1)};
  std::vector<Pt> eigen_expected_corners{make_pt(4, 0), make_pt(0, 4),
    make_pt(-2, 2), make_pt(2, -2)};
};

// Actual points copied from autoware_auto_geometry/test/test_bounding_box.hpp.
// This test needs to make sure cluster points are navigated and passed to bounding box
// computation properly and, in case of DetectedObjects, needs to make sure BoundingBox is copied
// over to DetectedObjects properly.
TEST_F(BoundingBoxComputationTest, BasicLfit2d)
{
  auto clusters = make_clusters(
    {pt_vector, pt_vector});

  BoundingBoxArray boxes_msg = compute_bounding_boxes(clusters, BboxMethod::LFit, false);
  ASSERT_EQ(boxes_msg.boxes.size(), 2U);
  for (const auto & box : boxes_msg.boxes) {
    test_corners(box, lfit_expected_corners, 0.25F);
  }

  DetectedObjects objects_msg = convert_to_detected_objects(boxes_msg);
  ASSERT_EQ(objects_msg.objects.size(), 2U);
  for (const auto & object : objects_msg.objects) {
    test_object_msg(object, lfit_expected_corners, 0.25F);
  }
}

TEST_F(BoundingBoxComputationTest, BasicEigen2d) {
  auto clusters = make_clusters(
    {pt_vector, pt_vector});

  BoundingBoxArray boxes_msg = compute_bounding_boxes(clusters, BboxMethod::Eigenbox, false);
  ASSERT_EQ(boxes_msg.boxes.size(), 2U);
  for (const auto & box : boxes_msg.boxes) {
    test_corners(box, eigen_expected_corners, 0.25F);
  }

  DetectedObjects objects_msg = convert_to_detected_objects(boxes_msg);
  ASSERT_EQ(objects_msg.objects.size(), 2U);
  for (const auto & object : objects_msg.objects) {
    test_object_msg(object, eigen_expected_corners, 0.25F);
  }
}

TEST_F(BoundingBoxComputationTest, BasicLfit3d)
{
  auto pt_vector_3d = pt_vector;
  pt_vector_3d[0U].z = -2.F;
  pt_vector_3d[1U].z = 2.F;
  auto clusters = make_clusters({pt_vector_3d, pt_vector_3d});

  BoundingBoxArray boxes_msg = compute_bounding_boxes(clusters, BboxMethod::LFit, true);
  DetectedObjects objects_msg = convert_to_detected_objects(boxes_msg);

  auto expected_corners_3d = lfit_expected_corners;
  for (auto & pt : expected_corners_3d) {
    pt.z = -2.F;
  }

  ASSERT_EQ(objects_msg.objects.size(), 2U);
  for (const auto & object : objects_msg.objects) {
    test_object_msg(object, expected_corners_3d, 0.25F);
    EXPECT_EQ(object.shape.height, 4.0F);
  }
}

TEST_F(BoundingBoxComputationTest, BasicEigen3d)
{
  auto pt_vector_3d = pt_vector;
  pt_vector_3d[0U].z = -2.F;
  pt_vector_3d[1U].z = 2.F;
  auto clusters = make_clusters({pt_vector_3d, pt_vector_3d});

  BoundingBoxArray boxes_msg = compute_bounding_boxes(clusters, BboxMethod::Eigenbox, true);
  DetectedObjects objects_msg = convert_to_detected_objects(boxes_msg);

  auto expected_corners_3d = eigen_expected_corners;
  for (auto & pt : expected_corners_3d) {
    pt.z = -2.F;
  }

  ASSERT_EQ(objects_msg.objects.size(), 2U);
  for (const auto & object : objects_msg.objects) {
    test_object_msg(object, expected_corners_3d, 0.25F);
    EXPECT_EQ(object.shape.height, 4.0F);
  }
}


#endif   // TEST_BOUNDING_BOX_COMPUTATION_HPP_
