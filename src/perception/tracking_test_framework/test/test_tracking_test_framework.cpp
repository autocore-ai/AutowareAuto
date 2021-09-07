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

#include <gtest/gtest.h>

#include <geometry/common_2d.hpp>
#include <tracking_test_framework/scene.hpp>

#include <memory>
#include <utility>
#include <vector>

namespace
{
// Tolerance value to be used for floating point comparisons
constexpr auto epsilon = 1e-3F;
}  // namespace

// Test for line intersection with line
TEST(TestTrackingTestFramework, TestLineIntersectionWithLine) {
  autoware::tracking_test_framework::Line l1 {
    Eigen::Vector2f{-1.0F, -1.0F}, Eigen::Vector2f{2.0F, 0.5F}};
  autoware::tracking_test_framework::Line l2 {
    Eigen::Vector2f{1.0F, 0.0F}, Eigen::Vector2f{1.0F, 2.0F}};
  EigenStlVector<Eigen::Vector2f> intersection_1 = l1.intersect_with_line(l2, true);
  ASSERT_EQ(intersection_1.size(), 1UL);
  EXPECT_NEAR(intersection_1[0].x(), 1.0F, epsilon);
  EXPECT_NEAR(intersection_1[0].y(), 0.0F, epsilon);

  // Since line can intersect with line only at one point setting closest point only to false
  // should still return same result
  EigenStlVector<Eigen::Vector2f> intersection_2 = l1.intersect_with_line(l2, false);
  ASSERT_EQ(intersection_2.size(), 1UL);
  EXPECT_NEAR(intersection_2[0].x(), 1.0F, epsilon);
  EXPECT_NEAR(intersection_2[0].y(), 0.0F, epsilon);

  // Intersection of parallel lines should return empty vector
  autoware::tracking_test_framework::Line l3 {
    Eigen::Vector2f{-1.0F, -3.0F}, Eigen::Vector2f{1.0F, 1.0F}};
  autoware::tracking_test_framework::Line l4 {
    Eigen::Vector2f{-3.0F, -2.0F}, Eigen::Vector2f{-1.0F, 2.0F}};
  EXPECT_TRUE(l3.intersect_with_line(l4, true).empty());

  // Non parallel lines which do not intersect should return empty vector
  autoware::tracking_test_framework::Line l5 {
    Eigen::Vector2f{-1.0F, -1.0F}, Eigen::Vector2f{-1.0F, 1.5F}};
  autoware::tracking_test_framework::Line l6 {
    Eigen::Vector2f{1.0F, -0.5F}, Eigen::Vector2f{-0.5F, 1.0F}};
  EXPECT_TRUE(l5.intersect_with_line(l6, true).empty());
}

// Test for line intersection with rectangle
TEST(TestTrackingTestFramework, TestLineIntersectionWithRectangle) {
  autoware::tracking_test_framework::Rectangle rect {
    Eigen::Vector2f{0.0F, 0.0F}, Eigen::Vector2f{2.0F, 3.0F}, 0.0F};
  autoware::tracking_test_framework::Line l1 {
    Eigen::Vector2f{-2.0F, 1.0F}, Eigen::Vector2f{2.0F, 1.0F}};
  EigenStlVector<Eigen::Vector2f> intersection_1 = rect.intersect_with_line(l1, true);
  ASSERT_EQ(intersection_1.size(), 1UL);
  EXPECT_NEAR(intersection_1[0].x(), -1.0F, epsilon);
  EXPECT_NEAR(intersection_1[0].y(), 1.0F, epsilon);

  // Check if multiple intersections returned when closest_point is set to false
  EigenStlVector<Eigen::Vector2f> intersection_2 = rect.intersect_with_line(l1, false);
  ASSERT_GT(intersection_2.size(), 1UL);
  EXPECT_NEAR(intersection_2[0].x(), 1.0F, epsilon);
  EXPECT_NEAR(intersection_2[0].y(), 1.0F, epsilon);
  EXPECT_NEAR(intersection_2[1].x(), -1.0F, epsilon);
  EXPECT_NEAR(intersection_2[1].y(), 1.0F, epsilon);

  // Check non intersecting line case it should return empty vector
  autoware::tracking_test_framework::Line l2 {
    Eigen::Vector2f{-2.0F, 0.0F}, Eigen::Vector2f{-2.0F, -1.5F}};
  EXPECT_TRUE(rect.intersect_with_line(l2, true).empty());
}

// Test for line intersection with circle
TEST(TestTrackingTestFramework, TestLineIntersectionWithCircle) {
  autoware::tracking_test_framework::Circle circle {Eigen::Vector2f {1.0F, 1.0F}, 1.0F};

  // Intersection with line vertical to circle
  autoware::tracking_test_framework::Line vertical_line {
    Eigen::Vector2f{0.0F, 0.0F}, Eigen::Vector2f{0.0F, 2.0F}};
  EigenStlVector<Eigen::Vector2f> intersection_1 = circle.intersect_with_line(vertical_line, true);
  ASSERT_EQ(intersection_1.size(), 1UL);
  EXPECT_NEAR(intersection_1[0].x(), 0.0F, epsilon);
  EXPECT_NEAR(intersection_1[0].y(), 1.0F, epsilon);

  // Intersection with line horizontal to circle
  autoware::tracking_test_framework::Line horizontal_line {
    Eigen::Vector2f{0, 0}, Eigen::Vector2f{2, 0}};
  EigenStlVector<Eigen::Vector2f> intersection_2 =
    circle.intersect_with_line(horizontal_line, true);
  ASSERT_EQ(intersection_2.size(), 1UL);
  EXPECT_NEAR(intersection_2[0].x(), 1.0F, epsilon);
  EXPECT_NEAR(intersection_2[0].y(), 0.0F, epsilon);

  // Intersection with line diagonal to circle
  autoware::tracking_test_framework::Line diagonal_line {
    Eigen::Vector2f{0, 0}, Eigen::Vector2f{2, 2}};
  EigenStlVector<Eigen::Vector2f> intersection_3 = circle.intersect_with_line(diagonal_line, true);
  ASSERT_EQ(intersection_3.size(), 1UL);
  EXPECT_NEAR(intersection_3[0].x(), 0.2928932F, epsilon);
  EXPECT_NEAR(intersection_3[0].y(), 0.2928932F, epsilon);

  // Get second intersection point of line diagonal to circle
  EigenStlVector<Eigen::Vector2f> intersection_4 = circle.intersect_with_line(diagonal_line, false);
  ASSERT_EQ(intersection_4.size(), 2UL);
  EXPECT_NEAR(intersection_4[0].x(), 0.2928932F, epsilon);
  EXPECT_NEAR(intersection_4[0].y(), 0.2928932F, epsilon);
  EXPECT_NEAR(intersection_4[1].x(), 1.7071067F, epsilon);
  EXPECT_NEAR(intersection_4[1].y(), 1.7071067F, epsilon);
}


TEST(TestTrackingTestFramework, TestSceneWithSingleLidarBeamSingleObject) {
  // Lidar with single beam
  autoware::tracking_test_framework::Lidar lidar{Eigen::Vector2f{0.0, 0.0}, 1, 2.0};

  std::vector<std::unique_ptr<autoware::tracking_test_framework::TrackedObject>> objects;
  // Tracked Pedestrian cluster
  objects.emplace_back(
    std::make_unique<autoware::tracking_test_framework::Pedestrian>(
      Eigen::Vector2f{1.0, 1.0}, 2, 0.0, 0.0));

  // Setup scene
  autoware::tracking_test_framework::Scene scene{lidar, std::move(objects)};
  auto intersection = scene.get_detected_objects_array(true);
  ASSERT_EQ(intersection.objects.size(), 1UL);
  EXPECT_NEAR(intersection.objects[0].shape.polygon.points.front().x, 1.0F, epsilon);
  EXPECT_NEAR(intersection.objects[0].shape.polygon.points.front().y, 0.0F, epsilon);

  // When object is moved ahead in the time dt, it goes out of range of lidar beam
  scene.move_all_objects(std::chrono::seconds(1));
  auto new_intersection = scene.get_detected_objects_array(true);

  EXPECT_TRUE(new_intersection.objects.empty());
}

TEST(TestTrackingTestFramework, TestSceneWithMultipleLidarBeamsMultipleObject) {
  // Lidar with multiple beams
  autoware::tracking_test_framework::Lidar lidar{Eigen::Vector2f{0.0, 0.0}, 4, 2.0};

  std::vector<std::unique_ptr<autoware::tracking_test_framework::TrackedObject>> objects;
  // Tracked Car cluster
  objects.emplace_back(
    std::make_unique<autoware::tracking_test_framework::Car>(
      Eigen::Vector2f{0.0, 0.0}, 3, 0.0, 0.0, Eigen::Vector2f{2.0, 3.0}));
  // Tracked Pedestrian cluster
  objects.emplace_back(
    std::make_unique<autoware::tracking_test_framework::Pedestrian>(
      Eigen::Vector2f{1.0, 1.0}, 2, 0.0, 0.0));

  // Setup scene
  autoware::tracking_test_framework::Scene scene{lidar, std::move(objects)};

  // Even though pedestrian and car are hit by lidar at a common point here we return closest
  // object to the ray only since the farther object should be occluded by using closest_only flag
  auto intersections = scene.get_detected_objects_array(true);
  ASSERT_EQ(intersections.objects.size(), 1UL);

  // Intersection points with lidar
  geometry_msgs::msg::Polygon polygon{};
  std::vector<Eigen::Vector2f> intersection_pts {Eigen::Vector2f{1.0F, 0.0F},
    Eigen::Vector2f{-0.8660F, 1.5F}, Eigen::Vector2f{-0.8660F, -1.5F},
    Eigen::Vector2f{1.0F, 1.748455e-7F}};
  std::vector<autoware::common::types::PointXYZIF> points_vec{};

  for (const auto & point : intersection_pts) {
    points_vec.push_back(
      autoware::tracking_test_framework::utils::
      get_point_from_vector_2d(point));
  }

  // Assert if the intersection points are lying on or inside the fitted bounding box
  for (const auto & point : points_vec) {
    geometry_msgs::msg::Point32 pt;
    pt.x = point.x;
    pt.y = point.y;
    pt.z = 0.0F;
    ASSERT_TRUE(
      autoware::common::geometry::is_point_inside_polygon_2d(
        intersections.objects[0].shape.polygon.points.begin(),
        intersections.objects[0].shape.polygon.points.end(), pt));
  }

  // Check if the area of the fitted bounding box is less than equal to the area of the 2D tracked
  // object here the car.
  const float bbox_area = autoware::common::geometry::area_2d(
    intersections.objects[0].shape.polygon.points.begin(),
    intersections.objects[0].shape.polygon.points.end());
  EXPECT_GT(6.0F, bbox_area);
}

TEST(TestTrackingTestFramework, Test1232FixScalingFactor) {
  autoware::tracking_test_framework::Lidar lidar{Eigen::Vector2f{0.0, 0.0}, 720, 200.0};
  std::vector<std::unique_ptr<autoware::tracking_test_framework::TrackedObject>> objects;
// Tracked Car cluster
  objects.emplace_back(
    std::make_unique<autoware::tracking_test_framework::Car>(
      Eigen::Vector2f{100.0, 100.0}, 3, 0.0, 0.0, Eigen::Vector2f{2.0, 3.0}));
  autoware::tracking_test_framework::Scene scene{lidar, std::move(objects)};
  auto detections_msg = scene.get_detected_objects_array(true);
  ASSERT_EQ(detections_msg.objects.size(), 1U);
}
