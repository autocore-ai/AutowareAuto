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

#include <gtest/gtest.h>

#include <tracking_test_framework/shapes.hpp>

#include <vector>
namespace
{
// Tolerance value to be used for floating point comparisons
constexpr auto epsilon = 1e-3F;
}  // namespace

// Test for line intersection with line
TEST(test_tracking_test_framework, test_line_intersection_with_line) {
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
TEST(test_tracking_test_framework, test_line_intersection_with_rectangle) {
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
TEST(test_tracking_test_framework, test_line_intersection_with_circle) {
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
