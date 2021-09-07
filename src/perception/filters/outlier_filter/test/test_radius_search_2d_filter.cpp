// Copyright 2021 Tier IV, Inc
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

#include <vector>
#include <memory>

#include "gtest/gtest.h"
#include "pcl_conversions/pcl_conversions.h"

#include "outlier_filter/radius_search_2d_filter.hpp"
#include "outlier_filter_test_utils.hpp"


using RadiusSearch2DFilter =
  autoware::perception::filters::outlier_filter::radius_search_2d_filter::RadiusSearch2DFilter;

// TEST METHODS
/* TEST 1: A single point pointcloud
 *
 *   x -> removed
 *
 */
TEST(RadiusSearch2DFilter, TestSinglePoint) {
  auto filter = std::make_shared<RadiusSearch2DFilter>(1.0, 5);
  std::vector<pcl::PointXYZ> points = {
    make_point(0.0f, 0.0f, 0.0f)};
  auto time0 = std::chrono::system_clock::now();
  auto t0 = to_msg_time(time0);
  auto input = make_pc(points, t0);

  // Run the filter
  pcl::PointCloud<pcl::PointXYZ> output;
  filter->filter(input, output);

  // Perform checks on the output pointcloud
  // For this test the single pointcloud is considered an outlier and will be removed
  check_pc({}, output);
}

/* TEST 2: A simple radial pointcloud
 *   x        x
 * x x x -> x x x
 *   x        x
 */
TEST(RadiusSearch2DFilter, TestSimpleCloud) {
  auto filter = std::make_shared<RadiusSearch2DFilter>(1.0, 5);
  std::vector<pcl::PointXYZ> points = {
    make_point(0.0f, 0.0f, 0.0f),
    make_point(0.2f, 0.0f, 0.0f),
    make_point(0.0f, 0.2f, 0.0f),
    make_point(-0.2f, 0.0f, 0.0f),
    make_point(0.0f, -0.2f, 0.0f)};
  auto time0 = std::chrono::system_clock::now();
  auto t0 = to_msg_time(time0);
  auto input = make_pc(points, t0);

  // Run the filter
  pcl::PointCloud<pcl::PointXYZ> output;
  filter->filter(input, output);

  // Perform checks on the output pointcloud
  // For this test the pointcloud should remain the same
  check_pc(points, output);
}

/* TEST 3: A simple radial pointcloud with one outlier
 *   x       x        x
 * x x x         -> x x x
 *   x                x
 */
TEST(RadiusSearch2DFilter, TestOutlierPoint) {
  auto filter = std::make_shared<RadiusSearch2DFilter>(0.5, 5);
  std::vector<pcl::PointXYZ> points = {
    make_point(0.0f, 0.0f, 0.0f),
    make_point(0.2f, 0.0f, 0.0f),
    make_point(0.0f, 0.2f, 0.0f),
    make_point(-0.2f, 0.0f, 0.0f),
    make_point(0.0f, -0.2f, 0.0f),
    make_point(0.8f, 0.2f, 0.0f)};
  auto time0 = std::chrono::system_clock::now();
  auto t0 = to_msg_time(time0);
  auto input = make_pc(points, t0);

  // Run the filter
  pcl::PointCloud<pcl::PointXYZ> output;
  filter->filter(input, output);

  // Perform checks on the output pointcloud
  // For this test the single pointcloud is considered an outlier and will be removed
  // Pop the last element since it is the outlier
  points.pop_back();
  check_pc(points, output);
}

/* TEST 4: A simple radial pointcloud, minimum number of neighbours increased
 *   x
 * x x x -> all removed
 *   x
 */
TEST(RadiusSearch2DFilter, TestIncreaseMinNeighbours) {
  auto filter = std::make_shared<RadiusSearch2DFilter>(1.0, 10);
  std::vector<pcl::PointXYZ> points = {
    make_point(0.0f, 0.0f, 0.0f),
    make_point(0.2f, 0.0f, 0.0f),
    make_point(0.0f, 0.2f, 0.0f),
    make_point(-0.2f, 0.0f, 0.0f),
    make_point(0.0f, -0.2f, 0.0f)};
  auto time0 = std::chrono::system_clock::now();
  auto t0 = to_msg_time(time0);
  auto input = make_pc(points, t0);

  // Run the filter
  pcl::PointCloud<pcl::PointXYZ> output;
  filter->filter(input, output);

  // Perform checks on the output pointcloud
  // Min neighbours increased, not enough neighbours all points should fail checks
  check_pc({}, output);
}

/* TEST 5: A simple radial pointcloud, minimum radius decreased
 *   x
 * x x x -> all removed
 *   x
 */
TEST(RadiusSearch2DFilter, TestDecreaseSearchRadius) {
  auto filter = std::make_shared<RadiusSearch2DFilter>(0.1, 5);
  std::vector<pcl::PointXYZ> points = {
    make_point(0.0f, 0.0f, 0.0f),
    make_point(0.2f, 0.0f, 0.0f),
    make_point(0.0f, 0.2f, 0.0f),
    make_point(-0.2f, 0.0f, 0.0f),
    make_point(0.0f, -0.2f, 0.0f)};
  auto time0 = std::chrono::system_clock::now();
  auto t0 = to_msg_time(time0);
  auto input = make_pc(points, t0);

  // Run the filter
  pcl::PointCloud<pcl::PointXYZ> output;
  filter->filter(input, output);

  // Perform checks on the output pointcloud
  // Min neighbours increased, not enough neighbours all points should fail checks
  check_pc({}, output);
}
