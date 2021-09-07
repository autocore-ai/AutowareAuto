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

#include <memory>
#include <vector>

#include "gtest/gtest.h"
#include "outlier_filter/voxel_grid_outlier_filter.hpp"
#include "outlier_filter_test_utils.hpp"


using VoxelGridOutlierFilter =
  autoware::perception::filters::outlier_filter::voxel_grid_outlier_filter::VoxelGridOutlierFilter;

// TEST METHODS
/* TEST 1: Four equispaced points
 *     |
 *  x  |  x
 *     |
 * --------- -> the same
 *     |
 *  x  |  x
 *     |
 */
TEST(VoxelGridOutlierFilterTest, TestFourEquispacedPoints) {
  auto filter =
    std::make_shared<VoxelGridOutlierFilter>(1.0f, 1.0f, 1.0f, static_cast<uint32_t>(1));
  std::vector<pcl::PointXYZ> points = {
    make_point(-1.0f, 1.0f, 0.0f),
    make_point(-1.0f, -1.0f, 0.0f),
    make_point(1.0f, 1.0f, 0.0f),
    make_point(1.0f, -1.0f, 0.0f)};
  auto time0 = std::chrono::system_clock::now();
  auto t0 = to_msg_time(time0);
  auto input = make_pc(points, t0);

  // Run the filter
  pcl::PointCloud<pcl::PointXYZ> output;
  filter->filter(input, output);

  // Perform the check
  check_pc(points, output);
}

/* TEST 2: Four equispaced points + one close point
 *     |            |
 *  xx |  x      xx |
 *     |            |
 * --------- -> ---------
 *     |            |
 *  x  |  x         |
 *     |            |
 */
TEST(VoxelGridOutlierFilterTest, TestTwoClosePoints) {
  auto filter =
    std::make_shared<VoxelGridOutlierFilter>(1.0f, 1.0f, 1.0f, static_cast<uint32_t>(2));
  std::vector<pcl::PointXYZ> points = {
    make_point(-1.0f, 1.0f, 0.0f),
    make_point(-0.8f, 1.0f, 0.0f),
    make_point(-1.0f, -1.0f, 0.0f),
    make_point(1.0f, 1.0f, 0.0f),
    make_point(1.0f, -1.0f, 0.0f)};
  auto time0 = std::chrono::system_clock::now();
  auto t0 = to_msg_time(time0);
  auto input = make_pc(points, t0);

  // Run the filter
  pcl::PointCloud<pcl::PointXYZ> output;
  filter->filter(input, output);

  // Perform the check
  std::vector<pcl::PointXYZ> filter_points = {
    make_point(-1.0f, 1.0f, 0.0f),
    make_point(-0.8f, 1.0f, 0.0f)
  };
  check_pc(filter_points, output);
}

/* TEST 3: Line of 10 points
 * Due to the gridding structure the grid has a range of [0.0 - 1.0)
 *      |              |
 * xxxxxxxxxxx    xxxxxxxxxx
 *      |              |
 * ----------- -> -----------
 */
TEST(VoxelGridOutlierFilterTest, TestLine) {
  auto filter =
    std::make_shared<VoxelGridOutlierFilter>(1.0f, 1.0f, 1.0f, static_cast<uint32_t>(2));
  std::vector<pcl::PointXYZ> points = {
    make_point(-1.0f, 1.0f, 0.0f),
    make_point(-0.8f, 1.0f, 0.0f),
    make_point(-0.6f, 1.0f, 0.0f),
    make_point(-0.4f, 1.0f, 0.0f),
    make_point(-0.2f, 1.0f, 0.0f),
    make_point(0.0f, 1.0f, 0.0f),
    make_point(0.2f, 1.0f, 0.0f),
    make_point(0.4f, 1.0f, 0.0f),
    make_point(0.6f, 1.0f, 0.0f),
    make_point(0.8f, 1.0f, 0.0f),
    make_point(1.0f, 1.0f, 0.0f)};
  auto time0 = std::chrono::system_clock::now();
  auto t0 = to_msg_time(time0);
  auto input = make_pc(points, t0);

  // Run the filter
  pcl::PointCloud<pcl::PointXYZ> output;
  filter->filter(input, output);

  // Perform the check
  // Voxel is exclusive of the range, hence the final value is clipped
  points.pop_back();
  check_pc(points, output);
}
