// Copyright 2019 Apex.AI, Inc.
// Co-developed by Tier IV, Inc. and Apex.AI, Inc.
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
#include "test_ndt_map.hpp"
#include <tuple>
#include <vector>

namespace autoware
{
namespace localization
{
namespace ndt
{

constexpr uint32_t TestMapValid::point_step;
constexpr uint32_t TestMapValid::num_points;
constexpr uint32_t TestMapValid::data_size;
constexpr uint32_t TestMapValid::width;
constexpr uint32_t TestMapValid::row_step;

TEST_F(TestMapValid, map_pcl_meta_validation) {
  using PointField = sensor_msgs::msg::PointField;

  // Some invalid fields:
  const auto pf10 = make_pf("x", 8U, PointField::FLOAT32, 1U);
  const auto pf11 = make_pf("x", 4U, PointField::FLOAT64, 1U);
  const auto pf12 = make_pf("x", 8U, PointField::FLOAT64, 2U);
  const auto pf13 = make_pf("xyz", 8U, PointField::FLOAT64, 1U);

  const std::vector<PointField> correct_field_set{pf1, pf2, pf3, pf4, pf5, pf6, pf7, pf8, pf9};

  const std::vector<std::vector<PointField>> incorrect_field_sets{
    {pf1, pf2, pf3, pf4, pf5, pf6, pf7, pf8, pf9, pf10},  // Extra field
    {pf2, pf1, pf3, pf4, pf5, pf6, pf7, pf8, pf9},  // Misordered
    {pf10, pf1, pf2, pf3, pf4, pf5, pf6, pf7, pf8, pf9},  // Extra field in start
    {pf1, pf2, pf10, pf3, pf4, pf5, pf6, pf7, pf8, pf9},  // Extra field in middle
    {pf10, pf2, pf3, pf4, pf5, pf6, pf7, pf8, pf9},
    {pf11, pf2, pf3, pf4, pf5, pf6, pf7, pf8, pf9},
    {pf12, pf2, pf3, pf4, pf5, pf6, pf7, pf8, pf9},
    {pf13, pf2, pf3, pf4, pf5, pf6, pf7, pf8, pf9},
    {pf13, pf2, pf3, pf4, pf5, pf6, pf7, pf8, pf9},
    {pf1, pf2, pf3, pf4, pf5, pf6, pf7, pf8},  // missing field
    {}
  };

  auto correct_pcl = make_pcl(correct_field_set, 1U, data_size, row_step, width, point_step);

  EXPECT_GT(validate_pcl_map(correct_pcl), 0U);

  for (const auto & incorrect_field_set : incorrect_field_sets) {
    auto incorrect_pcl = make_pcl(incorrect_field_set, 1U, data_size, row_step, width, point_step);
    EXPECT_EQ(validate_pcl_map(incorrect_pcl), 0U);
  }

  // Check with invalid height
  EXPECT_EQ(validate_pcl_map(make_pcl(correct_field_set, 0U, data_size, row_step, width,
    point_step)), 0U);
  // Check with invalid point step
  EXPECT_EQ(validate_pcl_map(make_pcl(correct_field_set, 1U, data_size, row_step, width,
    point_step - 1U)), 0U);
}

TEST_F(TestMapValid, map_pcl_size_validation) {
  const std::vector<PointField> field_set{pf1, pf2, pf3, pf4, pf5, pf6, pf7, pf8, pf9};

  EXPECT_EQ(validate_pcl_map(make_pcl(field_set, 1U, data_size, row_step, width,
    point_step)), num_points);

  // Data larger than expected means that the return value will exclude the excess data
  // data.size(), row_step and width should be consistent, otherwise the minimum of 3 will
  // determine the returned size
  EXPECT_EQ(validate_pcl_map(
      make_pcl(field_set, 1U, data_size + point_step, row_step, width, point_step)), num_points);
  EXPECT_EQ(validate_pcl_map(
      make_pcl(field_set, 1U, data_size, row_step + point_step, width, point_step)), num_points);
  EXPECT_EQ(validate_pcl_map(
      make_pcl(field_set, 1U, data_size, row_step, width + point_step, point_step)), num_points);

  // Even if the metadata and the data size is consistent, if the size is not divisible to
  // point_step, the excess data will be ignored.
  ASSERT_NE((data_size + 1U) % point_step, 0U);  // Required for the test to work.
  EXPECT_EQ(validate_pcl_map(
      make_pcl(field_set, 1U, data_size + 1U, row_step + 1U, width + 1U, point_step)), num_points);

  // Data shorter than expected: return the greatest number of points that can be read
  // given the point_step
  EXPECT_EQ(validate_pcl_map(make_pcl(field_set, 1U, data_size - 1U, row_step, width,
    point_step)), num_points - 1U);
  EXPECT_EQ(validate_pcl_map(make_pcl(field_set, 1U, data_size, row_step - 1U, width,
    point_step)), num_points - 1U);
  EXPECT_EQ(validate_pcl_map(make_pcl(field_set, 1U, data_size, row_step, width - 1U,
    point_step)), num_points - 1U);
  EXPECT_EQ(validate_pcl_map(
      make_pcl(field_set, 1U, data_size - 2 * point_step, row_step, width,
      point_step)), num_points - 2U);
  EXPECT_EQ(validate_pcl_map(
      make_pcl(field_set, 1U, data_size - 2 * point_step - 1U, row_step, width,
      point_step)), num_points - 3U);
}

}  // namespace ndt
}  // namespace localization
}  // namespace autoware
