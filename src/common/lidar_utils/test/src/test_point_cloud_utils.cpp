// Copyright 2017-2019 Apex.AI, Inc.
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

#include "common/types.hpp"
#include "lidar_utils/lidar_utils.hpp"
#include <gtest/gtest.h>

namespace autoware
{
namespace common
{
namespace lidar_utils
{
PointCloud2::SharedPtr create_custom_pcl(
    const std::vector<std::string> field_names,
    const uint32_t cloud_size)
{
  using sensor_msgs::msg::PointCloud2;
  PointCloud2::SharedPtr msg = std::make_shared<PointCloud2>();
  const auto field_size = field_names.size();
  msg->height = 1U;
  msg->width = cloud_size;
  msg->fields.resize(field_size);
  for (uint32_t i = 0U; i < field_size; i++)
  {
    msg->fields[i].name = field_names[i];
  }
  msg->point_step = 0U;
  for (uint32_t idx = 0U; idx < field_size; ++idx)
  {
    msg->fields[idx].offset = static_cast<uint32_t>(idx * sizeof(float32_t));
    msg->fields[idx].datatype = sensor_msgs::msg::PointField::FLOAT32;
    msg->fields[idx].count = 1U;
    msg->point_step += static_cast<uint32_t>(sizeof(float32_t));
  }
  const std::size_t capacity = msg->point_step * cloud_size;
  msg->data.clear();
  msg->data.reserve(capacity);
  for (std::size_t i = 0; i < capacity; ++i)
  {
    msg->data.emplace_back(0U); // initialize all values equal to 0
  }
  msg->row_step = msg->point_step * msg->width;
  msg->is_bigendian = false;
  msg->is_dense = false;
  msg->header.frame_id = "base_link";
  return msg;
}

} // namespace lidar_utils
} // namespace common

TEST(point_cloud_utils, has_intensity_and_throw_if_no_xyz_test)
{
  const uint32_t mini_cloud_size = 10U;

  std::vector<std::string> right_field_names{"x", "y", "z", "intensity"};
  std::vector<std::string> not_intensity_field_names{"x", "y", "z", "not_intensity"};
  std::vector<std::string> three_field_names{"x", "y", "z"};
  std::vector<std::string> five_field_names{"x", "y", "z", "intensity", "timestamp"};
  std::vector<std::string> invalid_field_names{"x", "y"};
  std::vector<std::string> wrong_x_field_names{"h", "y", "z"};
  std::vector<std::string> wrong_y_field_names{"x", "h", "z"};
  std::vector<std::string> wrong_z_field_names{"x", "y", "h"};
  const auto correct_pc = create_custom_pcl(right_field_names, mini_cloud_size);
  const auto not_intensity_pc = create_custom_pcl(not_intensity_field_names, mini_cloud_size);
  const auto three_fields_pc = create_custom_pcl(three_field_names, mini_cloud_size);
  const auto five_fields_pc = create_custom_pcl(five_field_names, mini_cloud_size);
  const auto invalid_pc = create_custom_pcl(invalid_field_names, mini_cloud_size);
  const auto no_x_pc = create_custom_pcl(wrong_x_field_names, mini_cloud_size);
  const auto no_y_pc = create_custom_pcl(wrong_y_field_names, mini_cloud_size);
  const auto no_z_pc = create_custom_pcl(wrong_z_field_names, mini_cloud_size);

  EXPECT_THROW(has_intensity_and_throw_if_no_xyz(invalid_pc), std::runtime_error);
  EXPECT_THROW(has_intensity_and_throw_if_no_xyz(no_x_pc), std::runtime_error);
  EXPECT_THROW(has_intensity_and_throw_if_no_xyz(no_y_pc), std::runtime_error);
  EXPECT_THROW(has_intensity_and_throw_if_no_xyz(no_z_pc), std::runtime_error);
  EXPECT_FALSE(has_intensity_and_throw_if_no_xyz(not_intensity_pc));
  EXPECT_FALSE(has_intensity_and_throw_if_no_xyz(three_fields_pc));
  EXPECT_TRUE(has_intensity_and_throw_if_no_xyz(correct_pc));
  EXPECT_TRUE(has_intensity_and_throw_if_no_xyz(five_fields_pc));
}

ASSERT_TRUE(fabsf(autoware::common::lidar_utils::fast_atan2(0.0f, 0.0f) -
                  atan2f(0.0f, 0.0f)) < FAST_ATAN2_MAX_ERROR);
ASSERT_TRUE(fabsf(autoware::common::lidar_utils::fast_atan2(1.0f, 0.0f) -
                  atan2f(1.0f, 0.0f)) < FAST_ATAN2_MAX_ERROR);
ASSERT_TRUE(fabsf(autoware::common::lidar_utils::fast_atan2(-1.0f, 0.0f) -
                  atan2f(-1.0f, 0.0f)) < FAST_ATAN2_MAX_ERROR);
ASSERT_TRUE(fabsf(autoware::common::lidar_utils::fast_atan2(0.0f, 1.0f) -
                  atan2f(0.0f, 1.0f)) < FAST_ATAN2_MAX_ERROR);
ASSERT_TRUE(fabsf(autoware::common::lidar_utils::fast_atan2(0.0f, -1.0f) -
                  atan2f(0.0f, -1.0f)) < FAST_ATAN2_MAX_ERROR);
} // namespace autoware

TEST(fast_atan2, max_error)
{
  float max_error = 0;
  for (float f = 0; f < autoware::common::types::TAU; f += 0.00001f)
  {
    float x = cos(f);
    float y = sin(f);
    max_error = ::std::max(
        max_error,
        fabsf(atan2f(y, x) - autoware::common::lidar_utils::fast_atan2(y, x)));
  }
  ASSERT_TRUE(max_error < FAST_ATAN2_MAX_ERROR);
}