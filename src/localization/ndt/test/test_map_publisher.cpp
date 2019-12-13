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
#include <ndt/map_publisher.hpp>
#include <pcl/io/pcd_io.h>
#include <lidar_utils/point_cloud_utils.hpp>
#include <lidar_utils/point_cloud_utils.hpp>


namespace autoware
{
namespace localization
{
namespace ndt
{
TEST(PCDLoadTest, basics) {
  constexpr auto num_points = 5U;
  pcl::PointCloud<pcl::PointXYZI> dummy_cloud{};
  sensor_msgs::msg::PointCloud2 msg;
  common::lidar_utils::init_pcl_msg(msg, "base_link", num_points);
  const std::string test_fname = "PCDLoadTest_test_pcd_file.pcd";
  const std::string non_existing_fname = "NON_EXISTING_FILE_PCDLoadTest.XYZ";
  for (auto i = 0U; i < num_points; i++) {
    pcl::PointXYZI pt;
    pt.x = static_cast<float>(i);
    pt.y = static_cast<float>(i);
    pt.z = static_cast<float>(i);
    pt.intensity = static_cast<float>(i);
    dummy_cloud.push_back(pt);
  }
  pcl::io::savePCDFile(test_fname, dummy_cloud);

  ASSERT_TRUE(std::ifstream{test_fname}.good());
  ASSERT_FALSE(std::ifstream{non_existing_fname}.good());
  EXPECT_THROW(read_from_pcd(non_existing_fname, msg), std::runtime_error);
  EXPECT_NO_THROW(read_from_pcd(test_fname, msg));

  sensor_msgs::PointCloud2ConstIterator<float> x_it(msg, "x");
  sensor_msgs::PointCloud2ConstIterator<float> y_it(msg, "y");
  sensor_msgs::PointCloud2ConstIterator<float> z_it(msg, "z");
  sensor_msgs::PointCloud2ConstIterator<float> intensity_it(msg, "intensity");

  auto counter = 0.0F;

  while (x_it != x_it.end() &&
    y_it != y_it.end() &&
    z_it != z_it.end())
  {
    EXPECT_FLOAT_EQ(*x_it, counter);
    EXPECT_FLOAT_EQ(*y_it, counter);
    EXPECT_FLOAT_EQ(*z_it, counter);
    EXPECT_FLOAT_EQ(*intensity_it, counter);

    ++x_it;
    ++y_it;
    ++z_it;
    ++intensity_it;
    counter += 1.0F;
  }
  EXPECT_FLOAT_EQ(counter, num_points);
  remove(test_fname.c_str());
}

}  // namespace ndt
}  // namespace localization
}  // namespace autoware
