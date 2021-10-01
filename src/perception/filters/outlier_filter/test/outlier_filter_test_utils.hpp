// Copyright 2021 Tier IV, Inc.
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

#ifndef OUTLIER_FILTER_TEST_UTILS_HPP_
#define OUTLIER_FILTER_TEST_UTILS_HPP_

#include <vector>

#include "gtest/gtest.h"

#include "geometry_msgs/msg/point32.hpp"
#include "pcl_conversions/pcl_conversions.h"
#include "lidar_utils/point_cloud_utils.hpp"
#include "point_cloud_msg_wrapper/point_cloud_msg_wrapper.hpp"


using PointXYZ = geometry_msgs::msg::Point32;

namespace
{
// Helper methods
// TODO(jilada): refactor in #1150
// Note: this method is slightly different to accommodate for the PCL point cloud type
pcl::PointCloud<pcl::PointXYZ> make_pc(
  std::vector<pcl::PointXYZ> points,
  builtin_interfaces::msg::Time stamp)
{
  using autoware::common::types::PointXYZIF;
  sensor_msgs::msg::PointCloud2 msg;
  point_cloud_msg_wrapper::PointCloud2Modifier<PointXYZIF> modifier{msg, "base_link"};
  modifier.reserve(points.size());

  for (auto point : points) {
    PointXYZIF pt;
    pt.x = point.x;
    pt.y = point.y;
    pt.z = point.z;
    pt.intensity = 1.0;
    modifier.push_back(pt);
  }

  msg.header.stamp = stamp;
  pcl::PointCloud<pcl::PointXYZ> pcl_cloud;
  pcl::moveFromROSMsg(msg, pcl_cloud);

  return pcl_cloud;
}

// TODO(jilada): refactor in #1150
pcl::PointXYZ make_point(float x, float y, float z)
{
  pcl::PointXYZ p;
  p.x = x;
  p.y = y;
  p.z = z;
  return p;
}

// TODO(jilada): refactor in #1150
builtin_interfaces::msg::Time to_msg_time(
  const std::chrono::system_clock::time_point time_point)
{
  const auto tse = time_point.time_since_epoch();
  if (tse < std::chrono::nanoseconds(0LL)) {
    throw std::invalid_argument("ROS 2 builtin interfaces time does not support negative a epoch.");
  }
  builtin_interfaces::msg::Time result;

  result.sec = static_cast<decltype(result.sec)>(
    std::chrono::duration_cast<std::chrono::seconds>(tse).count());

  result.nanosec = static_cast<decltype(result.nanosec)>((
      tse - std::chrono::duration_cast<std::chrono::seconds>(tse)).count());

  return result;
}

// TODO(jilada): refactor in #1150
// Note: this method is slightly different to accommodate for the PCL point cloud type
void check_pc(
  std::vector<pcl::PointXYZ> new_points,
  pcl::PointCloud<pcl::PointXYZ> & filtered_pc)
{
  // Check that points are of equal size
  EXPECT_EQ(new_points.size(), filtered_pc.points.size());

  // Compare points
  auto pc_it = filtered_pc.begin();
  auto p_it = new_points.begin();

  while (pc_it != filtered_pc.end() && p_it != new_points.end()) {
    // Check values
    ASSERT_FLOAT_EQ(pc_it->x, p_it->x);
    ASSERT_FLOAT_EQ(pc_it->y, p_it->y);
    ASSERT_FLOAT_EQ(pc_it->z, p_it->z);

    // Update iterators
    pc_it++;
    p_it++;
  }

  // Check that both iterators have reach the end
  ASSERT_EQ(pc_it, filtered_pc.end());
  ASSERT_EQ(p_it, new_points.end());
}
}  // namespace

#endif  // OUTLIER_FILTER_TEST_UTILS_HPP_
