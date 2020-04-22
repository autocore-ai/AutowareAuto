// Copyright 2020 Apex.AI, Inc.
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

#ifndef POINT_CLOUD_MAPPING__PC_MAP_TEST_HPP_
#define POINT_CLOUD_MAPPING__PC_MAP_TEST_HPP_

#include <sensor_msgs/msg/point_cloud2.hpp>
#include <common/types.hpp>
#include <point_cloud_mapping/map.hpp>

namespace autoware
{
namespace mapping
{
namespace point_cloud_mapping
{
/// Initialize a pc, insert points in the vector and return it.
sensor_msgs::msg::PointCloud2 make_pc(
  const std::vector<common::types::PointXYZIF> & pts,
  const std::string & frame = "map");

/// Create a point cloud with with a size of `size`. The magnitudes of fields ranges as follows:
/// (offset, offset, offset, offset) ... (offset + size, offset + size, offset + size, offset + size)
/// where the fields correspond to: (x, y, z, intensity)
sensor_msgs::msg::PointCloud2 make_pc(
  std::size_t size, std::size_t offset = 0,
  const std::string & frame = "map");


}  // namespace point_cloud_mapping
}  // namespace mapping
}  // namespace autoware

#endif  // POINT_CLOUD_MAPPING__PC_MAP_TEST_HPP_
