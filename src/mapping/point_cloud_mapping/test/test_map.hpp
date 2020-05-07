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
using PclCloud = pcl::PointCloud<pcl::PointXYZI>;

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

void check_pc(PclCloud & pc, std::size_t size);

void add_cell(
  sensor_msgs::msg::PointCloud2 & msg, uint32_t & pc_idx,
  const std::array<float_t, 4U> & center, float_t fixed_deviation);

class VoxelMapContext
{
public:
  using PointXYZ = geometry_msgs::msg::Point32;
  static constexpr std::size_t NUM_PTS_PER_CELL{7U};
  static constexpr float32_t FIXED_DEVIATION{0.3};
  VoxelMapContext();

protected:
  PointXYZ m_min_point;
  PointXYZ m_max_point;
  PointXYZ m_voxel_size;
  uint64_t m_capacity{10U};
};

// This functions similar to `make_pc(size, offset, frame)` but instead of inserting one
// point at each step, 1 point and 6 surrrounding equally distanced points are added to
// emulate a dense map getting 7:1 ratio.
sensor_msgs::msg::PointCloud2 make_pc_deviated(
  std::size_t size, std::size_t offset,
  const std::string & frame, float_t deviation);

std::vector<common::types::PointXYZIF> get_cells(
  const std::array<float_t, 4U> & center,
  float_t fixed_deviation);

}  // namespace point_cloud_mapping
}  // namespace mapping
}  // namespace autoware

#endif  // POINT_CLOUD_MAPPING__PC_MAP_TEST_HPP_
