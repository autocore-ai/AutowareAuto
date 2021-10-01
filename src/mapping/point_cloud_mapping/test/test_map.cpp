// Copyright 2020 the Autoware Foundation
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

/// \copyright Copyright 2020 the Autoware Foundation
/// All rights reserved.

#include "test_map.hpp"

#include <point_cloud_mapping/point_cloud_map.hpp>
#include <gtest/gtest.h>

#include <algorithm>
#include <limits>
#include <set>
#include <string>
#include <vector>

using autoware::mapping::point_cloud_mapping::DummyLocalizationMap;
using autoware::mapping::point_cloud_mapping::MapUpdateType;
using autoware::mapping::point_cloud_mapping::PclCloud;
using autoware::mapping::point_cloud_mapping::VoxelMapContext;
using autoware::mapping::point_cloud_mapping::DualVoxelMap;

class VoxelMapTest : public ::testing::Test, public VoxelMapContext {};

void DummyLocalizationMap::clear() {}

TEST_F(VoxelMapTest, BasicIo) {
  constexpr auto capacity = 10U;
  constexpr auto map_frame = "map";
  constexpr auto false_frame = ".asdasd..";
  auto map_size = 0U;
  ASSERT_NE(false_frame, map_frame);
  const auto grid_config = autoware::perception::filters::voxel_grid::Config(
    m_min_point, m_max_point, m_voxel_size, m_capacity);

  DualVoxelMap<DummyLocalizationMap> map{grid_config, map_frame, DummyLocalizationMap{}};

  auto add_update = [&map, &map_size, capacity](auto increment_size,
      MapUpdateType expected_update_type,
      const std::string & frame) {
      auto capped_increment = std::min(increment_size, (capacity - map_size));

      const auto pc = autoware::mapping::point_cloud_mapping::make_pc_deviated(
        increment_size,
        map_size, frame,
        FIXED_DEVIATION);
      const auto summary = map.update(pc);
      EXPECT_EQ(summary.update_type, expected_update_type);

      EXPECT_EQ(summary.num_added_pts, capped_increment * NUM_PTS_PER_CELL);

      map_size += capped_increment;
      ASSERT_LE(map_size, capacity);
      EXPECT_EQ(map.size(), map_size);

      // Let's check the content!
      const std::string fname_prefix{"map_test_fname"};
      const auto fname = fname_prefix + ".pcd";
      map.write(fname_prefix);
      PclCloud pcl_cloud;
      pcl::io::loadPCDFile(fname, pcl_cloud);
      autoware::mapping::point_cloud_mapping::check_pc(pcl_cloud, map_size);
      remove(fname.c_str());
    };
  // First insert: NEW
  EXPECT_NO_THROW(add_update(5U, MapUpdateType::NEW, map_frame));
  // Fully insert into existing map: UPDATE
  EXPECT_NO_THROW(add_update(3U, MapUpdateType::UPDATE, map_frame));
  // Capacity reached, partially inserted into existing map: PARTIAL_UPDATE
  EXPECT_NO_THROW(add_update(3U, MapUpdateType::PARTIAL_UPDATE, map_frame));
  // Map is already full, nothing is inserted: NO_CHANGE
  EXPECT_NO_THROW(add_update(3U, MapUpdateType::NO_CHANGE, map_frame));

  map.clear();
  map_size = 0U;
  EXPECT_EQ(map.size(), map_size);
  EXPECT_NO_THROW(add_update(2U, MapUpdateType::NEW, map_frame));
  // frame mismatch = exception
  EXPECT_THROW(add_update(2U, MapUpdateType::NEW, false_frame), std::runtime_error);
}

//////////////////////// helper function implementations ///////////////////////

void autoware::mapping::point_cloud_mapping::check_pc(PclCloud & pc, std::size_t size)
{
  EXPECT_EQ(pc.size(), size);
  std::set<size_t> read_pts;
  std::transform(
    pc.begin(), pc.end(), std::inserter(read_pts, read_pts.end()), [](
      const auto & pt) {
      EXPECT_NEAR(pt.x, pt.y, std::numeric_limits<decltype(pt.y)>::epsilon() * 10);
      EXPECT_NEAR(pt.z, pt.y, std::numeric_limits<decltype(pt.y)>::epsilon() * 10);
      return static_cast<size_t>(pt.x);
    });
  for (auto i = 0U; i < size; ++i) {
    EXPECT_NE(read_pts.find(i), read_pts.end());
  }
}

sensor_msgs::msg::PointCloud2 autoware::mapping::point_cloud_mapping::make_pc(
  const std::vector<autoware::common::types::PointXYZI> & pts,
  const std::string & frame)
{
  sensor_msgs::msg::PointCloud2 pc;
  using autoware::common::types::PointXYZI;
  point_cloud_msg_wrapper::PointCloud2Modifier<PointXYZI> modifier{pc, frame};
  for (const auto & pt : pts) {
    modifier.push_back(pt);
  }
  return pc;
}

sensor_msgs::msg::PointCloud2 autoware::mapping::point_cloud_mapping::make_pc(
  std::size_t size, std::size_t offset,
  const std::string & frame)
{
  std::vector<autoware::common::types::PointXYZI> pts(size);
  auto idx = 0U;
  std::generate(
    pts.begin(), pts.end(), [&idx, offset]() {
      auto val = static_cast<float_t>((idx++) + offset);
      return autoware::common::types::PointXYZI{val, val, val, val};
    });
  return autoware::mapping::point_cloud_mapping::make_pc(pts, frame);
}

sensor_msgs::msg::PointCloud2 autoware::mapping::point_cloud_mapping::make_pc_deviated(
  std::size_t size, std::size_t offset,
  const std::string & frame, float_t deviation)
{
  std::vector<autoware::common::types::PointXYZI> pts;
  pts.reserve(VoxelMapContext::NUM_PTS_PER_CELL * size);
  for (auto idx = 0U; idx < size; ++idx) {
    auto val = static_cast<float_t>((idx) + offset);
    const auto & cells = autoware::mapping::point_cloud_mapping::get_cells(
      {val, val, val, val},
      deviation);
    pts.insert(pts.end(), cells.begin(), cells.end());
  }
  return autoware::mapping::point_cloud_mapping::make_pc(pts, frame);
}

VoxelMapContext::VoxelMapContext()
{
  // Starting from (-0.5, -0.5, -0.5) with a voxel size of 1.0 makes the voxel centers
  // start from 0.0
  m_min_point.x = -0.5F;
  m_min_point.y = -0.5F;
  m_min_point.z = -0.5F;
  m_max_point.x = 20.5F;
  m_max_point.y = 20.5F;
  m_max_point.z = 20.5F;
  m_voxel_size.x = 1.0F;
  m_voxel_size.y = 1.0F;
  m_voxel_size.z = 1.0F;
}


// Get the point `center` and 6 additional points in a fixed distance from the center
// resulting in 7 points with random but bounded covariance.
std::vector<autoware::common::types::PointXYZI> autoware::mapping::point_cloud_mapping::get_cells(
  const std::array<float_t, 4U> & center,
  float_t fixed_deviation)
{
  std::vector<autoware::common::types::PointXYZI> pts;
  pts.reserve(VoxelMapContext::NUM_PTS_PER_CELL);
  pts.push_back({center[0], center[1], center[2], center[3]});
  for (auto idx = 0U; idx < 3U; idx++) {
    for (auto mode = 0u; mode < 2u; mode++) {
      auto deviated_pt = center;
      if (mode == 0U) {
        deviated_pt[idx] += fixed_deviation;
      } else {
        deviated_pt[idx] -= fixed_deviation;
      }
      pts.push_back({deviated_pt[0], deviated_pt[1], deviated_pt[2], deviated_pt[3]});
    }
  }
  return pts;
}
