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

#include <point_cloud_mapping/map.hpp>
#include <gtest/gtest.h>
#include <lidar_utils/point_cloud_utils.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include "pc_map_test.hpp"
namespace autoware
{
namespace mapping
{
namespace point_cloud_mapping
{

TEST(PointCloudMapTest, basic_io) {
  constexpr auto capacity = 10U;
  constexpr auto map_frame = "map";
  constexpr auto false_frame = ".asdasd..";
  auto map_size = 0U;
  ASSERT_NE(false_frame, map_frame);

  PlainPointCloudMap map{capacity, map_frame};

  auto check_pc = [](sensor_msgs::msg::PointCloud2 & pc, auto size) {
      std::array<sensor_msgs::PointCloud2ConstIterator<float_t>, 4U> its{
        sensor_msgs::PointCloud2ConstIterator<float_t>(pc, "x"),
        sensor_msgs::PointCloud2ConstIterator<float_t>(pc, "y"),
        sensor_msgs::PointCloud2ConstIterator<float_t>(pc, "z"),
        sensor_msgs::PointCloud2ConstIterator<float_t>(pc, "intensity")
      };
      sensor_msgs::PointCloud2Modifier modifier(pc);
      EXPECT_EQ(modifier.size(), size);
      for (auto i = 0U; i < size; ++i) {
        for (auto & it: its) {
          EXPECT_FLOAT_EQ(*it, static_cast<float_t>(i));
          ++it;
        }
      }
    };
  auto add_update = [&map, &map_size, capacity, check_pc](auto increment_size,
      MapUpdateType expected_update_type,
      const std::string & frame) {
      geometry_msgs::msg::PoseWithCovarianceStamped dummy;
      auto capped_increment = std::min(increment_size, (capacity - map_size));

      const auto pc = make_pc(increment_size, map_size, frame);
      const auto summary = map.try_add_observation(pc, dummy);
      EXPECT_EQ(summary.update_type, expected_update_type);
      EXPECT_EQ(summary.num_added_pts, capped_increment);

      map_size += capped_increment;
      ASSERT_LE(map_size, capacity);
      EXPECT_EQ(map.full(), (map_size == capacity));

      auto map_pc = map.get();
      check_pc(map_pc, map_size);
    };

  EXPECT_FALSE(map.full());
  // First insert: NEW
  EXPECT_NO_THROW(add_update(5U, MapUpdateType::NEW, map_frame));
  // Fully insert into existing map: UPDATE
  EXPECT_NO_THROW(add_update(3U, MapUpdateType::UPDATE, map_frame));
  // Capacity reached, partially inserted into existing map: PARTIAL_UPDATE
  EXPECT_NO_THROW(add_update(3U, MapUpdateType::PARTIAL_UPDATE, map_frame));
  // Map is already full, nothing is inserted: NO_CHANGE
  EXPECT_NO_THROW(add_update(3U, MapUpdateType::NO_CHANGE, map_frame));
  EXPECT_TRUE(map.full());
  map.clear();
  map_size = 0U;
  EXPECT_FALSE(map.full());
  auto map_pc = map.get();
  sensor_msgs::PointCloud2Modifier modifier{map_pc};
  EXPECT_EQ(modifier.size(), 0U);
  EXPECT_NO_THROW(add_update(2U, MapUpdateType::NEW, map_frame));
  // frame mismatch = exception
  EXPECT_THROW(add_update(2U, MapUpdateType::NEW, false_frame), std::runtime_error);
}

//////////////////////// helper function implementations ///////////////////////

sensor_msgs::msg::PointCloud2 make_pc(
  const std::vector<common::types::PointXYZIF> & pts,
  const std::string & frame)
{
  sensor_msgs::msg::PointCloud2 pc;
  common::lidar_utils::init_pcl_msg(pc, frame, pts.size());
  auto idx = 0U;
  for (const auto & pt : pts) {
    common::lidar_utils::add_point_to_cloud(pc, pt, idx);
  }
  return pc;
}

sensor_msgs::msg::PointCloud2 make_pc(
  std::size_t size, std::size_t offset,
  const std::string & frame)
{
  std::vector<common::types::PointXYZIF> pts(size);
  auto idx = 0U;
  std::generate(pts.begin(), pts.end(), [&idx, offset]() {
      auto val = static_cast<float_t>((idx++) + offset);
      return common::types::PointXYZIF{val, val, val, val};
    });
  return make_pc(pts, frame);
}


}  // namespace point_cloud_mapping
}  // namespace mapping
}  // namespace autoware
