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

#ifndef TEST_NDT_MAP_HPP_
#define TEST_NDT_MAP_HPP_

#include <ndt/ndt_map.hpp>
#include <geometry/spatial_hash_config.hpp>
#include <voxel_grid/config.hpp>
#include <geometry_msgs/msg/point32.hpp>
#include <lidar_utils/point_cloud_utils.hpp>
#include <Eigen/Core>
#include <vector>
#include <set>
#include <map>

namespace autoware
{
namespace localization
{
namespace ndt
{

sensor_msgs::msg::PointCloud2 make_pcl(
  const std::vector<sensor_msgs::msg::PointField> & fields,
  uint32_t height,
  uint32_t data_size,
  uint32_t row_step,
  uint32_t width,
  uint32_t point_step)
{
  sensor_msgs::msg::PointCloud2 msg;
  msg.data.resize(data_size);
  msg.fields = fields;
  msg.row_step = row_step;
  msg.height = height;
  msg.width = width;
  msg.point_step = point_step;
  return msg;
}

sensor_msgs::msg::PointField make_pf(
  std::string name, uint32_t offset, uint8_t datatype,
  uint32_t count)
{
  sensor_msgs::msg::PointField pf;
  pf.name = name;
  pf.offset = offset;
  pf.datatype = datatype;
  pf.count = count;
  return pf;
}

class TestMapValid : public ::testing::Test
{
public:
  using PointField = sensor_msgs::msg::PointField;
  TestMapValid()
  : pf1{make_pf("x", 0U, PointField::FLOAT64, 1U)},
    pf2{make_pf("y", 1U * sizeof(float64_t), PointField::FLOAT64, 1U)},
    pf3{make_pf("z", 2U * sizeof(float64_t), PointField::FLOAT64, 1U)},
    pf4{make_pf("cov_xx", 3U * sizeof(float64_t), PointField::FLOAT64, 1U)},
    pf5{make_pf("cov_xy", 4U * sizeof(float64_t), PointField::FLOAT64, 1U)},
    pf6{make_pf("cov_xz", 5U * sizeof(float64_t), PointField::FLOAT64, 1U)},
    pf7{make_pf("cov_yy", 6U * sizeof(float64_t), PointField::FLOAT64, 1U)},
    pf8{make_pf("cov_yz", 7U * sizeof(float64_t), PointField::FLOAT64, 1U)},
    pf9{make_pf("cov_zz", 8U * sizeof(float64_t), PointField::FLOAT64, 1U)},
    pf10{make_pf("cell_id", 8U * sizeof(float64_t) + sizeof(uint32_t), PointField::UINT32, 1U)} {}

protected:
  // Correct fields.
  const PointField pf1;
  const PointField pf2;
  const PointField pf3;
  const PointField pf4;
  const PointField pf5;
  const PointField pf6;
  const PointField pf7;
  const PointField pf8;
  const PointField pf9;
  const PointField pf10;
  static constexpr uint32_t point_step{static_cast<uint32_t>(9 * sizeof(float64_t)) +
    sizeof(uint32_t)};
  static constexpr uint32_t num_points{50U};
  static constexpr uint32_t data_size{point_step * num_points};
  static constexpr uint32_t width{num_points};
  static constexpr uint32_t row_step{data_size};
};

using PointXYZ = geometry_msgs::msg::Point32;

common::lidar_utils::PointXYZIF get_point_from_vector(const Eigen::Vector3d & v)
{
  return common::lidar_utils::PointXYZIF{
    static_cast<float>(v(0)),
    static_cast<float>(v(1)),
    static_cast<float>(v(2))};
}

// add the point `center` and 4 additional points in a fixed distance from the center
// resulting in 7 points with random but bounded covariance
void add_cell(
  sensor_msgs::msg::PointCloud2 & msg, uint32_t & pc_idx,
  const Eigen::Vector3d & center, double fixed_deviation)
{
  common::lidar_utils::add_point_to_cloud(msg, get_point_from_vector(center), pc_idx);

  std::vector<Eigen::Vector3d> points;
  for (auto idx = 0U; idx < 3U; idx++) {
    for (auto mode = 0u; mode < 2u; mode++) {
      auto deviated_pt = center;
      if (mode == 0U) {
        deviated_pt(idx) += fixed_deviation;
      } else {
        deviated_pt(idx) -= fixed_deviation;
      }
      points.push_back(deviated_pt);
      EXPECT_TRUE(common::lidar_utils::add_point_to_cloud(msg, get_point_from_vector(deviated_pt),
        pc_idx));
    }
  }
}


class NDTMapTest : public ::testing::Test
{
protected:
  static constexpr int POINTS_PER_DIM{5U};
  // how much should the points diverge from the center. It's fixed as there's no randomness.
  static constexpr float FIXED_DEVIATION{0.3};

  NDTMapTest()
  {
    // TODO(yunus.caliskan): Use the map manager for special cloud formatting.
    // init with a size to account for all the points in the map
    common::lidar_utils::init_pcl_msg(m_pc, "map",
      POINTS_PER_DIM * POINTS_PER_DIM * POINTS_PER_DIM * 7);
    // Grid and spatial hash uses these boundaries. The setup allows for a grid of 125 cells: 5x5x5
    // where the centroid coordinates range from the integers 1 to 5 and the voxel size is 1
    m_min_point.x = 0.5F;
    m_min_point.y = 0.5F;
    m_min_point.z = 0.5F;
    m_max_point.x = 5.5F;
    m_max_point.y = 5.5F;
    m_max_point.z = 5.5F;
    m_voxel_size.x = 1.0F;
    m_voxel_size.y = 1.0F;
    m_voxel_size.z = 1.0F;
  }

  void build_pc(const perception::filters::voxel_grid::Config & cfg)
  {
    uint32_t pc_idx = 0U;
    for (auto x = 1U; x <= POINTS_PER_DIM; ++x) {
      for (auto y = 1U; y <= POINTS_PER_DIM; ++y) {
        for (auto z = 1U; z <= POINTS_PER_DIM; ++z) {
          Eigen::Vector3d center{static_cast<double>(x), static_cast<double>(y),
            static_cast<double>(z)};
          add_cell(m_pc, m_pc_idx, center, FIXED_DEVIATION);
          m_voxel_centers[cfg.index(center)] = center;
        }
      }
    }
  }

  uint32_t m_pc_idx{0U};
  sensor_msgs::msg::PointCloud2 m_pc;
  std::map<uint64_t, Eigen::Vector3d> m_voxel_centers;
  PointXYZ m_min_point;
  PointXYZ m_max_point;
  PointXYZ m_voxel_size;
  uint64_t m_capacity{1024U};
};


}  // namespace ndt
}  // namespace localization
}  // namespace autoware

#endif  // TEST_NDT_MAP_HPP_
