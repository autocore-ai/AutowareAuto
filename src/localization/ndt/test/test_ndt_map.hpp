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
#include <vector>
#include <string>

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
    pf9{make_pf("cov_zz", 8U * sizeof(float64_t), PointField::FLOAT64, 1U)} {}

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
  static constexpr uint32_t point_step{static_cast<uint32_t>(9 * sizeof(float64_t))};
  static constexpr uint32_t num_points{50U};
  static constexpr uint32_t data_size{point_step * num_points};
  static constexpr uint32_t width{data_size};
  static constexpr uint32_t row_step{data_size};
};
}  // namespace ndt
}  // namespace localization
}  // namespace autoware

#endif  // TEST_NDT_MAP_HPP_
