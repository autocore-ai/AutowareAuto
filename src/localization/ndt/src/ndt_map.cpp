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

#include <ndt/ndt_map.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <algorithm>
#include <string>

namespace autoware
{
namespace localization
{
namespace ndt
{
uint32_t validate_pcl_map(const sensor_msgs::msg::PointCloud2 & msg)
{
  // lambda to check the fields of a PointField
  auto field_valid = [](const auto & field, auto data_type, auto offset, auto count) {
      return (field.datatype == data_type) && (field.offset == offset) && (field.count == count);
    };

  auto ret = 0U;
  const auto double_field_size =
    static_cast<uint32_t>(sizeOfPointField(sensor_msgs::msg::PointField::FLOAT64));
  const auto uint_field_size =
    static_cast<uint32_t>(sizeOfPointField(sensor_msgs::msg::PointField::UINT32));

  // TODO(cvasfi): Possibly allow additional fields that don't change the order of the fields
  constexpr auto expected_num_fields = 10U;

  // Check general pc metadata
  if ((msg.fields.size()) != expected_num_fields ||
    (msg.point_step != (((expected_num_fields - 1U) * double_field_size) + uint_field_size)) ||
    (msg.height != 1U))
  {
    return 0U;
  }

  // check PointField fields
  // Get ID of the last field before cell_ID for reverse iterating. (used to calculate offset)
  auto double_field_idx = expected_num_fields - 2U;
  if (!std::all_of(msg.fields.rbegin() + 1U, msg.fields.rend(),  // check all float fields
    [&double_field_idx, &field_valid, double_field_size](auto & field) {
      return field_valid(field, sensor_msgs::msg::PointField::FLOAT64,
      ((double_field_idx--) * double_field_size), 1U);
    }) ||
    !field_valid(msg.fields[9U], sensor_msgs::msg::PointField::UINT32,  // check the cell id field
    (9U * double_field_size), 1U) )
  {
    return 0U;
  }

  // Check field names
  if ((msg.fields[0U].name != "x") ||
    (msg.fields[1U].name != "y") ||
    (msg.fields[2U].name != "z") ||
    (msg.fields[3U].name != "cov_xx") ||
    (msg.fields[4U].name != "cov_xy") ||
    (msg.fields[5U].name != "cov_xz") ||
    (msg.fields[6U].name != "cov_yy") ||
    (msg.fields[7U].name != "cov_yz") ||
    (msg.fields[8U].name != "cov_zz") ||
    (msg.fields[9U].name != "cell_id"))
  {
    return 0U;
  }


  // If the actual size and the meta data is in conflict, use the minimum length to be safe.
  const auto min_data_length = std::min(static_cast<decltype(msg.row_step)>(msg.data.size()),
      std::min(msg.row_step, msg.width * msg.point_step));
  // Trim the length to make it divisible to point_step, excess data cannot be read.
  const auto safe_data_length = min_data_length - (min_data_length % msg.point_step);
  // Return number of points that can safely be read from the point cloud
  ret = safe_data_length / msg.point_step;

  return ret;
}
}  // namespace ndt
}  // namespace localization
}  // namespace autoware
