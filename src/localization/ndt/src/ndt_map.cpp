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

namespace autoware
{
namespace localization
{
namespace ndt
{
constexpr uint32_t NDTVoxel::NUM_POINT_THRESHOLD;

uint32_t validate_pcl_map(const sensor_msgs::msg::PointCloud2 & msg)
{
  auto ret = 0U;
  const auto point_field_size =
    static_cast<uint32_t>(sizeOfPointField(sensor_msgs::msg::PointField::FLOAT64));
  // TODO(cvasfi): Possibly allow additional fields that don't change the order of the fields
  constexpr auto expected_num_fields = 9U;

  // Initial meta data check
  if ((msg.fields.size()) != expected_num_fields ||
    (msg.point_step != expected_num_fields * point_field_size) ||
    (msg.height != 1U))
  {
    return 0U;
  }

  // Field metadata check
  for (auto field_idx = 0U; field_idx < msg.fields.size(); ++field_idx) {
    const auto & field = msg.fields[field_idx];
    if ((field.datatype != sensor_msgs::msg::PointField::FLOAT64) || (field.count != 1U) ||
      (field.offset != (field_idx * point_field_size)))
    {
      return 0U;
    }
  }

  if ((msg.fields[0U].name == "x") &&
    (msg.fields[1U].name == "y") &&
    (msg.fields[2U].name == "z") &&
    (msg.fields[3U].name == "cov_xx") &&
    (msg.fields[4U].name == "cov_xy") &&
    (msg.fields[5U].name == "cov_xz") &&
    (msg.fields[6U].name == "cov_yy") &&
    (msg.fields[7U].name == "cov_yz") &&
    (msg.fields[8U].name == "cov_zz"))
  {
    // Data validation

    // If the actual size and the meta data is in conflict, use the minimum length to be safe.
    const auto min_data_length = std::min(static_cast<decltype(msg.row_step)>(msg.data.size()),
        std::min(msg.row_step, msg.width));
    // Trim the length to make it divisible to point_step, excess data cannot be read.
    const auto safe_data_length = min_data_length - (min_data_length % msg.point_step);
    // Return number of points that can safely be read from the point cloud
    ret = safe_data_length / msg.point_step;
  }
  return ret;
}
}  // namespace ndt
}  // namespace localization
}  // namespace autoware
