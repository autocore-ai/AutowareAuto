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
#ifndef NDT__NDT_MAP_HPP_
#define NDT__NDT_MAP_HPP_

#include <sensor_msgs/msg/point_cloud2.hpp>
#include <ndt/visibility_control.hpp>

namespace autoware
{
namespace localization
{
namespace ndt
{
using float64_t = double;
/// Function that checks if the pcl message format is valid as an ndt map. The point cloud should
/// have the following fields: x, y, z, cov_xx, cov_xy, cov_xz, cov_yy, cov_yz, cov_zz. The data
/// type for the fields should be double.
/// \param msg Point cloud message
/// \return Safe and usable number of points. the formula is:
/// `min(data.size(), width, row_step) / point_step`
/// If the cloud is assessed to be invalid (i.e. due to invalid fields), then 0 is returned.
uint32_t NDT_PUBLIC validate_pcl_map(const sensor_msgs::msg::PointCloud2 & msg);

}  // namespace ndt
}  // namespace localization
}  // namespace autoware

#endif  // NDT__NDT_MAP_HPP_
