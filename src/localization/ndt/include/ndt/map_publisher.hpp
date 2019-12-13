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

#ifndef NDT__MAP_PUBLISHER_HPP_
#define NDT__MAP_PUBLISHER_HPP_

#include <ndt/visibility_control.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <string>

namespace autoware
{
namespace localization
{
namespace ndt
{

/// Read the pcd file into a PointCloud2 message. Throws if the file cannot be read.
/// \param[in] file_name Name of the pcd file.
/// \param[out] msg PointCloud2 message
void NDT_PUBLIC read_from_pcd(const std::string & file_name, sensor_msgs::msg::PointCloud2 & msg);

}  // namespace ndt
}  // namespace localization
}  // namespace autoware

#endif  // NDT__MAP_PUBLISHER_HPP_
