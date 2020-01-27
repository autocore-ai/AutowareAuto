// Copyright 2017-2019 Apex.AI, Inc.
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

#ifndef RAY_GROUND_CLASSIFIER__VISIBILITY_CONTROL_HPP_
#define RAY_GROUND_CLASSIFIER__VISIBILITY_CONTROL_HPP_

#include <string>
#include <sensor_msgs/msg/point_cloud2.hpp>

namespace autoware
{
namespace common
{
namespace lidar_utils
{
sensor_msgs::msg::PointCloud2::SharedPtr create_custom_pcl(
    const std::vector<std::string> field_names,
    const uint32_t cloud_size);
}
} // namespace common
} // namespace autoware
#endif // RAY_GROUND_CLASSIFIER_NODES__RAY_GROUND_CLASSIFIER_CLOUD_NODE_HPP_
