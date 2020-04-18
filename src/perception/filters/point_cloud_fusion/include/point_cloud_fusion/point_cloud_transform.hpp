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

#ifndef POINT_CLOUD_FUSION__POINT_CLOUD_TRANSFORM_HPP_
#define POINT_CLOUD_FUSION__POINT_CLOUD_TRANSFORM_HPP_
#include <tf2/convert.h>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <common/types.hpp>
#include <string>
#include <cstdlib>
#include <limits>

using autoware::common::types::bool8_t;

namespace autoware
{
namespace perception
{
namespace filters
{
namespace point_cloud_fusion
{
/// \brief Function to check if an input transform is identity
/// \param t_in transform to be checked
/// \return True if the transform is identity. False otherwise.
inline bool8_t is_identity(const geometry_msgs::msg::TransformStamped & t_in)
{
  auto ret = false;
  constexpr auto eps = std::numeric_limits<decltype(t_in.transform.translation.x)>::epsilon();
  if ((abs(t_in.transform.translation.x) < eps) &&
    (abs(t_in.transform.translation.y) < eps) &&
    (abs(t_in.transform.translation.z) < eps) &&
    (abs(t_in.transform.rotation.x) <= eps) &&
    (abs(t_in.transform.rotation.y) <= eps) &&
    (abs(t_in.transform.rotation.z) <= eps) &&
    (abs(t_in.transform.rotation.w - 1.0) <= (eps)))
  {
    ret = true;
  }
  return ret;
}

}  // namespace point_cloud_fusion
}  // namespace filters
}  // namespace perception
}  // namespace autoware

#endif  // POINT_CLOUD_FUSION__POINT_CLOUD_TRANSFORM_HPP_
