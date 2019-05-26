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
#include <string>
#include <cstdlib>
#include <limits>

namespace tf2
{
// TODO(yunus.caliskan): This function is simply copied from `tf2_sensor_msgs` and it is to be
//  removed when `tf2_sensor_msgs` is deployed in the ade.
template<>
inline
void doTransform(
  const sensor_msgs::msg::PointCloud2 & p_in, sensor_msgs::msg::PointCloud2 & p_out,
  const geometry_msgs::msg::TransformStamped & t_in)
{
  p_out = p_in;
  p_out.header = t_in.header;
  Eigen::Transform<float, 3, Eigen::Affine> t =
    Eigen::Translation3f(t_in.transform.translation.x, t_in.transform.translation.y,
      t_in.transform.translation.z) * Eigen::Quaternion<float>(
    t_in.transform.rotation.w, t_in.transform.rotation.x,
    t_in.transform.rotation.y, t_in.transform.rotation.z);

  sensor_msgs::PointCloud2ConstIterator<float> x_in(p_in, std::string("x"));
  sensor_msgs::PointCloud2ConstIterator<float> y_in(p_in, std::string("y"));
  sensor_msgs::PointCloud2ConstIterator<float> z_in(p_in, std::string("z"));

  sensor_msgs::PointCloud2Iterator<float> x_out(p_out, std::string("x"));
  sensor_msgs::PointCloud2Iterator<float> y_out(p_out, std::string("y"));
  sensor_msgs::PointCloud2Iterator<float> z_out(p_out, std::string("z"));

  Eigen::Vector3f point;
  for (; x_in != x_in.end(); ++x_in, ++y_in, ++z_in, ++x_out, ++y_out, ++z_out) {
    point = t * Eigen::Vector3f(*x_in, *y_in, *z_in);
    *x_out = point.x();
    *y_out = point.y();
    *z_out = point.z();
  }
}
}  // namespace tf2

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
inline bool is_identity(const geometry_msgs::msg::TransformStamped & t_in)
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
