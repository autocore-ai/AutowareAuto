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

#ifndef NDT__NDT_REPRESENTATIONS_HPP_
#define NDT__NDT_REPRESENTATIONS_HPP_

#include <lidar_utils/point_cloud_utils.hpp>
#include <ndt/visibility_control.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/transform.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <helper_functions/crtp.hpp>
#include <Eigen/Core>
#include <vector>
#include <algorithm>

namespace autoware
{
namespace localization
{
namespace ndt
{
using float64_t = double;
using float32_t = float;
using Real = float64_t;
/// Class Representing a base NDT grid cell. Functionality is implemented by CRTP derived classes.
/// \tparam Derived Implementation class
template<typename Derived>
class NDTNormal : public common::helper_functions::crtp<Derived>
{
public:
  using Point = Eigen::Vector3d;
  using Covariance = Eigen::Matrix3d;

  /// Function returning the mean value of the normal distribution
  /// \return Centroid of the unit.
  const Point & centroid() const
  {
    return this->impl().centroid_();
  }

  /// Function returning the covariance of the normal distribution
  /// \return Covariance of the unit
  const Covariance & covariance() const
  {
    return this->impl().covariance_();
  }
};

template<typename Derived, typename NdtUnit = common::lidar_utils::PointXYZIF>
class NDTScanBase
{
public:
  using Point = NdtUnit;
  using IterT = typename std::vector<NdtUnit>::iterator;
  // get the point for a given index.
  NdtUnit & get(size_t index);

  IterT begin();

  IterT end();

private:
  const Derived & impl() const
  {
    return *static_cast<const Derived *>(this);
  }
};


class P2DNDTScan : public NDTScanBase<P2DNDTScan, common::lidar_utils::PointXYZIF>
{
  using PointT = common::lidar_utils::PointXYZIF;
  const std::vector<PointT> & cells(double x, double y, double z);

  void advance(size_t advancement);
  PointT next();
  PointT prev();
  PointT front();
  PointT back();
  void transform(const geometry_msgs::msg::Transform &);

private:
  sensor_msgs::msg::PointCloud2 m_cloud;
};

}  // namespace ndt
}  // namespace localization
}  // namespace autoware

#endif  // NDT__NDT_REPRESENTATIONS_HPP_
