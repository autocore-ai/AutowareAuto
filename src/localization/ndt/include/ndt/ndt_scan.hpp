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

#ifndef NDT__NDT_SCAN_HPP_
#define NDT__NDT_SCAN_HPP_

#include <helper_functions/crtp.hpp>
#include <ndt/visibility_control.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <Eigen/Core>
#include <vector>

namespace autoware
{
namespace localization
{
namespace ndt
{
/// CRTP Base class defining the required minimal API of an NDTScan.
/// \tparam Derived Derived class
/// \tparam NDTUnit The unit representing a single element within the scan.
/// \tparam IteratorT The type of iterator to iterate the scan.
template<typename Derived, typename NDTUnit, typename IteratorT>
class NDTScanBase : public common::helper_functions::crtp<Derived>
{
public:
  using Point = NDTUnit;
  /// Get iterator pointing to the beginning of the internal container.
  /// \return Begin iterator.
  IteratorT begin()
  {
    return this->impl().begin_();
  }

  /// Get iterator pointing to the end of the internal container.
  /// \return End iterator.
  IteratorT end()
  {
    return this->impl().end_();
  }

  /// Clear the states and the internal cache of the scan.
  void clear()
  {
    return this->impl().clear_();
  }

  /// Check if there is any data in the scan.
  /// \return True if the internal container is empty.
  bool empty()
  {
    return this->impl().empty_();
  }

  /// Insert a point cloud into the NDTScan. This is the step where the pointcloud is
  /// converted into the ndt scan representation.
  /// \param msg Point cloud to insert.
  void insert(const sensor_msgs::msg::PointCloud2 & msg)
  {
    this->impl().insert_(msg);
  }

  /// Number of points inside the scan.
  /// \return Number of points
  uint32_t size()
  {
    return this->impl().size_();
  }
};

/// Represents a lidar scan in a P2D optimization problem. It is a wrapper around an
/// std::vector<Eigen::Vector3d>
class NDT_PUBLIC P2DNDTScan : public NDTScanBase<P2DNDTScan,
    Eigen::Vector3d, std::vector<Eigen::Vector3d>::iterator>
{
public:
  using Container = std::vector<Eigen::Vector3d>;
  using iterator = Container::iterator;

  // Make sure the given iterator type in the template is compatible with the used container.
  // container should have `iterator` type/alias defined.
  static_assert(std::is_same<decltype(std::declval<NDTScanBase>().begin()), iterator>::value,
    "P2DNDTScan: The iterator type parameter should match the "
    "iterator of the container.");

  /// Constructor
  /// \param msg Point cloud message to initialize this scan with.
  /// \param capacity Capacity of the scan. It should be configured according to the max. expected
  /// point cloud message size from the lidar.
  P2DNDTScan(
    const sensor_msgs::msg::PointCloud2 & msg,
    std::size_t capacity)
  {
    insert_(msg);
    m_points.reserve(capacity);
  }

  /// Constructor
  /// \param capacity Capacity of the scan. It should be configured according to the max. expected
  /// point cloud message size from the lidar.
  explicit P2DNDTScan(std::size_t capacity) {m_points.reserve(capacity);}

  /// Insert a point cloud into the NDTScan. This is the step where the pointcloud is
  /// converted into the ndt scan representation.
  /// \param msg Point cloud to insert.
  void insert_(const sensor_msgs::msg::PointCloud2 & msg)
  {
    if (!m_points.empty()) {
      m_points.clear();
    }
    constexpr auto container_full_error = "received a lidar scan with more points than the "
      "ndt scan representation can contain. Please re-configure the scan"
      "representation accordingly.";

    if (msg.width > m_points.capacity()) {
      throw std::length_error(container_full_error);
    }

    // TODO(yunus.caliskan): Can we avoid copying and use PointCloud2 directly? #102
    // Also validate map?
    sensor_msgs::PointCloud2ConstIterator<float> x_it(msg, "x");
    sensor_msgs::PointCloud2ConstIterator<float> y_it(msg, "y");
    sensor_msgs::PointCloud2ConstIterator<float> z_it(msg, "z");

    while (x_it != x_it.end() &&
      y_it != y_it.end() &&
      z_it != z_it.end())
    {
      if (m_points.size() == m_points.capacity()) {
        throw std::length_error(container_full_error);
      }
      m_points.emplace_back(*x_it, *y_it, *z_it);
      ++x_it;
      ++y_it;
      ++z_it;
    }
  }

  /// Get iterator pointing to the beginning of the internal container.
  /// \return Begin iterator.
  iterator begin_()
  {
    return m_points.begin();
  }

  /// Get iterator pointing to the end of the internal container.
  /// \return End iterator.
  iterator end_()
  {
    return m_points.end();
  }

  /// Check if there is any data in the scan.
  /// \return True if the internal container is empty.
  bool empty_()
  {
    return m_points.empty();
  }

  /// Clear the states and the internal cache of the scan.
  void clear_()
  {
    m_points.clear();
  }

  /// Number of points inside the scan.
  /// \return Number of points
  uint32_t size_()
  {
    return m_points.size();
  }

private:
  Container m_points;
};

}  // namespace ndt
}  // namespace localization
}  // namespace autoware

#endif  // NDT__NDT_SCAN_HPP_
