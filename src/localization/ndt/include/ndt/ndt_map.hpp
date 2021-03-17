// Copyright 2019 the Autoware Foundation
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
//
// Co-developed by Tier IV, Inc. and Apex.AI, Inc.

#ifndef NDT__NDT_MAP_HPP_
#define NDT__NDT_MAP_HPP_

#include <ndt/ndt_common.hpp>
#include <ndt/ndt_voxel.hpp>
#include <ndt/ndt_voxel_view.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <time_utils/time_utils.hpp>
#include <vector>
#include <limits>
#include <unordered_map>
#include <utility>
#include <string>
#include "common/types.hpp"

using autoware::common::types::float32_t;

namespace autoware
{
namespace localization
{
namespace ndt
{
/// Function that checks if the pcl message format is valid as an ndt map. The point cloud should
/// have the following fields: x, y, z, cov_xx, cov_xy, cov_xz, cov_yy, cov_yz, cov_zz. The data
/// type for the fields should be double.
/// \param msg Point cloud message
/// \return Safe and usable number of points. the formula is:
/// `min(data.size(), width, row_step) / point_step`
/// If the cloud is assessed to be invalid (i.e. due to invalid fields), then 0 is returned.
uint32_t NDT_PUBLIC validate_pcl_map(const sensor_msgs::msg::PointCloud2 & msg);

/////////////////////////////////////////////

template<typename Derived, typename VoxelT>
class NDTMapBase : public common::helper_functions::crtp<Derived>
{
public:
  using Grid = std::unordered_map<uint64_t, VoxelT>;
  using Point = Eigen::Vector3d;
  using Config = autoware::perception::filters::voxel_grid::Config;
  using TimePoint = std::chrono::system_clock::time_point;
  using VoxelViewVector = std::vector<VoxelView<VoxelT>>;

  /// Constructor
  /// \param voxel_grid_config Voxel grid config to configure the underlying voxel grid.
  explicit NDTMapBase(const Config & voxel_grid_config)
  : m_config(voxel_grid_config), m_map(m_config.get_capacity())
  {
    m_output_vector.reserve(1U);
  }

  // Maps should be moved rather than being copied.
  NDTMapBase(const NDTMapBase &) = delete;
  NDTMapBase & operator=(const NDTMapBase &) = delete;

  // Explicitly declaring to default is needed since we explicitly deleted the copy methods.
  NDTMapBase(NDTMapBase &&) = default;
  NDTMapBase & operator=(NDTMapBase &&) = default;

  /// Lookup the cell at location.
  /// \param x x coordinate
  /// \param y y coordinate
  /// \param z z coordinate
  /// \return A vector containing the cell at given coordinates. A vector is used to support
  /// near-neighbour cell queries in the future.
  const VoxelViewVector & cell(float32_t x, float32_t y, float32_t z) const
  {
    return cell(Point({x, y, z}));
  }

  /// Lookup the cell at location.
  /// \param pt point to lookup
  /// \return A vector containing the cell at given coordinates. A vector is used to support
  /// near-neighbour cell queries in the future.
  const VoxelViewVector & cell(const Point & pt) const
  {
    // TODO(yunus.caliskan): revisit after multi-cell lookup support.
    m_output_vector.clear();
    const auto vx_it = m_map.find(m_config.index(pt));
    // Only return a voxel if it's occupied (i.e. has enough points to compute covariance.)
    if (vx_it != m_map.end() && vx_it->second.usable()) {
      m_output_vector.emplace_back(vx_it->second);
    }
    return m_output_vector;
  }

  /// Insert a point cloud to the map.
  /// \param msg PointCloud2 message to add.
  void insert(const sensor_msgs::msg::PointCloud2 & msg)
  {
    m_stamp = ::time_utils::from_message(msg.header.stamp);
    m_frame_id = msg.header.frame_id;
    this->impl().insert_(msg);
  }

  /// Get size of the map
  /// \return Number of voxels in the map. This number includes the voxels that do not have
  /// enough numbers to be used yet.
  uint64_t size() const noexcept
  {
    return m_map.size();
  }

  /// Get size of the cell.
  /// \return A point representing the dimensions of the cell.
  auto cell_size() const noexcept
  {
    return m_config.get_voxel_size();
  }

  /// \brief Returns an const iterator to the first element of the map
  /// \return Iterator
  typename Grid::const_iterator begin() const noexcept
  {
    return cbegin();
  }
  /// \brief Returns an iterator to the first element of the map
  /// \return Iterator
  typename Grid::iterator begin() noexcept
  {
    return m_map.begin();
  }
  /// \brief Returns a const iterator to the first element of the map
  /// \return Iterator
  typename Grid::const_iterator cbegin() const noexcept
  {
    return m_map.cbegin();
  }
  /// \brief Returns a const iterator to one past the last element of the map
  /// \return Iterator
  typename Grid::const_iterator end() const noexcept
  {
    return cend();
  }
  /// \brief Returns an iterator to one past the last element of the map
  /// \return Iterator
  typename Grid::iterator end() noexcept
  {
    return m_map.end();
  }
  /// \brief Returns a const iterator to one past the last element of the map
  /// \return Iterator
  typename Grid::const_iterator cend() const noexcept
  {
    return m_map.cend();
  }

  /// Clear all voxels in the map
  void clear() noexcept
  {
    m_map.clear();
  }

  /// Get map's time stamp.
  /// \return map's time stamp.
  TimePoint stamp() const noexcept
  {
    return m_stamp;
  }

  /// \brief Set the contents of the pointcloud as the new map.
  /// \param msg Pointcloud to be inserted.
  void set(const sensor_msgs::msg::PointCloud2 & msg)
  {
    clear();
    insert(msg);
  }

  /// Get map's frame id.
  /// \return Frame id of the map.
  const std::string & frame_id() const noexcept
  {
    return m_frame_id;
  }

  /// \brief Check if the map is valid.
  /// \return True if the map and frame ID are not empty and the stamp is initialized.
  bool valid()
  {
    return (!m_map.empty()) && (!m_frame_id.empty());
  }

protected:
  /// Get voxel index given a point.
  /// \param pt point
  /// \return voxel index
  auto index(const Point & pt) const
  {
    return m_config.index(pt);
  }

  /// Get a reference to the voxel at the given index. If no voxel exists, a default constructed
  /// Voxel is inserted.
  /// \param idx
  /// \return
  VoxelT & voxel(uint64_t idx)
  {
    return m_map[idx];
  }

  auto emplace(uint64_t key, const VoxelT && vx)
  {
    return m_map.emplace(key, std::move(vx));
  }

private:
  mutable VoxelViewVector m_output_vector;
  const Config m_config;
  Grid m_map;
  TimePoint m_stamp{};
  std::string m_frame_id{};
};


/// Ndt Map for a dynamic voxel type. This map representation is only to be used
/// when a dense point cloud is intended to be represented as a map. (i.e. by the map publisher)
class NDT_PUBLIC DynamicNDTMap
  : public NDTMapBase<DynamicNDTMap, DynamicNDTVoxel>
{
public:
  using Voxel = DynamicNDTVoxel;
  using Grid = std::unordered_map<uint64_t, Voxel>;
  using Config = autoware::perception::filters::voxel_grid::Config;
  using Point = Eigen::Vector3d;

  using NDTMapBase::NDTMapBase;

  /// Insert the dense point cloud to the map. This is intended for converting a dense
  /// point cloud into the ndt representation. Ideal for reading dense pcd files.
  /// \param msg PointCloud2 message to add.
  void insert_(const sensor_msgs::msg::PointCloud2 & msg)
  {
    sensor_msgs::PointCloud2ConstIterator<float32_t> x_it(msg, "x");
    sensor_msgs::PointCloud2ConstIterator<float32_t> y_it(msg, "y");
    sensor_msgs::PointCloud2ConstIterator<float32_t> z_it(msg, "z");

    while (x_it != x_it.end() &&
      y_it != y_it.end() &&
      z_it != z_it.end())
    {
      const auto pt = Point({*x_it, *y_it, *z_it});
      const auto voxel_idx = index(pt);
      voxel(voxel_idx).add_observation(pt);  // Add or insert new voxel.

      ++x_it;
      ++y_it;
      ++z_it;
    }
    // try to stabilizie the covariance after inserting all the points
    for (auto & vx_it : *this) {
      auto & vx = vx_it.second;
      (void) vx.try_stabilize();
    }
  }
};

/// NDT map using StaticNDTVoxels. This class is to be used when the pointcloud
/// messages to be inserted already have the correct format (see validate_pcl_map(...)) and
/// represent a transformed map. No centroid/covariance computation is done during run-time.
class NDT_PUBLIC StaticNDTMap
  : public NDTMapBase<StaticNDTMap, StaticNDTVoxel>
{
public:
  using NDTMapBase::NDTMapBase;
  using Voxel = StaticNDTVoxel;

  /// Insert point cloud message representing the map to the map representation instance.
  /// Map is assumed to have correct format (see `validate_pcl_map(...)`) and was generated
  /// by a dense map representation with identical configuration to this representation.
  /// \param msg PointCloud2 message to add. Each point in this cloud should correspond to a
  /// single voxel in the underlying voxel grid. This is checked via the `cell_id` field in the pcl
  /// message which is expected to be equal to the voxel grid ID in the map's voxel grid. Since
  /// the grid's index will be a long value to avoid overflows, `cell_id` field should be an array
  /// of 2 unsigned integers. That is because there is no direct long support as a PointField.
  void insert_(const sensor_msgs::msg::PointCloud2 & msg)
  {
    if (validate_pcl_map(msg) == 0U) {
      // throwing rather than silently failing since ndt matching cannot be done with an
      // empty/incorrect map
      throw std::runtime_error(
              "Point cloud representing the ndt map is either empty"
              "or does not have the correct format.");
    }

    sensor_msgs::PointCloud2ConstIterator<Real> x_it(msg, "x");
    sensor_msgs::PointCloud2ConstIterator<Real> y_it(msg, "y");
    sensor_msgs::PointCloud2ConstIterator<Real> z_it(msg, "z");
    sensor_msgs::PointCloud2ConstIterator<Real> icov_xx_it(msg, "icov_xx");
    sensor_msgs::PointCloud2ConstIterator<Real> icov_xy_it(msg, "icov_xy");
    sensor_msgs::PointCloud2ConstIterator<Real> icov_xz_it(msg, "icov_xz");
    sensor_msgs::PointCloud2ConstIterator<Real> icov_yy_it(msg, "icov_yy");
    sensor_msgs::PointCloud2ConstIterator<Real> icov_yz_it(msg, "icov_yz");
    sensor_msgs::PointCloud2ConstIterator<Real> icov_zz_it(msg, "icov_zz");
    sensor_msgs::PointCloud2ConstIterator<uint32_t> cell_id_it(msg, "cell_id");

    while (x_it != x_it.end() &&
      y_it != y_it.end() &&
      z_it != z_it.end() &&
      icov_xx_it != icov_xx_it.end() &&
      icov_xy_it != icov_xy_it.end() &&
      icov_xz_it != icov_xz_it.end() &&
      icov_yy_it != icov_yy_it.end() &&
      icov_yz_it != icov_yz_it.end() &&
      icov_zz_it != icov_zz_it.end() &&
      cell_id_it != cell_id_it.end())
    {
      const Point centroid{*x_it, *y_it, *z_it};
      const auto voxel_idx = index(centroid);

      // Since no native usigned long support is vailable for a point field
      // the `cell_id_it` points to an array of two 32 bit integers to represent
      // a long number. So the assignments must be done via memcpy.
      Grid::key_type received_idx = 0U;
      std::memcpy(&received_idx, &cell_id_it[0U], sizeof(received_idx));

      // If the pointcloud does not represent a voxel grid of identical configuration,
      // report the error
      if (voxel_idx != received_idx) {
        throw std::domain_error(
                "NDTVoxelMap: Pointcloud representing the ndt map"
                "does not have a matching grid configuration with "
                "the map representation it is being inserted to. The cell IDs do not matchb");
      }

      Eigen::Matrix3d inv_covariance;
      inv_covariance << *icov_xx_it, *icov_xy_it, *icov_xz_it,
        *icov_xy_it, *icov_yy_it, *icov_yz_it,
        *icov_xz_it, *icov_yz_it, *icov_zz_it;
      const Voxel vx{centroid, inv_covariance};

      const auto insert_res = emplace(voxel_idx, Voxel{centroid, inv_covariance});
      if (!insert_res.second) {
        // if a voxel already exist at this point, replace.
        insert_res.first->second = vx;
      }

      ++x_it;
      ++y_it;
      ++z_it;
      ++icov_xx_it;
      ++icov_xy_it;
      ++icov_xz_it;
      ++icov_yy_it;
      ++icov_yz_it;
      ++icov_zz_it;
      ++cell_id_it;
    }
  }
};

}  // namespace ndt
}  // namespace localization

namespace common
{
namespace geometry
{
namespace point_adapter
{
/// Point adapters for eigen vector
/// These adapters are necessary for the VoxelGrid to know how to access
/// the coordinates from an eigen vector.
template<>
inline NDT_PUBLIC auto x_(const Eigen::Vector3d & pt)
{
  return static_cast<float32_t>(pt(0));
}

template<>
inline NDT_PUBLIC auto y_(const Eigen::Vector3d & pt)
{
  return static_cast<float32_t>(pt(1));
}

template<>
inline NDT_PUBLIC auto z_(const Eigen::Vector3d & pt)
{
  return static_cast<float32_t>(pt(2));
}

template<>
inline NDT_PUBLIC auto & xr_(Eigen::Vector3d & pt)
{
  return pt(0);
}

template<>
inline NDT_PUBLIC auto & yr_(Eigen::Vector3d & pt)
{
  return pt(1);
}

template<>
inline NDT_PUBLIC auto & zr_(Eigen::Vector3d & pt)
{
  return pt(2);
}
}  // namespace point_adapter
}  // namespace geometry
}  // namespace common
}  // namespace autoware
#endif  // NDT__NDT_MAP_HPP_
