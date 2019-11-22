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

#include <ndt/ndt_representations.hpp>
#include <voxel_grid/voxels.hpp>
#include <voxel_grid/voxel_grid.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <geometry/spatial_hash.hpp>
#include <vector>

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

/// Voxel implementation for the NDT map. This class fuses the voxel and NDTNormal
/// APIs to fully represent an NDT grid cell
class NDT_PUBLIC NDTVoxel : public perception::filters::voxel_grid::Voxel<Eigen::Vector3d>,
  public NDTNormal<NDTVoxel>
{
public:
  using PointT = Eigen::Vector3d;

  // TODO(yunus.caliskan): make this configurable.
  // Number of points a voxel should have to count as occupied. Set to the dimension of a 3D point.
  static constexpr uint32_t NUM_POINT_THRESHOLD = 3U;

  /// Add a point to the cell
  /// \param pt Point to add to the voxel.
  void add_observation(const PointT & pt)
  {
    if (m_cov_computed) {
      m_covariance.setZero();
      m_cov_point_count = 0U;
      m_cov_computed = false;
    }
    // TODO(yunus.caliskan): Resolve the overload resolution issue!
    // compute the centroid
    // This section is identical to the CentroidVoxel. However calling
    // `CentroidVoxel<Eigen::Vector3d>::add_observation(pt);` overloeads eigen's arithmetic
    // operators due to overload resolution.
    const auto last = static_cast<Real>(Voxel<PointT>::count());
    Voxel<PointT>::set_count(Voxel<PointT>::count() + 1U);
    const auto count_inv = Real{1.0} / static_cast<Real>(Voxel<PointT>::count());
    // Incremental update: u' = ((u * n) + x) / (n + 1), u = mean, x = obs, n = count
    const PointT centroid = ((Voxel<PointT>::get() * last) + pt) * count_inv;
    Voxel<PointT>::set_centroid(centroid);
  }

  /// \brief Use Config and index to set up any important information, in this case no-op
  /// \param[in] cfg The configuration object for the parent voxel grid
  /// \param[in] idx The index for this particular voxel
  //lint -e{9175} NOLINT this is to match a parent API
  void configure(const perception::filters::voxel_grid::Config & cfg, const uint64_t idx)
  {
    (void)cfg;
    (void)idx;
  }

  /// Adds the effect of one point to the covariance of the voxel. All points residing in the
  /// voxel should be included in the covariance computation for a correct result.
  /// \param pt point to add for covariance calculation.
  void add_point_for_covariance(const PointT & pt)
  {
    const auto inv_count = Real{1.0} / static_cast<Real>(count() - 1U);
    const auto & centroid = centroid_();
    m_covariance += inv_count * ((pt - centroid) * ((pt - centroid).transpose()));
    ++m_cov_point_count;
    if (m_cov_point_count == count()) {
      m_cov_computed = true;
    }
  }

  // Hiding the original function with a modified version that uses a custom threshold for
  // interfacing with NDTMap
  bool occupied() const
  {
    return count() >= NUM_POINT_THRESHOLD;
  }

  /// Returns the covariance of the points in the voxel.
  /// \return covariance of the cell
  const Eigen::Matrix3d & covariance_() const
  {
    if (!occupied()) {
      throw std::out_of_range("NDTVoxel: Cannot get covariance from an unoccupied voxel");
    }
    if (count() != m_cov_point_count) {
      throw std::length_error("NDTVoxel: Not all points are used in the covariance computation. "
              "Make sure to call `add_point_for_covariance() for each point in the voxel.`");
    }
    return m_covariance;
  }
  /// Returns the mean of the points in the cell
  /// \return centroid of the cell
  const Eigen::Vector3d & centroid_() const
  {
    // Using the overloaded function as the parent function will use the hidden occupancy check
    if (!occupied()) {
      throw std::out_of_range("NDTVoxel: Cannot get centroid from an unoccupied voxel");
    }
    return get();
  }

  /// Returns true if enough points are used in the covariance computation.
  bool cov_computed() const
  {
    return m_cov_computed;
  }

private:
  size_t m_cov_point_count{0U};
  bool m_cov_computed{false};
  Eigen::Matrix3d m_covariance;
};
/////////////////////////////////////////////

/// Class representing an NDT map. It utilizes a voxel grid for organizing the cells and a spatial
/// hash for managing the beighboring cells lookup. `NDTVoxelMapOutput` is used to wrap the output.
class NDT_PUBLIC NDTVoxelMap : public perception::filters::voxel_grid::VoxelGrid<NDTVoxel>,
  public NDTMapBase<NDTVoxelMap, NDTVoxel>
{
public:
  using GridT = perception::filters::voxel_grid::VoxelGrid<NDTVoxel>;
  using HashT = common::geometry::spatial_hash::SpatialHash3d<GridT::IT>;

  /// Constructor
  /// \param voxel_grid_config config instance for the voxel grid
  /// \param hash_config config instance for the spatial hash
  NDTVoxelMap(
    const perception::filters::voxel_grid::Config & voxel_grid_config)
  : VoxelGrid(voxel_grid_config) {}

  /// Insert the point cloud to the map. It works by first adding the points to the cells
  /// they correspond to and then Adding the pointers to these cells to a spatial hash to allow
  /// for fast lookup later.
  /// \param msg PointCloud2 message to add.
  void insert(const sensor_msgs::msg::PointCloud2 & msg)
  {
    insert_to_grid(msg);
  }
  /// Return the nearest neighbouring cells given coordinates. Search radius is set in the spatial
  /// hash config.
  /// \param x x coordinate
  /// \param y y coordinate
  /// \param z z coordinate
  /// \return A vector containing NDT voxels. Each element conforms to the API of NDTUnit
  const NDTVoxel & cell_(float_t x, float_t y, float_t z)
  {
    return get_voxel(Eigen::Vector3d({x, y, z}));
  }

  /// Clear the data from the map.
  void clear()
  {
    VoxelGrid::clear();
  }

private:
  void insert_to_grid(const sensor_msgs::msg::PointCloud2 & msg)
  {
    sensor_msgs::PointCloud2ConstIterator<float> x_it(msg, "x");
    sensor_msgs::PointCloud2ConstIterator<float> y_it(msg, "y");
    sensor_msgs::PointCloud2ConstIterator<float> z_it(msg, "z");
    sensor_msgs::PointCloud2ConstIterator<float> intensity_it(msg, "intensity");

    while (x_it != x_it.end() &&
      y_it != y_it.end() &&
      z_it != z_it.end() &&
      intensity_it != intensity_it.end())
    {
      VoxelGrid::insert(Eigen::Vector3d({*x_it, *y_it, *z_it}));

      ++x_it;
      ++y_it;
      ++z_it;
      ++intensity_it;
    }

    // TODO(yunus.caliskan): this pointcloud iteration mess will be refactored in #102
    x_it = sensor_msgs::PointCloud2ConstIterator<float>(msg, "x");
    y_it = sensor_msgs::PointCloud2ConstIterator<float>(msg, "y");
    z_it = sensor_msgs::PointCloud2ConstIterator<float>(msg, "z");
    intensity_it = sensor_msgs::PointCloud2ConstIterator<float>(msg, "intensity");

    // TODO(yunus.caliskan) explore incremental covariance calculation methods instead
    // Covariance has to be computed after the mean is computed. The pointcloud is iterated
    // for a second time instead of having and managing a history of points in the voxel.
    while (x_it != x_it.end() &&
      y_it != y_it.end() &&
      z_it != z_it.end() &&
      intensity_it != intensity_it.end())
    {
      Eigen::Vector3d pt({*x_it, *y_it, *z_it});
      auto & vx = get_voxel(pt);
      // Only compute covariance if there are enough points in a voxel.
      if (vx.count() == 0U) {
        // All of the points are inserted above, so each point should return a voxel
        // that contains at least itself.
        throw std::length_error("A used voxel cannot be empty.");
      }
      if (vx.occupied()) {
        vx.add_point_for_covariance(pt);
      }

      ++x_it;
      ++y_it;
      ++z_it;
      ++intensity_it;
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
inline NDT_PUBLIC auto & xr_(const Eigen::Vector3d & pt)
{
  return pt(0);
}

template<>
inline NDT_PUBLIC auto & yr_(const Eigen::Vector3d & pt)
{
  return pt(1);
}

template<>
inline NDT_PUBLIC auto & zr_(const Eigen::Vector3d & pt)
{
  return pt(2);
}


/// Point adapters for NDTVoxel grid iterator.

template<>
inline NDT_PUBLIC auto x_(
  const perception::filters::voxel_grid::VoxelGrid<localization::ndt::NDTVoxel>::IT & pt)
{
  return static_cast<float32_t>(pt->second.get()(0));
}
template<>
inline NDT_PUBLIC auto y_(
  const perception::filters::voxel_grid::VoxelGrid<localization::ndt::NDTVoxel>::IT & pt)
{
  return static_cast<float32_t>(pt->second.get()(1));
}
template<>
inline NDT_PUBLIC auto z_(
  const perception::filters::voxel_grid::VoxelGrid<localization::ndt::NDTVoxel>::IT & pt)
{
  return static_cast<float32_t>(pt->second.get()(2));
}

template<>
inline NDT_PUBLIC auto & xr_(
  const perception::filters::voxel_grid::VoxelGrid<localization::ndt::NDTVoxel>::IT & pt)
{
  return pt->second.get()(0);
}
template<>
inline NDT_PUBLIC auto & yr_(
  const perception::filters::voxel_grid::VoxelGrid<localization::ndt::NDTVoxel>::IT & pt)
{
  return pt->second.get()(1);
}
template<>
inline NDT_PUBLIC auto & zr_(
  const perception::filters::voxel_grid::VoxelGrid<localization::ndt::NDTVoxel>::IT & pt)
{
  return pt->second.get()(2);
}

}  // namespace point_adapter
}  // namespace geometry
}  // namespace common
}  // namespace autoware
#endif  // NDT__NDT_MAP_HPP_
