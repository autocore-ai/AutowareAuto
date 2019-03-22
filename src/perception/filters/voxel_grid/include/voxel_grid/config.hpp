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

/// \file
/// \brief This file defines the configuration class for the voxel grid data structure

#ifndef VOXEL_GRID__CONFIG_HPP_
#define VOXEL_GRID__CONFIG_HPP_

#include <voxel_grid/visibility_control.hpp>
#include <geometry_msgs/msg/point32.hpp>
#include <cmath>


namespace autoware
{
namespace perception
{
namespace filters
{
namespace voxel_grid
{

// TODO(cvasfi) move this definition to a shared/common place

template<typename T>
inline T clamp(const T val, const T min, const T max)
{
  return (val < min) ? min : ((val > max) ? max : val);
}

// TODO(cvasfi) move this definition to a shared/common place
struct PointXYZIF
{
  float x, y, z, intensity;
  uint16_t id;
  static constexpr uint16_t END_OF_SCAN_ID = 65535u;
};

using PointXYZ = geometry_msgs::msg::Point32;


/// \brief A configuration class for the VoxelGrid data structure, also includes some helper
///        functionality for computing indices and centroids of voxels.
class VOXEL_GRID_PUBLIC Config
{
public:
  static constexpr float MIN_VOXEL_SIZE_M = 0.01F;
  /// \brief Constructor
  /// \param[in] min_point The minimum corner of the receptive field (box) for the voxel grid
  /// \param[in] max_point The maximum corner of the receptive field (box) for the voxel grid
  /// \param[in] voxel_size The size of each voxel
  /// \param[in] capacity The preallocated size of the voxel grid
  Config(
    const PointXYZ & min_point,
    const PointXYZ & max_point,
    const PointXYZ & voxel_size,
    const uint64_t capacity);
  /// \brief Gets the minimum corner of the voxel grid
  /// \return Fixed value
  const PointXYZ & get_min_point() const;
  /// \brief Gets the maximum corner of the voxel grid
  /// \return Fixed value
  const PointXYZ & get_max_point() const;
  /// \brief Gets the voxel size of the voxel grid
  /// \return Fixed value
  const PointXYZ & get_voxel_size() const;
  /// \brief Gets the capacity of the voxel grid
  /// \return Fixed value
  uint64_t get_capacity() const;
  /// \brief Computes index for a given point given the voxelgrid configuration parameters
  /// \param[in] pt The point for which the voxel index will be computed
  /// \return The index of the voxel for which this point will fall into
  /// \tparam PointT The point type taken. Assumed to have public float members x, y, and z.
  ///                Also assumed to have a default constructor.
  template<typename PointT>
  uint64_t index(const PointT & pt) const
  {
    // pt needs x, y and z
    const uint64_t idx = static_cast<uint64_t>(
      std::floor((clamp(pt.x, m_min_point.x, m_max_point.x) - m_min_point.x) * m_voxel_size_inv.x));
    const uint64_t jdx = static_cast<uint64_t>(
      std::floor((clamp(pt.y, m_min_point.y, m_max_point.y) - m_min_point.y) * m_voxel_size_inv.y));
    const uint64_t kdx = static_cast<uint64_t>(
      std::floor((clamp(pt.z, m_min_point.z, m_max_point.z) - m_min_point.z) * m_voxel_size_inv.z));
    return idx + (jdx * m_y_stride) + (kdx * m_z_stride);
  }
  /// \brief Computes the centroid for a given voxel index
  /// \param[in] index The index for a given voxel
  /// \return A point for whom the x, y and z fields are filled out
  /// \tparam PointT The point type returned. Assumed to have public float members x, y, and z.
  ///                Also assumed to have a default constructor.
  ///                Only the x, y, and z fields will be filled out.
  template<typename PointT>
  PointT centroid(const uint64_t index) const
  {
    // 'deserialize' indices
    const uint64_t zdx = index / m_z_stride;
    const uint64_t jdx = (index % m_z_stride);
    const uint64_t ydx = jdx / m_y_stride;
    const uint64_t xdx = jdx % m_y_stride;
    // compute centroid of voxel
    PointT pt;
    pt.x = ((static_cast<float>(xdx) + 0.5F) * m_voxel_size.x) + m_min_point.x;
    pt.y = ((static_cast<float>(ydx) + 0.5F) * m_voxel_size.y) + m_min_point.y;
    pt.z = ((static_cast<float>(zdx) + 0.5F) * m_voxel_size.z) + m_min_point.z;
    return pt;
  }

private:
  /// \brief Sanity check a range in a basis direction
  /// \return The number of voxels in the given basis direction (aka width in units of voxels)
  /// \param[in] min The lower bound in the specified basis direction
  /// \param[in] max The upper bound in the specified basis direction
  /// \param[in] size The voxel size in the specified basis direction
  uint64_t check_basis_direction(
    const float min,
    const float max,
    const float size) const;

  PointXYZ m_min_point;
  PointXYZ m_max_point;
  PointXYZ m_voxel_size;
  PointXYZ m_voxel_size_inv;
  uint64_t m_y_stride;
  uint64_t m_z_stride;
  uint64_t m_capacity;
};  // class Config
}  // namespace voxel_grid
}  // namespace filters
}  // namespace perception
}  // namespace autoware

#endif  // VOXEL_GRID__CONFIG_HPP_
