// Copyright 2021 Tier IV, Inc.
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

/// \copyright Copyright 2021 Tier IV, Inc.
/// \file
/// \brief This file defines the VoxelGridOutlierFilter class.

#ifndef OUTLIER_FILTER__VOXEL_GRID_OUTLIER_FILTER_HPP_
#define OUTLIER_FILTER__VOXEL_GRID_OUTLIER_FILTER_HPP_

#include <memory>

#include "outlier_filter/visibility_control.hpp"

#include "sensor_msgs/msg/point_cloud2.hpp"
#include "pcl/filters/voxel_grid.h"
#include "pcl/search/pcl_search.h"


namespace autoware
{
namespace perception
{
namespace filters
{
namespace outlier_filter
{
/** \brief Namespace for the VoxelGridOutlierFilter library */
namespace voxel_grid_outlier_filter
{

/** \class VoxelGridOutlierFilter
 * \brief Library for applying voxel-based filtering on a PCL pointcloud
 */
class OUTLIER_FILTER_PUBLIC VoxelGridOutlierFilter
{
public:
  /** \brief Constructor for the VoxelGridOutlierFilter class
   * \param voxel_size_x Voxel leaf size of side X
   * \param voxel_size_y Voxel leaf size of side Y
   * \param voxel_size_z Voxel leaf size of side Z
   * \param voxel_points_threshold Minimum number of points per voxel
   */
  explicit OUTLIER_FILTER_PUBLIC VoxelGridOutlierFilter(
    float voxel_size_x, float voxel_size_y,
    float voxel_size_z, uint32_t voxel_points_threshold);

  /** \brief Filter function that runs the radius search algorithm.
   * \param input The input point cloud for filtering
   * \param output The output point cloud
   */
  void OUTLIER_FILTER_PUBLIC filter(
    const pcl::PointCloud<pcl::PointXYZ> & input,
    pcl::PointCloud<pcl::PointXYZ> & output);

  /** \brief Update dynamically configurable parameters
   * \param voxel_size_x Parameter that updates the voxel_size_x_ member variable
   * \param voxel_size_y Parameter that updates the voxel_size_y_ member variable
   * \param voxel_size_z Parameter that updates the voxel_size_z_ member variable
   * \param voxel_points_threshold Parameter that updates the voxel_points_threshold member variable
   */
  void OUTLIER_FILTER_PUBLIC update_parameters(
    float voxel_size_x, float voxel_size_y,
    float voxel_size_z,
    uint32_t voxel_points_threshold)
  {
    voxel_size_x_ = voxel_size_x;
    voxel_size_y_ = voxel_size_y;
    voxel_size_z_ = voxel_size_z;
    voxel_points_threshold_ = voxel_points_threshold;
  }

private:
  /** \brief Voxel leaf size of side X */
  float voxel_size_x_;

  /** \brief Voxel leaf size of side Y */
  float voxel_size_y_;

  /** \brief Voxel leaf size of side Z */
  float voxel_size_z_;

  /** \brief Minimum number of points per voxel */
  uint32_t voxel_points_threshold_;

  /** \brief Assembles a 3D grid over the pointcloud for filtering */
  std::shared_ptr<pcl::VoxelGrid<pcl::PointXYZ>> voxel_filter_;
};

}  // namespace voxel_grid_outlier_filter
}  // namespace outlier_filter
}  // namespace filters
}  // namespace perception
}  // namespace autoware

#endif  // OUTLIER_FILTER__VOXEL_GRID_OUTLIER_FILTER_HPP_
