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
/// \brief This file defines the RadiusSearch2DFilter class.

#ifndef OUTLIER_FILTER__RADIUS_SEARCH_2D_FILTER_HPP_
#define OUTLIER_FILTER__RADIUS_SEARCH_2D_FILTER_HPP_

#include <vector>
#include <memory>

#include "outlier_filter/visibility_control.hpp"

#include "pcl/search/pcl_search.h"
#include "sensor_msgs/msg/point_cloud2.hpp"


namespace autoware
{
namespace perception
{
namespace filters
{
namespace outlier_filter
{
/** \brief Namespace for the RadiusSearch2DFilter library */
namespace radius_search_2d_filter
{

/** \class RadiusSearch2DFilter
 * \brief Library for using a radius based 2D filtering algorithm on a PCL pointcloud
 */
class OUTLIER_FILTER_PUBLIC RadiusSearch2DFilter
{
public:
  /** \brief Constructor for the RadiusSearch2DFilter class
   * \param search_radius Radius bounding a point's neighbors
   * \param min_neighbors Minimum number of surrounding neighbors for a point
   */
  OUTLIER_FILTER_PUBLIC RadiusSearch2DFilter(double search_radius, int min_neighbors);

  /** \brief Filter function that runs the radius search algorithm.
   * \param input The input point cloud for filtering
   * \param output The output point cloud
   */
  void OUTLIER_FILTER_PUBLIC filter(
    const pcl::PointCloud<pcl::PointXYZ> & input,
    pcl::PointCloud<pcl::PointXYZ> & output);

  /** \brief Update dynamically configurable parameters
   * \param search_radius Parameter that updates the search_radius_ member variable
   * \param min_neighbors Parameter that updates the min_neighbors_ member variable
   */
  void OUTLIER_FILTER_PUBLIC update_parameters(double search_radius, int min_neighbors)
  {
    search_radius_ = search_radius;
    min_neighbors_ = min_neighbors;
  }

private:
  /** \brief Radius bounding a point's neighbors */
  double search_radius_;

  /** \brief Minimum number of surrounding neighbors for a point to not be considered an outlier */
  int min_neighbors_;

  /** \brief PCL Search object used to perform the radial search */
  std::shared_ptr<pcl::search::Search<pcl::PointXY>> kd_tree_;
};
}  // namespace radius_search_2d_filter
}  // namespace outlier_filter
}  // namespace filters
}  // namespace perception
}  // namespace autoware

#endif  // OUTLIER_FILTER__RADIUS_SEARCH_2D_FILTER_HPP_
