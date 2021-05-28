// Copyright 2021 The Autoware Foundation
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

/// \copyright Copyright 2021 The Autoware Foundation
/// \file
/// \brief This file defines the polygon_remover class.

#ifndef POLYGON_REMOVER__POLYGON_REMOVER_HPP_
#define POLYGON_REMOVER__POLYGON_REMOVER_HPP_

#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Polygon_2_algorithms.h>
#include <polygon_remover/visibility_control.hpp>
#include <common/types.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <visualization_msgs/msg/marker.h>
#include <geometry_msgs/msg/polygon.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <cmath>
#include <algorithm>
#include <memory>
#include <limits>
#include <string>
#include <vector>

namespace autoware
{
namespace perception
{
namespace filters
{
namespace polygon_remover
{

class POLYGON_REMOVER_PUBLIC PolygonRemover
{
public:
  using SharedPtr = std::shared_ptr<PolygonRemover>;
  using ConstSharedPtr = const std::shared_ptr<PolygonRemover>;

  using PointXYZI = common::types::PointXYZI;
  using PointCloud2 = sensor_msgs::msg::PointCloud2;
  using Polygon = geometry_msgs::msg::Polygon;
  using Marker = visualization_msgs::msg::Marker;
  typedef CGAL::Exact_predicates_inexact_constructions_kernel K;
  typedef K::Point_2 PointCgal;
  using bool8_t = autoware::common::types::bool8_t;

  explicit PolygonRemover(bool8_t will_visualize);

  /// \brief Removes the given geometry_msgs polygon from the given cloud and returns it.
  /// \param cloud_in Input Point Cloud Shared Pointer
  /// \param polygon_in Input Polygon
  /// \return Filtered Point Cloud Shared Pointer
  static PointCloud2::SharedPtr remove_polygon_geometry_from_cloud(
    const PointCloud2::ConstSharedPtr & cloud_in,
    const Polygon::ConstSharedPtr & polygon_in);

  /// \brief Removes the given cgal polygon from the given cloud and returns it.
  /// \param cloud_in_ptr Input Point Cloud Shared Pointer
  /// \param polyline_polygon Input Polygon
  /// \return Filtered Point Cloud Shared Pointer
  static PointCloud2::SharedPtr  remove_polygon_cgal_from_cloud(
    const PointCloud2::ConstSharedPtr & cloud_in_ptr,
    const std::vector<PointCgal> & polyline_polygon);

  /// \brief Converts a polygon from geometry_msgs::msg::Polygon to std::vector<PointCgal>
  /// \param polygon_in Input Polygon in geometry_msgs::msg::Polygon
  /// \return Polygon as std::vector<PointCgal>
  static std::vector<PointCgal> polygon_geometry_to_cgal(
    const Polygon::ConstSharedPtr & polygon_in);

  /// \brief Updates the stored polygon to be used later on
  /// \param polygon_in Input Polygon
  void update_polygon(const Polygon::ConstSharedPtr & polygon_in);

  /// \brief Removes the stored polygon from the point cloud and returns the filtered point cloud.
  /// \param cloud_in Input Point Cloud Shared Pointer
  /// \return Filtered Point Cloud Shared Pointer
  PointCloud2::SharedPtr remove_updated_polygon_from_cloud(
    const PointCloud2::ConstSharedPtr & cloud_in);

  bool8_t polygon_is_initialized() const;

  /// \brief Set the frame id of marker which will be used for visualization
  /// \param frame_id Frame id
  void set_marker_frame_id(const std::string & frame_id);

  /// \brief Returns the marker for visualization.
  /// \return Marker
  const Marker & get_marker() const;

private:
  bool8_t polygon_is_initialized_;
  bool8_t will_visualize_;
  std::vector<PointCgal> polygon_cgal_;
  Marker marker_;
};


}  // namespace polygon_remover
}  // namespace filters
}  // namespace perception
}  // namespace autoware

#endif  // POLYGON_REMOVER__POLYGON_REMOVER_HPP_
