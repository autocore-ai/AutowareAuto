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

#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Polygon_2_algorithms.h>
#include <point_cloud_msg_wrapper/point_cloud_msg_wrapper.hpp>
#include <common/types.hpp>
#include <geometry_msgs/msg/polygon.hpp>
#include <tuple>
#include <memory>
#include <string>
#include <vector>
#include "polygon_remover/polygon_remover.hpp"

namespace autoware
{
namespace perception
{
namespace filters
{
namespace polygon_remover
{
using common::types::PointXYZI;
using sensor_msgs::msg::PointCloud2;
using Polygon = geometry_msgs::msg::Polygon;
using K = CGAL::Exact_predicates_inexact_constructions_kernel;
using PointCgal = K::Point_2;
using autoware::common::types::bool8_t;

PolygonRemover::PolygonRemover(bool8_t will_visualize)
: polygon_is_initialized_{false},
  will_visualize_{will_visualize}
{
}

PointCloud2::SharedPtr PolygonRemover::remove_polygon_geometry_from_cloud(
  const PointCloud2::ConstSharedPtr & cloud_in,
  const Polygon::ConstSharedPtr & polygon_in)
{
  return remove_polygon_cgal_from_cloud(
    cloud_in,
    polygon_geometry_to_cgal(polygon_in));
}

PointCloud2::SharedPtr PolygonRemover::remove_polygon_cgal_from_cloud(
  const PointCloud2::ConstSharedPtr & cloud_in_ptr,
  const std::vector<PointCgal> & polyline_polygon)
{
  PointCloud2::SharedPtr cloud_filtered_ptr = std::make_shared<PointCloud2>();

  using CloudModifier = point_cloud_msg_wrapper::PointCloud2Modifier<PointXYZI>;
  using CloudView = point_cloud_msg_wrapper::PointCloud2View<PointXYZI>;

  CloudModifier cloud_modifier_filtered(*cloud_filtered_ptr, "");
  cloud_filtered_ptr->header = cloud_in_ptr->header;

  CloudView cloud_view_in(*cloud_in_ptr);
  cloud_modifier_filtered.resize(static_cast<uint32_t>(cloud_view_in.size()));

  auto point_is_outside_polygon = [&polyline_polygon](const PointXYZI & point) {
      auto result = CGAL::bounded_side_2(
        polyline_polygon.begin(), polyline_polygon.end(),
        PointCgal(point.x, point.y), K());
      return result == CGAL::ON_UNBOUNDED_SIDE;  // not INSIDE or ON the polygon
    };

  auto new_end = std::copy_if(
    cloud_view_in.cbegin(),
    cloud_view_in.cend(),
    cloud_modifier_filtered.begin(),
    [&point_is_outside_polygon](const PointXYZI & point) {
      return point_is_outside_polygon(point);
    });

  cloud_modifier_filtered.resize(
    static_cast<uint32_t>(std::distance(
      cloud_modifier_filtered.begin(),
      new_end)));
  return cloud_filtered_ptr;
}

std::vector<PointCgal> PolygonRemover::polygon_geometry_to_cgal(
  const Polygon::ConstSharedPtr & polygon_in)
{
  std::vector<PointCgal> polyline_polygon;
  if (polygon_in->points.size() < 3) {
    throw std::length_error("Polygon vertex count should be larger than 2.");
  }
  const auto & vertices_in = polygon_in->points;
  polyline_polygon.resize(vertices_in.size());
  std::transform(
    polygon_in->points.begin(),
    polygon_in->points.end(),
    polyline_polygon.begin(),
    [](const geometry_msgs::msg::Point32 & p_in) {
      return PointCgal(p_in.x, p_in.y);
    });
  return polyline_polygon;
}

void PolygonRemover::update_polygon(const Polygon::ConstSharedPtr & polygon_in)
{
  polygon_cgal_ = polygon_geometry_to_cgal(polygon_in);
  if (will_visualize_) {
    marker_.ns = "ns_polygon_remover";
    marker_.id = 0;
    marker_.type = visualization_msgs::msg::Marker::LINE_LIST;
    marker_.action = visualization_msgs::msg::Marker::ADD;
    marker_.pose.orientation.w = 1.0;
    marker_.scale.x = 0.2;
    marker_.color.a = 1.0;
    marker_.color.r = 1.0;
    marker_.color.g = 1.0;
    marker_.color.b = 1.0;

    auto make_point = [](float x, float y, float z) {
        geometry_msgs::msg::Point point;
        point.x = static_cast<double>(x);
        point.y = static_cast<double>(y);
        point.z = static_cast<double>(z);
        return point;
      };

    for (size_t index_cur = 0; index_cur < polygon_cgal_.size(); ++index_cur) {
      const auto & vertex = polygon_cgal_.at(index_cur);

      // Take the last segment into consideration to connect the loop
      size_t index_next = index_cur == polygon_cgal_.size() - 1 ? 0UL : index_cur + 1;
      const auto & vertex_next = polygon_cgal_.at(index_next);

      // Build upper ring
      auto vertex_up_cur = make_point(
        static_cast<float>(vertex.x()),
        static_cast<float>(vertex.y()), 5.0F);
      auto vertex_up_next =
        make_point(
        static_cast<float>(vertex_next.x()),
        static_cast<float>(vertex_next.y()), 5.0F);
      marker_.points.emplace_back(vertex_up_cur);
      marker_.points.emplace_back(vertex_up_next);

      // Build lower ring
      auto vertex_down_cur =
        make_point(static_cast<float>(vertex.x()), static_cast<float>(vertex.y()), -5.0F);
      auto vertex_down_next =
        make_point(
        static_cast<float>(vertex_next.x()),
        static_cast<float>(vertex_next.y()), -5.0F);
      marker_.points.emplace_back(vertex_down_cur);
      marker_.points.emplace_back(vertex_down_next);

      // Connect up and down vertices
      marker_.points.emplace_back(vertex_up_cur);
      marker_.points.emplace_back(vertex_down_cur);
    }
  }
  polygon_is_initialized_ = true;
}

PointCloud2::SharedPtr PolygonRemover::remove_updated_polygon_from_cloud(
  const PointCloud2::ConstSharedPtr & cloud_in)
{
  if (will_visualize_) {
    set_marker_frame_id(cloud_in->header.frame_id);
  }
  if (!polygon_is_initialized_) {
    throw std::runtime_error(
            "Polygon is not initialized. Please use `update_polygon` first.");
  }
  return remove_polygon_cgal_from_cloud(cloud_in, polygon_cgal_);
}

bool8_t PolygonRemover::polygon_is_initialized() const
{
  return polygon_is_initialized_;
}

void PolygonRemover::set_marker_frame_id(const std::string & frame_id)
{
  marker_.header.frame_id = frame_id;
}

const PolygonRemover::Marker & PolygonRemover::get_marker() const
{
  return marker_;
}

}  // namespace polygon_remover
}  // namespace filters
}  // namespace perception
}  // namespace autoware
