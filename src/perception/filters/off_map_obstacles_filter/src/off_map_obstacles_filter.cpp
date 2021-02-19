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

#include "off_map_obstacles_filter/off_map_obstacles_filter.hpp"

#include <cmath>
#include <memory>
#include <vector>

#include "common/types.hpp"

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "Eigen/Geometry"
#include "lanelet2_core/geometry/Lanelet.h"
#include "lanelet2_core/geometry/Point.h"
#include "lanelet2_core/geometry/Polygon.h"
#include "lanelet2_core/LaneletMap.h"

#include "boost/geometry/geometries/geometries.hpp"
#include "boost/geometry.hpp"

namespace autoware
{

namespace off_map_obstacles_filter
{

using float32_t = autoware::common::types::float32_t;
using float64_t = autoware::common::types::float64_t;
namespace utils = lanelet::utils;

OffMapObstaclesFilter::OffMapObstaclesFilter(
  std::shared_ptr<lanelet::LaneletMap> map,
  float64_t overlap_threshold)
: m_map{map}, m_overlap_threshold{overlap_threshold} {}

/// \param bbox The bounding box with coordinates in the base_link frame.
/// \param map_from_base_link The transform that transforms things from base_link to map.
/// \return A polygon with the four corners of the bounding box projected into 2D.
static lanelet::Polygon2d polygon_for_bbox(
  const Eigen::Isometry2f & map_from_base_link,
  const autoware_auto_msgs::msg::BoundingBox & bbox)
{
  // Create lanelet polygon from bounding box
  // We can neglect z here already, I had a peek into bounding_box.cpp and it's basically 2d only
  const Eigen::Vector2f dx {0.5f * bbox.size.x, 0.0f};
  const Eigen::Vector2f dy {0.0f, 0.5f * bbox.size.y};
  const Eigen::Vector2f centroid {bbox.centroid.x, bbox.centroid.y};
  // Why the heck is this offset by π/2?
  const float32_t yaw =
    std::atan2(
    bbox.orientation.z,
    bbox.orientation.w) * 2.0f + autoware::common::types::PI_2;

  const Eigen::Rotation2D<float32_t> orientation {yaw};

  // Construct the points in clockwise order – required by lanelet::Polygon2d
  const Eigen::Vector2f p0 = map_from_base_link * (centroid + orientation *
    (Eigen::Vector2f::Zero() - dx - dy));
  const Eigen::Vector2f p1 = map_from_base_link * (centroid + orientation *
    (Eigen::Vector2f::Zero() - dx + dy));
  const Eigen::Vector2f p2 = map_from_base_link * (centroid + orientation *
    (Eigen::Vector2f::Zero() + dx + dy));
  const Eigen::Vector2f p3 = map_from_base_link * (centroid + orientation *
    (Eigen::Vector2f::Zero() + dx - dy));

  lanelet::Polygon2d bbox_poly;
  bbox_poly.push_back(lanelet::Point2d{utils::getId(), p0.x(), p0.y()});
  bbox_poly.push_back(lanelet::Point2d{utils::getId(), p1.x(), p1.y()});
  bbox_poly.push_back(lanelet::Point2d{utils::getId(), p2.x(), p2.y()});
  bbox_poly.push_back(lanelet::Point2d{utils::getId(), p3.x(), p3.y()});
  return bbox_poly;
}

/// \brief Convert TransformStamped to Isometry2f – this assumes that the direction of the z axis
/// is the same in the two frames.
/// \param tfs A TransformStamped. Its rotation quaternion needs to be approximately 0 in x and y.
/// \return An Isometry2d that can be used to transform Eigen Vectors.
static Eigen::Isometry2d convert_2d_transform(const geometry_msgs::msg::TransformStamped & tfs)
{
  const auto & tf = tfs.transform;
  Eigen::Isometry2d translation;
  // Eigen does this only in assigments, not constructors.
  translation = Eigen::Translation2d {tf.translation.x, tf.translation.y};
  const float64_t yaw = std::atan2(tf.rotation.z, tf.rotation.w) * 2.0;
  Eigen::Isometry2d rotation;
  rotation = Eigen::Rotation2D<float64_t>{yaw};
  const Eigen::Isometry2d isometry = translation * rotation;
  return isometry;
}

visualization_msgs::msg::MarkerArray OffMapObstaclesFilter::bboxes_in_map_frame_viz(
  const geometry_msgs::msg::TransformStamped & map_from_base_link,
  const autoware_auto_msgs::msg::BoundingBoxArray & msg) const
{
  const Eigen::Isometry2d map_from_base_link_isometry = convert_2d_transform(map_from_base_link);

  visualization_msgs::msg::MarkerArray array;
  int id = 0;
  for (const auto & bbox : msg.boxes) {
    const auto polygon = polygon_for_bbox(map_from_base_link_isometry.cast<float32_t>(), bbox);
    visualization_msgs::msg::Marker marker;
    marker.header = msg.header;
    marker.header.frame_id = "map";
    marker.ns = "off_map_obstacles_filter";
    marker.id = id++;
    marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.position.x = 0.0;
    marker.pose.position.y = 0.0;
    marker.pose.position.z = 0.0;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.1;
    marker.scale.y = 0.1;
    marker.scale.z = 0.1;
    marker.color.r = 1.0f;
    marker.color.g = 0.0f;
    marker.color.b = 0.3f;
    marker.color.a = 1.0f;
    marker.lifetime.sec = 1;
    geometry_msgs::msg::Point p;
    p.z = 0.0;
    p.x = polygon[0].x();
    p.y = polygon[0].y();
    marker.points.push_back(p);
    p.x = polygon[1].x();
    p.y = polygon[1].y();
    marker.points.push_back(p);
    p.x = polygon[2].x();
    p.y = polygon[2].y();
    marker.points.push_back(p);
    p.x = polygon[3].x();
    p.y = polygon[3].y();
    marker.points.push_back(p);
    p.x = polygon[0].x();
    p.y = polygon[0].y();
    marker.points.push_back(p);
    array.markers.push_back(marker);
  }
  return array;
}

/// \brief Checks if a bbox is on the map.
/// \param map The lanelet map, correctly transformed into the map frame.
/// \param map_from_base_link An Isometry2d that can be used to transform Eigen Vectors.
/// \param overlap_threshold What fraction of a bbox needs to overlap the map to be considered
/// "on the map".
/// \param bbox An obstacle bounding box.
static bool bbox_is_on_map(
  const lanelet::LaneletMap & map,
  const Eigen::Isometry2f & map_from_base_link,
  const float64_t overlap_threshold,
  const autoware_auto_msgs::msg::BoundingBox & bbox)
{
  const lanelet::Polygon2d bbox_poly = polygon_for_bbox(map_from_base_link, bbox);
  // See https://github.com/fzi-forschungszentrum-informatik/Lanelet2/blob/master/lanelet2_core/doc/GeometryPrimer.md
  const lanelet::ConstHybridPolygon2d bbox_poly_hybrid = utils::toHybrid(bbox_poly);
  const float64_t total_area = lanelet::geometry::area(bbox_poly_hybrid);

  // Now find possibly-intersecting lanelets and areas
  const lanelet::BoundingBox2d bbox_bbox = lanelet::geometry::boundingBox2d(bbox_poly);
  const std::vector<lanelet::ConstLanelet> ll_candidates = map.laneletLayer.search(bbox_bbox);
  const std::vector<lanelet::ConstArea> area_candidates = map.areaLayer.search(bbox_bbox);

  // The output point type needs to be the same, but it seems easier to use boost's builtin polygon
  // as the (multi)polygon type for the output.
  typedef boost::geometry::model::polygon<Eigen::Vector2d> polygon_t;
  typedef boost::geometry::model::multi_polygon<polygon_t> mpolygon_t;
  mpolygon_t output;

  // For each of them, check if an intersection exists
  float64_t overlap_area = 0.0;
  for (const auto candidate : ll_candidates) {
    // Annoying – this seems to be the only way to do the intersection
    lanelet::Polygon2d ll_poly;
    for (const auto p : candidate.polygon2d()) {
      ll_poly.push_back(lanelet::Point2d{utils::getId(), p.x(), p.y()});
    }
    lanelet::ConstHybridPolygon2d ll_poly_hybrid = utils::toHybrid(ll_poly);
    boost::geometry::intersection(ll_poly_hybrid, bbox_poly_hybrid, output);
    overlap_area += boost::geometry::area(output);
    if (overlap_area / total_area >= overlap_threshold) {
      return true;
    }
  }
  for (const auto candidate : area_candidates) {
    if (!candidate.hasAttribute("subtype") || !candidate.hasAttribute("cad_id")) {continue;}
    if (candidate.attribute("subtype") != "parking_access" &&
      candidate.attribute("subtype") != "parking_spot") {continue;}
    // Annoying – this seems to be the only way to do the intersection
    lanelet::Polygon2d area_poly;
    for (const auto p : candidate.outerBoundPolygon()) {
      area_poly.push_back(lanelet::Point2d{utils::getId(), p.x(), p.y()});
    }
    lanelet::ConstHybridPolygon2d area_poly_hybrid = utils::toHybrid(area_poly);
    boost::geometry::intersection(area_poly_hybrid, bbox_poly_hybrid, output);
    overlap_area += boost::geometry::area(output);
    if (overlap_area / total_area >= overlap_threshold) {
      return true;
    }
  }
  return false;
}

void OffMapObstaclesFilter::remove_off_map_bboxes(
  const geometry_msgs::msg::TransformStamped & map_from_base_link,
  autoware_auto_msgs::msg::BoundingBoxArray & msg) const
{
  const Eigen::Isometry2d map_from_base_link_isometry = convert_2d_transform(map_from_base_link);
  msg.boxes.erase(
    std::remove_if(
      msg.boxes.begin(),
      msg.boxes.end(),
      [this, &map_from_base_link_isometry](const auto & bbox) {
        return !bbox_is_on_map(
          *this->m_map, map_from_base_link_isometry.cast<float32_t>(),
          this->m_overlap_threshold, bbox);
      }),
    msg.boxes.end());
}

}  // namespace off_map_obstacles_filter

}  // namespace autoware
