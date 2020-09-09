// Copyright 2020 Tier IV, Inc
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef HAD_MAP_UTILS__HAD_MAP_VISUALIZATION_HPP_
#define HAD_MAP_UTILS__HAD_MAP_VISUALIZATION_HPP_

#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/point32.hpp>
#include <geometry_msgs/msg/polygon.hpp>
#include <rclcpp/rclcpp.hpp>
#include <common/types.hpp>

#include <lanelet2_core/LaneletMap.h>

#include <chrono>
#include <unordered_set>
#include <memory>
#include <string>
#include <cmath>
#include <vector>

#include "had_map_utils/visibility_control.hpp"

using autoware::common::types::float32_t;
using autoware::common::types::float64_t;
using autoware::common::types::bool8_t;

namespace autoware
{
namespace common
{
namespace had_map_utils
{
void HAD_MAP_UTILS_PUBLIC setColor(
  std_msgs::msg::ColorRGBA * cl,
  const float32_t & r, const float32_t & g, const float32_t & b, const float32_t & a);

void HAD_MAP_UTILS_PUBLIC  setMarkerHeader(
  visualization_msgs::msg::Marker * m, const int32_t & id, const rclcpp::Time & t,
  const std::string & frame_id, const std::string & ns,
  const std_msgs::msg::ColorRGBA & c, const int32_t & action, const int32_t & type,
  const float32_t & lss);

void HAD_MAP_UTILS_PUBLIC lineString2Marker(
  const rclcpp::Time & t,
  const lanelet::LineString3d & ls, visualization_msgs::msg::Marker * line_strip,
  const std::string & frame_id, const std::string & ns, const std_msgs::msg::ColorRGBA & c,
  const float32_t & lss);

void HAD_MAP_UTILS_PUBLIC lineString2Marker(
  const rclcpp::Time & t,
  const lanelet::ConstLineString3d & ls, visualization_msgs::msg::Marker * line_strip,
  const std::string & frame_id, const std::string & ns, const std_msgs::msg::ColorRGBA & c,
  const float32_t & lss);

visualization_msgs::msg::MarkerArray HAD_MAP_UTILS_PUBLIC
lineStringsAsMarkerArray(
  const rclcpp::Time & t,
  const std::string & ns,
  const lanelet::LineStrings3d & linestrings,
  const std_msgs::msg::ColorRGBA & c);

visualization_msgs::msg::MarkerArray HAD_MAP_UTILS_PUBLIC laneletsBoundaryAsMarkerArray(
  const rclcpp::Time & t,
  const lanelet::ConstLanelets & lanelets,
  const std_msgs::msg::ColorRGBA & c,
  const bool8_t & viz_centerline);

void HAD_MAP_UTILS_PUBLIC basicPolygon2Marker(
  const rclcpp::Time & t,
  const int32_t & line_id, const lanelet::BasicPolygon3d & pg,
  visualization_msgs::msg::Marker * line_strip,
  const std::string & frame_id, const std::string & ns,
  const std_msgs::msg::ColorRGBA & c, const float32_t & lss);

visualization_msgs::msg::MarkerArray HAD_MAP_UTILS_PUBLIC areasBoundaryAsMarkerArray(
  const rclcpp::Time & t,
  const std::string & ns, const lanelet::Areas & areas,
  const std_msgs::msg::ColorRGBA & c);

visualization_msgs::msg::MarkerArray HAD_MAP_UTILS_PUBLIC polygonsBoundaryAsMarkerArray(
  const rclcpp::Time & t,
  const std::string & ns, const lanelet::Polygons3d & s,
  const std_msgs::msg::ColorRGBA & c);

void HAD_MAP_UTILS_PUBLIC bbox2Marker(
  const rclcpp::Time & t, const int32_t & line_id,
  const float64_t lower[], const float64_t upper[],
  visualization_msgs::msg::Marker * line_strip,
  const std::string & frame_id, const std::string & ns,
  const std_msgs::msg::ColorRGBA & c, const float32_t & lss);

visualization_msgs::msg::MarkerArray HAD_MAP_UTILS_PUBLIC boundingBoxAsMarkerArray(
  const rclcpp::Time & t,
  const std::string & ns, const float64_t upper[], const float64_t lower[],
  const std_msgs::msg::ColorRGBA & c);

void HAD_MAP_UTILS_PUBLIC lineString2Triangle(
  const lanelet::LineString3d & ls,
  std::vector<geometry_msgs::msg::Polygon> * triangles);

void HAD_MAP_UTILS_PUBLIC lanelet2Triangle(
  const lanelet::ConstLanelet & ll,
  std::vector<geometry_msgs::msg::Polygon> * triangles);

void HAD_MAP_UTILS_PUBLIC polygon2Triangle(
  const geometry_msgs::msg::Polygon & polygon,
  std::vector<geometry_msgs::msg::Polygon> * triangles);

void HAD_MAP_UTILS_PUBLIC lineString2Polygon(
  const lanelet::LineString3d & ls,
  geometry_msgs::msg::Polygon * polygon);

void HAD_MAP_UTILS_PUBLIC lanelet2Polygon(
  const lanelet::ConstLanelet & ll,
  geometry_msgs::msg::Polygon * polygon);

visualization_msgs::msg::MarkerArray HAD_MAP_UTILS_PUBLIC lineStringsAsTriangleMarkerArray(
  const rclcpp::Time & t, const std::string & ns, const lanelet::LineStrings3d & linestrings,
  const std_msgs::msg::ColorRGBA & c);

visualization_msgs::msg::MarkerArray HAD_MAP_UTILS_PUBLIC laneletsAsTriangleMarkerArray(
  const rclcpp::Time & t, const std::string & ns, const lanelet::ConstLanelets & lanelets,
  const std_msgs::msg::ColorRGBA & c);

}  // namespace had_map_utils
}  // namespace common
}  // namespace autoware

#endif  // HAD_MAP_UTILS__HAD_MAP_VISUALIZATION_HPP_
