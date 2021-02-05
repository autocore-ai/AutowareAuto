// Copyright 2020 The Autoware Foundation
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

/// \brief main function for lanelet2_map_provider


#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>

#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <common/types.hpp>

#include <chrono>
#include <memory>
#include <unordered_set>

#include "lanelet2_map_provider/lanelet2_map_provider.hpp"
#include "autoware_auto_msgs/srv/had_map_service.hpp"
#include "autoware_auto_msgs/msg/had_map_bin.hpp"

#include "had_map_utils/had_map_conversion.hpp"
#include "had_map_utils/had_map_query.hpp"
#include "had_map_utils/had_map_visualization.hpp"

#include "lanelet2_map_provider/lanelet2_map_visualizer.hpp"

using autoware::common::types::bool8_t;
using autoware::common::types::float64_t;

using namespace std::chrono_literals;

namespace autoware
{
namespace lanelet2_map_provider
{
void insertMarkerArray(
  visualization_msgs::msg::MarkerArray & a1,
  const visualization_msgs::msg::MarkerArray & a2)
{
  if (a2.markers.size() > 0) {
    a1.markers.insert(a1.markers.end(), a2.markers.begin(), a2.markers.end());
  }
}

void Lanelet2MapVisualizer::visualize_map_callback(
  rclcpp::Client<autoware_auto_msgs::srv::HADMapService>::SharedFuture response)
{
  auto msg = response.get()->map;

  std::shared_ptr<lanelet::LaneletMap> sub_map = std::make_shared<lanelet::LaneletMap>();
  autoware::common::had_map_utils::fromBinaryMsg(msg, sub_map);

  auto lls = autoware::common::had_map_utils::getConstLaneletLayer(sub_map);

  std_msgs::msg::ColorRGBA color_lane_bounds, color_parking_bounds,
    color_parking_access_bounds, color_geom_bounds,
    color_lanelets, color_parking, color_parking_access,
    color_pickup_dropoff;
  autoware::common::had_map_utils::setColor(
    &color_lane_bounds, 1.0f, 1.0f, 1.0f, 1.0f);
  autoware::common::had_map_utils::setColor(
    &color_parking_bounds, 1.0f, 1.0f, 1.0f, 1.0f);
  autoware::common::had_map_utils::setColor(
    &color_parking_access_bounds, 1.0f, 1.0f, 1.0f, 1.0f);
  autoware::common::had_map_utils::setColor(
    &color_geom_bounds, 0.0f, 0.0f, 1.0f, 1.0f);
  autoware::common::had_map_utils::setColor(
    &color_lanelets, 0.2f, 0.5f, 0.6f, 0.6f);
  autoware::common::had_map_utils::setColor(
    &color_parking, 0.3f, 0.3f, 0.7f, 0.5f);
  autoware::common::had_map_utils::setColor(
    &color_parking_access, 0.3f, 0.7f, 0.3f, 0.5f);
  autoware::common::had_map_utils::setColor(
    &color_pickup_dropoff, 0.9f, 0.2f, 0.1f, 0.7f);

  visualization_msgs::msg::MarkerArray map_marker_array;

  rclcpp::Time marker_t = rclcpp::Time(0);
  insertMarkerArray(
    map_marker_array,
    autoware::common::had_map_utils::laneletsBoundaryAsMarkerArray(
      marker_t, lls,
      color_lane_bounds, true));
  insertMarkerArray(
    map_marker_array,
    autoware::common::had_map_utils::laneletsAsTriangleMarkerArray(
      marker_t,
      "lanelet_triangles", lls, color_lanelets));

  // for parking spots defined as areas (LaneletOSM definition)
  auto ll_areas = autoware::common::had_map_utils::getAreaLayer(sub_map);
  auto ll_parking_areas = autoware::common::had_map_utils::subtypeAreas(ll_areas, "parking_spot");
  auto ll_parking_access_areas = autoware::common::had_map_utils::subtypeAreas(
    ll_areas,
    "parking_access");

  insertMarkerArray(
    map_marker_array,
    autoware::common::had_map_utils::areasBoundaryAsMarkerArray(
      marker_t, "parking_area_bounds",
      ll_parking_areas, color_parking_bounds));
  insertMarkerArray(
    map_marker_array,
    autoware::common::had_map_utils::areasBoundaryAsMarkerArray(
      marker_t, "parking_access_area_bounds",
      ll_parking_access_areas, color_parking_bounds));
  insertMarkerArray(
    map_marker_array,
    autoware::common::had_map_utils::areasAsTriangleMarkerArray(
      marker_t, "parking_area_triangles",
      ll_parking_areas, color_parking));
  insertMarkerArray(
    map_marker_array,
    autoware::common::had_map_utils::areasAsTriangleMarkerArray(
      marker_t, "parking_access_area_triangles",
      ll_parking_access_areas, color_parking_access));

  // Periodic publishing is a temp. hack until the rviz in ade has transient_local qos support.
  m_timer = this->create_wall_timer(
    std::chrono::seconds(1),
    [this, map_marker_array]() {m_viz_pub->publish(map_marker_array);});
}

Lanelet2MapVisualizer::Lanelet2MapVisualizer(const rclcpp::NodeOptions & options)
: Node("lanelet2_map_visualizer", options)
{
  m_client =
    this->create_client<autoware_auto_msgs::srv::HADMapService>("HAD_Map_Service");

  m_viz_pub = this->create_publisher<visualization_msgs::msg::MarkerArray>(
    "viz_had_map",
    rclcpp::QoS(rclcpp::KeepLast(5U)).transient_local());

  auto request = std::make_shared<autoware_auto_msgs::srv::HADMapService::Request>();
  bool8_t use_geom_bounds = false;

  float64_t d = 20.0;
  float64_t pos[] = {25.0, 25.0, 0.0};
  float64_t upper[] = {pos[0] + d, pos[1] + d, 0.0};
  float64_t lower[] = {pos[0] - d, pos[1] - d, 0.0};
  request->geom_upper_bound.clear();
  request->geom_lower_bound.clear();
  if (!use_geom_bounds) {
    request->requested_primitives.push_back(
      autoware_auto_msgs::srv::HADMapService::Request::FULL_MAP);
  } else {
    request->requested_primitives.push_back(
      autoware_auto_msgs::srv::HADMapService::Request::DRIVEABLE_GEOMETRY);
    for (size_t i = 0; i < 3; i++) {
      request->geom_upper_bound.push_back(upper[i]);
      request->geom_lower_bound.push_back(lower[i]);
    }
  }
  while (!m_client->wait_for_service(1s)) {
    if (!rclcpp::ok()) {
      RCLCPP_INFO(this->get_logger(), "Interrupted while waiting for the service. Exiting");
      return;
    }
    RCLCPP_INFO(this->get_logger(), "Service not available, waiting again...");
  }

  auto result = m_client->async_send_request(
    request,
    std::bind(&Lanelet2MapVisualizer::visualize_map_callback, this, std::placeholders::_1));
}

}  // namespace lanelet2_map_provider

}  // namespace autoware

RCLCPP_COMPONENTS_REGISTER_NODE(
  autoware::lanelet2_map_provider::Lanelet2MapVisualizer)
