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

/// \copyright Copyright 2020 The Autoware Foundation

#include "lanelet2_map_provider/lanelet2_map_provider_node.hpp"

#include <tf2_ros/buffer_interface.h>
#include <tf2_ros/visibility_control.h>
#include <tf2/buffer_core.h>
#include <tf2/time.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <geometry_msgs/msg/transform_stamped.h>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/time_source.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <common/types.hpp>
#include <chrono>
#include <string>
#include <memory>
#include <utility>

#include "autoware_auto_msgs/srv/had_map_service.hpp"
#include "autoware_auto_msgs/msg/had_map_bin.hpp"
#include "had_map_utils/had_map_conversion.hpp"
#include "had_map_utils/had_map_query.hpp"

using autoware::common::types::bool8_t;

namespace autoware
{
namespace lanelet2_map_provider
{
Lanelet2MapProviderNode::Lanelet2MapProviderNode(const rclcpp::NodeOptions & options)
: Node("Lanelet2MapProvider", options)
{
  const std::string map_filename = declare_parameter("map_osm_file").get<std::string>();
  const float64_t origin_offset_lat = declare_parameter("origin_offset_lat", 0.0);
  const float64_t origin_offset_lon = declare_parameter("origin_offset_lon", 0.0);
  if (has_parameter("latitude") && has_parameter("longitude") && has_parameter("elevation")) {
    const float64_t origin_lat = declare_parameter("latitude").get<float64_t>();
    const float64_t origin_lon = declare_parameter("longitude").get<float64_t>();
    const float64_t origin_alt = declare_parameter("elevation").get<float64_t>();
    LatLonAlt map_origin{origin_lat, origin_lon, origin_alt};
    m_map_provider = std::make_unique<Lanelet2MapProvider>(
      map_filename, map_origin, origin_offset_lat, origin_offset_lon);
  } else {
    /// This could potentially also read the same yaml that the ndt map publisher reads
    auto earth_from_map = get_map_origin();
    m_map_provider = std::make_unique<Lanelet2MapProvider>(
      map_filename, std::move(
        earth_from_map), origin_offset_lat, origin_offset_lon);
  }

  m_map_service =
    this->create_service<autoware_auto_msgs::srv::HADMapService>(
    "HAD_Map_Service", std::bind(
      &Lanelet2MapProviderNode::handle_request, this,
      std::placeholders::_1, std::placeholders::_2));
}

geometry_msgs::msg::TransformStamped Lanelet2MapProviderNode::get_map_origin()
{
  std::unique_ptr<tf2_ros::Buffer> buffer = nullptr;
  rclcpp::Clock::SharedPtr clock = std::make_shared<rclcpp::Clock>(RCL_ROS_TIME);
  rclcpp::TimeSource timesource;
  timesource.attachClock(clock);
  buffer = std::make_unique<tf2_ros::Buffer>(clock);
  std::shared_ptr<tf2_ros::TransformListener> transform_listener_ =
    std::make_shared<tf2_ros::TransformListener>(*buffer);

  rclcpp::WallRate loop_rate(std::chrono::milliseconds(100));

  while (rclcpp::ok()) {
    try {
      geometry_msgs::msg::TransformStamped tfs =
        buffer->lookupTransform("earth", "map", tf2::TimePointZero);
      // No exception – we got the transform
      return tfs;
    } catch (tf2::TransformException & ex) {
      RCLCPP_INFO(
        this->get_logger(),
        "Waiting for earth to map transform - please start ndt_map_publisher .... : %s",
        ex.what());
    }
    loop_rate.sleep();
  }
  throw std::runtime_error("rclcpp error");
}


// This function should extract requested correct submap from the fullmap
// convert to binary format and then fill response
void Lanelet2MapProviderNode::handle_request(
  std::shared_ptr<autoware_auto_msgs::srv::HADMapService_Request> request,
  std::shared_ptr<autoware_auto_msgs::srv::HADMapService_Response> response)
{
  autoware_auto_msgs::msg::HADMapBin msg;
  msg.header.frame_id = "map";

  // TODO(simon) add map version and format information to message header
  // msg.format_version = format_version;
  // msg.map_version = map_version;

  auto primitive_sequence = request->requested_primitives;

  // special case where we send existing map as is
  if (primitive_sequence.size() == 1 && *(primitive_sequence.begin()) ==
    autoware_auto_msgs::srv::HADMapService_Request::FULL_MAP)
  {
    autoware::common::had_map_utils::toBinaryMsg(m_map_provider->m_map, msg);
    response->map = msg;
    return;
  }

  // check if geom bounds are set in request (ie - they are non zero)
  auto upper_bound = request->geom_upper_bound;
  auto lower_bound = request->geom_lower_bound;
  bool8_t geom_bound_requested = (upper_bound.size() == 3) && (lower_bound.size() == 3);

  lanelet::LaneletMapPtr requested_map;
  lanelet::Lanelets requested_lanelets;
  lanelet::Areas requested_areas;
  lanelet::LineStrings3d requested_linestrings;
  lanelet::BoundingBox2d geom_bbox;

  if (geom_bound_requested) {
    geom_bbox = lanelet::BoundingBox2d(
      lanelet::BasicPoint2d(lower_bound[0], lower_bound[1]),
      lanelet::BasicPoint2d(upper_bound[0], upper_bound[1]));
  }

  for (auto primitive : primitive_sequence) {
    if (primitive == autoware_auto_msgs::srv::HADMapService_Request::DRIVEABLE_GEOMETRY) {
      lanelet::LineStrings3d linestrings;

      if (!geom_bound_requested) {
        requested_lanelets =
          autoware::common::had_map_utils::getLaneletLayer(m_map_provider->m_map);
        requested_areas = autoware::common::had_map_utils::getAreaLayer(m_map_provider->m_map);
        linestrings = autoware::common::had_map_utils::getLineStringLayer(
          m_map_provider->m_map);
      } else {
        requested_lanelets = m_map_provider->m_map->laneletLayer.search(geom_bbox);
        requested_areas = m_map_provider->m_map->areaLayer.search(geom_bbox);
        linestrings = m_map_provider->m_map->lineStringLayer.search(geom_bbox);
      }
      lanelet::LineStrings3d parking_linestrings =
        autoware::common::had_map_utils::subtypeLineStrings(
        linestrings, "parking_spot");
      lanelet::LineStrings3d parking_access_linestrings =
        autoware::common::had_map_utils::subtypeLineStrings(
        linestrings, "parking_access");
      lanelet::LineStrings3d pickup_dropoff_linestrings =
        autoware::common::had_map_utils::subtypeLineStrings(
        linestrings, "parking_spot,drop_off,pick_up");
      requested_linestrings.insert(
        requested_linestrings.end(),
        parking_linestrings.begin(),
        parking_linestrings.end());
      requested_linestrings.insert(
        requested_linestrings.end(),
        parking_access_linestrings.begin(),
        parking_access_linestrings.end());
      requested_linestrings.insert(
        requested_linestrings.end(),
        pickup_dropoff_linestrings.begin(),
        pickup_dropoff_linestrings.end());
    }
  }
  requested_map = lanelet::utils::createMap({requested_lanelets}, {requested_areas});
  for (auto i = requested_linestrings.begin(); i != requested_linestrings.end(); i++) {
    requested_map->add(*i);
  }
  autoware::common::had_map_utils::toBinaryMsg(requested_map, msg);
  response->map = msg;
}

}  // namespace lanelet2_map_provider

}  // namespace autoware

RCLCPP_COMPONENTS_REGISTER_NODE(
  autoware::lanelet2_map_provider::Lanelet2MapProviderNode)
