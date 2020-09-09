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
: Node("Lanelet2MapProvider", options),
  m_origin_set(false),
  m_verbose(true),
  m_map_filename(declare_parameter("map_osm_file").get<std::string>())
{
  declare_parameter("latitude");
  declare_parameter("longitude");
  declare_parameter("elevation");

  if (get_parameter("latitude", m_origin_lat) &&
    get_parameter("longitude", m_origin_lon) &&
    get_parameter("elevation", m_origin_ele))
  {
    m_origin_set = true;
  }
  init();
}

void Lanelet2MapProviderNode::get_map_origin()
{
  std::unique_ptr<tf2_ros::Buffer> buffer = nullptr;
  rclcpp::Clock::SharedPtr clock = std::make_shared<rclcpp::Clock>(RCL_ROS_TIME);
  rclcpp::TimeSource timesource;
  timesource.attachClock(clock);
  buffer = std::make_unique<tf2_ros::Buffer>(clock);
  std::shared_ptr<tf2_ros::TransformListener> transform_listener_ =
    std::make_shared<tf2_ros::TransformListener>(*buffer);

  rclcpp::WallRate loop_rate(std::chrono::milliseconds(100));
  bool8_t got_map_origin = false;

  while (!got_map_origin && rclcpp::ok()) {
    try {
      geometry_msgs::msg::TransformStamped tfs =
        buffer->lookupTransform("earth", "map", tf2::TimePointZero);
      m_earth_to_map = tfs;
      got_map_origin = true;
    } catch (tf2::TransformException & ex) {
      RCLCPP_INFO(
        this->get_logger(),
        "Waiting for earth to map transform - please start ndt_map_publisher .... : %s",
        ex.what());
    }
    loop_rate.sleep();
  }
}

void Lanelet2MapProviderNode::init()
{
  m_map_provider = std::make_unique<Lanelet2MapProvider>(m_map_filename);

  if (!m_origin_set) {
    get_map_origin();
  }
  load_map();

  m_map_service =
    this->create_service<autoware_auto_msgs::srv::HADMapService>(
    "HAD_Map_Service", std::bind(&Lanelet2MapProviderNode::handle_request, this,
    std::placeholders::_1, std::placeholders::_2));
}

void Lanelet2MapProviderNode::load_map()
{
  if (m_origin_set) {
    m_map_provider->set_geographic_coords(m_origin_lat, m_origin_lon, m_origin_ele);
  } else {
    m_map_provider->set_earth_to_map_transform(m_earth_to_map);
    m_map_provider->calculate_geographic_coords();
  }
  m_map_provider->load_map();
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

  // special  case where we send existing map as is
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
      requested_linestrings.insert(
        requested_linestrings.end(),
        parking_linestrings.begin(),
        parking_linestrings.end());
      requested_linestrings.insert(
        requested_linestrings.end(),
        parking_access_linestrings.begin(),
        parking_access_linestrings.end());
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
