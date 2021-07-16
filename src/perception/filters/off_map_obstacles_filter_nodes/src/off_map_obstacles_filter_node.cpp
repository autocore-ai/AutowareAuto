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
//
// Co-developed by Tier IV, Inc. and Apex.AI, Inc.

#include "off_map_obstacles_filter_nodes/off_map_obstacles_filter_node.hpp"

#include <functional>
#include <memory>
#include <string>

#include "had_map_utils/had_map_conversion.hpp"

#include "common/types.hpp"
#include "tf2_ros/buffer_interface.h"

using namespace std::literals::chrono_literals;

namespace autoware
{
namespace off_map_obstacles_filter_nodes
{

using bool8_t = autoware::common::types::bool8_t;
using float64_t = autoware::common::types::float64_t;
using HADMapService = autoware_auto_msgs::srv::HADMapService;
using ObstacleMsg = autoware_auto_msgs::msg::BoundingBoxArray;
using MarkerArray = visualization_msgs::msg::MarkerArray;

OffMapObstaclesFilterNode::OffMapObstaclesFilterNode(const rclcpp::NodeOptions & options)
:  Node("off_map_obstacles_filter", options),
  m_sub_ptr(create_subscription<ObstacleMsg>(
      "bounding_boxes_in", rclcpp::QoS{10},
      [this](const ObstacleMsg::SharedPtr msg) {process_bounding_boxes(msg);})),
  m_pub_ptr(create_publisher<ObstacleMsg>("bounding_boxes_out", rclcpp::QoS{10})),
  m_map_client_ptr(
    create_client<HADMapService>(
      "HAD_Map_Service",
      rmw_qos_profile_services_default)),
  m_tf2_buffer(this->get_clock()),
  m_tf2_listener(m_tf2_buffer),
  m_overlap_threshold(declare_parameter("overlap_threshold").get<float64_t>())
{
  while (!m_map_client_ptr->wait_for_service(1s)) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(get_logger(), "Interrupted while waiting for service.");
      rclcpp::shutdown();
      return;
    }
    RCLCPP_INFO(get_logger(), "Waiting for map provider");
  }
  auto request = std::make_shared<HADMapService::Request>();
  request->requested_primitives.push_back(HADMapService::Request::FULL_MAP);
  m_map_client_ptr->async_send_request(
    request,
    std::bind(&OffMapObstaclesFilterNode::map_response, this, std::placeholders::_1));

  // Bounding box arrays can be visualized directly – this visualization is for checking that the
  // conversion + transformation of bboxes to polygons in the map frame is correct. It shouldn't
  // be needed by most people.
  if (declare_parameter("publish_polygon_viz", rclcpp::ParameterValue(false)).get<bool8_t>()) {
    m_marker_pub_ptr = create_publisher<MarkerArray>("bounding_boxes_viz", rclcpp::QoS{10});
  }
}

void OffMapObstaclesFilterNode::map_response(
  const rclcpp::Client<HADMapService>::SharedFuture future)
{
  auto lanelet_map_ptr = std::make_shared<lanelet::LaneletMap>();
  autoware::common::had_map_utils::fromBinaryMsg(future.get()->map, lanelet_map_ptr);
  m_filter = std::make_unique<OffMapObstaclesFilter>(lanelet_map_ptr, m_overlap_threshold);
}

void OffMapObstaclesFilterNode::process_bounding_boxes(const ObstacleMsg::SharedPtr msg) const
{
  if (!m_filter) {
    RCLCPP_INFO(get_logger(), "Did not filter boxes because no map was available.");
    m_pub_ptr->publish(*msg);
  }
  if (msg->header.frame_id != "base_link") {
    // Using a different frame would not work since the code relies on the z axis to be aligned
    // with the map frame's z axis to do its projection onto the map.
    // That could be generalized in principle – but if the message is arriving in a different
    // frame, probably someone else than euclidean_cluster_node is sending it, and we can't be
    // sure anymore the 2.5D assumption is valid either. And that assumption is more fundamental
    // because the lanelet::Polygon2d needs its vertices to be in counterclockwise order, and
    // it's kind of difficult to ensure that for arbitrary rotated boxes.
    throw std::runtime_error(
            "Bounding boxes are in unexpected frame '" + msg->header.frame_id +
            "'");
  }
  try {
    geometry_msgs::msg::TransformStamped map_from_base_link = m_tf2_buffer.lookupTransform(
      "map", "base_link", tf2_ros::fromMsg(msg->header.stamp),
      tf2::durationFromSec(0.1));
    m_filter->remove_off_map_bboxes(map_from_base_link, *msg);
    if (m_marker_pub_ptr) {
      const auto marker_array = m_filter->bboxes_in_map_frame_viz(map_from_base_link, *msg);
      m_marker_pub_ptr->publish(marker_array);
    }
  } catch (...) {
    RCLCPP_INFO(get_logger(), "Did not filter boxes because no transform was available.");
  }
  m_pub_ptr->publish(*msg);
}

}  // namespace off_map_obstacles_filter_nodes
}  // namespace autoware

#include "rclcpp_components/register_node_macro.hpp"

// This acts as an entry point, allowing the component to be
// discoverable when its library is being loaded into a running process
RCLCPP_COMPONENTS_REGISTER_NODE(autoware::off_map_obstacles_filter_nodes::OffMapObstaclesFilterNode)
