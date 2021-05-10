// Copyright 2019 the Autoware Foundation
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

#include <rclcpp/node_options.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <tf2/buffer_core.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/utils.h>
#include <time_utils/time_utils.hpp>
#include <motion_common/motion_common.hpp>

#include <autoware_auto_msgs/msg/complex32.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <lanelet2_global_planner_nodes/lanelet2_global_planner_node.hpp>
#include <std_msgs/msg/string.hpp>
#include <common/types.hpp>

#include <chrono>
#include <cmath>
#include <string>
#include <memory>
#include <vector>

using namespace std::chrono_literals;
using autoware::common::types::bool8_t;
using autoware::common::types::float32_t;
using autoware::common::types::float64_t;
using autoware::common::types::TAU;
using autoware::planning::lanelet2_global_planner::Lanelet2GlobalPlanner;
using autoware_auto_msgs::msg::Complex32;
using std::placeholders::_1;

namespace autoware
{
namespace planning
{
namespace lanelet2_global_planner_nodes
{

autoware_auto_msgs::msg::TrajectoryPoint convertToTrajectoryPoint(
  const geometry_msgs::msg::Pose & pose)
{
  autoware_auto_msgs::msg::TrajectoryPoint pt;
  pt.x = static_cast<float>(pose.position.x);
  pt.y = static_cast<float>(pose.position.y);
  const auto angle = tf2::getYaw(pose.orientation);
  pt.heading = ::motion::motion_common::from_angle(angle);
  return pt;
}

Lanelet2GlobalPlannerNode::Lanelet2GlobalPlannerNode(
  const rclcpp::NodeOptions & node_options)
: Node("lanelet2_global_planner_node", node_options),
  tf_listener(tf_buffer, std::shared_ptr<rclcpp::Node>(this, [](auto) {}), false)
{
  start_pose_init = false;
  // Global planner instance init
  lanelet2_global_planner = std::make_shared<Lanelet2GlobalPlanner>();
  // Subcribers Goal Pose
  goal_pose_sub_ptr =
    this->create_subscription<geometry_msgs::msg::PoseStamped>(
    "goal_pose", rclcpp::QoS(10),
    std::bind(&Lanelet2GlobalPlannerNode::goal_pose_cb, this, _1));

  // Subcribers Current Pose
  current_pose_sub_ptr =
    this->create_subscription<autoware_auto_msgs::msg::VehicleKinematicState>(
    "vehicle_kinematic_state", rclcpp::QoS(10),
    std::bind(&Lanelet2GlobalPlannerNode::current_pose_cb, this, _1));

  // Global path publisher
  global_path_pub_ptr =
    this->create_publisher<autoware_auto_msgs::msg::HADMapRoute>(
    "global_path", rclcpp::QoS(10));

  // Create map client
  map_client = this->create_client<autoware_auto_msgs::srv::HADMapService>("HAD_Map_Client");

  // Request binary map from the map loader node
  this->request_osm_binary_map();
}

void Lanelet2GlobalPlannerNode::request_osm_binary_map()
{
  while (rclcpp::ok() && !map_client->wait_for_service(1s)) {
    RCLCPP_WARN(this->get_logger(), "HAD map service not available yet. Waiting...");
  }
  if (!rclcpp::ok()) {
    RCLCPP_ERROR(
      this->get_logger(),
      "Client interrupted while waiting for map service to appear. Exiting.");
  }

  auto request = std::make_shared<autoware_auto_msgs::srv::HADMapService_Request>();
  request->requested_primitives.push_back(
    autoware_auto_msgs::srv::HADMapService_Request::FULL_MAP);

  auto result = map_client->async_send_request(request);
  if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result) !=
    rclcpp::executor::FutureReturnCode::SUCCESS)
  {
    RCLCPP_ERROR(this->get_logger(), "Service call failed");
    throw std::runtime_error("Lanelet2GlobalPlannerNode: Map service call fail");
  }

  // copy message to map
  autoware_auto_msgs::msg::HADMapBin msg = result.get()->map;

  // Convert binary map msg to lanelet2 map and set the map for global path planner
  lanelet2_global_planner->osm_map = std::make_shared<lanelet::LaneletMap>();
  autoware::common::had_map_utils::fromBinaryMsg(msg, lanelet2_global_planner->osm_map);

  // parse lanelet global path planner elements
  lanelet2_global_planner->parse_lanelet_element();
}

void Lanelet2GlobalPlannerNode::goal_pose_cb(
  const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
  if (!start_pose_init) {
    RCLCPP_ERROR(this->get_logger(), "Current pose has not been set!");
    return;
  }
  // transform and set the starting and goal point in the map frame
  goal_pose.header = msg->header;
  goal_pose.pose = msg->pose;
  geometry_msgs::msg::PoseStamped goal_pose_map = goal_pose;

  if (goal_pose.header.frame_id != "map") {
    if (!transform_pose_to_map(goal_pose, goal_pose_map)) {
      // return: nothing happen
      return;
    } else {
      goal_pose = goal_pose_map;
    }
  }

  auto start = convertToTrajectoryPoint(start_pose.pose);
  auto end = convertToTrajectoryPoint(goal_pose.pose);

  // get routes
  std::vector<lanelet::Id> route;
  if (lanelet2_global_planner->plan_route(start, end, route)) {
    // send out the global path
    std_msgs::msg::Header msg_header;
    msg_header.stamp = rclcpp::Clock().now();
    msg_header.frame_id = "map";
    this->send_global_path(route, start, end, msg_header);
  } else {
    RCLCPP_ERROR(this->get_logger(), "Global route has not been found!");
  }
}

void Lanelet2GlobalPlannerNode::current_pose_cb(
  const autoware_auto_msgs::msg::VehicleKinematicState::SharedPtr msg)
{
  // convert msg to geometry_msgs::msg::Pose
  start_pose.pose.position.x = msg->state.x;
  start_pose.pose.position.y = msg->state.y;
  start_pose.pose.position.z = 0.0;
  start_pose.pose.orientation =
    motion::motion_common::to_quat<geometry_msgs::msg::Quaternion>(
    msg->state.heading);
  start_pose.header = msg->header;

  // transform to "map" frame if needed
  if (start_pose.header.frame_id != "map") {
    geometry_msgs::msg::PoseStamped start_pose_map = start_pose;

    if (!transform_pose_to_map(start_pose, start_pose_map)) {
      // transform failed
      start_pose_init = false;
    } else {
      // transform ok: set start_pose to the pose in map
      start_pose = start_pose_map;
      start_pose_init = true;
    }
  } else {
    // No transform required
    start_pose_init = true;
  }
}

void Lanelet2GlobalPlannerNode::send_global_path(
  const std::vector<lanelet::Id> & route,
  const autoware_auto_msgs::msg::TrajectoryPoint & start_point,
  const autoware_auto_msgs::msg::TrajectoryPoint & end_point, const std_msgs::msg::Header & header)
{
  // the maximum of PlanTrajectory message is 100
  if (route.size() > 100) {
    RCLCPP_ERROR(this->get_logger(), "Route size is exceeded the limit of 100");
    return;
  }

  // build trajectory message
  // parking id = first/first to last
  // drivable area = second/second to last
  // main route = other
  autoware_auto_msgs::msg::HADMapRoute global_route;
  global_route.header = header;

  autoware_auto_msgs::msg::RoutePoint start_route_point;
  start_route_point.position.x = start_point.x;
  start_route_point.position.y = start_point.y;
  start_route_point.heading = start_point.heading;

  autoware_auto_msgs::msg::RoutePoint end_route_point;
  end_route_point.position.x = end_point.x;
  end_route_point.position.y = end_point.y;
  end_route_point.heading = end_point.heading;

  global_route.start_point = start_route_point;
  global_route.goal_point = end_route_point;

  for (const auto & route_id : route) {
    // add data to the global path
    autoware_auto_msgs::msg::MapPrimitive primitive;
    primitive.id = route_id;
    primitive.primitive_type = lanelet2_global_planner->get_primitive_type(route_id);

    autoware_auto_msgs::msg::HADMapSegment new_segment;
    new_segment.preferred_primitive_id = primitive.id;
    new_segment.primitives.push_back(primitive);
    global_route.segments.push_back(new_segment);
  }
  // publish the global path
  global_path_pub_ptr->publish(global_route);
}

bool8_t Lanelet2GlobalPlannerNode::transform_pose_to_map(
  const geometry_msgs::msg::PoseStamped & pose_in,
  geometry_msgs::msg::PoseStamped & pose_out)
{
  std::string source_frame = pose_in.header.frame_id;
  // lookup transform validity
  if (!tf_buffer.canTransform("map", source_frame, tf2::TimePointZero)) {
    RCLCPP_ERROR(this->get_logger(), "Failed to transform Pose to map frame");
    return false;
  }

  // transform pose into map frame
  geometry_msgs::msg::TransformStamped tf_map;
  try {
    tf_map = tf_buffer.lookupTransform(
      "map", source_frame,
      time_utils::from_message(pose_in.header.stamp));
  } catch (const tf2::ExtrapolationException &) {
    // currently falls back to retrive newest transform available for availability,
    // Do validation of time stamp in the future
    tf_map = tf_buffer.lookupTransform("map", source_frame, tf2::TimePointZero);
  }

  // apply transform
  tf2::doTransform(pose_in, pose_out, tf_map);
  return true;
}

}  // namespace lanelet2_global_planner_nodes
}  // namespace planning
}  // namespace autoware

RCLCPP_COMPONENTS_REGISTER_NODE(
  autoware::planning::lanelet2_global_planner_nodes::Lanelet2GlobalPlannerNode)
