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

#ifndef  LANELET2_GLOBAL_PLANNER_NODE__LANELET2_GLOBAL_PLANNER_NODE_HPP_
#define  LANELET2_GLOBAL_PLANNER_NODE__LANELET2_GLOBAL_PLANNER_NODE_HPP_
// ros2
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <std_msgs/msg/string.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2/buffer_core.h>
#include <tf2_ros/transform_listener.h>
#include <time_utils/time_utils.hpp>

// autoware
#include <lanelet2_global_planner_node/visibility_control.hpp>
#include <lanelet2_global_planner/lanelet2_global_planner.hpp>
#include <lanelet2_core/LaneletMap.h>
#include <common/types.hpp>
#include <autoware_auto_msgs/srv/had_map_service.hpp>
#include <autoware_auto_msgs/msg/had_map_bin.hpp>
#include <autoware_auto_msgs/msg/route.hpp>
#include <autoware_auto_msgs/msg/vehicle_kinematic_state.hpp>
#include <had_map_utils/had_map_conversion.hpp>
#include <common/types.hpp>

// c++
#include <chrono>
#include <string>
#include <memory>
#include <vector>
#include <cmath>

using namespace std::chrono_literals;
using autoware::common::types::bool8_t;
using autoware::common::types::float64_t;
using autoware::motion::planning::lanelet2_global_planner::Lanelet2GlobalPlanner;

namespace autoware
{
namespace motion
{
namespace planning
{
namespace lanelet2_global_planner_node
{
class LANELET2_GLOBAL_PLANNER_NODE_PUBLIC Lanelet2GlobalPlannerNode : public rclcpp::Node
{
public:
  explicit Lanelet2GlobalPlannerNode(const rclcpp::NodeOptions & node_options);

  void request_osm_binary_map();
  void goal_pose_cb(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
  void current_pose_cb(const autoware_auto_msgs::msg::VehicleKinematicState::SharedPtr msg);
  void send_global_path(
    const std::vector<lanelet::Id> & route,
    const autoware_auto_msgs::msg::TrajectoryPoint & start_point,
    const autoware_auto_msgs::msg::TrajectoryPoint & end_point,
    const std_msgs::msg::Header & header);
  bool8_t transform_pose_to_map(
    const geometry_msgs::msg::PoseStamped & pose_in, geometry_msgs::msg::PoseStamped & pose_out);

private:
  std::shared_ptr<Lanelet2GlobalPlanner> lanelet2_global_planner;
  rclcpp::Client<autoware_auto_msgs::srv::HADMapService>::SharedPtr map_client;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pose_sub_ptr;
  rclcpp::Subscription<autoware_auto_msgs::msg::VehicleKinematicState>::SharedPtr
    current_pose_sub_ptr;
  rclcpp::Publisher<autoware_auto_msgs::msg::Route>::SharedPtr global_path_pub_ptr;
  geometry_msgs::msg::PoseStamped start_pose;
  geometry_msgs::msg::PoseStamped goal_pose;
  bool8_t start_pose_init;
  tf2::BufferCore tf_buffer;
  tf2_ros::TransformListener tf_listener;
};
}  // namespace lanelet2_global_planner_node
}  // namespace planning
}  // namespace motion
}  // namespace autoware

#endif  // LANELET2_GLOBAL_PLANNER_NODE__LANELET2_GLOBAL_PLANNER_NODE_HPP_
