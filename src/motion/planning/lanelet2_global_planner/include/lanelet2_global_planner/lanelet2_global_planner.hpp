// Copyright 2019 Apex.AI, Inc.
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

#ifndef  LANELET2_GLOBAL_PLANNER__LANELET2_GLOBAL_PLANNER_HPP_
#define  LANELET2_GLOBAL_PLANNER__LANELET2_GLOBAL_PLANNER_HPP_
// ros2
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <std_msgs/msg/string.hpp>

// lanelet2
#include <lanelet2_core/primitives/Lanelet.h>
#include <lanelet2_io/Io.h>
#include <lanelet2_projection/UTM.h>
#include <lanelet2_routing/Route.h>
#include <lanelet2_routing/RoutingCost.h>
#include <lanelet2_routing/RoutingGraph.h>
#include <lanelet2_routing/RoutingGraphContainer.h>
#include <lanelet2_traffic_rules/TrafficRulesFactory.h>
// autoware
#include <lanelet2_global_planner/visibility_control.hpp>
#include <common/types.hpp>
// c++
#include <chrono>
#include <string>
#include <memory>
#include <vector>
#include <cmath>
#include <unordered_map>

using autoware::common::types::float64_t;
using autoware::common::types::bool8_t;

namespace autoware
{
namespace motion
{
namespace planning
{
namespace lanelet2_global_planner
{
class LANELET2_GLOBAL_PLANNER_PUBLIC Lanelet2GlobalPlannerNode : public rclcpp::Node
{
public:
  explicit Lanelet2GlobalPlannerNode(const rclcpp::NodeOptions & node_options);

  void load_osm_map(const std::string & file, float64_t lat, float64_t lon, float64_t alt);
  void parse_lanelet_element();
  bool8_t plan_route(
    const lanelet::Point3d & start, const lanelet::Point3d & end,
    std::vector<lanelet::Id> & route) const;
  lanelet::Id find_nearparking_from_point(const lanelet::Point3d & point) const;
  lanelet::Id find_nearroute_from_parking(const lanelet::Id & park_id) const;
  lanelet::Id find_lane_id(const lanelet::Id & cad_id) const;
  std::vector<lanelet::Id> get_lane_route(
    const lanelet::Id & from_id,
    const lanelet::Id & to) const;
  bool8_t compute_parking_center(lanelet::Id & parking_id, lanelet::Point3d & parking_center) const;
  float64_t p2p_euclidean(const lanelet::Point3d & p1, const lanelet::Point3d & p2) const;
  std::vector<lanelet::Id> str2num_lanes(const std::string & str) const;

private:
  std::unique_ptr<lanelet::LaneletMap> osm_map;
  std::vector<lanelet::Id> parking_id_list;
  std::unordered_map<lanelet::Id, std::vector<lanelet::Id>> parking_lane_map;
  std::unordered_map<lanelet::Id, lanelet::Id> near_road_map;
};
}  // namespace lanelet2_global_planner
}  // namespace planning
}  // namespace motion
}  // namespace autoware

#endif  // LANELET2_GLOBAL_PLANNER__LANELET2_GLOBAL_PLANNER_HPP_
