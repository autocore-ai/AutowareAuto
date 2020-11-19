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
#include <autoware_auto_msgs/msg/trajectory_point.hpp>
#include <common/types.hpp>
// c++
#include <chrono>
#include <string>
#include <memory>
#include <vector>
#include <cmath>
#include <unordered_map>
#include <regex>

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

using autoware_auto_msgs::msg::TrajectoryPoint;

class LANELET2_GLOBAL_PLANNER_PUBLIC Lanelet2GlobalPlanner
{
public:
  explicit Lanelet2GlobalPlanner();

  void load_osm_map(const std::string & file, float64_t lat, float64_t lon, float64_t alt);
  void parse_lanelet_element();
  bool8_t plan_route(
    TrajectoryPoint & start, TrajectoryPoint & end,
    std::vector<lanelet::Id> & route) const;
  TrajectoryPoint refine_pose_by_parking_spot(
    const lanelet::Id & parking_id,
    const TrajectoryPoint & input_point) const;
  bool8_t point_in_parking_spot(
    const lanelet::Point3d & point, const lanelet::Id & parking_id) const;
  std::string get_primitive_type(const lanelet::Id & prim_id);
  lanelet::Id find_nearparking_from_point(const lanelet::Point3d & point) const;
  lanelet::Id find_nearroute_from_parking(const lanelet::Id & park_id) const;
  lanelet::Id find_parkingaccess_from_parking(const lanelet::Id & park_id) const;
  std::vector<lanelet::Id> find_lane_from_parkingaccess(const lanelet::Id & parkaccess_id) const;
  lanelet::Id find_lane_id(const lanelet::Id & cad_id) const;
  std::vector<lanelet::Id> get_lane_route(
    const std::vector<lanelet::Id> & from_id,
    const std::vector<lanelet::Id> & to) const;
  bool8_t compute_parking_center(lanelet::Id & parking_id, lanelet::Point3d & parking_center) const;
  float64_t p2p_euclidean(const lanelet::Point3d & p1, const lanelet::Point3d & p2) const;
  std::vector<lanelet::Id> lanelet_chr2num(const std::string & str) const;
  std::vector<lanelet::Id> lanelet_str2num(const std::string & str) const;
  std::shared_ptr<lanelet::LaneletMap> osm_map;

private:
  std::vector<lanelet::Id> parking_id_list;
  std::unordered_map<lanelet::Id, std::vector<lanelet::Id>> parking_lane_map;
  std::unordered_map<lanelet::Id, std::vector<lanelet::Id>> parking2access_map;
  std::unordered_map<lanelet::Id, std::vector<lanelet::Id>> access2lane_map;
  std::unordered_map<lanelet::Id, lanelet::Id> near_road_map;
};
}  // namespace lanelet2_global_planner
}  // namespace planning
}  // namespace motion
}  // namespace autoware

#endif  // LANELET2_GLOBAL_PLANNER__LANELET2_GLOBAL_PLANNER_HPP_
