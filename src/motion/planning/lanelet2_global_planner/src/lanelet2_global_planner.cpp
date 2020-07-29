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

#include <rclcpp/node_options.hpp>
#include <rclcpp_components/register_node_macro.hpp>

#include <lanelet2_global_planner/lanelet2_global_planner.hpp>
#include <std_msgs/msg/string.hpp>
#include <common/types.hpp>

#include <string>
#include <unordered_map>
#include <utility>
#include <vector>
#include <algorithm>
#include <memory>

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
Lanelet2GlobalPlannerNode::Lanelet2GlobalPlannerNode(
  const rclcpp::NodeOptions & node_options)
: Node("lanelet2_global_planner_node", node_options)
{
}

void Lanelet2GlobalPlannerNode::load_osm_map(
  const std::string & file,
  float64_t lat, float64_t lon, float64_t alt)
{
  if (osm_map) {
    osm_map.reset();
  }
  osm_map = load(file, lanelet::projection::UtmProjector(
        lanelet::Origin({lat, lon, alt})));

  // throw map load error
  if (!osm_map) {
    throw std::runtime_error("Lanelet2GlobalPlannerNode: Map load fail");
  }
}

void Lanelet2GlobalPlannerNode::parse_lanelet_element()
{
  if (osm_map) {
    // parsing lanelet lane
    typedef std::unordered_map<lanelet::Id, lanelet::Id>::iterator it_lane;
    std::pair<it_lane, bool8_t> result_lane;
    for (const auto & lanelet : osm_map->laneletLayer) {
      // filter-out non road type
      if (lanelet.hasAttribute("subtype") &&
        lanelet.hasAttribute("cad_id") &&
        lanelet.attribute("subtype") == "road")
      {
        lanelet::Id lane_id = lanelet.id();
        lanelet::Id lane_cad_id = *lanelet.attribute("cad_id").asId();
        result_lane = near_road_map.emplace(lane_cad_id, lane_id);
        if (!result_lane.second) {
          throw std::runtime_error("Lanelet2GlobalPlannerNode: Parsing osm lane map fail");
        }
      }
    }

    // Pair of Map Iterator and bool8_t value
    typedef std::unordered_map<lanelet::Id, std::vector<lanelet::Id>>::iterator it;
    std::pair<it, bool8_t> result;
    for (const auto & linestring : osm_map->lineStringLayer) {
      // loop through parking attribute
      if (linestring.hasAttribute("subtype") &&
        linestring.hasAttribute("cad_id") &&
        linestring.hasAttribute("near_roads") &&
        linestring.attribute("subtype") == "parking")
      {
        // build vector of parking id
        lanelet::Id parking_id = linestring.id();
        parking_id_list.push_back(parking_id);

        // near road id value ("near_roads")
        std::string lanes_str = linestring.attribute("near_roads").value();
        std::vector<lanelet::Id> near_road_ids = str2num_lanes(lanes_str);
        result = parking_lane_map.emplace(parking_id, near_road_ids);
        if (!result.second) {
          throw std::runtime_error("Lanelet2GlobalPlannerNode: Parsing osm linestring map fail");
        }
      }
    }
  }
}

bool8_t Lanelet2GlobalPlannerNode::plan_route(
  const lanelet::Point3d & start,
  const lanelet::Point3d & end, std::vector<lanelet::Id> & route) const
{
  // find near start-end parking:return id
  lanelet::Id near_parking_start = find_nearparking_from_point(start);
  lanelet::Id near_parking_end = find_nearparking_from_point(end);
  // locate near roads/lanes: return cad_id
  lanelet::Id road_start = find_nearroute_from_parking(near_parking_start);
  lanelet::Id road_end = find_nearroute_from_parking(near_parking_end);
  // find lane id from near_roads id: return lane id
  lanelet::Id lane_id_start = find_lane_id(road_start);
  lanelet::Id lane_id_end = find_lane_id(road_end);
  // plan a route using lanelet2 lib: vector lane id
  route = get_lane_route(lane_id_start, lane_id_end);
  if (route.size() > 0) {
    return true;
  } else {
    return false;
  }
}

lanelet::Id Lanelet2GlobalPlannerNode::find_nearparking_from_point(const lanelet::Point3d & point)
const
{
  // loop through parking space to find the closest distance error
  float64_t min_dist = 1e9;
  auto i = std::min_element(std::begin(parking_id_list),
      std::end(parking_id_list),
      [ =, &min_dist](lanelet::Id park_id1, lanelet::Id park_id2)
      {
        // compute parking center point
        lanelet::Point3d p1;
        lanelet::Point3d p2;
        if (compute_parking_center(park_id1, p1) &&
        compute_parking_center(park_id2, p2))
        {
          float64_t dist1 = p2p_euclidean(p1, point);
          float64_t dist2 = p2p_euclidean(p2, point);
          min_dist = (dist1 < dist2) ? dist1 : dist2;
          return dist1 < dist2;
        } else {
          return false;
        }
      });

  // get parking id
  // Improvement- Check if the parking point is too far away?
  //              Check if min_dist below the threshold
  return parking_id_list[std::distance(std::begin(parking_id_list), i)];
}

lanelet::Id Lanelet2GlobalPlannerNode::find_nearroute_from_parking(const lanelet::Id & park_id)
const
{
  lanelet::Id lane_id = -1;
  if (osm_map->lineStringLayer.exists(park_id)) {
    // search the map
    auto it = parking_lane_map.find(park_id);
    if (it != parking_lane_map.end()) {
      // could be more than one id in the vector<Id>
      // pick the first near road
      // this version only give the first one for now
      lane_id = it->second.at(0);
    } else {
      RCLCPP_WARN(
        this->get_logger(),
        "Find near route parking: Parking id not found in the map");
    }
  } else {
    RCLCPP_WARN(
      this->get_logger(),
      "Find near route parking: Parking id not found in the map");
  }
  return lane_id;
}

lanelet::Id Lanelet2GlobalPlannerNode::find_lane_id(const lanelet::Id & cad_id) const
{
  lanelet::Id lane_id = -1;
  // search the map
  auto it = near_road_map.find(cad_id);
  if (it != near_road_map.end()) {
    // pick the first near road (this version only give the first one for now)
    lane_id = it->second;
  } else {
    RCLCPP_WARN(
      this->get_logger(),
      "Find lane id: Lane id not found in the map");
  }
  return lane_id;
}

std::vector<lanelet::Id> Lanelet2GlobalPlannerNode::get_lane_route(
  const lanelet::Id & from_id, const lanelet::Id & to_id) const
{
  std::vector<lanelet::Id> lane_ids;
  lanelet::traffic_rules::TrafficRulesPtr trafficRules =
    lanelet::traffic_rules::TrafficRulesFactory::create(lanelet::Locations::Germany,
      lanelet::Participants::Vehicle);
  lanelet::routing::RoutingGraphUPtr routingGraph =
    lanelet::routing::RoutingGraph::build(*osm_map, *trafficRules);

  // plan a shortest path without a lane change
  lanelet::ConstLanelet fromLanelet = osm_map->laneletLayer.get(from_id);
  lanelet::ConstLanelet toLanelet = osm_map->laneletLayer.get(to_id);
  lanelet::Optional<lanelet::routing::Route> route = routingGraph->getRoute(
    fromLanelet, toLanelet, 0);
  lanelet::routing::LaneletPath shortestPath = route->shortestPath();
  lanelet::LaneletSequence fullLane = route->fullLane(fromLanelet);

  // op for the use of shortest path in this implementation
  if (!shortestPath.empty() && !fullLane.empty()) {
    lane_ids = fullLane.ids();
  } else {
    throw std::runtime_error("Lanelet2GlobalPlannerNode: Finding route error");
  }
  return lane_ids;
}

bool8_t Lanelet2GlobalPlannerNode::compute_parking_center(
  lanelet::Id & parking_id, lanelet::Point3d & parking_center) const
{
  bool8_t result = false;
  // Point3d parking_center;
  if (osm_map->lineStringLayer.exists(parking_id) &&
    osm_map->lineStringLayer.get(parking_id).size() > 0U)
  {
    // sum x,y,z points
    float64_t mean_x = 0.0;
    float64_t mean_y = 0.0;
    float64_t mean_z = 0.0;
    lanelet::LineString3d ls3d = osm_map->lineStringLayer.get(parking_id);
    size_t num_points = ls3d.size();
    std::for_each(ls3d.begin(), ls3d.end(), [&](lanelet::Point3d p)
      {
        mean_x += p.x() / static_cast<float64_t>(num_points);
        mean_y += p.y() / static_cast<float64_t>(num_points);
        mean_z += p.z() / static_cast<float64_t>(num_points);
      });
    lanelet::Point3d p(lanelet::utils::getId(), mean_x, mean_y, mean_z);
    parking_center = p;
    result = true;
  }
  return result;
}

float64_t Lanelet2GlobalPlannerNode::p2p_euclidean(
  const lanelet::Point3d & p1,
  const lanelet::Point3d & p2) const
{
  Eigen::Vector3d pd = p1.basicPoint() - p2.basicPoint();
  Eigen::Vector3d pd2 = pd.array().square();
  return std::sqrt(pd2.x() + pd2.y() + pd2.z());
}

std::vector<lanelet::Id> Lanelet2GlobalPlannerNode::str2num_lanes(const std::string & str) const
{
  // expecting e.g. str = "[u'429933', u'430462']";
  // extract number at 3-8, 14-19
  std::string prefix_str = "'";
  size_t pos = 0U;
  uint32_t counter = 0U;
  uint32_t start = 0U;
  uint32_t end = 0U;
  std::vector<lanelet::Id> lanes;
  while ((pos = str.find(prefix_str, pos)) != std::string::npos) {
    ++counter;
    if (counter % 2 == 0U) {
      end = pos;
      std::string num_str = str.substr(start + 1, end - start - 1);
      lanelet::Id num_id = static_cast<lanelet::Id>(std::atoi(num_str.c_str()));
      lanes.push_back(num_id);
    } else {
      start = pos;
    }
    pos++;
  }
  return lanes;
}
}  // namespace lanelet2_global_planner
}  // namespace planning
}  // namespace motion
}  // namespace autoware

RCLCPP_COMPONENTS_REGISTER_NODE(
  autoware::motion::planning::lanelet2_global_planner::Lanelet2GlobalPlannerNode)
