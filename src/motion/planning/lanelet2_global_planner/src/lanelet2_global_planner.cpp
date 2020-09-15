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

#include <lanelet2_global_planner/lanelet2_global_planner.hpp>
#include <common/types.hpp>

#include <string>
#include <unordered_map>
#include <utility>
#include <vector>
#include <algorithm>
#include <memory>
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
Lanelet2GlobalPlanner::Lanelet2GlobalPlanner()
{
}

void Lanelet2GlobalPlanner::load_osm_map(
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
    throw std::runtime_error("Lanelet2GlobalPlanner: Map load fail");
  }
}

void Lanelet2GlobalPlanner::parse_lanelet_element()
{
  if (osm_map) {
    // parsing lanelet layer
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
          throw std::runtime_error("Lanelet2GlobalPlanner: Parsing osm lane map fail");
        }
      }
    }

    // parsing lane elements
    typedef std::unordered_map<lanelet::Id, std::vector<lanelet::Id>>::iterator it;
    std::pair<it, bool8_t> result;
    for (const auto & linestring : osm_map->lineStringLayer) {
      // Map version 1 (no parking access element): mapping a parking to lanes
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
        std::vector<lanelet::Id> near_road_ids = lanelet_chr2num(lanes_str);
        result = parking_lane_map.emplace(parking_id, near_road_ids);
        if (!result.second) {
          throw std::runtime_error("Lanelet2GlobalPlanner: Parsing osm linestring map fail");
        }
      }

      // Map version 2 (with parking access element):
      // mapping a parking spot to parking accesses
      if (linestring.hasAttribute("subtype") &&
        linestring.hasAttribute("cad_id") &&
        linestring.hasAttribute("parking_accesses") &&
        (linestring.attribute("subtype") == "parking_spot" ||
        linestring.attribute("subtype") == "parking_spot,drop_off,pick_up"))
      {
        // build vector of parking id
        lanelet::Id parking_id = linestring.id();
        parking_id_list.push_back(parking_id);
        // get associate parking accesses
        std::string parking_access_str = linestring.attribute("parking_accesses").value();
        std::vector<lanelet::Id> parking_access_ids = lanelet_str2num(parking_access_str);
        // insert to a map
        result = parking2access_map.emplace(parking_id, parking_access_ids);
        if (!result.second) {
          throw std::runtime_error("Lanelet2GlobalPlanner: Insert parking2access_map fail");
        }
      }

      // mapping a parking access to lanes
      if (linestring.hasAttribute("subtype") &&
        linestring.hasAttribute("cad_id") &&
        linestring.hasAttribute("ref_lanelet") &&
        linestring.attribute("subtype") == "parking_access")
      {
        // get associate lanes
        lanelet::Id parking_access_id = linestring.id();
        std::string near_lane_str = linestring.attribute("ref_lanelet").value();
        std::vector<lanelet::Id> near_lane_ids = lanelet_str2num(near_lane_str);
        // insert to a map
        result = access2lane_map.emplace(parking_access_id, near_lane_ids);
        if (!result.second) {
          throw std::runtime_error("Lanelet2GlobalPlanner: Insert access2lane_map fail");
        }
      }
    }  // end for
  }
}

bool8_t Lanelet2GlobalPlanner::plan_route(
  const lanelet::Point3d & start,
  const lanelet::Point3d & end, std::vector<lanelet::Id> & route) const
{
  // find near start-end parking:return id
  lanelet::Id near_parking_start = find_nearparking_from_point(start);
  lanelet::Id near_parking_end = find_nearparking_from_point(end);
  // find connecting a parking access from a parking spot
  lanelet::Id parkingaccess_start = find_parkingaccess_from_parking(near_parking_start);
  lanelet::Id parkingaccess_end = find_parkingaccess_from_parking(near_parking_end);
  // find connecting a lane from a parking access
  lanelet::Id lane_start = find_lane_from_parkingaccess(parkingaccess_start);
  lanelet::Id lane_end = find_lane_from_parkingaccess(parkingaccess_end);
  // plan a route using lanelet2 lib: vector lane id
  route = get_lane_route(lane_start, lane_end);
  if (route.size() > 0) {
    // parking, parking access, routes, parking access, parking
    route.insert(route.begin(), {near_parking_start, parkingaccess_start});
    route.insert(route.end(), {parkingaccess_end, near_parking_end});
    return true;
  } else {
    return false;
  }
}

lanelet::Id Lanelet2GlobalPlanner::find_nearparking_from_point(const lanelet::Point3d & point)
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

lanelet::Id Lanelet2GlobalPlanner::find_nearroute_from_parking(const lanelet::Id & park_id)
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
    }
  }
  return lane_id;
}

lanelet::Id Lanelet2GlobalPlanner::find_parkingaccess_from_parking(const lanelet::Id & park_id)
const
{
  lanelet::Id parking_access_id = -1;
  if (osm_map->lineStringLayer.exists(park_id)) {
    // search the map
    auto it_parking = parking2access_map.find(park_id);
    if (it_parking != parking2access_map.end()) {
      // could be more than one id in the vector<Id>
      // pick the first parking access for now
      parking_access_id = it_parking->second.at(0);
    }
  }
  return parking_access_id;
}

lanelet::Id Lanelet2GlobalPlanner::find_lane_from_parkingaccess(const lanelet::Id & parkaccess_id)
const
{
  lanelet::Id lane_id = -1;
  if (osm_map->lineStringLayer.exists(parkaccess_id)) {
    // search the map
    auto it_lane = access2lane_map.find(parkaccess_id);
    if (it_lane != access2lane_map.end()) {
      // pick the first leane (this version only give the first one for now)
      lane_id = it_lane->second.at(0);
    }
  }
  return lane_id;
}

lanelet::Id Lanelet2GlobalPlanner::find_lane_id(const lanelet::Id & cad_id) const
{
  lanelet::Id lane_id = -1;
  // search the map
  auto it = near_road_map.find(cad_id);
  if (it != near_road_map.end()) {
    // pick the first near road (this version only give the first one for now)
    lane_id = it->second;
  }
  return lane_id;
}

std::vector<lanelet::Id> Lanelet2GlobalPlanner::get_lane_route(
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

  // check route validity before continue further
  if (!route) {
    // return empty lane ids to be catch by the caller
    return lane_ids;
  }

  // op for the use of shortest path in this implementation
  lanelet::routing::LaneletPath shortestPath = route->shortestPath();
  lanelet::LaneletSequence fullLane = route->fullLane(fromLanelet);
  if (!shortestPath.empty() && !fullLane.empty()) {
    lane_ids = fullLane.ids();
  }
  return lane_ids;
}

bool8_t Lanelet2GlobalPlanner::compute_parking_center(
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

float64_t Lanelet2GlobalPlanner::p2p_euclidean(
  const lanelet::Point3d & p1,
  const lanelet::Point3d & p2) const
{
  Eigen::Vector3d pd = p1.basicPoint() - p2.basicPoint();
  Eigen::Vector3d pd2 = pd.array().square();
  return std::sqrt(pd2.x() + pd2.y() + pd2.z());
}

std::vector<lanelet::Id> Lanelet2GlobalPlanner::lanelet_chr2num(const std::string & str) const
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

std::vector<lanelet::Id> Lanelet2GlobalPlanner::lanelet_str2num(const std::string & str) const
{
  // expecting no space comma e.g. str = "1523,4789,4852";
  std::vector<lanelet::Id> result_nums;
  std::regex delimiter(",");
  std::sregex_token_iterator first{str.begin(), str.end(), delimiter, -1}, last;
  std::vector<std::string> tokens{first, last};
  for (auto t : tokens) {
    lanelet::Id num_id = static_cast<lanelet::Id>(std::atoi(t.c_str()));
    result_nums.emplace_back(num_id);
  }
  return result_nums;
}
}  // namespace lanelet2_global_planner
}  // namespace planning
}  // namespace motion
}  // namespace autoware
