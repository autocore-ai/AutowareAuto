// Copyright 2021 the Autoware Foundation
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
#include <lanelet2_core/geometry/Area.h>

#include <common/types.hpp>
#include <geometry/common_2d.hpp>
#include <motion_common/motion_common.hpp>

#include <algorithm>
#include <limits>
#include <memory>
#include <regex>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

using autoware::common::types::float64_t;
using autoware::common::types::bool8_t;

namespace autoware
{
namespace planning
{
namespace lanelet2_global_planner
{

void Lanelet2GlobalPlanner::load_osm_map(
  const std::string & file,
  float64_t lat, float64_t lon, float64_t alt)
{
  if (osm_map) {
    osm_map.reset();
  }
  osm_map = load(
    file, lanelet::projection::UtmProjector(
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
          throw std::runtime_error("Lanelet2GlobalPlanner: Parsing osm lane from map fail");
        }
      }
    }

    // parsing lane elements
    typedef std::unordered_map<lanelet::Id, std::vector<lanelet::Id>>::iterator it;
    std::pair<it, bool8_t> result;
    for (const auto & area : osm_map->areaLayer) {
      // Map version 1 (no parking access element): mapping a parking to lanes
      if (area.hasAttribute("subtype") &&
        area.hasAttribute("cad_id") &&
        area.hasAttribute("near_roads") &&
        area.attribute("subtype") == "parking")
      {
        // build vector of parking id
        lanelet::Id parking_id = area.id();
        parking_id_list.push_back(parking_id);

        // near road id value ("near_roads")
        std::string lanes_str = area.attribute("near_roads").value();
        std::vector<lanelet::Id> near_road_ids = lanelet_chr2num(lanes_str);
        result = parking_lane_map.emplace(parking_id, near_road_ids);
        if (!result.second) {
          throw std::runtime_error("Lanelet2GlobalPlanner: Parsing osm parking_lane from map fail");
        }
      }

      // Map version 2 (with parking access element):
      // mapping a parking spot to parking accesses
      if (area.hasAttribute("subtype") &&
        area.hasAttribute("parking_accesses") &&
        (area.attribute("subtype") == "parking_spot" ||
        area.attribute("subtype") == "parking_spot,drop_off,pick_up"))
      {
        // build vector of parking id
        lanelet::Id parking_id = area.id();
        parking_id_list.push_back(parking_id);
        // get associate parking accesses
        std::string parking_access_str = area.attribute("parking_accesses").value();
        std::vector<lanelet::Id> parking_access_ids = lanelet_str2num(parking_access_str);
        // insert to a map
        result = parking2access_map.emplace(parking_id, parking_access_ids);
        if (!result.second) {
          throw std::runtime_error("Lanelet2GlobalPlanner: Insert parking2access_map fail");
        }
      }

      // mapping a parking access to lanes
      if (area.hasAttribute("subtype") &&
        area.hasAttribute("ref_lanelet") &&
        area.attribute("subtype") == "parking_access")
      {
        // get associate lanes
        lanelet::Id parking_access_id = area.id();
        std::string near_lane_str = area.attribute("ref_lanelet").value();
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

// Function currently assumes the following:
//   - The route can start in a parking spot or lanelet
//   - The route can end in a parking spot or lanelet
bool8_t Lanelet2GlobalPlanner::plan_route(
  TrajectoryPoint & start_point,
  TrajectoryPoint & end_point, std::vector<lanelet::Id> & route) const
{
  const lanelet::Point3d start(lanelet::utils::getId(), start_point.x, start_point.y, 0.0);
  const lanelet::Point3d end(lanelet::utils::getId(), end_point.x, end_point.y, 0.0);

  std::vector<lanelet::Id> lane_start;
  std::vector<lanelet::Id> lane_end;
  lanelet::Id near_parking_start, near_parking_end;
  lanelet::Id parkingaccess_start, parkingaccess_end;
  auto start_in_parking = false, end_in_parking = false;

  // Look for parking spot only when some exist
  if (!parking_id_list.empty()) {
    // Find nearest parking spot IDs for start and end points
    near_parking_start = find_nearparking_from_point(start);
    near_parking_end = find_nearparking_from_point(end);

    // Determine if start/end position is in the closest parking spot
    start_in_parking = point_in_parking_spot(start, near_parking_start);
    end_in_parking = point_in_parking_spot(end, near_parking_end);
  }

  if (start_in_parking) {
    // find connecting parking access from a parking spot
    parkingaccess_start = find_parkingaccess_from_parking(near_parking_start);
    // find connecting lane from a parking access
    lane_start = find_lane_from_parkingaccess(parkingaccess_start);
    start_point = refine_pose_by_parking_spot(near_parking_start, start_point);
  } else {
    // Two nearest lanelets should, in theory, be the lanelets for each direction
    auto nearest_lanelets = osm_map->laneletLayer.nearest(start, 2);
    if (nearest_lanelets.empty()) {
      std::cerr << "Couldn't find nearest lanelet to start." << std::endl;
    } else {
      for (const auto & lanelet : nearest_lanelets) {
        lane_start.push_back(lanelet.id());
      }
    }
  }

  if (end_in_parking) {
    // find connecting parking access from a parking spot
    parkingaccess_end = find_parkingaccess_from_parking(near_parking_end);
    // find connecting lane from a parking access
    lane_end = find_lane_from_parkingaccess(parkingaccess_end);
    // refine goal point by parking spot
    end_point = refine_pose_by_parking_spot(near_parking_end, end_point);
  } else {
    // Two nearest lanelets should, in theory, be the lanelets for each direction
    auto nearest_lanelets = osm_map->laneletLayer.nearest(end, 2);
    if (nearest_lanelets.empty()) {
      std::cerr << "Couldn't find nearest lanelet to goal." << std::endl;
    } else {
      for (const auto & lanelet : nearest_lanelets) {
        lane_end.push_back(lanelet.id());
      }
    }
  }

  // plan a route using lanelet2 lib
  route = get_lane_route(lane_start, lane_end);

  if (route.size() > 0) {
    if (start_in_parking) {
      // parking, parking access, route
      route.insert(route.begin(), {near_parking_start, parkingaccess_start});
    }
    if (end_in_parking) {
      // route, parking access, parking
      route.insert(route.end(), {parkingaccess_end, near_parking_end});
    }

    return true;
  } else {
    return false;
  }
}

TrajectoryPoint Lanelet2GlobalPlanner::refine_pose_by_parking_spot(
  const lanelet::Id & parking_id,
  const TrajectoryPoint & input_point)
const
{
  // This function refines the pose within a parking spot into better pose where it is
  // located at the center of parking spot and is oriented parallel to the parking spot
  //
  // TODO(mitsudome-r): this function makes the following assumptions about parking spot:
  // 1. parking spot is made of 5 points where first and last point overlap
  // 2. parking spot is rectangle in shape
  // We probably want to encode more information in the map in first place if we want to
  // make this function more robust.

  // do not refine point at failure
  if (!osm_map->areaLayer.exists(parking_id)) {
    return input_point;
  }
  const auto parking_poly = osm_map->areaLayer.get(parking_id).outerBoundPolygon().basicPolygon();

  // we assume that parking spot is rectangle shape which has 5 points
  // where first and last point overlap
  if (parking_poly.size() != 5) {
    return input_point;
  }

  // find centerline of parking spot
  const auto & p0 = parking_poly[0];
  const auto & p1 = parking_poly[1];
  const auto & p2 = parking_poly[2];
  const auto & p3 = parking_poly[3];

  TrajectoryPoint center_p1, center_p2;

  // find longer side of parking spot
  if (lanelet::geometry::distance2d(p0, p1) > lanelet::geometry::distance2d(p1, p2)) {
    const auto c_p1 = (p0 + p3) / 2.0;
    const auto c_p2 = (p1 + p2) / 2.0;
    center_p1.x = static_cast<float>(c_p1.x());
    center_p1.y = static_cast<float>(c_p1.y());
    center_p2.x = static_cast<float>(c_p2.x());
    center_p2.y = static_cast<float>(c_p2.y());
  } else {
    const auto c_p1 = (p0 + p1) / 2.0;
    const auto c_p2 = (p2 + p3) / 2.0;
    center_p1.x = static_cast<float>(c_p1.x());
    center_p1.y = static_cast<float>(c_p1.y());
    center_p2.x = static_cast<float>(c_p2.x());
    center_p2.y = static_cast<float>(c_p2.y());
  }

  // calculate refined point
  TrajectoryPoint refined_point;

  // Get centerpoint of centerline
  refined_point.x = (center_p1.x + center_p2.x) / 2.0f;
  refined_point.y = (center_p1.y + center_p2.y) / 2.0f;

  const auto direction_vector = autoware::common::geometry::minus_2d(center_p2, center_p1);
  const auto angle_center_line = std::atan2(direction_vector.y, direction_vector.x);
  const auto heading_center_line = ::motion::motion_common::from_angle(angle_center_line);
  const auto angle_diff =
    std::abs(::motion::motion_common::to_angle(input_point.heading - heading_center_line));

  if (static_cast<double>(angle_diff) < M_PI / 2.0) {
    refined_point.heading = heading_center_line;
  } else {
    refined_point.heading =
      ::motion::motion_common::from_angle(static_cast<double>(angle_center_line) + M_PI);
  }

  return refined_point;
}

bool8_t Lanelet2GlobalPlanner::point_in_parking_spot(
  const lanelet::Point3d & point, const lanelet::Id & parking_id)
const
{
  if (osm_map->areaLayer.exists(parking_id)) {
    const auto parking_area = osm_map->areaLayer.get(parking_id);
    const auto parking_poly = parking_area.outerBoundPolygon().basicPolygon();

    return lanelet::geometry::within(point, parking_poly);
  } else {
    return false;
  }
}

std::string Lanelet2GlobalPlanner::get_primitive_type(const lanelet::Id & prim_id)
{
  if (osm_map->laneletLayer.exists(prim_id)) {
    return "lane";
  } else if (osm_map->areaLayer.exists(prim_id)) {
    const auto area = osm_map->areaLayer.get(prim_id);
    if (area.hasAttribute("subtype") &&
      area.hasAttribute("parking_accesses") &&
      (area.attribute("subtype") == "parking_spot" ||
      area.attribute("subtype") == "parking_spot,drop_off,pick_up"))
    {
      return "parking";
    } else if (area.hasAttribute("subtype") &&  // NOLINT
      area.hasAttribute("ref_lanelet") &&
      area.attribute("subtype") == "parking_access")
    {
      return "drivable_area";
    } else {
      return "unknown";
    }
  } else {
    return "unknown";
  }
}

lanelet::Id Lanelet2GlobalPlanner::find_nearparking_from_point(const lanelet::Point3d & point)
const
{
  // loop through parking space to find the closest distance error
  float64_t min_dist = 1e9;
  auto it = std::min_element(
    std::begin(parking_id_list),
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
  return parking_id_list[static_cast<size_t>(std::distance(std::begin(parking_id_list), it))];
}

lanelet::Id Lanelet2GlobalPlanner::find_nearroute_from_parking(const lanelet::Id & park_id)
const
{
  lanelet::Id lane_id = -1;
  if (osm_map->areaLayer.exists(park_id)) {
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
  if (osm_map->areaLayer.exists(park_id)) {
    // search the map
    auto it_parking = parking2access_map.find(park_id);
    if (it_parking != parking2access_map.end()) {
      // just in case if there is more than one id in the vector<Id>
      // pick the first parking access for now
      // it should be one-to-one anyway
      parking_access_id = it_parking->second.at(0);
    }
  }
  return parking_access_id;
}

std::vector<lanelet::Id> Lanelet2GlobalPlanner::find_lane_from_parkingaccess(
  const lanelet::Id & parkaccess_id) const
{
  std::vector<lanelet::Id> lane_id{};
  if (osm_map->areaLayer.exists(parkaccess_id)) {
    // search the map
    auto it_lane = access2lane_map.find(parkaccess_id);
    if (it_lane != access2lane_map.end()) {
      // return available lane ids
      lane_id = it_lane->second;
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
  const std::vector<lanelet::Id> & from_id, const std::vector<lanelet::Id> & to_id) const
{
  lanelet::traffic_rules::TrafficRulesPtr trafficRules =
    lanelet::traffic_rules::TrafficRulesFactory::create(
    lanelet::Locations::Germany,
    lanelet::Participants::Vehicle);
  lanelet::routing::RoutingGraphUPtr routingGraph =
    lanelet::routing::RoutingGraph::build(*osm_map, *trafficRules);

  // plan a shortest path without a lane change from the given from:to combination
  float64_t shortest_length = std::numeric_limits<float64_t>::max();
  std::vector<lanelet::Id> shortest_route;
  for (auto start_id : from_id) {
    for (auto end_id : to_id) {
      lanelet::ConstLanelet fromLanelet = osm_map->laneletLayer.get(start_id);
      lanelet::ConstLanelet toLanelet = osm_map->laneletLayer.get(end_id);
      lanelet::Optional<lanelet::routing::Route> route = routingGraph->getRoute(
        fromLanelet, toLanelet, 0);

      // check route validity before continue further
      if (route) {
        // op for the use of shortest path in this implementation
        lanelet::routing::LaneletPath shortestPath = route->shortestPath();
        lanelet::LaneletSequence fullLane = route->fullLane(fromLanelet);
        const auto route_length = route->length2d();
        if (!shortestPath.empty() && !fullLane.empty() && shortest_length > route_length) {
          // add to the list
          shortest_length = route_length;
          shortest_route = fullLane.ids();
        }
      }
    }
  }

  return shortest_route;
}

bool8_t Lanelet2GlobalPlanner::compute_parking_center(
  lanelet::Id & parking_id, lanelet::Point3d & parking_center) const
{
  bool8_t result = false;
  // Point3d parking_center;
  if (osm_map->areaLayer.exists(parking_id) &&
    osm_map->areaLayer.get(parking_id).outerBoundPolygon().size() > 0U)
  {
    // sum x,y,z points
    float64_t mean_x = 0.0;
    float64_t mean_y = 0.0;
    float64_t mean_z = 0.0;
    const auto area3d = osm_map->areaLayer.get(parking_id).outerBoundPolygon().basicPolygon();
    size_t num_points = area3d.size();
    std::for_each(
      area3d.begin(), area3d.end(), [&](lanelet::BasicPoint3d p)
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
  size_t counter = 0U;
  size_t start = 0U;
  size_t end = 0U;
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
}  // namespace autoware
