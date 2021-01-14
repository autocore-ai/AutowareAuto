// Copyright 2020 Embotech AG, Zurich, Switzerland, inspired by Christopher Ho's mpc code
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <CGAL/Boolean_set_operations_2.h>

#include <common/types.hpp>
#include <parking_planner_nodes/parking_planner_node.hpp>
#include <parking_planner/parking_planner.hpp>
#include <parking_planner/configuration.hpp>
#include <parking_planner/geometry.hpp>
#include <autoware_auto_msgs/msg/trajectory_point.hpp>
#include <autoware_auto_msgs/msg/trajectory.hpp>
#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_core/primitives/Polygon.h>
#include <autoware_auto_msgs/srv/had_map_service.hpp>
#include <had_map_utils/had_map_conversion.hpp>
#include <had_map_utils/had_map_visualization.hpp>
#include <had_map_utils/had_map_query.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <autoware_auto_tf2/tf2_autoware_auto_msgs.hpp>
#include <autoware_auto_msgs/msg/had_map_bin.hpp>
#include <geometry/common_2d.hpp>
#include <geometry/convex_hull.hpp>
#include <geometry/hull_pockets.hpp>
#include <geometry/bounding_box/rotating_calipers.hpp>
#include <motion_common/motion_common.hpp>
#include <chrono>
#include <algorithm>
#include <memory>
#include <limits>
#include <vector>
#include <list>
#include <string>
#include <utility>
#include <thread>

using Kernel = CGAL::Exact_predicates_exact_constructions_kernel;
using CGAL_Point = Kernel::Point_2;
using CGAL_Polygon = CGAL::Polygon_2<Kernel>;
using CGAL_Polygon_with_holes = CGAL::Polygon_with_holes_2<Kernel>;

using motion::motion_common::to_angle;
using motion::motion_common::from_angle;
using namespace std::chrono_literals;

namespace autoware
{
namespace motion
{
namespace planning
{
namespace parking_planner_nodes
{

// Bring in some classes from the parking planner and give them names
// that distinguish them from their Autoware counterparts
using ParkerVehicleState = autoware::motion::planning::parking_planner::VehicleState<float64_t>;
using ParkerVehicleCommand = autoware::motion::planning::parking_planner::VehicleCommand<float64_t>;
using ParkerNLPCostWeights = autoware::motion::planning::parking_planner::NLPCostWeights<float64_t>;
using ParkerModelParameters =
  autoware::motion::planning::parking_planner::BicycleModelParameters<float64_t>;
using ParkingPolytope = autoware::motion::planning::parking_planner::Polytope2D<float64_t>;
using ParkingPlanner = autoware::motion::planning::parking_planner::ParkingPlanner;
using ParkingStatus = autoware::motion::planning::parking_planner::PlanningStatus;
using ParkingTrajectory = autoware::motion::planning::parking_planner::Trajectory<float64_t>;
using PlanningStatus = autoware::motion::planning::parking_planner::PlanningStatus;
using ParkingPoint = autoware::motion::planning::parking_planner::Point2D<float64_t>;

using autoware_auto_msgs::msg::TrajectoryPoint;
using autoware_auto_msgs::msg::BoundingBoxArray;
using autoware_auto_msgs::msg::BoundingBox;

using Point = geometry_msgs::msg::Point32;
using Polygon = geometry_msgs::msg::Polygon;
using autoware::common::types::float32_t;
using autoware::common::types::float64_t;
using autoware::common::geometry::convex_hull;
using autoware::common::geometry::ccw;
using autoware::common::geometry::hull_pockets;
using autoware::common::geometry::minus_2d;
using autoware::common::geometry::plus_2d;
using autoware::common::geometry::norm_2d;
using autoware::common::geometry::times_2d;
using autoware::common::geometry::get_normal;
using autoware::common::geometry::bounding_box::minimum_perimeter_bounding_box;


using lanelet::Point3d;
using lanelet::ConstLanelet;
using lanelet::Polygon3d;
using lanelet::LineString3d;
using lanelet::utils::getId;

ParkingPlannerNode::ParkingPlannerNode(
  const rclcpp::NodeOptions & options)
: TrajectoryPlannerNodeBase("parking_planner", "plan_parking_trajectory", options)
{
  const auto f32_param = [this](const std::string & name) {
      return static_cast<Real>(declare_parameter(name).get<float32_t>());
    };

  const VehicleConfig vehicle_param{
    f32_param("vehicle.cg_to_front_m"),
    f32_param("vehicle.cg_to_rear_m"),
    f32_param("vehicle.front_corner_stiffness"),
    f32_param("vehicle.rear_corner_stiffness"),
    f32_param("vehicle.mass_kg"),
    f32_param("vehicle.yaw_inertia_kgm2"),
    f32_param("vehicle.width_m"),
    f32_param("vehicle.front_overhang_m"),
    f32_param("vehicle.rear_overhang_m")
  };

  const ParkerNLPCostWeights optimization_weights{
    f32_param("optimization_weights.steering"),
    f32_param("optimization_weights.throttle"),
    f32_param("optimization_weights.goal")
  };

  const ParkerVehicleState lower_state_bounds{
    f32_param("state_bounds.lower.x_m"),
    f32_param("state_bounds.lower.y_m"),
    f32_param("state_bounds.lower.velocity_mps"),
    f32_param("state_bounds.lower.heading_rad"),
    f32_param("state_bounds.lower.steering_rad")
  };

  const ParkerVehicleState upper_state_bounds{
    f32_param("state_bounds.upper.x_m"),
    f32_param("state_bounds.upper.y_m"),
    f32_param("state_bounds.upper.velocity_mps"),
    f32_param("state_bounds.upper.heading_rad"),
    f32_param("state_bounds.upper.steering_rad")
  };

  const ParkerVehicleCommand lower_command_bounds{
    f32_param("command_bounds.lower.steering_rate_rps"),
    f32_param("command_bounds.lower.throttle_mps2"),
  };

  const ParkerVehicleCommand upper_command_bounds{
    f32_param("command_bounds.upper.steering_rate_rps"),
    f32_param("command_bounds.upper.throttle_mps2"),
  };

  init(
    vehicle_param, optimization_weights, lower_state_bounds, upper_state_bounds,
    lower_command_bounds, upper_command_bounds);
}

void ParkingPlannerNode::init(
  const VehicleConfig & vehicle_param,
  const ParkerNLPCostWeights & optimization_weights,
  const ParkerVehicleState & lower_state_bounds,
  const ParkerVehicleState & upper_state_bounds,
  const ParkerVehicleCommand & lower_command_bounds,
  const ParkerVehicleCommand & upper_command_bounds
)
{
  const ParkerModelParameters model_parameters(
    vehicle_param.length_cg_front_axel(),
    vehicle_param.length_cg_rear_axel(),
    vehicle_param.width(),
    vehicle_param.front_overhang(),
    vehicle_param.rear_overhang()
  );

  // Create and set a base planner object that we'll copy for individual planning runs
  m_planner = std::make_unique<autoware::motion::planning::parking_planner::ParkingPlanner>(
    model_parameters, optimization_weights,
    lower_state_bounds,
    upper_state_bounds, lower_command_bounds, upper_command_bounds);

  m_debug_obstacles_publisher = this->create_publisher<visualization_msgs::msg::MarkerArray>(
    "parking_debug_obstacles",
    rclcpp::QoS(rclcpp::KeepLast(5U)).transient_local());

  m_debug_trajectory_publisher = this->create_publisher<autoware_auto_msgs::msg::Trajectory>(
    "parking_debug_trajectory",
    rclcpp::QoS(rclcpp::KeepLast(5U)).transient_local());


  m_debug_start_end_publisher = this->create_publisher<autoware_auto_msgs::msg::BoundingBoxArray>(
    "parking_debug_start_end",
    rclcpp::QoS(rclcpp::KeepLast(5U)).transient_local());
}

// --- Helpers for "execute_planning" ---
static Point lanelet_point_to_point(const lanelet::ConstPoint3d & lanelet_point)
{
  Point point;
  point.x = static_cast<float32_t>(lanelet_point.x());
  point.y = static_cast<float32_t>(lanelet_point.y());
  return point;
}

static bool are_points_equal(const Point & p1, const Point & p2)
{
  return norm_2d(minus_2d(p1, p2)) < std::numeric_limits<float32_t>::epsilon();
}

template<typename Iter1, typename Iter2>
static typename std::vector<std::list<Point>> get_pocket_hulls(
  const Iter1 polygon_start,
  const Iter1 polygon_end,
  const Iter2 convex_hull_start,
  const Iter2 convex_hull_end
)
{
  // - Get pockets in that convex hull (hence the above rotation)
  const auto pocket_list = hull_pockets(
    polygon_start, polygon_end,
    convex_hull_start, convex_hull_end);

  // - Create owned convex hulls of the pockets
  std::vector<std::list<Point>> owned_pocket_hulls;
  for (const auto & pocket_vector : pocket_list) {
    // Convert the pocket to an std::list for convex_hull() to mutate
    auto pocket = std::list<Point>{pocket_vector.begin(), pocket_vector.end()};
    {
      // Inner scope: we don't want those new identifiers to stay available
      const auto pocket_end = convex_hull(pocket);
      typename std::list<Point>::const_iterator pocket_begin = pocket.begin();
      // We only care about the convex hull, throw away interior points
      pocket.resize(static_cast<uint32_t>(std::distance(pocket_begin, pocket_end)));
    }
    owned_pocket_hulls.emplace_back(pocket);
  }

  return owned_pocket_hulls;
}

template<typename Iter>
static typename std::vector<std::list<Point>> get_outer_boxes(
  const Polygon3d & drivable_area,
  const Iter convex_hull_start,
  const Iter convex_hull_end
)
{
  std::vector<std::list<Point>> outer_boxes{};
  for (uint32_t k = {}; k < drivable_area.numSegments(); ++k) {
    // Create an owned segment vector of points, for comparisons with std::search
    const std::vector<Point> segment_vector{
      lanelet_point_to_point(drivable_area.segment(k).first),
      lanelet_point_to_point(drivable_area.segment(k).second),
    };

    // Check if segment is on the convex hull. The second part of the condition
    // checks the "rollover segment", as in the segment obtained by connecting the
    // final point with the first point again.
    if ( ( std::search(
        convex_hull_start, convex_hull_end,
        segment_vector.begin(), segment_vector.end(), are_points_equal) != convex_hull_end ) ||
      (are_points_equal(segment_vector[0], *(std::prev(convex_hull_end))) &&
      are_points_equal(segment_vector[1], *convex_hull_start) ) )
    {
      auto orthogonal = get_normal(minus_2d(segment_vector[1], segment_vector[0]));
      orthogonal = times_2d(orthogonal, -1.0f / norm_2d(orthogonal));

      const auto box = std::list<Point>{
        segment_vector[0],
        plus_2d(segment_vector[0], times_2d(orthogonal, 0.2f)),
        plus_2d(segment_vector[1], times_2d(orthogonal, 0.2f)),
        segment_vector[1],
      };
      outer_boxes.emplace_back(box);
    }
  }

  return outer_boxes;
}


static ParkerVehicleState convert_trajectorypoint_to_vehiclestate(const TrajectoryPoint & point)
{
  return ParkerVehicleState{point.x, point.y, point.longitudinal_velocity_mps,
    to_angle(point.heading), point.front_wheel_angle_rad};
}

static std::vector<ParkingPolytope> convert_drivable_area_to_obstacles(
  const Polygon3d & drivable_area)
{
  // - Convert lanelet polygon to a list of points. We'll keep this list for later. Using
  //   a list and not a vector because the convex hull does as well and we'll do comparisons
  //   involving the two in iterator form.
  std::vector<Point> drivable_area_points{};
  for (uint32_t k = {}; k < drivable_area.numSegments(); ++k) {
    drivable_area_points.emplace_back(
      lanelet_point_to_point(
        drivable_area.segment(k).
        first));
  }

  // - Get convex hull of drivable surface
  std::list<Point> drivable_area_hull{drivable_area_points.begin(), drivable_area_points.end()};
  {
    // Inner scope: we don't want those new identifiers to stay available
    const auto drivable_area_hull_end = convex_hull(drivable_area_hull);
    const typename decltype(drivable_area_hull)::const_iterator
    drivable_area_hull_begin = drivable_area_hull.begin();
    // We only care about the convex hull, throw away interior points
    drivable_area_hull.resize(
      static_cast<uint32_t>(std::distance(
        drivable_area_hull_begin,
        drivable_area_hull_end)));
  }

  // - Find a point that is on the convex hull and rotate the drivable area points to start there
  auto first_hull_point = std::find_first_of(
    drivable_area_points.begin(), drivable_area_points.end(),
    drivable_area_hull.begin(), drivable_area_hull.end(), are_points_equal);
  std::rotate(drivable_area_points.begin(), first_hull_point, drivable_area_points.end() );

  // - Get pockets in that convex hull (hence the above rotation)
  const auto owned_pocket_hulls = get_pocket_hulls(
    drivable_area_points.begin(), drivable_area_points.end(),
    drivable_area_hull.begin(), drivable_area_hull.end());

  // - Get outer boxes where necessary
  const auto outer_boxes = get_outer_boxes(
    drivable_area, drivable_area_hull.begin(),
    drivable_area_hull.end());

  // - Merge outer boxes and pocket hulls into one big list
  auto all_hulls = owned_pocket_hulls;
  all_hulls.insert(all_hulls.end(), outer_boxes.begin(), outer_boxes.end());

  // - Compute parking polytopes from the convex hulls of the outer boxes as well as the pockets
  std::vector<ParkingPolytope> obstacles{};
  for (const auto & hull : all_hulls) {
    std::vector<ParkingPoint> parking_points{};
    for (auto it = hull.begin(); it != hull.end(); ++it) {
      parking_points.emplace_back(ParkingPoint(it->x, it->y));
    }
    // Parking polytopes need ccw, lanelet does clockwise
    const auto point_p2g = [](const ParkingPoint & pt) {
        Point out;
        out.x = static_cast<float32_t>(std::get<0>(pt.get_coord()));
        out.y = static_cast<float32_t>(std::get<1>(pt.get_coord()));
        return out;
      };
    if (parking_points.size() >= 3 &&
      ccw(
        point_p2g(parking_points[0]), point_p2g(parking_points[1]),
        point_p2g(parking_points[2])) )
    {
      std::reverse(parking_points.begin(), parking_points.end());
    }
    obstacles.emplace_back(ParkingPolytope(parking_points));
  }

  return obstacles;
}

static AutowareTrajectory convert_parking_planner_to_autoware_trajectory(
  const ParkingTrajectory & parking_trajectory)
{
  // These constants come from parking_planner/configuration.hpp
  float32_t time_from_start = 0.0;
  const float32_t time_step = static_cast<float32_t>(
    autoware::motion::planning::parking_planner::INTEGRATION_STEP_SIZE);

  // Create one trajectory point for each parking planner trajectory point.
  AutowareTrajectory trajectory{};
  trajectory.header.frame_id = "map";
  for (const auto & step : parking_trajectory) {
    auto parking_state = step.get_state();
    auto parking_command = step.get_command();

    TrajectoryPoint pt{};
    pt.x = static_cast<float32_t>(parking_state.get_x());
    pt.y = static_cast<float32_t>(parking_state.get_y());
    pt.heading = from_angle(static_cast<float32_t>(parking_state.get_heading()));
    pt.longitudinal_velocity_mps = static_cast<float32_t>(parking_state.get_velocity());
    pt.lateral_velocity_mps = 0.0f;  // The parking planner has the kinematic model, there
                                     // is no lateral velocity in that
    pt.front_wheel_angle_rad = static_cast<float32_t>(parking_state.get_steering());
    // The parking planner does not consider mass at this point
    pt.acceleration_mps2 = static_cast<float32_t>(parking_command.get_throttle());
    pt.heading_rate_rps = static_cast<float32_t>(parking_command.get_steering_rate());
    pt.rear_wheel_angle_rad = 0.0f;  // The parking planner does not support rear wheel steering

    float32_t t_s = std::floor(time_from_start);
    float32_t t_ns = (time_from_start - t_s) * 1.0e9f;
    pt.time_from_start.sec = static_cast<int32_t>(t_s);
    pt.time_from_start.nanosec = static_cast<uint32_t>(t_ns);

    trajectory.points.push_back(pt);

    time_from_start += time_step;
  }

  return trajectory;
}

HADMapService::Request ParkingPlannerNode::create_map_request(const Route & route)
{
  HADMapService::Request request{};
  request.requested_primitives.push_back(
    autoware_auto_msgs::srv::HADMapService_Request::DRIVEABLE_GEOMETRY);

  const auto BOX_PADDING = 10.0f;
  request.geom_upper_bound.push_back(
    std::fmax(
      route.start_point.x,
      route.goal_point.x) + BOX_PADDING);
  request.geom_upper_bound.push_back(
    std::fmax(
      route.start_point.y,
      route.goal_point.y) + BOX_PADDING);
  request.geom_upper_bound.push_back(0.0);
  request.geom_lower_bound.push_back(
    std::fmin(
      route.start_point.x,
      route.goal_point.x) - BOX_PADDING);
  request.geom_lower_bound.push_back(
    std::fmin(
      route.start_point.y,
      route.goal_point.y) - BOX_PADDING);
  request.geom_lower_bound.push_back(0.0);
  return request;
}

// TODO(s.me) this is getting a bit long, break up
static Polygon3d coalesce_drivable_areas(
  const Route & route,
  const lanelet::LaneletMapPtr & lanelet_map_ptr)
{
  CGAL_Polygon_with_holes drivable_area;

  for (const auto & map_primitive : route.primitives) {
    // Attempt to obtain a polygon from the primitive ID
    Polygon current_area_polygon{};
    if (lanelet_map_ptr->lineStringLayer.exists(map_primitive.id) ) {
      // The ID corresponds to a linestring, so the find() call below should not become null
      LineString3d current_area = *lanelet_map_ptr->lineStringLayer.find(map_primitive.id);
      autoware::common::had_map_utils::lineString2Polygon(current_area, &current_area_polygon);
    } else if (lanelet_map_ptr->laneletLayer.exists(map_primitive.id)) {
      // The ID corresponds to a lanelet, so the find() call below should not become null
      ConstLanelet current_area = *lanelet_map_ptr->laneletLayer.find(map_primitive.id);
      autoware::common::had_map_utils::lanelet2Polygon(current_area, &current_area_polygon);
    } else {
      // This might happen if a primitive is on the route, but outside of the bounding box that we
      // query the map for. Not sure how to deal with this at this point though.
      std::cerr << "Error: primitive ID " << map_primitive.id << " not found, skipping" <<
        std::endl;
    }

    if (drivable_area.outer_boundary().size() > 0) {
      // Convert current_area_polygon to a CGAL_Polygon and make sure the orientation is correct
      CGAL_Polygon to_join{};
      CGAL_Polygon_with_holes temporary_union;
      const auto first_point = current_area_polygon.points.begin();
      for (auto area_point_it =
        current_area_polygon.points.begin();
        // Stop if we run out of points, or if we encounter the first point again
        area_point_it < current_area_polygon.points.end() &&
        !(first_point != area_point_it && first_point->x == area_point_it->x &&
        first_point->y == area_point_it->y);
        area_point_it++)
      {
        to_join.push_back(CGAL_Point(area_point_it->x, area_point_it->y));
      }

      if (to_join.is_clockwise_oriented() ) {
        to_join.reverse_orientation();
      }

      // Merge this CGAL polygon with the growing drivable_area. We need an intermediate merge
      // result because as far as I can tell from the CGAL docs, I can't "join to" a polygon
      // in-place with the join() interface.
      const auto polygons_overlap = CGAL::join(drivable_area, to_join, temporary_union);
      if (!polygons_overlap && !drivable_area.outer_boundary().is_empty()) {
        // TODO(s.me) cancel here? Right now we just ignore that polygon, if it doesn't
        // overlap with the rest, there is no way to get to it anyway
        std::cerr << "Error: polygons in union do not overlap!" << std::endl;
      } else {
        drivable_area = temporary_union;
      }
    } else {
      // Otherwise, just set the current drivable area equal to the area to add to it, because
      // CGAL seems to do "union(empty, non-empty) = empty" for some reason.
      const auto first_point = current_area_polygon.points.begin();
      for (auto area_point_it =
        current_area_polygon.points.begin();
        area_point_it < current_area_polygon.points.end() &&
        // Stop if we run out of points, or if we encounter the first point again
        !(first_point != area_point_it && first_point->x == area_point_it->x &&
        first_point->y == area_point_it->y);
        area_point_it++)
      {
        drivable_area.outer_boundary().push_back(CGAL_Point(area_point_it->x, area_point_it->y));
      }
      if (drivable_area.outer_boundary().is_clockwise_oriented() ) {
        drivable_area.outer_boundary().reverse_orientation();
      }
    }
  }

  // At this point, all the polygons from the route should be merged into drivable_area,
  // and we now need to turn this back into a lanelet polygon.
  std::vector<Point3d> lanelet_drivable_area_points{};
  lanelet_drivable_area_points.reserve(drivable_area.outer_boundary().size());
  for (auto p = drivable_area.outer_boundary().vertices_begin();
    p != drivable_area.outer_boundary().vertices_end(); p++)
  {
    lanelet_drivable_area_points.emplace_back(
      Point3d(
        getId(), CGAL::to_double(p->x()),
        CGAL::to_double(p->y()), 0.0));
  }
  Polygon3d lanelet_drivable_area(getId(), lanelet_drivable_area_points);
  return lanelet_drivable_area;
}

void ParkingPlannerNode::debug_publish_obstacles(
  const std::vector<ParkingPolytope> & obstacles)
{
  visualization_msgs::msg::MarkerArray map_marker_array;
  rclcpp::Time marker_t = rclcpp::Time(0);

  std_msgs::msg::ColorRGBA color_obstacles;
  autoware::common::had_map_utils::setColor(&color_obstacles, 1.0f, 1.0f, 1.0f, 1.0f);

  std::vector<LineString3d> all_obstacle_linestrings{};
  for (const auto & the_obstacle : obstacles) {
    LineString3d obstacle_linestring(getId(), {});
    for (const auto & pt : the_obstacle.get_vertices() ) {
      obstacle_linestring.push_back(
        Point3d(
          getId(), std::get<0>(pt.get_coord()),
          std::get<1>(pt.get_coord()), 0.0));
    }
    all_obstacle_linestrings.push_back(obstacle_linestring);
  }

  m_debug_obstacles_publisher->publish(
    autoware::common::had_map_utils::lineStringsAsTriangleMarkerArray(
      marker_t, "parking_debug_obstacles",
      all_obstacle_linestrings,
      color_obstacles));
}

void ParkingPlannerNode::debug_publish_start_and_end(
  const ParkerVehicleState & start,
  const ParkerVehicleState & end
)
{
  const auto params = m_planner->get_parameters();
  const auto bbox_from_state = [&params](const ParkerVehicleState & state) -> BoundingBox
    {
      // Shorthands to keep the formulas sane
      const double h = state.get_heading();
      const double xcog = state.get_x(), ycog = state.get_y();
      const double lf = params.get_length_front() + params.get_front_overhang();
      const double lr = params.get_length_rear() + params.get_rear_overhang();
      const double wh = params.get_vehicle_width() * 0.5;
      const double ch = std::cos(h), sh = std::sin(h);

      // We need a list for the bounding box call later
      std::list<Point> vehicle_corners;

      { // Front left
        auto p = Point{};
        p.x = static_cast<float>(xcog + (lf * ch) - (wh * sh));
        p.y = static_cast<float>(ycog + (lf * sh) + (wh * ch));
        vehicle_corners.push_back(p);
      }
      { // Front right
        auto p = Point{};
        p.x = static_cast<float>(xcog + (lf * ch) + (wh * sh));
        p.y = static_cast<float>(ycog + (lf * sh) - (wh * ch));
        vehicle_corners.push_back(p);
      }
      { // Rear right
        auto p = Point{};
        p.x = static_cast<float>(xcog - (lr * ch) + (wh * sh));
        p.y = static_cast<float>(ycog - (lr * sh) - (wh * ch));
        vehicle_corners.push_back(p);
      }
      { // Rear right
        auto p = Point{};
        p.x = static_cast<float>(xcog - (lr * ch) - (wh * sh));
        p.y = static_cast<float>(ycog - (lr * sh) + (wh * ch));
        vehicle_corners.push_back(p);
      }

      return minimum_perimeter_bounding_box(vehicle_corners);
    };

  BoundingBoxArray bbox_array;
  bbox_array.header.frame_id = "map";
  bbox_array.boxes.resize(2);
  bbox_array.boxes[0] = bbox_from_state(start);
  bbox_array.boxes[0].vehicle_label = BoundingBox::CAR;
  bbox_array.boxes[1] = bbox_from_state(end);
  bbox_array.boxes[0].vehicle_label = BoundingBox::CAR;

  m_debug_start_end_publisher->publish(bbox_array);
}

AutowareTrajectory ParkingPlannerNode::plan_trajectory(
  const Route & route,
  const lanelet::LaneletMapPtr & lanelet_map_ptr)
{
  // ---- Merge the drivable areas into one lanelet::Polygon3d --------------------------
  // TODO(s.me) For experiments, we take dummy data here.
  const Polygon3d drivable_area = coalesce_drivable_areas(route, lanelet_map_ptr);

  // ---- Obtain "list of bounding obstacles" of drivable surface -----------------------
  const auto obstacles = convert_drivable_area_to_obstacles(drivable_area);

  // - Debugging: publish the obstacles as marker arrays for inspection
  this->debug_publish_obstacles(obstacles);

  // ---- Call the actual planner with the inputs we've assembled -----------------------
  const auto start_trajectory_point = route.start_point;
  const auto goal_trajectory_point = route.goal_point;
  const auto starting_state = convert_trajectorypoint_to_vehiclestate(start_trajectory_point);
  const auto goal_state = convert_trajectorypoint_to_vehiclestate(goal_trajectory_point);

  this->debug_publish_start_and_end(starting_state, goal_state);

  const auto planner_result = m_planner->plan(starting_state, goal_state, obstacles);
  std::cout << "NLP solution took " << planner_result.get_nlp_iterations() << " iterations" <<
    std::endl;

  if (planner_result.get_status() == ParkingStatus::NLP_ERROR) {
    return AutowareTrajectory{};
  }

  // ---- Convert the trajectory to an autoware trajectory message ----------------------
  auto trajectory =
    convert_parking_planner_to_autoware_trajectory(planner_result.get_trajectory());

  m_debug_trajectory_publisher->publish(trajectory);

  return trajectory;
}

}  // namespace parking_planner_nodes
}  // namespace planning
}  // namespace motion
}  // namespace autoware
RCLCPP_COMPONENTS_REGISTER_NODE( \
  autoware::motion::planning::parking_planner_nodes::ParkingPlannerNode)
