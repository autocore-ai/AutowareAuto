// Copyright 2020 Embotech AG, Zurich, Switzerland. Arm Limited. All rights reserved.
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

// This file contains the "main entry point" of the parking planner in the form of
// a class "ParkingPlanner" and its main method, "plan" (at the end of the file).
#include <CGAL/Boolean_set_operations_2.h>

#include <common/types.hpp>
#include <geometry/common_2d.hpp>
#include <geometry/convex_hull.hpp>
#include <geometry/hull_pockets.hpp>
#include <had_map_utils/had_map_visualization.hpp>
#include <motion_common/motion_common.hpp>

#include <iostream>
#include <vector>
#include <limits>
#include <list>

#include "parking_planner/configuration.hpp"
#include "parking_planner/parking_planner.hpp"
#include "parking_planner/parking_planner_types.hpp"
#include "parking_planner/astar_path_planner.hpp"

using motion::motion_common::from_angle;
namespace autoware
{

namespace motion
{

namespace planning
{

namespace parking_planner
{

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
using Point = geometry_msgs::msg::Point32;
using Kernel = CGAL::Exact_predicates_exact_constructions_kernel;
using CGAL_Point = Kernel::Point_2;
using CGAL_Polygon = CGAL::Polygon_2<Kernel>;
using CGAL_Polygon_with_holes = CGAL::Polygon_with_holes_2<Kernel>;

PlanningResult::PlanningResult(
  const Trajectory<float64_t> trajectory,
  const std::size_t nlp_iterations,
  const float64_t nlp_proc_time,
  const PlanningStatus status)
{
  m_trajectory = trajectory;
  m_nlp_iterations = nlp_iterations;
  m_nlp_proc_time = nlp_proc_time;
  m_status = status;
}


// ---- Initial trajectory guess computation that goes into NLP -----------------------------
static std::vector<VehicleState<float64_t>> resize_state_vector(
  const std::vector<VehicleState<float64_t>> & states_input,
  const std::size_t target_length)
{
  // Make the state trajectory as long as we need the output to be. This is done in a
  // somewhat naïve fashion, because it works well enough.

  // - If length matches, we just make an owned copy to make the interface the same
  //   as for the modified-length-versions
  const auto input_length = states_input.size();
  if (input_length == target_length) {
    return std::vector<VehicleState<float64_t>>(states_input);
  }

  // - Length does not match
  std::vector<VehicleState<float64_t>> resized_states{};
  if (input_length < target_length) {  // Grow the vector to the target length
    // Copy existing trajectory
    resized_states.insert(resized_states.end(), states_input.begin(), states_input.end() );

    // Repeat the last entry as many times as needed
    for (std::size_t k = {}; k < (target_length - input_length); k++) {
      resized_states.push_back(states_input.back() );
    }

  } else if (input_length > target_length) {  // Shrink the vector to the target length
    const std::size_t number_of_points_to_skip = input_length - target_length;

    // Pick a stride that's big enough to allow us to skip the necessary number of points
    std::size_t skip_length = 1;
    while (skip_length * target_length < input_length) {
      skip_length += 1;
    }

    // Skip points with the given skip length until we're close to the end
    auto read_iter = states_input.begin();
    std::size_t number_of_skipped_points = 0;
    for (; number_of_skipped_points < (number_of_points_to_skip - skip_length);
      number_of_skipped_points += skip_length)
    {
      resized_states.push_back(*read_iter);
      read_iter += static_cast<int64_t>(skip_length + 1);
    }

    // The final amount of points to skip is less than skip_length now
    read_iter += static_cast<int64_t>(number_of_points_to_skip - number_of_skipped_points);

    // Copy over the rest normally
    for (; read_iter != states_input.end(); read_iter++) {
      resized_states.push_back(*read_iter);
    }
  }

  // float64_t-check if the resizing worked properly
  if (resized_states.size() != target_length) {
    throw std::length_error("Resize code resized to the wrong length");
  }

  return resized_states;
}

// --- Helpers for "execute_planning" ---
static Point lanelet_point_to_point(const lanelet::ConstPoint3d & lanelet_point)
{
  Point point;
  point.x = static_cast<float32_t>(lanelet_point.x());
  point.y = static_cast<float32_t>(lanelet_point.y());
  return point;
}

bool are_points_equal(const Point & p1, const Point & p2)
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
  const lanelet::Polygon3d & drivable_area,
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

std::vector<Polytope2D<float64_t>> convert_drivable_area_to_obstacles(
  const lanelet::Polygon3d & drivable_area)
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
    const auto drivable_area_hull_end = autoware::common::geometry::convex_hull(drivable_area_hull);
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
  std::vector<Polytope2D<float64_t>> obstacles{};
  for (const auto & hull : all_hulls) {
    std::vector<Point2D<float64_t>> parking_points{};
    for (auto it = hull.begin(); it != hull.end(); ++it) {
      parking_points.emplace_back(Point2D<float64_t>(it->x, it->y));
    }
    // Parking polytopes need ccw, lanelet does clockwise
    const auto point_p2g = [](const Point2D<float64_t> & pt) {
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
    obstacles.emplace_back(Polytope2D<float64_t>(parking_points));
  }

  return obstacles;
}

autoware_auto_msgs::msg::Trajectory convert_parking_planner_to_autoware_trajectory(
  const Trajectory<float64_t> & parking_trajectory)
{
  // These constants come from parking_planner/configuration.hpp
  float32_t time_from_start = 0.0;
  const float32_t time_step = static_cast<float32_t>(INTEGRATION_STEP_SIZE);

  // Create one trajectory point for each parking planner trajectory point.
  autoware_auto_msgs::msg::Trajectory trajectory{};
  trajectory.header.frame_id = "map";
  for (const auto & step : parking_trajectory) {
    auto parking_state = step.get_state();
    auto parking_command = step.get_command();

    autoware_auto_msgs::msg::TrajectoryPoint pt{};
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

// TODO(s.me) this is getting a bit long, break up
lanelet::Polygon3d coalesce_drivable_areas(
  const autoware_auto_msgs::msg::Route & route,
  const lanelet::LaneletMapPtr & lanelet_map_ptr)
{
  CGAL_Polygon_with_holes drivable_area;

  for (const auto & map_primitive : route.primitives) {
    // Attempt to obtain a polygon from the primitive ID
    geometry_msgs::msg::Polygon current_area_polygon{};
    if (lanelet_map_ptr->lineStringLayer.exists(map_primitive.id) ) {
      // The ID corresponds to a linestring, so the find() call below should not become null
      lanelet::LineString3d current_area = *lanelet_map_ptr->lineStringLayer.find(map_primitive.id);
      autoware::common::had_map_utils::lineString2Polygon(current_area, &current_area_polygon);
    } else if (lanelet_map_ptr->laneletLayer.exists(map_primitive.id)) {
      // The ID corresponds to a lanelet, so the find() call below should not become null
      lanelet::ConstLanelet current_area = *lanelet_map_ptr->laneletLayer.find(map_primitive.id);
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
  std::vector<lanelet::Point3d> lanelet_drivable_area_points{};
  lanelet_drivable_area_points.reserve(drivable_area.outer_boundary().size());
  for (auto p = drivable_area.outer_boundary().vertices_begin();
    p != drivable_area.outer_boundary().vertices_end(); p++)
  {
    lanelet_drivable_area_points.emplace_back(
      lanelet::Point3d(
        lanelet::utils::getId(), CGAL::to_double(p->x()),
        CGAL::to_double(p->y()), 0.0));
  }
  lanelet::Polygon3d lanelet_drivable_area(lanelet::utils::getId(), lanelet_drivable_area_points);
  return lanelet_drivable_area;
}

Trajectory<float64_t> ParkingPlanner::create_trajectory_from_states(
  const std::vector<VehicleState<float64_t>> & states_input,
  const std::size_t desired_trajectory_length) const
{
  // Create a version of the states vector that is of the length we need
  const auto resized_states = resize_state_vector(states_input, desired_trajectory_length);

  // Splice those states into a Trajectory along with a guess of "the commands are just zero"
  Trajectory<float64_t> trajectory{};
  for (const auto & state : resized_states) {
    const auto input_guess = VehicleCommand<float64_t>{};
    trajectory.push_back(TrajectoryStep<float64_t>(input_guess, state));
  }

  return trajectory;
}


// ---- Main parking planner interface ------------------------------------------------------
ParkingPlanner::ParkingPlanner(
  const BicycleModelParameters<float64_t> & parameters,
  const NLPCostWeights<float64_t> & nlp_weights,
  const VehicleState<float64_t> & lower_state_bounds,
  const VehicleState<float64_t> & upper_state_bounds,
  const VehicleCommand<float64_t> & lower_command_bounds,
  const VehicleCommand<float64_t> & upper_command_bounds
)
: m_nlp_planner(nlp_weights, lower_state_bounds, upper_state_bounds,
    lower_command_bounds, upper_command_bounds), m_model_parameters(parameters)
{
  m_astar_planner = AstarPathPlanner();
}


PlanningResult ParkingPlanner::plan(
  const VehicleState<float64_t> & current_state,
  const VehicleState<float64_t> & goal_state,
  const std::vector<Polytope2D<float64_t>> & obstacles
) const
{
  // If the starting and target angles would be closer if we shift by 2*pi in either
  // direction, do that. This will help avoid the planner coming up with "loop" solutions.
  // TODO(s.me) this should be cleaned up after AVP
  const auto goal_heading = goal_state.get_heading();
  const auto current_heading = current_state.get_heading();
  const auto current_heading_difference = std::abs(goal_heading - current_heading);
  VehicleState<float64_t> adapted_goal_state = goal_state;
  const auto pi = 3.14159;
  if (std::abs(goal_heading - (2 * pi) - current_heading) < current_heading_difference) {
    adapted_goal_state.set_heading(goal_heading - (2 * pi) );
  } else if (std::abs(goal_heading + (2 * pi) - current_heading) < current_heading_difference) {
    adapted_goal_state.set_heading(goal_heading + (2 * pi) );
  }

  // Run discretized global A* planner
  const auto vehicle_bounding_box =
    BicycleModel<float64_t,
      float64_t>(m_model_parameters).compute_bounding_box(VehicleState<float64_t>{});
  const std::vector<VehicleState<float64_t>> astar_output =
    m_astar_planner.plan_astar(
    current_state,
    adapted_goal_state,
    vehicle_bounding_box,
    obstacles);

  // Translate A* planner output to a trajectory with commands
  const auto trajectory_guess = this->create_trajectory_from_states(astar_output, HORIZON_LENGTH);

  // Run NLP smoother, warm-started using the A* guess
  auto nlp_results = m_nlp_planner.plan_nlp(
    current_state, adapted_goal_state, trajectory_guess,
    obstacles, m_model_parameters);
  const auto smoothed_trajectory = nlp_results.m_trajectory;
  const std::size_t nlp_iterations =
    static_cast<std::size_t>(nlp_results.m_solve_info["iter_count"].to_int());
  const float64_t nlp_proc_time = nlp_results.m_solve_info["t_proc_total"].to_double();

  // Perform post-checking of trajectory for collisions and dynamics
  const auto checking_tolerance = 1e-4;
  const auto trajectory_ok = m_nlp_planner.check_trajectory(
    smoothed_trajectory, current_state,
    adapted_goal_state, obstacles, m_model_parameters, checking_tolerance);
  std::cout << "Trajectory ok is: " << trajectory_ok << std::endl;

  // Return final result
  if (trajectory_ok) {
    return PlanningResult(smoothed_trajectory, nlp_iterations, nlp_proc_time, PlanningStatus::OK);
  } else {
    return PlanningResult(
      smoothed_trajectory, nlp_iterations, nlp_proc_time,
      PlanningStatus::NLP_ERROR);
  }
}

}  // namespace parking_planner
}  // namespace planning
}  // namespace motion
}  // namespace autoware
