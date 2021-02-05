// Copyright 2020 Embotech AG, Zurich, Switzerland. Arm Ltd. inspired by Christopher Ho's mpc code
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

using motion::motion_common::to_angle;
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
using ParkingStatus = autoware::motion::planning::parking_planner::PlanningStatus;

using autoware_auto_msgs::msg::TrajectoryPoint;
using autoware_auto_msgs::msg::BoundingBoxArray;
using autoware_auto_msgs::msg::BoundingBox;

using Point = geometry_msgs::msg::Point32;
using autoware::common::types::float32_t;
using autoware::common::types::float64_t;
using autoware::common::geometry::bounding_box::minimum_perimeter_bounding_box;


using lanelet::Point3d;
using lanelet::Polygon3d;
using lanelet::LineString3d;
using lanelet::Area;
using lanelet::Areas;
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

static ParkerVehicleState convert_trajectorypoint_to_vehiclestate(const TrajectoryPoint & point)
{
  return ParkerVehicleState{point.x, point.y, point.longitudinal_velocity_mps,
    to_angle(point.heading), point.front_wheel_angle_rad};
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

void ParkingPlannerNode::debug_publish_obstacles(
  const std::vector<ParkingPolytope> & obstacles)
{
  visualization_msgs::msg::MarkerArray map_marker_array;
  rclcpp::Time marker_t = rclcpp::Time(0);

  std_msgs::msg::ColorRGBA color_obstacles;
  autoware::common::had_map_utils::setColor(&color_obstacles, 1.0f, 1.0f, 1.0f, 1.0f);

  Areas all_obstacle_areas;
  for (const auto & the_obstacle : obstacles) {
    Area area;
    LineString3d obstacle_bound(getId(), {});
    for (const auto & pt : the_obstacle.get_vertices() ) {
      obstacle_bound.push_back(
        Point3d(
          getId(), std::get<0>(pt.get_coord()),
          std::get<1>(pt.get_coord()), 0.0));
    }
    area.setOuterBound({obstacle_bound});
    all_obstacle_areas.push_back(area);
  }

  m_debug_obstacles_publisher->publish(
    autoware::common::had_map_utils::areasAsTriangleMarkerArray(
      marker_t, "parking_debug_obstacles",
      all_obstacle_areas,
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
  const Polygon3d drivable_area = parking_planner::coalesce_drivable_areas(route, lanelet_map_ptr);

  // ---- Obtain "list of bounding obstacles" of drivable surface -----------------------
  const auto obstacles = parking_planner::convert_drivable_area_to_obstacles(drivable_area);

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
  auto trajectory = parking_planner::convert_parking_planner_to_autoware_trajectory(
    planner_result.get_trajectory());

  m_debug_trajectory_publisher->publish(trajectory);

  return trajectory;
}

}  // namespace parking_planner_nodes
}  // namespace planning
}  // namespace motion
}  // namespace autoware
RCLCPP_COMPONENTS_REGISTER_NODE( \
  autoware::motion::planning::parking_planner_nodes::ParkingPlannerNode)
