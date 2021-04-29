// Copyright 2020 The Autoware Foundation
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

#include "behavior_planner_nodes/behavior_planner_node.hpp"
#include <had_map_utils/had_map_conversion.hpp>
#include <motion_common/config.hpp>
#include <geometry/common_2d.hpp>

#include <rclcpp_components/register_node_macro.hpp>

#include <string>
#include <memory>

namespace autoware
{
namespace behavior_planner_nodes
{

BehaviorPlannerNode::BehaviorPlannerNode(const rclcpp::NodeOptions & options)
:  Node("behavior_planner_node", options)
{
  init();
}

void BehaviorPlannerNode::init()
{
  using rclcpp::QoS;
  using namespace std::chrono_literals;

  // Setup planner
  const auto cg_to_front_m =
    static_cast<float32_t>(declare_parameter("vehicle.cg_to_front_m").get<float64_t>());
  const auto cg_to_rear_m =
    static_cast<float32_t>(declare_parameter("vehicle.cg_to_rear_m").get<float64_t>());
  const auto front_overhang_m =
    static_cast<float32_t>(declare_parameter("vehicle.front_overhang_m").get<float64_t>());
  const auto rear_overhang_m =
    static_cast<float32_t>(declare_parameter("vehicle.rear_overhang_m").get<float64_t>());
  const auto cg_to_vehicle_center =
    ( (cg_to_front_m + front_overhang_m) - (rear_overhang_m + cg_to_rear_m) ) * 0.5F;

  const behavior_planner::PlannerConfig config{
    static_cast<float32_t>(declare_parameter("goal_distance_thresh").get<float64_t>()),
    static_cast<float32_t>(declare_parameter("stop_velocity_thresh").get<float64_t>()),
    static_cast<float32_t>(declare_parameter("heading_weight").get<float64_t>()),
    static_cast<float32_t>(declare_parameter("subroute_goal_offset_lane2parking").get<float64_t>()),
    static_cast<float32_t>(declare_parameter("subroute_goal_offset_parking2lane").get<float64_t>()),
    cg_to_vehicle_center
  };

  m_planner = std::make_unique<behavior_planner::BehaviorPlanner>(config);

  // Setup Tf Buffer with listener
  rclcpp::Clock::SharedPtr clock = std::make_shared<rclcpp::Clock>(RCL_ROS_TIME);
  m_tf_buffer = std::make_shared<tf2_ros::Buffer>(clock);
  m_tf_listener = std::make_shared<tf2_ros::TransformListener>(
    *m_tf_buffer,
    std::shared_ptr<rclcpp::Node>(this, [](auto) {}), false);

  m_lane_planner_client = rclcpp_action::create_client<PlanTrajectoryAction>(
    this->get_node_base_interface(),
    this->get_node_graph_interface(),
    this->get_node_logging_interface(),
    this->get_node_waitables_interface(),
    "plan_lane_trajectory");
  m_parking_planner_client = rclcpp_action::create_client<PlanTrajectoryAction>(
    this->get_node_base_interface(),
    this->get_node_graph_interface(),
    this->get_node_logging_interface(),
    this->get_node_waitables_interface(),
    "plan_parking_trajectory");

  // wait until action clients are ready
  while (!m_lane_planner_client->wait_for_action_server(1s)) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(get_logger(), "Interrupted while waiting for action server.");
      rclcpp::shutdown();
      return;
    }
    RCLCPP_INFO(get_logger(), "Waiting for action server...");
  }
  while (!m_parking_planner_client->wait_for_action_server(1s)) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(get_logger(), "Interrupted while waiting for action server.");
      rclcpp::shutdown();
      return;
    }
    RCLCPP_INFO(get_logger(), "Waiting for action server...");
  }

  m_map_client = this->create_client<HADMapService>("HAD_Map_Service");
  while (!m_map_client->wait_for_service(1s)) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(get_logger(), "Interrupted while waiting for service.");
      rclcpp::shutdown();
      return;
    }
    RCLCPP_INFO(get_logger(), "Waiting for service...");
  }

  if (declare_parameter("enable_object_collision_estimator").get<bool>()) {
    m_modify_trajectory_client = this->create_client<ModifyTrajectory>("estimate_collision");
    while (!m_modify_trajectory_client->wait_for_service(1s)) {
      if (!rclcpp::ok()) {
        RCLCPP_ERROR(get_logger(), "Interrupted while waiting for service.");
        rclcpp::shutdown();
        return;
      }
      RCLCPP_INFO(get_logger(), "Waiting for service...");
    }
  }

  // Setup subscribers
  m_ego_state_sub = this->create_subscription<State>(
    "vehicle_state", QoS{10},
    [this](const State::SharedPtr msg) {on_ego_state(msg);});
  m_route_sub = this->create_subscription<HADMapRoute>(
    "route", QoS{10},
    [this](const HADMapRoute::SharedPtr msg) {on_route(msg);});
  m_vehicle_state_report_sub = this->create_subscription<VehicleStateReport>(
    "vehicle_state_report", QoS{10},
    [this](const VehicleStateReport::SharedPtr msg) {on_vehicle_state_report(msg);});

  // Setup publishers
  m_trajectory_pub =
    this->create_publisher<Trajectory>("trajectory", QoS{10});
  m_debug_trajectory_pub =
    this->create_publisher<Trajectory>("debug/full_trajectory", QoS{10});
  m_debug_checkpoints_pub =
    this->create_publisher<Trajectory>("debug/checkpoints", QoS{10});
  m_debug_subroute_pub =
    this->create_publisher<HADMapRoute>("debug/current_subroute", QoS{10});
  m_vehicle_state_command_pub =
    this->create_publisher<VehicleStateCommand>("vehicle_state_command", QoS{10});
}

void BehaviorPlannerNode::goal_response_callback(
  std::shared_future<PlanTrajectoryGoalHandle::SharedPtr> future)
{
  if (!future.get()) {
    RCLCPP_ERROR(get_logger(), "Goal was rejected by server");
    return;
  } else {
    // start requesting trajectory
    m_requesting_trajectory = true;
  }
}

void BehaviorPlannerNode::feedback_callback(
  PlanTrajectoryGoalHandle::SharedPtr goal_handle,
  const std::shared_ptr<const PlanTrajectoryAction::Feedback> feedback)
{
  // currently we don't need feedback.
  // This function will be deleted when action is replaced with synchronous service call
  (void) goal_handle;
  (void) feedback;
}

void BehaviorPlannerNode::result_callback(const PlanTrajectoryGoalHandle::WrappedResult & result)
{
  if (result.result->result == PlanTrajectoryAction::Result::SUCCESS &&
    !result.result->trajectory.points.empty())
  {
    RCLCPP_INFO(get_logger(), "Received trajectory from planner");
  } else {
    RCLCPP_ERROR(get_logger(), "Planner failed to calculate!!");
  }

  auto trajectory = result.result->trajectory;
  trajectory.header.frame_id = "map";
  m_debug_trajectory_pub->publish(trajectory);

  m_planner->set_trajectory(result.result->trajectory);

  // finished requesting trajectory
  m_requesting_trajectory = false;
}

State BehaviorPlannerNode::transform_to_map(const State & state)
{
  geometry_msgs::msg::TransformStamped tf;
  try {
    tf = m_tf_buffer->lookupTransform(
      "map", state.header.frame_id,
      time_utils::from_message(state.header.stamp));
  } catch (const tf2::ExtrapolationException &) {
    // TODO(mitsudome-r): currently falls back to retrive newest
    // transform available for availability,
    // Do validation in the future
    tf = m_tf_buffer->lookupTransform("map", state.header.frame_id, tf2::TimePointZero);
  }
  State transformed_state;
  motion::motion_common::doTransform(state, transformed_state, tf);
  transformed_state.header.frame_id = "map";
  transformed_state.header.stamp = state.header.stamp;
  return transformed_state;
}

void BehaviorPlannerNode::request_trajectory(const RouteWithType & route_with_type)
{
  using std::placeholders::_1;
  using std::placeholders::_2;

  const auto & route = route_with_type.route;
  const auto & planner_type = route_with_type.planner_type;

  auto action_goal = PlanTrajectoryAction::Goal();
  action_goal.sub_route = route;

  auto send_goal_options = rclcpp_action::Client<PlanTrajectoryAction>::SendGoalOptions();
  send_goal_options.goal_response_callback = std::bind(
    &BehaviorPlannerNode::goal_response_callback,
    this, _1);
  send_goal_options.feedback_callback = std::bind(
    &BehaviorPlannerNode::feedback_callback, this, _1,
    _2);
  send_goal_options.result_callback = std::bind(&BehaviorPlannerNode::result_callback, this, _1);

  switch (planner_type) {
    case behavior_planner::PlannerType::LANE:
      m_lane_planner_client->async_send_goal(action_goal, send_goal_options);
      RCLCPP_INFO(get_logger(), "Sent lane trajectory action goal");
      break;
    case behavior_planner::PlannerType::PARKING:
      m_parking_planner_client->async_send_goal(action_goal, send_goal_options);
      RCLCPP_INFO(get_logger(), "Sent parking trajectory action goal");
      break;
    default:
      break;
  }
  m_debug_subroute_pub->publish(route);
}

void BehaviorPlannerNode::on_ego_state(const State::SharedPtr & msg)
{
  // Do nothing if localization result is not received yet.
  if (!m_tf_buffer->canTransform("map", msg->header.frame_id, tf2::TimePointZero)) {
    RCLCPP_INFO(get_logger(), "Waiting for localization result to become available");
    return;
  }

  m_ego_state = transform_to_map(*msg);

  // do nothing if we haven't got route yet
  if (!m_planner->is_route_ready()) {
    return;
  }

  // check if we need new trajectory
  // make sure we are not requesting trajectory if we already have
  static auto previous_output_arrived_goal = std::chrono::system_clock::now();
  if (!m_requesting_trajectory) {
    if (m_planner->has_arrived_goal(m_ego_state)) {
      // TODO(mitsudome-r): replace this with throttled output in foxy
      const auto now = std::chrono::system_clock::now();
      const auto throttle_time = std::chrono::duration<float64_t>(3);
      if (now - previous_output_arrived_goal > throttle_time) {
        RCLCPP_INFO(get_logger(), "Trying to change gear");
        previous_output_arrived_goal = now;
      }
      RCLCPP_INFO_ONCE(get_logger(), "Reached goal. Wait for another route");
    } else if (m_planner->has_arrived_subroute_goal(m_ego_state)) {
      // send next subroute
      m_planner->set_next_subroute();
      request_trajectory(m_planner->get_current_subroute(m_ego_state));
      m_requesting_trajectory = true;
    } else if (m_planner->needs_new_trajectory(m_ego_state)) {
      // update trajectory for current subroute
      request_trajectory(m_planner->get_current_subroute(m_ego_state));
      m_requesting_trajectory = true;
    }
  }

  if (!m_planner->is_trajectory_ready()) {
    return;
  }

  static auto previous_output = std::chrono::system_clock::now();
  const auto desired_gear = m_planner->get_desired_gear(m_ego_state);
  if (desired_gear != m_current_gear) {
    const auto throttle_time = std::chrono::duration<float64_t>(3);

    // TODO(mitsudome-r): replace this with throttled output in foxy
    const auto now = std::chrono::system_clock::now();
    if (now - previous_output > throttle_time) {
      RCLCPP_INFO(get_logger(), "Trying to change gear");
      previous_output = now;
    }


    VehicleStateCommand gear_command;
    gear_command.gear = desired_gear;
    gear_command.mode = VehicleStateCommand::MODE_AUTONOMOUS;
    gear_command.stamp = msg->header.stamp;
    m_vehicle_state_command_pub->publish(gear_command);

    // send trajectory with current state so that velocity will be zero in order to change gear
    Trajectory trajectory;
    trajectory.header.frame_id = "map";
    trajectory.header.stamp = msg->header.stamp;
    trajectory.points.push_back(msg->state);
    m_trajectory_pub->publish(trajectory);
  } else {
    auto trajectory = m_planner->get_trajectory(m_ego_state);
    // trajectory.header = m_ego_state.header;
    trajectory.header.frame_id = "map";
    trajectory.header.stamp = msg->header.stamp;

    // If object collision estimation is enabled, send trajectory through the collision estimator
    if (m_modify_trajectory_client) {
      auto request = std::make_shared<ModifyTrajectory::Request>();
      request->original_trajectory = trajectory;
      auto result =
        m_modify_trajectory_client->async_send_request(
        request,
        std::bind(
          &BehaviorPlannerNode::modify_trajectory_response,
          this, std::placeholders::_1));
    } else {
      m_trajectory_pub->publish(trajectory);
    }
  }
}

void BehaviorPlannerNode::on_vehicle_state_report(const VehicleStateReport::SharedPtr & msg)
{
  m_current_gear = msg->gear;
}

void BehaviorPlannerNode::on_route(const HADMapRoute::SharedPtr & msg)
{
  if (m_requesting_trajectory) {
    RCLCPP_ERROR(
      get_logger(),
      "Route was rejected. Route cannot be updated while communicating with trajectory planners.");
    return;
  }

  if (!m_planner->is_vehicle_stopped(m_ego_state)) {
    RCLCPP_ERROR(
      get_logger(), "Route was rejected. Route cannot be update while the vehicle is moving");
    return;
  }

  RCLCPP_INFO(get_logger(), "Received route");

  m_route = msg;

  // TODO(mitsudome-r): replace it with bounded request
  auto request = std::make_shared<HADMapService::Request>();
  request->requested_primitives.push_back(HADMapService::Request::FULL_MAP);

  // TODO(mitsudome-r): If synchronized service request becomes available,
  // replace it with synchronized implementation
  auto result =
    m_map_client->async_send_request(
    request,
    std::bind(&BehaviorPlannerNode::map_response, this, std::placeholders::_1));
}

void BehaviorPlannerNode::modify_trajectory_response(
  rclcpp::Client<ModifyTrajectory>::SharedFuture future)
{
  auto trajectory = future.get()->modified_trajectory;

  // set current position with velocity zero to do emergency stop in case
  // collision estimator fails or if there is obstacle on first point
  if (trajectory.points.empty()) {
    auto stopping_point = m_ego_state.state;
    stopping_point.longitudinal_velocity_mps = 0.0;
    trajectory.points.push_back(stopping_point);
  }
  m_trajectory_pub->publish(trajectory);
}

void BehaviorPlannerNode::map_response(rclcpp::Client<HADMapService>::SharedFuture future)
{
  m_lanelet_map_ptr = std::make_shared<lanelet::LaneletMap>();
  autoware::common::had_map_utils::fromBinaryMsg(future.get()->map, m_lanelet_map_ptr);

  RCLCPP_INFO(get_logger(), "Received map");

  // TODO(mitsudome-r) move to handle_accepted() when synchronous service is available
  m_planner->set_route(*m_route, m_lanelet_map_ptr);

  const auto subroutes = m_planner->get_subroutes();
  Trajectory checkpoints;
  checkpoints.header.frame_id = "map";
  for (const auto subroute : subroutes) {
    TrajectoryPoint trajectory_start_point;
    trajectory_start_point.x = static_cast<float32_t>(subroute.route.start_point.position.x);
    trajectory_start_point.y = static_cast<float32_t>(subroute.route.start_point.position.y);
    trajectory_start_point.heading = subroute.route.start_point.heading;

    TrajectoryPoint trajectory_goal_point;
    trajectory_goal_point.x = static_cast<float32_t>(subroute.route.goal_point.position.x);
    trajectory_goal_point.y = static_cast<float32_t>(subroute.route.goal_point.position.y);
    trajectory_goal_point.heading = subroute.route.goal_point.heading;

    checkpoints.points.push_back(trajectory_start_point);
    checkpoints.points.push_back(trajectory_goal_point);
  }
  m_debug_checkpoints_pub->publish(checkpoints);
}
}  // namespace behavior_planner_nodes
}  // namespace autoware

RCLCPP_COMPONENTS_REGISTER_NODE(autoware::behavior_planner_nodes::BehaviorPlannerNode)
