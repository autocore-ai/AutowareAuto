// Copyright 2020 Embotech AG, Zurich, Switzerland
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

#include <gtest/gtest.h>
#include <parking_planner_nodes/parking_planner_node.hpp>
#include <autoware_auto_msgs/msg/trajectory.hpp>
#include <autoware_auto_msgs/action/plan_trajectory.hpp>
#include <motion_testing/motion_testing.hpp>
#include <rclcpp/rclcpp.hpp>
#include <motion_common/config.hpp>
#include <had_map_utils/had_map_conversion.hpp>

#include <chrono>
#include <algorithm>
#include <string>
#include <tuple>
#include <vector>
#include <memory>

using autoware_auto_msgs::srv::HADMapService;
using autoware::motion::planning::parking_planner_nodes::ParkingPlannerNode;
using motion::motion_testing::make_state;
using std::chrono::system_clock;
using autoware_auto_msgs::msg::Trajectory;

using autoware_auto_msgs::action::PlanTrajectory;
using State = autoware_auto_msgs::msg::VehicleKinematicState;
using GoalHandlePlanTrajectory = rclcpp_action::ServerGoalHandle<PlanTrajectory>;

using autoware::common::types::float32_t;
using autoware::common::types::float64_t;
using motion::motion_common::VehicleConfig;
using ParkerNLPCostWeights = autoware::motion::planning::parking_planner::NLPCostWeights<float64_t>;
using ParkerVehicleState = autoware::motion::planning::parking_planner::VehicleState<float64_t>;
using ParkerVehicleCommand = autoware::motion::planning::parking_planner::VehicleCommand<float64_t>;

static const rclcpp::NodeOptions get_test_options()
{
  rclcpp::NodeOptions options;

  // Type specified here because auto eduction fails
  const std::vector<std::tuple<std::string, float32_t>> overrides = {
    {"vehicle.cg_to_front_m", 1.0},
    {"vehicle.cg_to_rear_m", 1.0},
    {"vehicle.front_corner_stiffness", 0.5},
    {"vehicle.rear_corner_stiffness", 0.5},
    {"vehicle.mass_kg", 1500.0},
    {"vehicle.yaw_inertia_kgm2", 12.0},
    {"vehicle.width_m", 2.0},
    {"vehicle.front_overhang_m", 0.5},
    {"vehicle.rear_overhang_m", 0.5},
    {"optimization_weights.steering", 1.0},
    {"optimization_weights.throttle", 1.0},
    {"optimization_weights.goal", 0.0},
    {"state_bounds.lower.x_m", -100.0},
    {"state_bounds.lower.y_m", -100.0},
    {"state_bounds.lower.velocity_mps", -12.0},
    {"state_bounds.lower.heading_rad", -2.0 * 3.14156},
    {"state_bounds.lower.steering_rad", -0.52},
    {"state_bounds.upper.x_m", +100.0},
    {"state_bounds.upper.y_m", +100.0},
    {"state_bounds.upper.velocity_mps", +12.0},
    {"state_bounds.upper.heading_rad", +2.0 * 3.14156},
    {"state_bounds.upper.steering_rad", +0.52},
    {"command_bounds.lower.steering_rate_rps", -10.0},
    {"command_bounds.lower.throttle_mps2", -15.0},
    {"command_bounds.upper.steering_rate_rps", +10.0},
    {"command_bounds.upper.throttle_mps2", +15.0},
  };

  std::for_each(
    overrides.begin(), overrides.end(), [&options](auto x) {
      options.append_parameter_override(std::get<0>(x), std::get<1>(x));
    });

  return options;
}

// This doesn't do anything, just here so we can give the fake service something to call
static void send_fake_map(
  const std::shared_ptr<rmw_request_id_t> request_header,
  const std::shared_ptr<HADMapService::Request> request,
  const std::shared_ptr<HADMapService::Response> response)
{
  (void)request_header;
  (void)request;
  (void)response;
}


TEST(ParkerNodeSanityChecks, Basic)
{
  rclcpp::init(0, nullptr);

  // Create a fake map provider node
  auto fake_node = std::make_shared<rclcpp::Node>("fake_node");
  auto fake_map_service =
    fake_node->create_service<HADMapService>("HAD_Map_Service", &send_fake_map);

  // Create a planner node
  const auto options = get_test_options();
  auto plannernode = std::make_shared<ParkingPlannerNode>(options);

  rclcpp::executors::SingleThreadedExecutor exec;
  exec.add_node(plannernode);
  EXPECT_NO_THROW(exec.spin_some(std::chrono::milliseconds(100LL)));

  // TODO(s.me) test some actual things here once the integration with
  // the map provider is there, or maybe do that in integration tests.

  rclcpp::shutdown();
}
