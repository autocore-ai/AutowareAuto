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

#include <lane_planner_nodes/lane_planner_node.hpp>
#include <had_map_utils/had_map_conversion.hpp>

#include <memory>

#include "gtest/gtest.h"

using autoware::common::types::float32_t;
using autoware::common::types::float64_t;

using motion::motion_common::Real;
using motion::motion_common::VehicleConfig;

using autoware_auto_msgs::srv::HADMapService;
using autoware_auto_msgs::action::PlanTrajectory;
using autoware_auto_msgs::msg::HADMapRoute;
using autoware_auto_msgs::msg::HADMapSegment;
using autoware_auto_msgs::msg::MapPrimitive;

// returns a map with a lane has given number of points(n_points)
// length of the lane will be n_points meters in y direction
lanelet::LaneletMapPtr get_lanelet_map(
  const lanelet::Id & id, const float64_t velocity,
  const size_t n_points)
{
  lanelet::Points3d right_points, left_points, center_points;
  constexpr float64_t resolution = 1.0;
  for (size_t i = 0; i < n_points; i++) {
    const auto y = resolution * static_cast<float64_t>(i);
    left_points.push_back(lanelet::Point3d(lanelet::utils::getId(), -1, y, 0));
    right_points.push_back(lanelet::Point3d(lanelet::utils::getId(), 1, y, 0));
    center_points.push_back(lanelet::Point3d(lanelet::utils::getId(), 0, y, 0));
  }
  lanelet::LineString3d ls1(lanelet::utils::getId(), left_points);
  lanelet::LineString3d ls2(lanelet::utils::getId(), right_points);
  lanelet::LineString3d ls3(lanelet::utils::getId(), center_points);

  lanelet::Lanelet ll(id, ls1, ls2);
  ll.setCenterline(ls3);
  ll.setAttribute(lanelet::AttributeName::SpeedLimit, velocity);

  return lanelet::utils::createMap({ll});
}

HADMapRoute get_route(const int64_t lane_id, const float32_t length)
{
  HADMapRoute had_map_route;
  had_map_route.start_point.position.x = 0;
  had_map_route.start_point.position.y = 0;
  had_map_route.goal_point.position.x = 0;
  had_map_route.goal_point.position.y = length;

  MapPrimitive primitive;
  primitive.id = lane_id;
  HADMapSegment segment;
  segment.preferred_primitive_id = primitive.id;
  had_map_route.segments.push_back(segment);
  had_map_route.segments.front().primitives.push_back(primitive);

  return had_map_route;
}

class LanePlannerNodeTest : public ::testing::Test
{
public:
  void SetUp() override
  {
    ASSERT_FALSE(rclcpp::ok());
    rclcpp::init(0, nullptr);
    ASSERT_TRUE(rclcpp::ok());

    using std::placeholders::_1;
    using std::placeholders::_2;
    using std::placeholders::_3;

    m_fake_node = std::make_shared<rclcpp::Node>("fake_node");
    m_fake_map_service =
      m_fake_node->create_service<HADMapService>(
      "HAD_Map_Service",
      std::bind(&LanePlannerNodeTest::send_fake_map, this, _1, _2, _3));
    m_trajectory_client = rclcpp_action::create_client<PlanTrajectory>(
      m_fake_node,
      "plan_lane_trajectory");

    rclcpp::NodeOptions node_options{};
    node_options.append_parameter_override("vehicle.cg_to_front_m", 1.0F);
    node_options.append_parameter_override("vehicle.cg_to_rear_m", 1.0F);
    node_options.append_parameter_override("vehicle.front_corner_stiffness", 0.1F);
    node_options.append_parameter_override("vehicle.rear_corner_stiffness", 0.1F);
    node_options.append_parameter_override("vehicle.mass_kg", 1500.0F);
    node_options.append_parameter_override("vehicle.yaw_inertia_kgm2", 12.0F);
    node_options.append_parameter_override("vehicle.width_m", 2.0F);
    node_options.append_parameter_override("vehicle.front_overhang_m", 0.5F);
    node_options.append_parameter_override("vehicle.rear_overhang_m", 0.5F);

    node_options.append_parameter_override("gaussian_smoother.standard_deviation", 1.0F);
    node_options.append_parameter_override("gaussian_smoother.kernel_size", 5);

    node_options.append_parameter_override("lane_planner.trajectory_resolution", 1.0F);

    m_planner_ptr = std::make_shared<autoware::lane_planner_nodes::LanePlannerNode>(node_options);
    m_lane_id = lanelet::utils::getId();
  }

  void send_fake_map(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<HADMapService::Request> request,
    const std::shared_ptr<HADMapService::Response> response)
  {
    (void)request_header;
    (void)request;
    autoware_auto_msgs::msg::HADMapBin map_msg;
    const auto map_ptr = get_lanelet_map(m_lane_id, 1, 10);
    autoware::common::had_map_utils::toBinaryMsg(map_ptr, map_msg);
    response->map = map_msg;
  }

  std::shared_ptr<autoware::lane_planner_nodes::LanePlannerNode> m_planner_ptr;
  rclcpp::Node::SharedPtr m_fake_node{nullptr};
  rclcpp::Service<HADMapService>::SharedPtr m_fake_map_service;
  rclcpp_action::Client<PlanTrajectory>::SharedPtr m_trajectory_client;
  lanelet::Id m_lane_id;
};

TEST_F(LanePlannerNodeTest, PlanSimpleTrajectory)
{
  using namespace std::chrono_literals;

  const auto is_action_ready = m_trajectory_client->wait_for_action_server(5s);
  ASSERT_TRUE(is_action_ready);


  auto goal = PlanTrajectory::Goal();
  goal.sub_route = get_route(m_lane_id, 5);

  auto goal_handle_future = m_trajectory_client->async_send_goal(goal);

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(m_fake_node);
  executor.add_node(m_planner_ptr);

  const auto goal_return_code = executor.spin_until_future_complete(goal_handle_future, 10s);
  ASSERT_EQ(goal_return_code, rclcpp::executor::FutureReturnCode::SUCCESS);
  auto goal_handle = goal_handle_future.get();

  // check if goal was accepted
  ASSERT_NE(goal_handle, nullptr);

  auto result_future = m_trajectory_client->async_get_result(goal_handle);
  const auto result_return_code = executor.spin_until_future_complete(result_future, 10s);

  ASSERT_EQ(result_return_code, rclcpp::executor::FutureReturnCode::SUCCESS);

  auto result = result_future.get();

  // check if trajectory was returned
  // the value of trajectory itself should be tested in lane_planner package
  ASSERT_FALSE(result.result->trajectory.points.empty());
}
