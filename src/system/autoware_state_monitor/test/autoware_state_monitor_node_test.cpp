// Copyright 2021 The Autoware Foundation
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
//
// Co-developed by Tier IV, Inc. and Robotec.AI sp. z o.o.

#include "autoware_state_monitor/autoware_state_monitor_node.hpp"

#include <memory>

#include "gtest/gtest.h"

#include "tf2_ros/transform_broadcaster.h"
#include "geometry_msgs/msg/transform_stamped.hpp"

#include "autoware_state_monitor/state.hpp"

#include "test_utils.hpp"

using autoware::state_monitor::State;
using autoware::state_monitor::AutowareStateMonitorNode;
using namespace std::chrono_literals;

using autoware_auto_msgs::msg::VehicleStateReport;
using autoware_auto_msgs::msg::VehicleOdometry;
using autoware_auto_msgs::msg::HADMapRoute;
using autoware_auto_msgs::msg::Engage;
using autoware_auto_msgs::msg::AutowareState;


class AutowareStateMonitorNodeNoParametersTest : public ::testing::Test
{
public:
  void SetUp() override
  {
    ASSERT_FALSE(rclcpp::ok());
    rclcpp::init(0, nullptr);
    ASSERT_TRUE(rclcpp::ok());
  }

  void TearDown()
  {
    rclcpp::shutdown();
  }

protected:
  std::shared_ptr<AutowareStateMonitorNode> tested_node;
  std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> executor;
};

TEST_F(AutowareStateMonitorNodeNoParametersTest, no_parameters)
{
  rclcpp::NodeOptions node_options;
  tested_node = std::make_shared<AutowareStateMonitorNode>(node_options);
  executor.reset(new rclcpp::executors::SingleThreadedExecutor);
  executor->add_node(tested_node);
  executor->spin_some(100ms);
}

class AutowareStateMonitorNodeTest : public ::testing::Test
{
public:
  void SetUp() override
  {
    ASSERT_FALSE(rclcpp::ok());
    rclcpp::init(0, nullptr);
    ASSERT_TRUE(rclcpp::ok());

    // Created tested and helper nodes
    auto node_options = createNodeOptions();
    tested_node = std::make_shared<AutowareStateMonitorNode>(node_options);
    test_helper_node = rclcpp::Node::make_shared("test_helper_node");

    // Add nodes to executor
    executor.reset(new rclcpp::executors::SingleThreadedExecutor);
    executor->add_node(tested_node);
    executor->add_node(test_helper_node);

    // Spy
    autoware_state_spy = std::make_shared<Spy<AutowareState>>(
      test_helper_node, executor, "output/autoware_state");

    // Publishers
    pub_engage = test_helper_node->create_publisher<Engage>("input/engage", 1);
    pub_vehicle_state = test_helper_node->create_publisher<VehicleStateReport>(
      "input/vehicle_state_report", 1);
    pub_route = test_helper_node->create_publisher<HADMapRoute>("input/route", 1);
    pub_odometry = test_helper_node->create_publisher<VehicleOdometry>("input/odometry", 1);

    // TF
    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(
      tested_node->shared_from_this());
  }

  void TearDown()
  {
    rclcpp::shutdown();
  }

  rclcpp::NodeOptions createNodeOptions()
  {
    rclcpp::NodeOptions node_options;
    node_options.append_parameter_override("update_rate", 10.0f);
    node_options.append_parameter_override("arrived_distance_threshold", 1.0f);
    node_options.append_parameter_override("stopped_time_threshold", 1.0f);
    node_options.append_parameter_override("stopped_velocity_threshold_mps", 0.1f);
    node_options.append_parameter_override("wait_time_after_initializing", 0.0f);
    node_options.append_parameter_override("wait_time_after_planning", 0.0f);
    node_options.append_parameter_override("wait_time_after_arrived_goal", 0.0f);
    return node_options;
  }

  void expectState(const State & state)
  {
    auto msg = autoware_state_spy->expectMsg();
    EXPECT_EQ(msg.state, state);
  }

  void publishMapToBaseLinkTf()
  {
    geometry_msgs::msg::TransformStamped transform_stamped;
    transform_stamped.header.stamp = tested_node->get_clock()->now();
    transform_stamped.header.frame_id = "map";
    transform_stamped.child_frame_id = "base_link";

    transform_stamped.transform.translation.x = 1.0;
    transform_stamped.transform.translation.y = 1.0;
    transform_stamped.transform.translation.z = 0.0;

    transform_stamped.transform.rotation.x = 0.0;
    transform_stamped.transform.rotation.y = 0.0;
    transform_stamped.transform.rotation.z = 0.0;
    transform_stamped.transform.rotation.w = 1.0;

    tf_broadcaster_->sendTransform(transform_stamped);
  }

protected:
  std::shared_ptr<AutowareStateMonitorNode> tested_node;
  rclcpp::Node::SharedPtr test_helper_node;
  std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> executor;

  rclcpp::Publisher<Engage>::SharedPtr pub_engage;
  rclcpp::Publisher<VehicleStateReport>::SharedPtr pub_vehicle_state;
  rclcpp::Publisher<HADMapRoute>::SharedPtr pub_route;
  rclcpp::Publisher<VehicleOdometry>::SharedPtr pub_odometry;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  std::shared_ptr<Spy<AutowareState>> autoware_state_spy;
};

TEST_F(AutowareStateMonitorNodeTest, basic_states_sequence)
{
  // sequence: Initializing -> WaitingForRoute -> Planning
  //           -> WaitingForEngage -> Driving
  expectState(AutowareState::INITIALIZING);
  expectState(AutowareState::WAITING_FOR_ROUTE);

  HADMapRoute route;
  route.goal_point.position.x = 1.0;
  route.goal_point.position.y = 1.0;
  pub_route->publish(route);
  expectState(AutowareState::PLANNING);
  expectState(AutowareState::PLANNING);

  VehicleStateReport report;
  report.mode = 1;  // Autonomous
  pub_vehicle_state->publish(report);
  expectState(AutowareState::WAITING_FOR_ENGAGE);

  Engage engage;
  engage.engage = true;
  pub_engage->publish(engage);
  expectState(AutowareState::DRIVING);

  // Publish current tf map->base_link as (1.0, 1.0) --> vehicle close to goal
  publishMapToBaseLinkTf();

  // Velocity above the threshold --> still driving
  VehicleOdometry odometry;
  odometry.stamp = toTime(0.0);
  odometry.velocity_mps = 1.0;
  pub_odometry->publish(odometry);
  expectState(AutowareState::DRIVING);

  // Velocity below the threshold, odometry buffer have 1s length
  odometry.stamp = toTime(2.0);
  odometry.velocity_mps = 0.0;
  pub_odometry->publish(odometry);
  expectState(AutowareState::ARRIVED_GOAL);

  expectState(AutowareState::WAITING_FOR_ROUTE);
}

TEST_F(AutowareStateMonitorNodeTest, shutdown_service)
{
  // Create a client to call the shutdown service
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr shutdown_client =
    test_helper_node->create_client<std_srvs::srv::Trigger>("service/shutdown");

  // Wait till the service interface is ready
  while (!shutdown_client->wait_for_service(1s)) {
    EXPECT_EQ(rclcpp::ok(), true);
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
  }

  executor->spin_some(100ms);

  // Create a request message and sent it to the service interface
  auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
  auto result_future = shutdown_client->async_send_request(request);

  // Wait for the result to be returned
  while (result_future.wait_for(100ms) == std::future_status::timeout) {
    executor->spin_some(10ms);
  }
  auto result = result_future.get();
  EXPECT_EQ(result->success, false);
  EXPECT_EQ(result->message, "Shutdown timeout.");
}
