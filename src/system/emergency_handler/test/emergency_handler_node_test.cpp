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

#include "emergency_handler/emergency_handler_node.hpp"

#include <memory>
#include <string>

#include "gtest/gtest.h"
#include "rclcpp/rclcpp.hpp"

#include "test_utils.hpp"

using namespace std::chrono_literals;

using autoware_auto_msgs::msg::VehicleStateReport;
using autoware_auto_msgs::msg::VehicleOdometry;
using autoware_auto_msgs::msg::DrivingCapability;
using autoware_auto_msgs::msg::AutowareState;
using autoware_auto_msgs::msg::VehicleControlCommand;
using autoware_auto_msgs::msg::VehicleStateCommand;
using autoware_auto_msgs::msg::EmergencyState;
using autoware_auto_msgs::msg::HazardStatusStamped;
using autoware_auto_msgs::msg::HazardStatus;
using autoware::emergency_handler::EmergencyHandlerNode;

using diagnostic_msgs::msg::DiagnosticArray;
using diagnostic_msgs::msg::DiagnosticStatus;

class EmergencyHandlerNodeTest : public ::testing::Test
{
public:
  void SetUp() override
  {
    ASSERT_FALSE(rclcpp::ok());
    rclcpp::init(0, nullptr);
    ASSERT_TRUE(rclcpp::ok());

    // Created tested and helper nodes
    auto node_options = createNodeOptions();
    tested_node = std::make_shared<EmergencyHandlerNode>(node_options);
    test_node = rclcpp::Node::make_shared("test_node");

    // Add nodes to executor
    executor.reset(new rclcpp::executors::SingleThreadedExecutor);
    executor->add_node(tested_node);
    executor->add_node(test_node);

    // Publishers
    pub_autoware_state = test_node->create_publisher<AutowareState>("input/autoware_state", 1);
    pub_driving_capability = test_node->create_publisher<DrivingCapability>(
      "input/driving_capability", 1);
    pub_vehicle_control_command = test_node->create_publisher<VehicleControlCommand>(
      "input/prev_control_command", 1);
    pub_vehicle_state_report = test_node->create_publisher<VehicleStateReport>(
      "input/state_report", 1);
    pub_odometry = test_node->create_publisher<VehicleOdometry>("input/odometry", 1);

    // Register spies
    control_command_spy = std::make_shared<Spy<VehicleControlCommand>>(
      test_node, executor, "output/control_command");
    state_command_spy = std::make_shared<Spy<VehicleStateCommand>>(
      test_node, executor, "output/state_command");
    emergency_state_spy = std::make_shared<Spy<EmergencyState>>(
      test_node, executor, "output/is_emergency");
    hazard_status_spy = std::make_shared<Spy<HazardStatusStamped>>(
      test_node, executor, "output/hazard_status");
    diagnostic_spy = std::make_shared<Spy<DiagnosticArray>>(
      test_node, executor, "output/diagnostics_err");
  }

  void TearDown()
  {
    rclcpp::shutdown();
  }

protected:
  rclcpp::NodeOptions createNodeOptions()
  {
    rclcpp::NodeOptions node_options;
    node_options.append_parameter_override("update_rate", 10.0f);
    node_options.append_parameter_override("timeout_driving_capability", 0.5f);
    node_options.append_parameter_override("data_ready_timeout", 0.1f);
    node_options.append_parameter_override("use_emergency_hold", false);
    node_options.append_parameter_override("emergency_hazard_level", 2);
    return node_options;
  }

  void publishAutowareState(const uint8_t state)
  {
    AutowareState msg;
    msg.state = state;
    pub_autoware_state->publish(msg);
  }

  void publishVehicleStateReport(const uint8_t mode)
  {
    VehicleStateReport msg;
    msg.mode = mode;
    pub_vehicle_state_report->publish(msg);
  }

  diagnostic_msgs::msg::DiagnosticStatus createDiagnosticStatus(
    const int level, const std::string & hw_id,
    const std::string & name = "", const std::string & message = "")
  {
    diagnostic_msgs::msg::DiagnosticStatus diag;

    diag.level = static_cast<unsigned char>(level);
    diag.hardware_id = hw_id;
    diag.name = name;
    diag.message = message;

    return diag;
  }

  void expectNoEmergencyState()
  {
    {
      auto msg = hazard_status_spy->expectMsg();
      EXPECT_EQ(msg.status.level, HazardStatus::NO_FAULT);
      EXPECT_EQ(msg.status.diag_safe_fault.size(), 0);
      EXPECT_EQ(msg.status.diag_latent_fault.size(), 0);
      EXPECT_EQ(msg.status.diag_single_point_fault.size(), 0);
    }
    {
      auto msg = emergency_state_spy->expectMsg();
      EXPECT_EQ(msg.state, EmergencyState::NORMAL);
    }
    {
      auto msg = diagnostic_spy->expectMsg();
      ASSERT_EQ(msg.status.size(), 0);
    }

    // Emergency vehicle control commands and state commands shouldn't be published
    double timeout = 0.1;
    EXPECT_ANY_THROW(control_command_spy->expectMsg(timeout));
    EXPECT_ANY_THROW(state_command_spy->expectMsg(timeout));
  }

  std::shared_ptr<EmergencyHandlerNode> tested_node;
  rclcpp::Node::SharedPtr test_node;
  std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> executor;

  rclcpp::Publisher<AutowareState>::SharedPtr pub_autoware_state;
  rclcpp::Publisher<DrivingCapability>::SharedPtr pub_driving_capability;
  rclcpp::Publisher<VehicleControlCommand>::SharedPtr pub_vehicle_control_command;
  rclcpp::Publisher<VehicleStateReport>::SharedPtr pub_vehicle_state_report;
  rclcpp::Publisher<VehicleOdometry>::SharedPtr pub_odometry;

  std::shared_ptr<Spy<VehicleControlCommand>> control_command_spy;
  std::shared_ptr<Spy<VehicleStateCommand>> state_command_spy;
  std::shared_ptr<Spy<EmergencyState>> emergency_state_spy;
  std::shared_ptr<Spy<HazardStatusStamped>> hazard_status_spy;
  std::shared_ptr<Spy<DiagnosticArray>> diagnostic_spy;
};

TEST_F(EmergencyHandlerNodeTest, create_destroy)
{
}

TEST_F(EmergencyHandlerNodeTest, manual_mode_not_ignored_autoware_state_no_emergency_state)
{
  publishAutowareState(AutowareState::DRIVING);  // not ignored state
  publishVehicleStateReport(VehicleStateReport::MODE_MANUAL);

  DrivingCapability driving_capability;
  driving_capability.remote_control.level = HazardStatus::NO_FAULT;
  driving_capability.remote_control.emergency = false;

  pub_driving_capability->publish(driving_capability);

  expectNoEmergencyState();
}

TEST_F(EmergencyHandlerNodeTest, autonomous_mode_not_ignored_autoware_state_no_emergency_state)
{
  publishAutowareState(AutowareState::DRIVING);  // not ignored state
  publishVehicleStateReport(VehicleStateReport::MODE_AUTONOMOUS);

  DrivingCapability driving_capability;
  driving_capability.autonomous_driving.level = HazardStatus::NO_FAULT;
  driving_capability.autonomous_driving.emergency = false;
  pub_driving_capability->publish(driving_capability);

  expectNoEmergencyState();
}

TEST_F(EmergencyHandlerNodeTest, manual_mode_ignored_autoware_state_no_emergency_state)
{
  publishAutowareState(AutowareState::INITIALIZING);  // ignored state
  publishVehicleStateReport(VehicleStateReport::MODE_MANUAL);

  DrivingCapability driving_capability;
  driving_capability.remote_control.level = HazardStatus::SINGLE_POINT_FAULT;  // fault
  driving_capability.remote_control.emergency = true;
  pub_driving_capability->publish(driving_capability);

  expectNoEmergencyState();
}

TEST_F(EmergencyHandlerNodeTest, autonomous_mode_ignored_autoware_state_no_emergency_state)
{
  publishAutowareState(AutowareState::INITIALIZING);  // ignored state
  publishVehicleStateReport(VehicleStateReport::MODE_AUTONOMOUS);

  DrivingCapability driving_capability;
  driving_capability.autonomous_driving.level = HazardStatus::SINGLE_POINT_FAULT;  // fault
  driving_capability.autonomous_driving.emergency = true;
  pub_driving_capability->publish(driving_capability);

  expectNoEmergencyState();
}

TEST_F(EmergencyHandlerNodeTest, autonomous_mode_emergency_state_in_driving_capability)
{
  publishAutowareState(AutowareState::DRIVING);  // not ignored state
  publishVehicleStateReport(VehicleStateReport::MODE_AUTONOMOUS);

  DrivingCapability driving_capability;
  driving_capability.remote_control.level = HazardStatus::NO_FAULT;
  driving_capability.autonomous_driving.level = HazardStatus::SINGLE_POINT_FAULT;  // fault
  driving_capability.autonomous_driving.emergency = true;
  driving_capability.autonomous_driving.diag_single_point_fault.push_back(
    createDiagnosticStatus(DiagnosticStatus::ERROR, "test_hw", "test_name"));
  pub_driving_capability->publish(driving_capability);

  pub_odometry->publish(*prepareVehicleOdometryMsg(0.0));
  pub_vehicle_control_command->publish(VehicleControlCommand{});

  {
    auto msg = hazard_status_spy->expectMsg();
    EXPECT_EQ(msg.status.level, HazardStatus::SINGLE_POINT_FAULT);
    ASSERT_EQ(msg.status.diag_single_point_fault.size(), 1);
  }
  {
    auto msg = emergency_state_spy->expectMsg();
    EXPECT_EQ(msg.state, EmergencyState::MRM_OPERATING);
  }
  {
    auto msg = diagnostic_spy->expectMsg();
    ASSERT_EQ(msg.status.size(), 1);
    EXPECT_EQ(msg.status[0].hardware_id, "test_hw");
    EXPECT_EQ(msg.status[0].name, "test_name");
    EXPECT_EQ(msg.status[0].level, DiagnosticStatus::ERROR);
  }
  { // Check if control command and state command have been generated
    auto msg = control_command_spy->expectMsg();
    EXPECT_EQ(msg.velocity_mps, 0.0);
    EXPECT_EQ(msg.long_accel_mps2, -2.5);
    EXPECT_EQ(msg.front_wheel_angle_rad, 0);
  }
  {
    auto msg = state_command_spy->expectMsg();
    EXPECT_EQ(msg.blinker, VehicleStateCommand::BLINKER_HAZARD);
    EXPECT_EQ(msg.headlight, VehicleStateCommand::HEADLIGHT_NO_COMMAND);
    EXPECT_EQ(msg.wiper, VehicleStateCommand::WIPER_NO_COMMAND);
    EXPECT_EQ(msg.gear, VehicleStateCommand::GEAR_PARK);
  }
}

TEST_F(EmergencyHandlerNodeTest, heartbeat_checker_timeout)
{
  publishAutowareState(AutowareState::DRIVING);
  publishVehicleStateReport(VehicleStateReport::MODE_MANUAL);
  DrivingCapability driving_capability;
  driving_capability.remote_control.level = HazardStatus::NO_FAULT;
  pub_driving_capability->publish(driving_capability);

  { // Wait for hazard status with heardbeat timeout
    const auto t_start = test_node->get_clock()->now();
    bool received_heartbeat_timeout = false;
    const double timeout = 1.0;

    while (1) {
      if (!rclcpp::ok()) {
        break;
      }
      if ((test_node->get_clock()->now() - t_start).seconds() > timeout) {
        break;
      }

      auto msg = hazard_status_spy->expectMsg();
      if (msg.status.level == HazardStatus::SINGLE_POINT_FAULT &&
        msg.status.diag_single_point_fault.size() == 1 &&
        msg.status.diag_single_point_fault[0].hardware_id == "emergency_handler" &&
        msg.status.diag_single_point_fault[0].name == "heartbeat_timeout" &&
        msg.status.diag_single_point_fault[0].level == DiagnosticStatus::ERROR)
      {
        received_heartbeat_timeout = true;
        break;
      }
    }
    EXPECT_TRUE(received_heartbeat_timeout);
  }
}

TEST_F(EmergencyHandlerNodeTest, clear_emergency_service)
{
  publishAutowareState(AutowareState::DRIVING);
  publishVehicleStateReport(VehicleStateReport::MODE_MANUAL);
  DrivingCapability driving_capability;
  driving_capability.remote_control.level = HazardStatus::NO_FAULT;
  pub_driving_capability->publish(driving_capability);

  // Create a client to call the shutdown service
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr client =
    test_node->create_client<std_srvs::srv::Trigger>("service/clear_emergency");

  // Wait till the service interface is ready
  while (!client->wait_for_service(1s)) {
    EXPECT_EQ(rclcpp::ok(), true);
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
  }

  executor->spin_some(100ms);

  // Create a request message and sent it to the service interface
  auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
  auto result_future = client->async_send_request(request);

  // Wait for the result to be returned
  while (result_future.wait_for(100ms) == std::future_status::timeout) {
    executor->spin_some(10ms);
  }

  auto result = result_future.get();
  EXPECT_EQ(result->success, true);
  EXPECT_EQ(result->message, "Emergency state has been cleared.");
}

TEST_F(EmergencyHandlerNodeTest, clear_emergency_service_no_input_data)
{
  // Create a client to call the shutdown service
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr client =
    test_node->create_client<std_srvs::srv::Trigger>("service/clear_emergency");

  // Wait till the service interface is ready
  while (!client->wait_for_service(1s)) {
    EXPECT_EQ(rclcpp::ok(), true);
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
  }

  executor->spin_some(100ms);

  // Create a request message and sent it to the service interface
  auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
  auto result_future = client->async_send_request(request);

  EXPECT_ANY_THROW(executor->spin_some(100ms));
}

TEST_F(EmergencyHandlerNodeTest, input_data_timeout_no_input_data)
{
  // Node expect data after initialization: DrivingCapability, AutowareState, VehicleStateReport
  {
    auto msg = hazard_status_spy->expectMsg();
    EXPECT_EQ(msg.status.level, autoware_auto_msgs::msg::HazardStatus::SINGLE_POINT_FAULT);
    ASSERT_EQ(msg.status.diag_single_point_fault.size(), 1);
    EXPECT_EQ(msg.status.diag_single_point_fault[0].hardware_id, "emergency_handler");
    EXPECT_EQ(msg.status.diag_single_point_fault[0].name, "input_data_timeout");
    EXPECT_EQ(msg.status.diag_single_point_fault[0].level, DiagnosticStatus::ERROR);
  }
  {
    auto msg = emergency_state_spy->expectMsg();
    EXPECT_EQ(msg.state, EmergencyState::MRM_OPERATING);
  }
  {
    auto msg = diagnostic_spy->expectMsg();
    ASSERT_EQ(msg.status.size(), 1);
    EXPECT_EQ(msg.status[0].hardware_id, "emergency_handler");
    EXPECT_EQ(msg.status[0].name, "input_data_timeout");
    EXPECT_EQ(msg.status[0].level, DiagnosticStatus::ERROR);
  }
}

TEST_F(EmergencyHandlerNodeTest, input_data_timeout_because_of_missing_autoware_state)
{
  pub_driving_capability->publish(DrivingCapability{});
  pub_vehicle_state_report->publish(VehicleStateReport{});

  {
    auto msg = hazard_status_spy->expectMsg();
    EXPECT_EQ(msg.status.level, autoware_auto_msgs::msg::HazardStatus::SINGLE_POINT_FAULT);
    ASSERT_EQ(msg.status.diag_single_point_fault.size(), 1);
    EXPECT_EQ(msg.status.diag_single_point_fault[0].hardware_id, "emergency_handler");
    EXPECT_EQ(msg.status.diag_single_point_fault[0].name, "input_data_timeout");
    EXPECT_EQ(msg.status.diag_single_point_fault[0].level, DiagnosticStatus::ERROR);
  }
  {
    auto msg = emergency_state_spy->expectMsg();
    EXPECT_EQ(msg.state, EmergencyState::MRM_OPERATING);
  }
  {
    auto msg = diagnostic_spy->expectMsg();
    ASSERT_EQ(msg.status.size(), 1);
    EXPECT_EQ(msg.status[0].hardware_id, "emergency_handler");
    EXPECT_EQ(msg.status[0].name, "input_data_timeout");
    EXPECT_EQ(msg.status[0].level, DiagnosticStatus::ERROR);
  }
}

TEST_F(EmergencyHandlerNodeTest, input_data_timeout_because_of_missing_driving_capability)
{
  pub_autoware_state->publish(AutowareState{});
  pub_vehicle_state_report->publish(VehicleStateReport{});

  {
    auto msg = hazard_status_spy->expectMsg();
    EXPECT_EQ(msg.status.level, autoware_auto_msgs::msg::HazardStatus::SINGLE_POINT_FAULT);
    ASSERT_EQ(msg.status.diag_single_point_fault.size(), 1);
    EXPECT_EQ(msg.status.diag_single_point_fault[0].hardware_id, "emergency_handler");
    EXPECT_EQ(msg.status.diag_single_point_fault[0].name, "input_data_timeout");
    EXPECT_EQ(msg.status.diag_single_point_fault[0].level, DiagnosticStatus::ERROR);
  }
  {
    auto msg = emergency_state_spy->expectMsg();
    EXPECT_EQ(msg.state, EmergencyState::MRM_OPERATING);
  }
  {
    auto msg = diagnostic_spy->expectMsg();
    ASSERT_EQ(msg.status.size(), 1);
    EXPECT_EQ(msg.status[0].hardware_id, "emergency_handler");
    EXPECT_EQ(msg.status[0].name, "input_data_timeout");
    EXPECT_EQ(msg.status[0].level, diagnostic_msgs::msg::DiagnosticStatus::ERROR);
  }
}

TEST_F(EmergencyHandlerNodeTest, input_data_timeout_because_of_missing_vehicle_state_report)
{
  pub_autoware_state->publish(AutowareState{});
  pub_driving_capability->publish(DrivingCapability{});

  {
    auto msg = hazard_status_spy->expectMsg();
    EXPECT_EQ(msg.status.level, autoware_auto_msgs::msg::HazardStatus::SINGLE_POINT_FAULT);
    ASSERT_EQ(msg.status.diag_single_point_fault.size(), 1);
    EXPECT_EQ(msg.status.diag_single_point_fault[0].hardware_id, "emergency_handler");
    EXPECT_EQ(msg.status.diag_single_point_fault[0].name, "input_data_timeout");
    EXPECT_EQ(msg.status.diag_single_point_fault[0].level, DiagnosticStatus::ERROR);
  }
  {
    auto msg = emergency_state_spy->expectMsg();
    EXPECT_EQ(msg.state, EmergencyState::MRM_OPERATING);
  }
  {
    auto msg = diagnostic_spy->expectMsg();
    ASSERT_EQ(msg.status.size(), 1);
    EXPECT_EQ(msg.status[0].hardware_id, "emergency_handler");
    EXPECT_EQ(msg.status[0].name, "input_data_timeout");
    EXPECT_EQ(msg.status[0].level, diagnostic_msgs::msg::DiagnosticStatus::ERROR);
  }
}
