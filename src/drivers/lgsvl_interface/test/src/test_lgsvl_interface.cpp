// Copyright 2020 The Autoware Foundation
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

#include "test_lgsvl_interface.hpp"

TEST_F(LgsvlInterface_test, gear_mapping_state_command)
{
  bool test_completed = false;
  VSC expected_result;

  // Setup subscribtion
  auto handle_state_cmd = [&expected_result, &test_completed]
      (const VSC::SharedPtr msg) -> void {

      EXPECT_EQ(msg->gear, expected_result.gear);
      test_completed = true;
    };
  const auto sub_node = std::make_shared<rclcpp::Node>("test_lgsvl_interface_sub_state_command",
      "/gtest");
  auto sub_ptr = sub_node->create_subscription<lgsvl_interface::VSC>(sim_state_cmd_topic, rclcpp::QoS(
        10), handle_state_cmd);

  // Setup Node execution
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node_);
  executor.add_node(sub_node);
  wait_for_publisher(sub_ptr);


  auto wait_for_subscription_callback =
    [&test_completed, &expected_result, &executor, this](lgsvl_interface::GEAR_TYPE gear,
      lgsvl_interface::GEAR_TYPE expected_gear) -> void {
      auto max_test_dur = std::chrono::seconds(1);
      auto timed_out = false;
      test_completed = false;
      VSC vsc_msg;
      vsc_msg.gear = gear;
      expected_result.gear = expected_gear;
      auto start_time = std::chrono::system_clock::now();

      while (rclcpp::ok() && !test_completed) {
        auto state_command_status = lgsvl_interface_->send_state_command(vsc_msg);
        EXPECT_TRUE(state_command_status);
        executor.spin_some();
        rclcpp::sleep_for(std::chrono::milliseconds(50));
        if (std::chrono::system_clock::now() - start_time > max_test_dur) {
          timed_out = true;
          break;
        }
      }
      EXPECT_FALSE(timed_out);
      EXPECT_TRUE(test_completed);

      //cleaup
      rclcpp::sleep_for(std::chrono::milliseconds(100));
      executor.spin_some();
    };

  // Tests
  wait_for_subscription_callback(VSC::GEAR_DRIVE,
    static_cast<lgsvl_interface::GEAR_TYPE>(lgsvl_interface::LGSVL_GEAR::DRIVE));

  wait_for_subscription_callback(VSC::GEAR_REVERSE,
    static_cast<lgsvl_interface::GEAR_TYPE>(lgsvl_interface::LGSVL_GEAR::REVERSE));

  wait_for_subscription_callback(static_cast<lgsvl_interface::GEAR_TYPE>(99u),
    static_cast<lgsvl_interface::GEAR_TYPE>(lgsvl_interface::LGSVL_GEAR::DRIVE));
}

TEST_F(LgsvlInterface_test, gear_mapping_state_report)
{
  VSR vsr_msg;

  // Setup Node execution
  const auto pub_node = std::make_shared<rclcpp::Node>("test_lgsvl_interface_pub_state_report",
      "/gtest");
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node_);

  // Setup Publisher
  const auto pub_ptr = pub_node->create_publisher<VSR>(sim_state_rpt_topic, rclcpp::QoS{10});
  wait_for_subscriber(pub_ptr);

  auto publish_gear_and_wait =
    [&vsr_msg, &pub_ptr, &executor](lgsvl_interface::GEAR_TYPE gear) -> void {
      vsr_msg.gear = gear;
      pub_ptr->publish(vsr_msg);
      rclcpp::sleep_for(std::chrono::milliseconds(100));
      executor.spin_some();
    };

  // Tests
  EXPECT_EQ(lgsvl_interface_->get_state_report().gear, 0);

  publish_gear_and_wait(static_cast<lgsvl_interface::GEAR_TYPE>(lgsvl_interface::LGSVL_GEAR::DRIVE));
  EXPECT_EQ(lgsvl_interface_->get_state_report().gear, VSC::GEAR_DRIVE);

  publish_gear_and_wait(
    static_cast<lgsvl_interface::GEAR_TYPE>(lgsvl_interface::LGSVL_GEAR::REVERSE));
  EXPECT_EQ(lgsvl_interface_->get_state_report().gear, VSC::GEAR_REVERSE);

  publish_gear_and_wait(static_cast<lgsvl_interface::GEAR_TYPE>(99u));
  EXPECT_EQ(lgsvl_interface_->get_state_report().gear, VSC::GEAR_NEUTRAL);
}
