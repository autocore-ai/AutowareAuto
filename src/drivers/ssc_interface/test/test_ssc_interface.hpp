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

#ifndef TEST_SSC_INTERFACE_HPP_
#define TEST_SSC_INTERFACE_HPP_

#include <gtest/gtest.h>
#include <ssc_interface/visibility_control.hpp>
#include <ssc_interface/ssc_interface.hpp>
#include <memory>

using autoware_auto_msgs::msg::VehicleStateCommand;
using autoware_auto_msgs::msg::VehicleStateReport;

class SscInterface_test : public ::testing::Test
{
protected:
  void SetUp() override
  {
    rclcpp::init(0, nullptr);
    node_ = std::make_shared<rclcpp::Node>("ssc_interface_test_node", "/gtest");
    ssc_interface_ = std::make_unique<ssc_interface::SscInterface>(
      *node_, 1.0F, 1.0F, 3.0F, -3.0F, 1.5708F)
  }

  void TearDown() override
  {
    (void)rclcpp::shutdown();
  }

public:
  rclcpp::Node::SharedPtr node_;
  std::unique_ptr<ssc_interface::SscInterface> ssc_interface_;
};

template<typename T>
void wait_for_subscriber(
  const T & pub_ptr,
  const uint32_t num_expected_subs = 1U,
  std::chrono::milliseconds match_timeout = std::chrono::seconds{10U})
{
  const auto match_start = std::chrono::steady_clock::now();
  // Ensure map publisher has a map that is listening.
  while (pub_ptr->get_subscription_count() < num_expected_subs) {
    rclcpp::sleep_for(std::chrono::milliseconds(100));
    if (std::chrono::steady_clock::now() - match_start > match_timeout) {
      throw std::runtime_error("timed out waiting for subscriber");
    }
  }
}


template<typename T>
void wait_for_publisher(
  const T & sub_ptr,
  const uint32_t num_expected_pubs = 1U,
  std::chrono::milliseconds match_timeout = std::chrono::seconds{10U})
{
  const auto match_start = std::chrono::steady_clock::now();
  // Ensure map publisher has a map that is listening.
  while (sub_ptr->get_publisher_count() < num_expected_pubs) {
    rclcpp::sleep_for(std::chrono::milliseconds(100));
    if (std::chrono::steady_clock::now() - match_start > match_timeout) {
      throw std::runtime_error("timed out waiting for publisher");
    }
  }
}
#endif  // TEST_SSC_INTERFACE_HPP_
