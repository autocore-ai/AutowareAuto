// Copyright 2018-2020 the Autoware Foundation, Arm Limited
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
//
// Co-developed by Tier IV, Inc. and Apex.AI, Inc.

#include <xsens_nodes/xsens_imu_node.hpp>


//lint -e537 NOLINT  // cpplint vs pclint
#include <string>
//lint -e537 NOLINT  // cpplint vs pclint
#include <memory>
#include <vector>
#include <cstdio>
#include "rclcpp/rclcpp.hpp"

// this file is simply a main file to create a ros1 style standalone node
int32_t main(const int32_t argc, char ** const argv)
{
  int32_t ret = 0;

  try {
    rclcpp::init(argc, argv);

    auto vptr = std::make_shared<autoware::drivers::xsens_nodes::XsensImuNode>(
      "xsens_imu_driver_node");

    vptr->run();
  } catch (const std::exception & err) {
    // RCLCPP logging macros are not used in error handling because they would depend on vptr's
    // logger. This dependency would result in a crash when vptr is a nullptr
    std::cerr << err.what() << std::endl;
    ret = 2;
  } catch (...) {
    std::cerr << "Unknown error encountered, exiting..." << std::endl;
    ret = -1;
  }
  rclcpp::shutdown();
  return ret;
}
