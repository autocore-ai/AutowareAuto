// Copyright 2017-2018 Apex.AI, Inc.
// Co-developed by Tier IV, Inc. and Apex.AI, Inc.
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
#include <rcutils/cmdline_parser.h>
#include <rclcpp/rclcpp.hpp>
#include <common/types.hpp>

#include <memory>
#include <string>

#include "ray_ground_classifier_nodes/ray_ground_classifier_cloud_node.hpp"

using autoware::common::types::char8_t;

constexpr const char8_t * NODE_NAME = "ray_ground_classifier";

int32_t main(const int32_t argc, char * argv[])
{
  int32_t ret = 0;
  try {
    rclcpp::init(argc, argv);

    using
    autoware::perception::filters::ray_ground_classifier_nodes::RayGroundClassifierCloudNode;
    const auto nd_ptr = std::make_shared<RayGroundClassifierCloudNode>(NODE_NAME);
    if (lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE != nd_ptr->configure().id()) {
      throw std::runtime_error("Could not configure RayGroundClassifierCloudNode!");
    }
    if (lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE != nd_ptr->activate().id()) {
      throw std::runtime_error("Could not activate RayGroundClassifierCloudNode!");
    }
    rclcpp::executors::SingleThreadedExecutor exe;

    exe.add_node(nd_ptr->get_node_base_interface());

    exe.spin();

    rclcpp::shutdown();
  } catch (const std::exception & e) {
    // RCLCPP logging macros are not used in error handling because they would
    // depend on nd_ptr's logger. This dependency would result in a crash when
    // nd_ptr is a nullptr
    std::cerr << NODE_NAME << ": " << (e.what()) << std::endl;
    ret = 2;
    ret = 2;
  } catch (...) {
    std::cerr << NODE_NAME << ": Unknown exception caught. Exiting..." << std::endl;
    ret = -1;
  }
  return ret;
}
