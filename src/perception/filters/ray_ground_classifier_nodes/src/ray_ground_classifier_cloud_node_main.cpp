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

#include <memory>
#include <string>

#include "ray_ground_classifier_nodes/ray_ground_classifier_cloud_node.hpp"

#define DEFAULT_CONFIG \
  CMAKE_INSTALL_PREFIX "/" "share/ray_ground_classifier_nodes/vlp16_lexus.param.yaml"


int32_t main(const int32_t argc, char * argv[])
{
  int32_t ret = 0;
  try {
    const char * config_file = DEFAULT_CONFIG;
    const char * arg = rcutils_cli_get_option(argv, &argv[argc], "--config_file");
    if (nullptr != arg) {
      config_file = arg;
    }
    const char * node_name = "ray_ground_classifier";
    arg = rcutils_cli_get_option(argv, &argv[argc], "--node_name");
    if (nullptr != arg) {
      node_name = arg;
    }
    const char * node_namespace = "";
    arg = rcutils_cli_get_option(argv, &argv[argc], "--node_namespace");
    if (nullptr != arg) {
      node_namespace = arg;
    }
    // TODO(christopher.ho) #1270apex_app
    using
    autoware::perception::filters::ray_ground_classifier_nodes::RayGroundClassifierCloudNode;
    const auto nd_ptr = std::make_shared<RayGroundClassifierCloudNode>(
      node_name,
      node_namespace,
      config_file);
    if (lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE != nd_ptr->configure().id()) {
      throw std::runtime_error("Could not configure RayGroundClassifierCloudNode!");
    }
    if (lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE != nd_ptr->activate().id()) {
      throw std::runtime_error("Could not activate RayGroundClassifierCloudNode!");
    }
    rclcpp::executors::SingleThreadedExecutor exe;

    exe.add_node(nd_ptr->get_node_base_interface());

    exe.spin();

    if (!rclcpp::shutdown()) {
      throw std::runtime_error("rclcpp shutdown failed!");
    }
  } catch (const std::exception & e) {
    RCLCPP_INFO(rclcpp::get_logger("ray_ground_classifier_node"), e.what());
    ret = 2;
  } catch (...) {
    RCLCPP_INFO(
      rclcpp::get_logger("ray_ground_classifier_node"), "Unknown exception caught. Exiting...");
    ret = -1;
  }
  return ret;
}
