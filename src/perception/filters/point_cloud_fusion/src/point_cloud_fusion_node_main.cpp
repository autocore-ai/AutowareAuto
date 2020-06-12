// Copyright 2019 Apex.AI, Inc.
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

#include <point_cloud_fusion/point_cloud_fusion.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rcutils/cmdline_parser.h>
#include <common/types.hpp>
#include <string>
#include <memory>
#include <vector>
#include <cstdio>

using autoware::common::types::char8_t;


// this file is simply a main file to create a ros1 style standalone node
int32_t main(const int32_t argc, char ** const argv)
{
  int32_t ret = 0;
  try {
    const char8_t * node_name = "point_cloud_fusion_node";
    const char8_t * arg = rcutils_cli_get_option(argv, &argv[argc], "--node_name");
    if (nullptr != arg) {
      node_name = arg;
    }
    const char8_t * node_namespace = "";
    arg = rcutils_cli_get_option(argv, &argv[argc], "--node_namespace");
    if (nullptr != arg) {
      node_namespace = arg;
    }

    rclcpp::init(argc, argv);

    auto pc_ptr = std::make_shared<autoware::perception::filters::point_cloud_fusion
        ::PointCloudFusionNode>(node_name, node_namespace);

    rclcpp::spin(pc_ptr);
    rclcpp::shutdown();
  } catch (const rclcpp::ParameterTypeException & err) {
    // RCLCPP logging macros are not used in error handling because they would depend on vptr's
    // logger. This dependency would result in a crash when vptr is a nullptr
    std::cerr << "Parameter type error: \"" << err.what() << "\". Make sure to use a correct "
      "parameter file and use the argument `__params:=path/to/config.yaml` ." << std::endl;
    ret = 2;
  } catch (const std::exception & e) {
    std::cerr << e.what() << std::endl;
    ret = 2;
  } catch (...) {
    std::cerr << "Unknown error encountered, exiting..." << std::endl;
    ret = -1;
  }
  return ret;
}
