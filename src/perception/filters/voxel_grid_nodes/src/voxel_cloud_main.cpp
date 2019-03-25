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

#include <voxel_grid_nodes/voxel_cloud_node.hpp>
#include <rcutils/cmdline_parser.h>
#include <lifecycle_msgs/msg/state.hpp>
#include <lifecycle_msgs/msg/transition.hpp>
#include <rclcpp/rclcpp.hpp>
#include <boost/filesystem.hpp>
#include <iostream>
#include <memory>
#include <string>


int32_t main(const int32_t argc, char * argv[])
{
  rclcpp::init(argc, argv);
  boost::filesystem::path default_config_path = boost::filesystem::absolute(argv[0]).parent_path() /
    "param" / "vlp16_lexus_centroid.param.yaml";

  int32_t ret = 0;
  try {
    const char * config_file = default_config_path.string().c_str();
    const char * arg = rcutils_cli_get_option(argv, &argv[argc], "--config_file");
    if (nullptr != arg) {
      config_file = arg;
    }
    const char * node_name = "voxel_grid_cloud_node";
    arg = rcutils_cli_get_option(argv, &argv[argc], "--node_name");
    if (nullptr != arg) {
      node_name = arg;
    }
    const char * node_namespace = "";
    arg = rcutils_cli_get_option(argv, &argv[argc], "--node_namespace");
    if (nullptr != arg) {
      node_namespace = arg;
    }


    using autoware::perception::filters::voxel_grid_nodes::VoxelCloudNode;
    const auto nd_ptr = std::make_shared<VoxelCloudNode>(
      node_name,
      node_namespace,
      config_file);


    if (lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE != nd_ptr->configure().id()) {
      throw std::runtime_error("Could not configure VoxelCloudNode!");
    }
    if (lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE != nd_ptr->activate().id()) {
      throw std::runtime_error("Could not activate VoxelCloudNode!");
    }

    rclcpp::spin(nd_ptr->get_node_base_interface());

    rclcpp::shutdown();
  } catch (const std::exception & e) {
    // RCLCPP logging macros are not used in error handling because they would depend on nd_ptr's
    // logger. This dependency would result in a crash when nd_ptr is a nullptr
    std::cerr << (e.what());
    ret = 2;
  } catch (...) {
    std::cerr << "Unknown exception caught. Exiting...";
    ret = -1;
  }
  return ret;
}
