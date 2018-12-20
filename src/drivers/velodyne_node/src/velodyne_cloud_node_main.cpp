// Copyright 2018 Apex.AI, Inc.
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

#include <velodyne_node/velodyne_cloud_node.hpp>


//lint -e537 NOLINT  // cpplint vs pclint
#include <string>
//lint -e537 NOLINT  // cpplint vs pclint
#include <memory>
#include <vector>
#include <cstdio>
#include "boost/program_options.hpp"
#include "boost/filesystem.hpp"
#include "rclcpp/rclcpp.hpp"

// this file is simply a main file to create a ros1 style standalone node
int32_t main(const int32_t argc, char ** const argv)
{
  int32_t ret = 0;
  // get resource path relative to the executable
  boost::filesystem::path default_config_path = boost::filesystem::absolute(argv[0]).parent_path() /
    "param" / "vlp16_test.param.yaml";

  std::shared_ptr<autoware::drivers::velodyne_node::VelodyneCloudNode> vptr;

  try {
    rclcpp::init(argc, argv);
    boost::program_options::options_description desc("Allowed options");
    desc.add_options()("help", "produce help message")("config_file",
      boost::program_options::value<std::string>()->
      default_value(default_config_path.string()),
      "Parameter file used for the two nodes")("node_name",
      boost::program_options::value<std::string>()->
      default_value("vlp16_test_node"),
      "Name of velodyne cloud node")("node_namespace",
      boost::program_options::value<std::string>()->
      default_value(""),
      "Namespace of velodyne cloud driver")
    ;
    boost::program_options::variables_map vm;
    boost::program_options::store(
      boost::program_options::parse_command_line(argc, argv, desc), vm);
    boost::program_options::notify(vm);

    vptr = std::make_shared<autoware::drivers::velodyne_node::VelodyneCloudNode>(
      vm["node_name"].as<std::string>().c_str(),
      vm["node_namespace"].as<std::string>().c_str(),
      vm["config_file"].as<std::string>().c_str());

    vptr->run();

    rclcpp::shutdown();
  } catch (const std::exception & err) {
    // RCLCPP logging macros are not used in error handling because they would depend on vptr's
    // logger. This dependency would result in a crash when vptr is a nullptr
    std::cerr << err.what() << std::endl;
    ret = 2;
  } catch (...) {
    std::cerr << "Unknown error encountered, exiting..." << std::endl;
    ret = -1;
  }
  return ret;
}
