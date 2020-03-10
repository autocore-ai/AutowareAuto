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

#include <ndt_nodes/map_publisher.hpp>
#include <rclcpp/rclcpp.hpp>

int32_t main(const int32_t argc, char * argv[])
{
  rclcpp::init(argc, argv);

  int32_t ret = 0;
  try {
    autoware::localization::ndt_nodes::NDTMapPublisherNode map_publisher(
      "ndt_map_publisher_node", "");
    // Map publishes ones and stops execution.
    map_publisher.run();

    // Run until terminated by user to ensure the published map gets received at the subscribers.
    while (rclcpp::ok()) {
      rclcpp::sleep_for(std::chrono::milliseconds(100LL));
    }
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
