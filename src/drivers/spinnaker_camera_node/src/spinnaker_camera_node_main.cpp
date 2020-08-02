// Copyright 2020 Apex.AI, Inc.
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

#include <rclcpp/rclcpp.hpp>
#include <spinnaker_camera_node/spinnaker_camera_node.hpp>

#include <memory>

using autoware::drivers::camera::SpinnakerCameraNode;

int main(int argc, char * argv[])
{
  // force flush of the stdout buffer.
  // this ensures a correct sync of all prints
  // even when executed simultaneously within the launch file.
  setvbuf(stdout, NULL, _IONBF, BUFSIZ);
  int32_t ret = 0;

  try {
    rclcpp::init(argc, argv);
    rclcpp::executors::SingleThreadedExecutor exe;

    rclcpp::NodeOptions node_options;
    auto node = std::make_shared<SpinnakerCameraNode>(
      node_options);

    exe.add_node(node->get_node_base_interface());
    exe.spin();

    rclcpp::shutdown();
  } catch (const std::exception & err) {
    // RCLCPP logging macros are not used in error handling because they would depend on node's
    // logger. This dependency would result in a crash when node is a nullptr
    std::cerr << "Got exception: " << err.what() << std::endl;
    ret = 2;
  } catch (...) {
    std::cerr << "Unknown error encountered, exiting..." << std::endl;
    ret = -1;
  }

  return ret;
}
