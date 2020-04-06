// Copyright 2017-2020 Apex.AI, Inc.
// Co-developed by Tier IV, Inc. and Apex.AI, Inc.
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

#include <helper_functions/helper_functions.hpp>
#include <point_cloud_filter_transform_nodes/point_cloud_filter_transform_node.hpp>

#include <memory>
#include <string>

int32_t main(const int32_t argc, char * argv[])
{
  rclcpp::init(argc, argv);

  int32_t ret = 0;
  try {
    using autoware::perception::filters::point_cloud_filter_transform_nodes::
    PointCloud2FilterTransformNode;
    const auto nd_ptr = std::make_shared<PointCloud2FilterTransformNode>(
      "point_cloud_filter_transform_node");

    rclcpp::spin(nd_ptr);

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
