// Copyright 2019 Apex.AI, Inc.
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

#include <iostream>
//lint -e537 NOLINT repeated include file... cpplint disagrees with pclint
#include <memory>
#include "euclidean_cluster_nodes/euclidean_cluster_node.hpp"

int32_t main(const int32_t argc, char ** const argv)
{
  int32_t ret = 0;
  try {
    rclcpp::init(argc, argv);
    using autoware::perception::segmentation::euclidean_cluster_nodes::EuclideanClusterNode;
    const auto nd_ptr = std::make_shared<EuclideanClusterNode>("euclidean_cluster_cloud_node");

    rclcpp::spin(nd_ptr);

    if (!rclcpp::shutdown()) {
      throw std::runtime_error{"Could not shut down rclcpp"};
    }
  } catch (const std::exception & e) {
    std::cerr << e.what() << "\n";
    ret = 2;
  } catch (...) {
    std::cerr << "Unexpected exception caught! Exiting\n";
    ret = 255;
  }

  return ret;
}
