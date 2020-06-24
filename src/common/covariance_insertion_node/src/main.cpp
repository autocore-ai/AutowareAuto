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

/// \copyright Copyright 2020 Apex.AI, Inc.
/// All rights reserved.

#include "covariance_insertion_node/covariance_insertion_node.hpp"

int32_t main(const int32_t argc, char ** const argv)
{
  int32_t ret;
  try {
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions options;
    autoware::covariance_insertion_node::CovarianceInsertionNode node{options};
    ret = 0;
  } catch (const std::exception & e) {
    std::cerr << e.what() << "\n";
    ret = 1;
  } catch (...) {
    std::cerr << "Unknown error occured" << "\n";
    ret = 255;
  }
  rclcpp::shutdown();

  return ret;
}
