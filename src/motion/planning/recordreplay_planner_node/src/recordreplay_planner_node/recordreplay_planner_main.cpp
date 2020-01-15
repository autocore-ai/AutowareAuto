// Copyright 2020 Sandro Merkli, inspired by Christopher Ho's mpc code
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

#include "recordreplay_planner_node/recordreplay_planner_node.hpp"
#include <rclcpp/rclcpp.hpp>

#include <memory>

int32_t main(int32_t argc, char ** argv)
{
  rclcpp::init(argc, argv);

  using motion::planning::recordreplay_planner_node::RecordReplayPlannerNode;
  const auto nd = std::make_shared<RecordReplayPlannerNode>("recordreplay_planner", "");

  rclcpp::spin(nd);

  if (!rclcpp::shutdown()) {
    throw std::runtime_error{"rclcpp shutdown failed"};
  }
  return 0;
}
