// Copyright 2021 The Autoware Foundation
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

#include "hello_world/hello_world_node.hpp"

namespace autoware
{
namespace hello_world
{

HelloWorldNode::HelloWorldNode(const rclcpp::NodeOptions & options)
:  Node("hello_world", options),
  verbose(true)
{
  print_hello();
}

int32_t HelloWorldNode::print_hello() const
{
  return hello_world::print_hello();
}

}  // namespace hello_world
}  // namespace autoware

#include "rclcpp_components/register_node_macro.hpp"

// This acts as an entry point, allowing the component to be
// discoverable when its library is being loaded into a running process
RCLCPP_COMPONENTS_REGISTER_NODE(autoware::hello_world::HelloWorldNode)
