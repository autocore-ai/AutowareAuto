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

#include "hello_world/hello_world_node.hpp"

//lint -e537 NOLINT  // cpplint vs pclint
#include <string>

namespace autoware
{
namespace hello_world
{

hello_world_c::hello_world_c(const std::string & node_name)
:  Node(node_name.c_str()),
  verbose(true)
{
}

hello_world_c::~hello_world_c()
{
}

int32_t hello_world_c::print_hello() const
{
  return hello_world::print_hello();
}

}  // namespace hello_world
}  // namespace autoware
