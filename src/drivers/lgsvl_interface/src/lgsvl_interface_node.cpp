// Copyright 2020 Apex.AI, Inc.
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

#include "lgsvl_interface/lgsvl_interface_node.hpp"
#include "lgsvl_interface/lgsvl_interface.hpp"

#include <memory>
#include <string>

namespace lgsvl_interface
{

LgsvlInterfaceNode::LgsvlInterfaceNode(
  const std::string & node_name,
  const std::string & node_namespace)
: VehicleInterfaceNode{node_name, node_namespace}
{
  const auto sim_cmd_topic = declare_parameter("lgsvl.command_topic").get<std::string>();
  const auto sim_can_topic = declare_parameter("lgsvl.can_topic").get<std::string>();
  const Config cfg{
    declare_parameter("lgsvl.translator.throttle_scale").get<double>(),
    declare_parameter("lgsvl.translator.brake_scale").get<double>(),
    declare_parameter("lgsvl.translator.steer_scale").get<double>(),
    declare_parameter("lgsvl.translator.velocity_max").get<double>(),
    declare_parameter("lgsvl.translator.velocity_min").get<double>(),
    declare_parameter("lgsvl.translator.cg_to_front").get<double>(),
    declare_parameter("lgsvl.translator.cg_to_rear").get<double>(),
  };
  // Set up interface
  set_interface(std::make_unique<LgsvlInterface>(*this, sim_cmd_topic, sim_can_topic, cfg));
  // TODO(c.ho) low pass filter and velocity controller
}

}  // namespace lgsvl_interface
