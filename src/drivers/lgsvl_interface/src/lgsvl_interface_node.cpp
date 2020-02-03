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
#include <vector>

namespace lgsvl_interface
{

LgsvlInterfaceNode::LgsvlInterfaceNode(
  const std::string & node_name,
  const std::string & node_namespace)
: VehicleInterfaceNode{node_name, node_namespace}
{
  const auto sim_ctrl_cmd_topic =
    declare_parameter("lgsvl.control_command_topic").get<std::string>();
  const auto sim_state_cmd_topic =
    declare_parameter("lgsvl.state_command_topic").get<std::string>();
  const auto sim_state_rpt_topic =
    declare_parameter("lgsvl.state_report_topic").get<std::string>();
  // Optional
  const auto sim_odom_topic_param =
    declare_parameter("lgsvl.odometry_topic");
  const std::string sim_odom_topic =
    rclcpp::ParameterType::PARAMETER_NOT_SET == sim_odom_topic_param.get_type() ?
    "" :
    sim_odom_topic_param.get<std::string>();
  const auto kinematic_state_topic =
    declare_parameter("lgsvl.kinematic_state_topic").get<std::string>();
  const auto table = [this](const std::string & prefix_raw) -> Table1D {
      const std::string prefix = "lgsvl." + prefix_raw + ".";
      return Table1D{
      declare_parameter(prefix + "domain").get<std::vector<double>>(),
      declare_parameter(prefix + "range").get<std::vector<double>>()
      };
    };
  // Set up interface
  set_interface(std::make_unique<LgsvlInterface>(
      *this,
      sim_ctrl_cmd_topic,
      sim_state_cmd_topic,
      sim_state_rpt_topic,
      sim_odom_topic,
      kinematic_state_topic,
      table("throttle"),
      table("brake"),
      table("steer")
  ));
  // TODO(c.ho) low pass filter and velocity controller
}

}  // namespace lgsvl_interface
