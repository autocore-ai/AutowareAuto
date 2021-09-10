// Copyright 2021 The Autoware Foundation
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

#include "vehicle_constants_manager_nodes/vehicle_constants_manager_node.hpp"
#include <rclcpp/rclcpp.hpp>
#include <common/types.hpp>

#include <algorithm>
#include <map>
#include <string>
#include <utility>
#include <vector>

namespace autoware
{
namespace common
{
namespace vehicle_constants_manager_node
{

using common::types::float64_t;

VehicleConstantsManagerNode::VehicleConstantsManagerNode(const rclcpp::NodeOptions & options)
:  Node("vehicle_constants_manager_node", options)
{
  // Declare params from yaml/inputs to ROS2 parameter server
  const std::vector<std::string> params_primary_names{
    "wheel_radius",
    "wheel_width",
    "wheel_base",
    "wheel_tread",
    "overhang_front",
    "overhang_rear",
    "overhang_left",
    "overhang_right",
    "vehicle_height",
    "cg_to_rear",
    "tire_cornering_stiffness_front",
    "tire_cornering_stiffness_rear",
    "mass_vehicle",
    "inertia_yaw_kg_m2"};

  std::map<std::string, float64_t> map_param_prim_name_to_val;
  std::for_each(
    params_primary_names.cbegin(),
    params_primary_names.cend(),
    [&map_param_prim_name_to_val](const std::string & name) {
      map_param_prim_name_to_val.insert(std::make_pair(name, -1.0));
    });

  this->declare_parameters("", map_param_prim_name_to_val);
  const std::vector<rclcpp::Parameter> params_primary_values = this->get_parameters(
    params_primary_names);

  // Build the VehicleConstants object
  const auto vc = vehicle_constants_manager::VehicleConstants(
    params_primary_values.at(0).as_double(),
    params_primary_values.at(1).as_double(),
    params_primary_values.at(2).as_double(),
    params_primary_values.at(3).as_double(),
    params_primary_values.at(4).as_double(),
    params_primary_values.at(5).as_double(),
    params_primary_values.at(6).as_double(),
    params_primary_values.at(7).as_double(),
    params_primary_values.at(8).as_double(),
    params_primary_values.at(9).as_double(),
    params_primary_values.at(10).as_double(),
    params_primary_values.at(11).as_double(),
    params_primary_values.at(12).as_double(),
    params_primary_values.at(13).as_double()
  );

  // Publish the derived parameters to the parameter server
  const std::map<std::string, float64_t> map_param_deriv_name_to_val{
    std::make_pair("cg_to_front", vc.cg_to_front),
    std::make_pair("vehicle_length", vc.vehicle_length),
    std::make_pair("vehicle_width", vc.vehicle_width),
    std::make_pair("offset_longitudinal_min", vc.offset_longitudinal_min),
    std::make_pair("offset_longitudinal_max", vc.offset_longitudinal_max),
    std::make_pair("offset_lateral_min", vc.offset_lateral_min),
    std::make_pair("offset_lateral_max", vc.offset_lateral_max),
    std::make_pair("offset_height_min", vc.offset_height_min),
    std::make_pair("offset_height_max", vc.offset_height_max)
  };

  this->declare_parameters(this->get_namespace(), map_param_deriv_name_to_val, true);

  // Publish confirmation bool parameter
  this->declare_parameter<types::bool8_t>(
    "published_all",
    true,
    rcl_interfaces::msg::ParameterDescriptor(),
    true);

  // Pretty print
  RCLCPP_INFO_STREAM(get_logger(), "vehicle constants: \n" << vc.str_pretty());
}

}  // namespace vehicle_constants_manager_node
}  // namespace common
}  // namespace autoware

#include "rclcpp_components/register_node_macro.hpp"

// This acts as an entry point, allowing the component to be
// discoverable when its library is being loaded into a running process
RCLCPP_COMPONENTS_REGISTER_NODE(
  autoware::common::vehicle_constants_manager_node::VehicleConstantsManagerNode)
