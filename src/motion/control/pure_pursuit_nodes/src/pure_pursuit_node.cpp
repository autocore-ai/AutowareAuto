// Copyright 2019 Apex.AI, Inc.
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

#include <common/types.hpp>
#include <motion_common/motion_common.hpp>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>

#include <pure_pursuit_nodes/pure_pursuit_node.hpp>

#include <memory>
#include <string>

using autoware::common::types::float64_t;
using autoware::common::types::float32_t;
using autoware::common::types::bool8_t;

namespace autoware
{
namespace motion
{
namespace control
{
/// \brief Resources relating to the pure pursuit node package
namespace pure_pursuit_nodes
{
PurePursuitNode::PurePursuitNode(
  const rclcpp::NodeOptions & node_options)
: ControllerBaseNode{"pure_pursuit_node", "", node_options, "ctrl_cmd", "current_pose",
    "tf", "trajectory", "ctrl_diag"}
{
  pure_pursuit::Config cfg{
    static_cast<float32_t>(declare_parameter(
      "controller.minimum_lookahead_distance").get<float64_t>()),
    static_cast<float32_t>(declare_parameter(
      "controller.maximum_lookahead_distance").get<float64_t>()),
    static_cast<float32_t>(
      declare_parameter("controller.speed_to_lookahead_ratio").get<float64_t>()),
    declare_parameter("controller.is_interpolate_lookahead_point").get<bool8_t>(),
    declare_parameter("controller.is_delay_compensation").get<bool8_t>(),
    static_cast<float32_t>(declare_parameter(
      "controller.emergency_stop_distance").get<float64_t>()),
    static_cast<float32_t>(declare_parameter(
      "controller.speed_thres_traveling_direction").get<float64_t>()),
    static_cast<float32_t>(declare_parameter(
      "controller.dist_front_rear_wheels").get<float64_t>())};
  set_controller(std::make_unique<pure_pursuit::PurePursuit>(cfg));
}
}  // namespace pure_pursuit_nodes
}  // namespace control
}  // namespace motion
}  // namespace autoware

RCLCPP_COMPONENTS_REGISTER_NODE(autoware::motion::control::pure_pursuit_nodes::PurePursuitNode)
