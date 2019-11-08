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

#include <motion_common/motion_common.hpp>

#include <memory>
#include <string>

#include "pure_pursuit_nodes/pure_pursuit_node.hpp"

namespace autoware
{
namespace motion
{
namespace control
{
/// \brief Resources relating to the pure pursuit node package
namespace pure_pursuit_nodes
{
using float32_t = float;
////////////////////////////////////////////////////////////////////////////////
PurePursuitNode::PurePursuitNode(
  const std::string & node_name,
  const std::string & node_namespace)
: ControllerBaseNode{node_name, node_namespace}
{
  pure_pursuit::Config cfg{
    static_cast<float32_t>(declare_parameter(
      "controller.minimum_lookahead_distance").get<double>()),
    static_cast<float32_t>(declare_parameter(
      "controller.maximum_lookahead_distance").get<double>()),
    static_cast<float32_t>(
      declare_parameter("controller.speed_to_lookahead_ratio").get<double>()),
    declare_parameter("controller.is_interpolate_lookahead_point").get<bool>(),
    declare_parameter("controller.is_delay_compensation").get<bool>(),
    static_cast<float32_t>(declare_parameter(
      "controller.emergency_stop_distance").get<double>()),
    static_cast<float32_t>(declare_parameter(
      "controller.speed_thres_traveling_direction").get<double>()),
    static_cast<float32_t>(declare_parameter(
      "controller.dist_front_rear_wheels").get<double>())};
  set_controller(std::make_unique<pure_pursuit::PurePursuit>(cfg));
}
////////////////////////////////////////////////////////////////////////////////
PurePursuitNode::PurePursuitNode(
  const std::string & node_name,
  const std::string & pose_topic,
  const std::string & trajectory_topic,
  const std::string & command_topic,
  const std::string & diagnosis_topic,
  const std::string & tf_topic,
  const pure_pursuit::Config & cfg,
  const std::string & node_namespace)
: ControllerBaseNode{node_name, node_namespace, command_topic, pose_topic,
    tf_topic, trajectory_topic, diagnosis_topic}
{
  set_controller(std::make_unique<pure_pursuit::PurePursuit>(cfg));
}
}  // namespace pure_pursuit_nodes
}  // namespace control
}  // namespace motion
}  // namespace autoware
