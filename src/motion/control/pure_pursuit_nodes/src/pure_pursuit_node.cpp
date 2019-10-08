// Copyright 2017-2018 Apex.AI, Inc.
// All rights reserved.

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
