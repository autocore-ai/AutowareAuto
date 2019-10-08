/// \copyright Copyright 2017-2018 Apex.AI, Inc.
/// All rights reserved.
/// \file
/// \brief This file defines the pure pursuit node

#ifndef PURE_PURSUIT_NODES__PURE_PURSUIT_NODE_HPP_
#define PURE_PURSUIT_NODES__PURE_PURSUIT_NODE_HPP_

#include <controller_common_nodes/controller_base_node.hpp>
#include <pure_pursuit/pure_pursuit.hpp>

#include <string>

#include "pure_pursuit_nodes/visibility_control.hpp"

namespace autoware
{
namespace motion
{
namespace control
{
/// \brief Boilerplate Apex.OS nodes around pure_pursuit
namespace pure_pursuit_nodes
{
/// \brief Boilerplate node that subscribes to the current pose and
/// publishes a vehicle control command
class PURE_PURSUIT_NODES_PUBLIC PurePursuitNode
  : public ::motion::control::controller_common_nodes::ControllerBaseNode
{
public:
  /// \brief Parameter constructor
  /// \param[in] node_name Name of the node, controls which parameter set from the file is matched
  /// \param[in] node_namespace Name of the node's namespace, controls which parameters are used
  PurePursuitNode(
    const std::string & node_name,
    const std::string & node_namespace = "");

  /// \brief Explicit constructor
  /// \param[in] node_name Name of the node
  /// \param[in] pose_topic Name of input pose topic
  /// \param[in] trajectory_topic Name of input trajectory topic
  /// \param[in] command_topic Name of output control command topic
  /// \param[in] tf_topic Name of input tf topic
  /// \param[in] diagnosis_topic Name of output diagnosis topic
  /// \param[in] cfg Configuration object for PurePursuit
  /// \param[in] node_namespace Namespace of this node
  PurePursuitNode(
    const std::string & node_name,
    const std::string & pose_topic,
    const std::string & trajectory_topic,
    const std::string & command_topic,
    const std::string & diagnosis_topic,
    const std::string & tf_topic,
    const pure_pursuit::Config & cfg,
    const std::string & node_namespace = "");
};
}  // namespace pure_pursuit_nodes
}  // namespace control
}  // namespace motion
}  // namespace autoware

#endif  // PURE_PURSUIT_NODES__PURE_PURSUIT_NODE_HPP_
