// Copyright 2019 the Autoware Foundation
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
  /// \param[in] cfg Configuration object for PurePursuit
  /// \param[in] node_namespace Namespace of this node
  PurePursuitNode(
    const std::string & node_name,
    const pure_pursuit::Config & cfg,
    const std::string & node_namespace = "");
};
}  // namespace pure_pursuit_nodes
}  // namespace control
}  // namespace motion
}  // namespace autoware

#endif  // PURE_PURSUIT_NODES__PURE_PURSUIT_NODE_HPP_
