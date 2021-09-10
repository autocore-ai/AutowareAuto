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

/// @copyright Copyright 2021 The Autoware Foundation
/// @file vehicle_constants_manager_node.hpp
/// @brief This file defines the vehicle_constants_manager_nodes_node class.

#ifndef VEHICLE_CONSTANTS_MANAGER_NODES__VEHICLE_CONSTANTS_MANAGER_NODE_HPP_
#define VEHICLE_CONSTANTS_MANAGER_NODES__VEHICLE_CONSTANTS_MANAGER_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <vehicle_constants_manager/vehicle_constants_manager.hpp>
#include "vehicle_constants_manager_nodes/visibility_control.hpp"

namespace autoware
{
namespace common
{
namespace vehicle_constants_manager_node
{

/// @brief The node that reads the vehicle constants from a yaml file and publishes them with the
/// parameter server.
/// @details It reads and prints the vehicle constants with the help of vehicle_constants_manager
/// library package.
class VEHICLE_CONSTANTS_MANAGER_NODES_PUBLIC VehicleConstantsManagerNode : public rclcpp::Node
{
public:
  /// @brief The constructor of the Node.
  /// @throws std::runtime_error in possible initialization failure of
  /// `vehicle_constants_manager::VehicleConstants` object.
  /// @param options rclcpp::NodeOptions object which stands for encapsulation of options for node
  /// initialization.
  explicit VehicleConstantsManagerNode(const rclcpp::NodeOptions & options);

private:
};

}  // namespace vehicle_constants_manager_node
}  // namespace common
}  // namespace autoware

#endif  // VEHICLE_CONSTANTS_MANAGER_NODES__VEHICLE_CONSTANTS_MANAGER_NODE_HPP_
