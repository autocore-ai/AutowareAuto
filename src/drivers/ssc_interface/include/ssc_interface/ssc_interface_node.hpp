// Copyright 2020 The Autoware Foundation
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

/// \copyright Copyright 2020 The Autoware Foundation
/// \file
/// \brief This file defines the ssc_interface_node class.

#ifndef SSC_INTERFACE__SSC_INTERFACE_NODE_HPP_
#define SSC_INTERFACE__SSC_INTERFACE_NODE_HPP_

#include <ssc_interface/visibility_control.hpp>

#include <vehicle_interface/vehicle_interface_node.hpp>

#include <rclcpp/rclcpp.hpp>

#include <string>

namespace ssc_interface
{

/// \class SscInterfaceNode
/// \brief SSC Interface node version of VehicleInterfaceNode
class SSC_INTERFACE_PUBLIC SscInterfaceNode
  : public ::autoware::drivers::vehicle_interface::VehicleInterfaceNode
{
public:
  /// \brief default constructor, starts driver
  /// \param[in] options Options for the node
  /// \throw runtime error if failed to start threads or configure driver
  explicit SscInterfaceNode(const rclcpp::NodeOptions & options);
};

}  // namespace ssc_interface

#endif  // SSC_INTERFACE__SSC_INTERFACE_NODE_HPP_
