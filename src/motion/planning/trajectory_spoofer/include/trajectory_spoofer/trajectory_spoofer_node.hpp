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
/// \brief This file defines the TrajectorySpooferNode class.

#ifndef TRAJECTORY_SPOOFER__TRAJECTORY_SPOOFER_NODE_HPP_
#define TRAJECTORY_SPOOFER__TRAJECTORY_SPOOFER_NODE_HPP_

#include <trajectory_spoofer/trajectory_spoofer.hpp>

#include <rclcpp/rclcpp.hpp>

namespace autoware
{
namespace trajectory_spoofer
{
/// \class TrajectorySpooferNode
/// \brief ROS 2 Node for creating fake trajectories
class TRAJECTORY_SPOOFER_PUBLIC TrajectorySpooferNode
  : public rclcpp::Node
{
public:
  /// \brief default constructor, starts node
  /// \param[in] options an rclcpp::NodeOptions object to configure the node
  /// \throw runtime error if failed to start threads or configure node
  explicit TrajectorySpooferNode(const rclcpp::NodeOptions & options);

private:
  bool verbose;  ///< whether to use verbose output or not.
};
}  // namespace trajectory_spoofer
}  // namespace autoware

#endif  // TRAJECTORY_SPOOFER__TRAJECTORY_SPOOFER_NODE_HPP_
