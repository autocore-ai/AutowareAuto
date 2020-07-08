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

#include <autoware_auto_msgs/msg/vehicle_kinematic_state.hpp>
#include <autoware_auto_msgs/msg/trajectory.hpp>
#include <common/types.hpp>
#include <rclcpp/rclcpp.hpp>

#include <memory>
#include <string>

namespace autoware
{
namespace trajectory_spoofer
{
using Trajectory = autoware_auto_msgs::msg::Trajectory;
using VehicleKinematicState = autoware_auto_msgs::msg::VehicleKinematicState;
using std::placeholders::_1;

/// \class TrajectorySpooferNode
/// \brief ROS 2 Node for creating fake trajectories
class TRAJECTORY_SPOOFER_PUBLIC TrajectorySpooferNode
  : public rclcpp::Node
{
private:
  enum class TrajectoryType : uint8_t
  {
    STRAIGHT = 1u,
    CIRCLE = 2u,
  };

  bool8_t speed_ramp_on_;
  float32_t target_speed_;
  int32_t num_of_points_;
  TrajectoryType trajectory_type_;
  float32_t length_;
  float32_t radius_;

  Trajectory trajectory_;

  std::shared_ptr<TrajectorySpoofer> spoofer_;
  std::shared_ptr<rclcpp::Publisher<Trajectory>> trajectory_pub_;
  std::shared_ptr<rclcpp::Subscription<VehicleKinematicState>> state_sub_;

  TrajectoryType get_trajectory_type_from_string(const std::string & trajectory_type_string)
  {
    if (trajectory_type_string == "straight") {
      return TrajectoryType::STRAIGHT;
    } else if (trajectory_type_string == "circle") {
      return TrajectoryType::CIRCLE;
    } else {
      throw std::invalid_argument{"Unknown trajectory type"};
    }
  }

public:
  /// \brief default constructor, starts node
  /// \param[in] node_options an rclcpp::NodeOptions object to configure the node
  /// \throw runtime error if failed to start threads or configure node
  explicit TrajectorySpooferNode(const rclcpp::NodeOptions & node_options);

  void on_recv_state(VehicleKinematicState::SharedPtr msg);
};

}  // namespace trajectory_spoofer
}  // namespace autoware

#endif  // TRAJECTORY_SPOOFER__TRAJECTORY_SPOOFER_NODE_HPP_
