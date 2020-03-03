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

#include "trajectory_spoofer/trajectory_spoofer_node.hpp"
#include "trajectory_spoofer/trajectory_spoofer.hpp"

#include <memory>

//lint -e537 NOLINT  // cpplint vs pclint

namespace autoware
{
namespace trajectory_spoofer
{
TrajectorySpooferNode::TrajectorySpooferNode(const rclcpp::NodeOptions & options)
: Node("trajectory_spoofer", options), verbose_(true)
{
  speed_ramp_on_ = this->declare_parameter("speed_ramp_on", false);
  target_speed_ = static_cast<float32_t>(this->declare_parameter("target_speed", 10.0));

  if (speed_ramp_on_) {
    spoofer_ = std::make_shared<TrajectorySpoofer>(target_speed_);
  } else {
    spoofer_ = std::make_shared<TrajectorySpoofer>();
  }

  trajectory_pub_ = this->create_publisher<Trajectory>("dummy_trajectory", 10);
  state_sub_ = this->create_subscription<VehicleKinematicState>(
    "kinematic_state",
    rclcpp::QoS{10},
    [this](VehicleKinematicState::SharedPtr msg) {on_recv_state(msg);});
}

void TrajectorySpooferNode::on_recv_state(VehicleKinematicState::SharedPtr msg)
{
  (void)msg;
}
}  // namespace trajectory_spoofer
}  // namespace autoware
