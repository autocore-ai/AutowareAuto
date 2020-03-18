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
  num_of_points_ = static_cast<int32_t>(this->declare_parameter("num_of_points", 20));
  std::string trajectory_type_string = this->declare_parameter("trajectory_type", "straight");
  length_ = static_cast<float32_t>(this->declare_parameter("length", 10.0));
  radius_ = static_cast<float32_t>(this->declare_parameter("radius", 12.0));


  if (trajectory_type_string == "straight") {
    trajectory_type_ = TrajectoryType::STRAIGHT;
  } else if (trajectory_type_string == "circle") {
    trajectory_type_ = TrajectoryType::CIRCLE;
  } else {
    exit(0);
  }

  if (speed_ramp_on_) {
    spoofer_ = std::make_shared<TrajectorySpoofer>(target_speed_);
  } else {
    spoofer_ = std::make_shared<TrajectorySpoofer>();
  }

  trajectory_pub_ = this->create_publisher<Trajectory>("trajectory", 10);
  state_sub_ = this->create_subscription<VehicleKinematicState>(
    "vehicle_kinematic_state",
    rclcpp::QoS{10},
    [this](VehicleKinematicState::SharedPtr msg) {on_recv_state(msg);});
}

void TrajectorySpooferNode::on_recv_state(VehicleKinematicState::SharedPtr msg)
{

  Trajectory traj;
  switch (trajectory_type_) {
    // Straight line
    case TrajectoryType::STRAIGHT:
      traj = spoofer_->spoof_straight_trajectory(*msg, num_of_points_, length_, speed_ramp_on_);
      break;

    // Circle
    case TrajectoryType::CIRCLE:
      traj = spoofer_->spoof_circular_trajectory(*msg, num_of_points_, radius_);
      break;
  }

  traj.header.frame_id = msg->header.frame_id;

  if (traj.points.size() > 0) {
    trajectory_pub_->publish(traj);
  }
}
}  // namespace trajectory_spoofer
}  // namespace autoware
