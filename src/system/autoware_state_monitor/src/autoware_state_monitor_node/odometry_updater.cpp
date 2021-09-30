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
//
// Co-developed by Tier IV, Inc. and Robotec.AI sp. z o.o.

#include "autoware_state_monitor/odometry_updater.hpp"

#include "rclcpp/time.hpp"

namespace autoware
{
namespace state_monitor
{

OdometryUpdater::OdometryUpdater(
  OdometryBuffer & odometry_buffer,
  double buffer_length_sec)
: odometry_buffer_(odometry_buffer),
  buffer_length_sec_(buffer_length_sec)
{}

void OdometryUpdater::update(
  const autoware_auto_msgs::msg::VehicleOdometry::ConstSharedPtr msg)
{
  if (!msg) {
    return;
  }

  odometry_buffer_.push_back(msg);

  // Delete old data in buffer
  while (true) {
    const auto time_diff = rclcpp::Time(msg->stamp) -
      rclcpp::Time(odometry_buffer_.front()->stamp);

    if (time_diff.seconds() <= buffer_length_sec_) {
      break;
    }

    odometry_buffer_.pop_front();
  }
}

}  // namespace state_monitor
}  // namespace autoware
