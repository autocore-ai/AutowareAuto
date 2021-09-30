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

#ifndef AUTOWARE_STATE_MONITOR__ODOMETRY_UPDATER_HPP_
#define AUTOWARE_STATE_MONITOR__ODOMETRY_UPDATER_HPP_

#include "autoware_auto_msgs/msg/vehicle_odometry.hpp"
#include "autoware_state_monitor/visibility_control.hpp"
#include "autoware_state_monitor/odometry_buffer.hpp"

namespace autoware
{
namespace state_monitor
{

/// \brief Updates odometry buffer by adding new values and removing old to preserve length
class AUTOWARE_STATE_MONITOR_PUBLIC OdometryUpdater
{
public:
  /// \brief Create odometry updater
  /// \param odometry_buffer stores odometry values
  /// \param buffer_length_sec length of the buffer in seconds
  OdometryUpdater(OdometryBuffer & odometry_buffer, double buffer_length_sec);

  /// \brief Add new odometry message to buffer
  /// \param msg vehicle odometry message
  void update(const autoware_auto_msgs::msg::VehicleOdometry::ConstSharedPtr msg);

private:
  OdometryBuffer & odometry_buffer_;
  double buffer_length_sec_;
};

}  // namespace state_monitor
}  // namespace autoware

#endif  // AUTOWARE_STATE_MONITOR__ODOMETRY_UPDATER_HPP_
