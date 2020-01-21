// Copyright 2020 Apex.AI, Inc.
// Co-developed by Tier IV, Inc. and Apex.AI, Inc.
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
/// \file
/// \brief Base class for vehicle "translator"
#ifndef VEHICLE_INTERFACE__PLATFORM_INTERFACE_HPP_
#define VEHICLE_INTERFACE__PLATFORM_INTERFACE_HPP_

#include <vehicle_interface/visibility_control.hpp>
#include <autoware_auto_msgs/msg/raw_control_command.hpp>
#include <autoware_auto_msgs/msg/vehicle_control_command.hpp>
#include <autoware_auto_msgs/msg/vehicle_odometry.hpp>
#include <autoware_auto_msgs/msg/vehicle_state_command.hpp>
#include <autoware_auto_msgs/msg/vehicle_state_report.hpp>

#include <chrono>

namespace autoware
{
namespace drivers
{
namespace vehicle_interface
{

/// Interface class for specific vehicle implementations. Child classes which implement this
/// interface are expected to have wrap their own communication mechanism, and create a subclass
/// from the VehicleInterfaceNode
class PlatformInterface
{
public:
  /// Constructor
  PlatformInterface() = default;
  /// Destructor
  virtual ~PlatformInterface() = default;

  /// Try to receive data from the vehicle platform, and update StateReport and Odometry. If no
  /// data is received, return false, and the VehicleInterfaceNode will treat this as an error
  /// If the platform supports additional sensors/messages, then publishing can happen in this
  /// method.
  /// Exceptions may be thrown on errors
  /// \param[in] timeout The maximum amount of time to check/receive data
  /// \return True if data was received before the timeout, false otherwise
  virtual bool update(std::chrono::nanoseconds timeout) = 0;
  /// Send the state command to the vehicle platform. May require translation into a
  /// vehicle-specific representation and sending multiple messages
  /// Exceptions may be thrown on errors
  /// \param[in] msg The state command to send to the vehicle
  /// \return false if sending failed in some way, true otherwise
  virtual bool send_state_command(const autoware_auto_msgs::msg::VehicleStateCommand & msg) = 0;
  /// Send the state command to the vehicle platform. May require translation into a
  /// vehicle-specific representation and sending multiple messages.
  /// Exceptions may be thrown on errors
  /// \param[in] msg The control command to send to the vehicle
  /// \return false if sending failed in some way, true otherwise
  virtual bool send_control_command(const autoware_auto_msgs::msg::VehicleControlCommand & msg) = 0;
  /// Send the state command to the vehicle platform. May require translation into a
  /// vehicle-specific representation and sending multiple messages.
  /// Exceptions may be thrown on errors
  /// \param[in] msg The control command to send to the vehicle
  /// \return false if sending failed in some way, true otherwise
  virtual bool send_control_command(const autoware_auto_msgs::msg::RawControlCommand & msg) = 0;

  /// Get the most recent state of the vehicle. The State should be assumed to be constant unless
  /// data from the vehicle platform implies a state should be changed. For example, if the gear
  /// state is drive, the StateReport should be in drive until the vehicle platform reports that
  /// it is in neutral or some other gear state.
  /// \return A StateReport message intended to be published.
  const autoware_auto_msgs::msg::VehicleStateReport & get_state_report() const noexcept;
  /// Get the most recent odomoetry of the vehicle
  /// \return A Odometry message intended to be published.
  const autoware_auto_msgs::msg::VehicleOdometry & get_odometry() const noexcept;

protected:
  /// Get the underlying state report for modification
  autoware_auto_msgs::msg::VehicleStateReport & state_report() noexcept;
  /// Get the underlying odometry for modification
  autoware_auto_msgs::msg::VehicleOdometry & odometry() noexcept;

private:
  autoware_auto_msgs::msg::VehicleStateReport m_state_report{};
  autoware_auto_msgs::msg::VehicleOdometry m_odometry{};
};  // class PlatformInterface
}  // namespace vehicle_interface
}  // namespace drivers
}  // namespace autoware

#endif  // VEHICLE_INTERFACE__PLATFORM_INTERFACE_HPP_
