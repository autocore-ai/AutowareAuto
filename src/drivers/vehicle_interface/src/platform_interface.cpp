// Copyright 2020 the Autoware Foundation
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
#include "vehicle_interface/platform_interface.hpp"

namespace autoware
{
namespace drivers
{
namespace vehicle_interface
{
const autoware_auto_msgs::msg::VehicleStateReport &
PlatformInterface::get_state_report() const noexcept
{
  return m_state_report;
}

const autoware_auto_msgs::msg::VehicleOdometry & PlatformInterface::get_odometry() const noexcept
{
  return m_odometry;
}

autoware_auto_msgs::msg::VehicleStateReport & PlatformInterface::state_report() noexcept
{
  //lint -e{1536} This is an active design choice to not make getters and setters pure virtual NOLINT
  return m_state_report;
}

autoware_auto_msgs::msg::VehicleOdometry & PlatformInterface::odometry() noexcept
{
  //lint -e{1536} This is an active design choice to not make getters and setters pure virtual NOLINT
  return m_odometry;
}
}  // namespace vehicle_interface
}  // namespace drivers
}  // namespace autoware
