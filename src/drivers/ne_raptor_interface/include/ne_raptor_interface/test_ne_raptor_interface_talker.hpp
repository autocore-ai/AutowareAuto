// Copyright 2020 The Autoware Foundation
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

/// \copyright Copyright 2021 The Autoware Foundation
/// \file test_ne_raptor_interface_talker.hpp
/// \brief This file defines the NERaptorInterfaceTalker class.

#ifndef NE_RAPTOR_INTERFACE__TEST_NE_RAPTOR_INTERFACE_TALKER_HPP_
#define NE_RAPTOR_INTERFACE__TEST_NE_RAPTOR_INTERFACE_TALKER_HPP_

#include <gtest/gtest.h>

#include <ne_raptor_interface/ne_raptor_interface.hpp>

#include <cmath>
#include <cstdint>
#include <fstream>
#include <memory>

namespace autoware
{
namespace ne_raptor_interface
{

/// \brief Class that publishes messages to help test NERaptorInterface
class NERaptorInterfaceTalker : public rclcpp::Node
{
public:
  /// \brief Default constructor.
  /// \param[in] options NodeOptions object with required parameters and options
  explicit NERaptorInterfaceTalker(const rclcpp::NodeOptions & options);

  /// \brief Put in data to publish (overloaded function)
  /// \param[in] msg The control command to send to the vehicle (type BrakeReport)
  /// \return false if sending failed in some way, true otherwise
  bool8_t send_report(const BrakeReport & msg);

  /// \brief Put in data to publish (overloaded function)
  /// \param[in] msg The control command to send to the vehicle (type GearReport)
  /// \return false if sending failed in some way, true otherwise
  bool8_t send_report(const GearReport & msg);

  /// \brief Put in data to publish (overloaded function)
  /// \param[in] msg The control command to send to the vehicle (type MiscReport)
  /// \return false if sending failed in some way, true otherwise
  bool8_t send_report(const MiscReport & msg);

  /// \brief Put in data to publish (overloaded function)
  /// \param[in] msg The control command to send to the vehicle (type OtherActuatorsReport)
  /// \return false if sending failed in some way, true otherwise
  bool8_t send_report(const OtherActuatorsReport & msg);

  /// \brief Put in data to publish (overloaded function)
  /// \param[in] msg The control command to send to the vehicle (type SteeringReport)
  /// \return false if sending failed in some way, true otherwise
  bool8_t send_report(const SteeringReport & msg);

  /// \brief Put in data to publish (overloaded function)
  /// \param[in] msg The control command to send to the vehicle (type WheelSpeedReport)
  /// \return false if sending failed in some way, true otherwise
  bool8_t send_report(const WheelSpeedReport & msg);

private:
  // Publishers (from Raptor DBW)
  rclcpp::Publisher<BrakeReport>::SharedPtr t_brake_rpt_pub;
  rclcpp::Publisher<GearReport>::SharedPtr t_gear_rpt_pub;
  rclcpp::Publisher<MiscReport>::SharedPtr t_misc_rpt_pub;
  rclcpp::Publisher<OtherActuatorsReport>::SharedPtr t_other_acts_rpt_pub;
  rclcpp::Publisher<SteeringReport>::SharedPtr t_steering_rpt_pub;
  rclcpp::Publisher<WheelSpeedReport>::SharedPtr t_wheel_spd_rpt_pub;
};  // class NERaptorInterfaceTalker

}  // namespace ne_raptor_interface
}  // namespace autoware

#endif  // NE_RAPTOR_INTERFACE__TEST_NE_RAPTOR_INTERFACE_TALKER_HPP_
