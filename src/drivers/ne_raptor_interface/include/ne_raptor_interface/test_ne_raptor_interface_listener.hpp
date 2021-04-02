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
/// \file test_ne_raptor_interface_listener.hpp
/// \brief This file defines the NERaptorInterfaceListener class.

#ifndef NE_RAPTOR_INTERFACE__TEST_NE_RAPTOR_INTERFACE_LISTENER_HPP_
#define NE_RAPTOR_INTERFACE__TEST_NE_RAPTOR_INTERFACE_LISTENER_HPP_

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

/// \brief Class that listens for messages published while testing NERaptorInterface
class NERaptorInterfaceListener : public rclcpp::Node
{
public:
  /// \brief Default constructor.
  /// \param[in] options NodeOptions object with required parameters and options
  explicit NERaptorInterfaceListener(const rclcpp::NodeOptions & options);

  // Save received values
  AcceleratorPedalCmd l_accel_cmd;            /**< Last AcceleratorPedalCmd message received */
  BrakeCmd l_brake_cmd;                       /**< Last BrakeCmd message received */
  GearCmd l_gear_cmd;                         /**< Last GearCmd message received */
  GlobalEnableCmd l_enable_cmd;               /**< Last GlobalEnableCmd message received */
  MiscCmd l_misc_cmd;                         /**< Last MiscCmd message received */
  SteeringCmd l_steer_cmd;                    /**< Last MiscCmd message received */
  VehicleStateReport l_vehicle_state;         /**< Last VehicleStateReport message received */
  VehicleOdometry l_vehicle_odo;              /**< Last VehicleOdometry message received */
  VehicleKinematicState l_vehicle_kin_state;  /**< Last VehicleKinematicState message received */

  // Check whether values were received
  bool8_t l_got_accel_cmd,    /**< AcceleratorPedalCmd message was received */
    l_got_brake_cmd,          /**< BrakeCmd message was received */
    l_got_gear_cmd,           /**< GearCmd message was received */
    l_got_global_enable_cmd,  /**< GlobalEnableCmd message was received */
    l_got_misc_cmd,           /**< MiscCmd message was received */
    l_got_steer_cmd,          /**< SteeringCmd message was received */
    l_got_dbw_enable_cmd,     /**< DBW Enable message was received */
    l_got_dbw_disable_cmd,    /**< DBW Disable message was received */
    l_got_vehicle_state,      /**< VehicleStateReport message was received */
    l_got_vehicle_odo,        /**< VehicleOdometry message was received */
    l_got_vehicle_kin_state;  /**< VehicleKinematicState message was received */

private:
  // Subscribers (from Raptor DBW)
  rclcpp::SubscriptionBase::SharedPtr
    l_accel_cmd_sub,
    l_brake_cmd_sub,
    l_gear_cmd_sub,
    l_global_enable_cmd_sub,
    l_misc_cmd_sub,
    l_steer_cmd_sub,
    l_dbw_enable_cmd_sub,
    l_dbw_disable_cmd_sub;
  // Subscribers (from Autoware.Auto)
  rclcpp::SubscriptionBase::SharedPtr
    l_vehicle_state_sub,
    l_vehicle_odo_sub,
    l_vehicle_kin_state_sub;

  // Listener functions

  /// \brief Test if data was published
  /// \param[in] msg The message to receive (pointer of type AcceleratorPedalCmd)
  void on_accel_cmd(const AcceleratorPedalCmd::SharedPtr & msg);

  /// \brief Test if data was published
  /// \param[in] msg The message to receive (pointer of type BrakeCmd)
  void on_brake_cmd(const BrakeCmd::SharedPtr & msg);

  /// \brief Test if data was published
  /// \param[in] msg The message to receive (pointer of type GearCmd)
  void on_gear_cmd(const GearCmd::SharedPtr & msg);

  /// \brief Test if data was published
  /// \param[in] msg The message to receive (pointer of type GlobalEnableCmd)
  void on_global_enable_cmd(const GlobalEnableCmd::SharedPtr & msg);

  /// \brief Test if data was published
  /// \param[in] msg The message to receive (pointer of type MiscCmd)
  void on_misc_cmd(const MiscCmd::SharedPtr & msg);

  /// \brief Test if data was published
  /// \param[in] msg The message to receive (pointer of type SteeringCmd)
  void on_steer_cmd(const SteeringCmd::SharedPtr & msg);

  /// \brief Test if data was published
  /// \param[in] msg The message to receive (pointer of type Empty)
  void on_dbw_enable_cmd(const std_msgs::msg::Empty::SharedPtr & msg);

  /// \brief Test if data was published
  /// \param[in] msg The message to receive (pointer of type Empty)
  void on_dbw_disable_cmd(const std_msgs::msg::Empty::SharedPtr & msg);

  /// \brief Test if data was published
  /// \param[in] msg The message to receive (pointer of type VehicleStateReport)
  void on_vehicle_state(const VehicleStateReport::SharedPtr & msg);

  /// \brief Test if data was published
  /// \param[in] msg The message to receive (pointer of type VehicleOdometry)
  void on_vehicle_odo(const VehicleOdometry::SharedPtr & msg);

  /// \brief Test if data was published
  /// \param[in] msg The message to receive (pointer of type VehicleKinematicState)
  void on_vehicle_kin_state(const VehicleKinematicState::SharedPtr & msg);
};  // class NERaptorInterfaceListener

}  // namespace ne_raptor_interface
}  // namespace autoware

#endif  // NE_RAPTOR_INTERFACE__TEST_NE_RAPTOR_INTERFACE_LISTENER_HPP_
