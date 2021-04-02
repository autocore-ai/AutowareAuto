// Copyright 2021 The Autoware Foundation
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

#include "ne_raptor_interface/test_ne_raptor_interface_listener.hpp"
#include <memory>

namespace autoware
{
namespace ne_raptor_interface
{
NERaptorInterfaceListener::NERaptorInterfaceListener(const rclcpp::NodeOptions & options)
: Node("ne_raptor_interface_listener_node", "/gtest", options),
  l_accel_cmd{},
  l_brake_cmd{},
  l_gear_cmd{},
  l_enable_cmd{},
  l_misc_cmd{},
  l_steer_cmd{},
  l_vehicle_state{},
  l_vehicle_odo{},
  l_vehicle_kin_state{},
  l_got_accel_cmd{false},
  l_got_brake_cmd{false},
  l_got_gear_cmd{false},
  l_got_global_enable_cmd{false},
  l_got_misc_cmd{false},
  l_got_steer_cmd{false},
  l_got_dbw_enable_cmd{false},
  l_got_dbw_disable_cmd{false},
  l_got_vehicle_state{false},
  l_got_vehicle_odo{false},
  l_got_vehicle_kin_state{false}
{
  // Subscribers (from Raptor DBW)
  l_accel_cmd_sub =
    this->create_subscription<AcceleratorPedalCmd>(
    "accelerator_pedal_cmd", rclcpp::QoS{1},
    [this](AcceleratorPedalCmd::SharedPtr msg) {on_accel_cmd(msg);});

  l_brake_cmd_sub =
    this->create_subscription<BrakeCmd>(
    "brake_cmd", rclcpp::QoS{1},
    [this](BrakeCmd::SharedPtr msg) {on_brake_cmd(msg);});

  l_gear_cmd_sub =
    this->create_subscription<GearCmd>(
    "gear_cmd", rclcpp::QoS{1},
    [this](GearCmd::SharedPtr msg) {on_gear_cmd(msg);});

  l_global_enable_cmd_sub =
    this->create_subscription<GlobalEnableCmd>(
    "global_enable_cmd", rclcpp::QoS{1},
    [this](GlobalEnableCmd::SharedPtr msg) {on_global_enable_cmd(msg);});

  l_misc_cmd_sub =
    this->create_subscription<MiscCmd>(
    "misc_cmd", rclcpp::QoS{1},
    [this](MiscCmd::SharedPtr msg) {on_misc_cmd(msg);});

  l_steer_cmd_sub =
    this->create_subscription<SteeringCmd>(
    "steering_cmd", rclcpp::QoS{1},
    [this](SteeringCmd::SharedPtr msg) {on_steer_cmd(msg);});

  l_dbw_enable_cmd_sub =
    this->create_subscription<std_msgs::msg::Empty>(
    "enable", rclcpp::QoS{1},
    [this](std_msgs::msg::Empty::SharedPtr msg) {on_dbw_enable_cmd(msg);});

  l_dbw_disable_cmd_sub =
    this->create_subscription<std_msgs::msg::Empty>(
    "disable", rclcpp::QoS{1},
    [this](std_msgs::msg::Empty::SharedPtr msg) {on_dbw_disable_cmd(msg);});

  // Subscribers (from Autoware.Auto)
  l_vehicle_state_sub =
    this->create_subscription<VehicleStateReport>(
    "vehicle_state_report", rclcpp::QoS{10},
    [this](VehicleStateReport::SharedPtr msg) {on_vehicle_state(msg);});

  l_vehicle_odo_sub =
    this->create_subscription<VehicleOdometry>(
    "vehicle_odometry", rclcpp::QoS{10},
    [this](VehicleOdometry::SharedPtr msg) {on_vehicle_odo(msg);});

  l_vehicle_kin_state_sub =
    this->create_subscription<VehicleKinematicState>(
    "vehicle_kinematic_state", rclcpp::QoS{10},
    [this](VehicleKinematicState::SharedPtr msg) {on_vehicle_kin_state(msg);});
}

// Listener functions
void NERaptorInterfaceListener::on_accel_cmd(const AcceleratorPedalCmd::SharedPtr & msg)
{
  l_accel_cmd = *msg;
  l_got_accel_cmd = true;
}

void NERaptorInterfaceListener::on_brake_cmd(const BrakeCmd::SharedPtr & msg)
{
  l_brake_cmd = *msg;
  l_got_brake_cmd = true;
}

void NERaptorInterfaceListener::on_gear_cmd(const GearCmd::SharedPtr & msg)
{
  l_gear_cmd = *msg;
  l_got_gear_cmd = true;
}

void NERaptorInterfaceListener::on_global_enable_cmd(const GlobalEnableCmd::SharedPtr & msg)
{
  l_enable_cmd = *msg;
  l_got_global_enable_cmd = true;
}

void NERaptorInterfaceListener::on_misc_cmd(const MiscCmd::SharedPtr & msg)
{
  l_misc_cmd = *msg;
  l_got_misc_cmd = true;
}

void NERaptorInterfaceListener::on_steer_cmd(const SteeringCmd::SharedPtr & msg)
{
  l_steer_cmd = *msg;
  l_got_steer_cmd = true;
}

void NERaptorInterfaceListener::on_dbw_enable_cmd(const std_msgs::msg::Empty::SharedPtr & msg)
{
  if (msg != NULL) {
    l_got_dbw_enable_cmd = true;
  }
}

void NERaptorInterfaceListener::on_dbw_disable_cmd(const std_msgs::msg::Empty::SharedPtr & msg)
{
  if (msg != NULL) {
    l_got_dbw_disable_cmd = true;
  }
}

void NERaptorInterfaceListener::on_vehicle_state(const VehicleStateReport::SharedPtr & msg)
{
  l_vehicle_state = *msg;
  l_got_vehicle_state = true;
}

void NERaptorInterfaceListener::on_vehicle_odo(const VehicleOdometry::SharedPtr & msg)
{
  l_vehicle_odo = *msg;
  l_got_vehicle_odo = true;
}

void NERaptorInterfaceListener::on_vehicle_kin_state(const VehicleKinematicState::SharedPtr & msg)
{
  l_vehicle_kin_state = *msg;
  l_got_vehicle_kin_state = true;
}

}  // namespace ne_raptor_interface
}  // namespace autoware
