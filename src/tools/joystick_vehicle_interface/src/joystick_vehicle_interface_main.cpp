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

#include "joystick_vehicle_interface/joystick_vehicle_interface_node.hpp"
#include <rclcpp/rclcpp.hpp>

#include <memory>

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  const auto nd = std::make_shared<joystick_vehicle_interface::JoystickVehicleInterfaceNode>(
    "joystick_vehicle_interface",
    ""
  );
  rclcpp::spin(nd);

  (void)rclcpp::shutdown();
  return 0;
}
