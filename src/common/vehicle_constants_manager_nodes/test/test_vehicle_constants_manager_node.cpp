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

#include <rclcpp/rclcpp.hpp>
#include <gtest/gtest.h>

#include <vehicle_constants_manager/vehicle_constants_manager.hpp>
#include <memory>
#include <stdexcept>
#include <vector>
#include "vehicle_constants_manager_nodes/vehicle_constants_manager_node.hpp"

using float64_t = autoware::common::types::float64_t;

TEST(TestVehicleConstantsManagerNodes, TestCallFromAnotherNode) {
  rclcpp::init(0, nullptr);

  std::vector<rclcpp::Parameter> params;
  const float64_t wheel_radius = 0.37;
  params.emplace_back("wheel_radius", wheel_radius);
  params.emplace_back("wheel_width", 0.27);
  params.emplace_back("wheel_base", 2.734);
  params.emplace_back("wheel_tread", 1.571);
  params.emplace_back("overhang_front", 1.033);
  params.emplace_back("overhang_rear", 1.021);
  params.emplace_back("overhang_left", 0.3135);
  params.emplace_back("overhang_right", 0.3135);
  params.emplace_back("vehicle_height", 1.662);
  params.emplace_back("cg_to_rear", 1.367);
  params.emplace_back("tire_cornering_stiffness_front", 0.1);
  params.emplace_back("tire_cornering_stiffness_rear", 0.1);
  params.emplace_back("mass_vehicle", 2120.0);
  params.emplace_back("inertia_yaw_kg_m2", 12.0);

  rclcpp::NodeOptions node_options;
  node_options.parameter_overrides(params);

  const auto node_vehicle_constants_manager =
    std::make_shared<autoware::common::vehicle_constants_manager_node::VehicleConstantsManagerNode>(
    node_options);

  // Create it by passing it a sub node (emulating an outer node)
  using namespace std::chrono_literals;
  const auto vc = autoware::common::vehicle_constants_manager::try_get_vehicle_constants(
    node_vehicle_constants_manager->create_sub_node("test"), 300ms);

  // Get these parameters from the VehicleConstants object
  const float64_t vehicle_width = vc.vehicle_width;
  const float64_t overhang_left = vc.overhang_left;
  const float64_t wheel_tread = vc.wheel_tread;
  const float64_t overhang_right = vc.overhang_right;

  // Perform some known checks
  EXPECT_DOUBLE_EQ(wheel_radius, vc.wheel_radius);
  EXPECT_DOUBLE_EQ(vehicle_width, overhang_left + wheel_tread + overhang_right);
  rclcpp::shutdown();
}

TEST(TestVehicleConstantsManagerNodes, TestExpectException) {
  rclcpp::init(0, nullptr);

  // Initialize it with not enough parameters
  std::vector<rclcpp::Parameter> params;
  params.emplace_back("wheel_radius", 0.37);
  params.emplace_back("wheel_width", 0.27);
  params.emplace_back("wheel_base", 2.734);
  params.emplace_back("wheel_tread", 1.571);
  params.emplace_back("overhang_front", 1.033);
  params.emplace_back("overhang_rear", 1.021);
  params.emplace_back("overhang_left", 0.3135);

  rclcpp::NodeOptions node_options;
  node_options.parameter_overrides(params);

  using VehicleConstantsManagerNode =
    autoware::common::vehicle_constants_manager_node::VehicleConstantsManagerNode;

  VehicleConstantsManagerNode::SharedPtr node_vehicle_constants_manager;
  EXPECT_THROW(
    node_vehicle_constants_manager = std::make_shared<VehicleConstantsManagerNode>(node_options),
    std::runtime_error);
  rclcpp::shutdown();
}
