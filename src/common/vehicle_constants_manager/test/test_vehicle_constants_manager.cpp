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

#include <memory>
#include <stdexcept>

#include "gtest/gtest.h"
#include "vehicle_constants_manager/vehicle_constants_manager.hpp"


TEST(TestVehicleConstantsManager, TestInitializationConstructor) {
  using float64_t = autoware::common::types::float64_t;
  using VehicleConstants = autoware::common::vehicle_constants_manager::VehicleConstants;

  const float64_t wheel_radius = 0.37;
  const float64_t wheel_width = 0.27;
  const float64_t wheel_base = 2.734;
  const float64_t wheel_tread = 1.571;
  const float64_t overhang_front = 1.033;
  const float64_t overhang_rear = 1.021;
  const float64_t overhang_left = 0.3135;
  const float64_t overhang_right = 0.3135;
  const float64_t vehicle_height = 1.662;
  const float64_t cg_to_rear = 1.367;
  const float64_t tire_cornering_stiffness_front_n_per_deg = 0.1;
  const float64_t tire_cornering_stiffness_rear_n_per_deg = 0.1;
  const float64_t mass_vehicle = 2120.0;
  const float64_t inertia_yaw_kg_m_2 = 12.0;

  // Well set parameters
  EXPECT_NO_THROW(
    VehicleConstants vc(
      wheel_radius,
      wheel_width,
      wheel_base,
      wheel_tread,
      overhang_front,
      overhang_rear,
      overhang_left,
      overhang_right,
      vehicle_height,
      cg_to_rear,
      tire_cornering_stiffness_front_n_per_deg,
      tire_cornering_stiffness_rear_n_per_deg,
      mass_vehicle,
      inertia_yaw_kg_m_2
  ));

  // Center of gravity is not within wheel_base
  const float64_t bad_cg_to_rear = wheel_base + 1.0;
  EXPECT_THROW(
    VehicleConstants vc(
      wheel_radius,
      wheel_width,
      wheel_base,
      wheel_tread,
      overhang_front,
      overhang_rear,
      overhang_left,
      overhang_right,
      vehicle_height,
      bad_cg_to_rear,
      tire_cornering_stiffness_front_n_per_deg,
      tire_cornering_stiffness_rear_n_per_deg,
      mass_vehicle,
      inertia_yaw_kg_m_2
    ), std::runtime_error);

  // Some supposedly absolute parameters are negative
  EXPECT_THROW(
    VehicleConstants vc(
      wheel_radius,
      wheel_width,
      -wheel_base,
      wheel_tread,
      -overhang_front,
      overhang_rear,
      overhang_left,
      -overhang_right,
      vehicle_height,
      cg_to_rear,
      tire_cornering_stiffness_front_n_per_deg,
      tire_cornering_stiffness_rear_n_per_deg,
      mass_vehicle,
      inertia_yaw_kg_m_2), std::runtime_error);
}

TEST(TestVehicleConstantsManager, TestGetVehicleConstants) {
  rclcpp::init(0, nullptr);
  const rclcpp::Node::SharedPtr node = std::make_shared<rclcpp::Node>("some_node");
  using namespace std::chrono_literals;
  EXPECT_THROW(
    autoware::common::vehicle_constants_manager::try_get_vehicle_constants(node, 300ms),
    std::runtime_error);
  rclcpp::shutdown();
}
