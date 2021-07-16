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
//
// Co-developed by Tier IV, Inc. and Apex.AI, Inc.

#include <tracking_test_framework/tracked_object.hpp>

#include <cmath>
#include <memory>
#include <utility>

namespace autoware
{
namespace tracking_test_framework
{

TrackedObject::TrackedObject(
  const Eigen::Vector2f & starting_position,
  const autoware::common::types::float32_t starting_speed_mps,
  const autoware::common::types::float32_t orientation_degrees,
  const autoware::common::types::float32_t
  orientation_degrees_change, std::unique_ptr<Shape> shape)
: m_shape(std::move(shape)), m_position(starting_position), m_speed_mps(starting_speed_mps),
  m_orientation_rad(autoware::tracking_test_framework::utils::to_radians(orientation_degrees)),
  m_turn_rate_rad_per_sec(autoware::tracking_test_framework::utils::to_radians
      (orientation_degrees_change)) {}

void TrackedObject::move_object(const std::chrono::milliseconds dt_in_ms)
{
  const auto time_interval_secs = static_cast<float>(dt_in_ms.count()) / 1000.F;
  m_position += time_interval_secs * m_speed_mps * Eigen::Vector2f{cos(orientation()), sin(
      orientation())};
  m_orientation_rad += time_interval_secs * m_turn_rate_rad_per_sec;
  update_shape();
}

Car::Car(
  const Eigen::Vector2f & starting_position,
  const autoware::common::types::float32_t starting_speed,
  const autoware::common::types::float32_t orientation_degrees,
  const autoware::common::types::float32_t orientation_degrees_change,
  const Eigen::Vector2f & size)
: TrackedObject(starting_position, starting_speed,
    orientation_degrees, orientation_degrees_change, std::make_unique<Rectangle>(
      Rectangle{starting_position, size, orientation_degrees})), m_size(size) {}

void Car::update_shape()
{
  m_shape = std::make_unique<Rectangle>(Rectangle{this->position(), m_size, this->orientation()});
}

Pedestrian::Pedestrian(
  const Eigen::Vector2f & starting_position,
  const autoware::common::types::float32_t starting_speed,
  const autoware::common::types::float32_t orientation_degrees,
  const autoware::common::types::float32_t orientation_degrees_change)
: TrackedObject(starting_position, starting_speed, orientation_degrees,
    orientation_degrees_change, std::make_unique<Circle>(Circle{starting_position, 1.0f})) {}

void Pedestrian::update_shape()
{
  m_shape = std::make_unique<Circle>(Circle{this->position(), 1.0f});
}


}  // namespace tracking_test_framework
}  // namespace autoware
