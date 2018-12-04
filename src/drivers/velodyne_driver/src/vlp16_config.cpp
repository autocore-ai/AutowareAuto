// Copyright 2018 Apex.AI, Inc.
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

#include <limits>
#include <utility>

#include "velodyne_driver/vlp16_translator.hpp"


namespace autoware
{
namespace drivers
{
namespace velodyne_driver
{
////////////////////////////////////////////////////////////////////////////////
Vlp16Translator::Config::Config(
  const float rpm,
  const geometry_msgs::msg::Point32 offset_m,
  const geometry_msgs::msg::Point32 rotation_rad,
  const float min_distance_m,
  const float max_distance_m,
  const float min_angle_deg,
  const float max_angle_deg)
: m_rpm(rpm),
  m_offset_m(offset_m),
  m_rotation_rad(rotation_rad),
  m_min_distance_m(min_distance_m),
  m_max_distance_m(max_distance_m),
  m_min_angle_deg(min_angle_deg),
  m_max_angle_deg(max_angle_deg)
{
  if (m_max_distance_m < m_min_distance_m) {
    throw std::runtime_error("Velodyne Driver: Invalid max/min radial configuration");
  }
}
////////////////////////////////////////////////////////////////////////////////
float Vlp16Translator::Config::get_rpm() const
{
  return m_rpm;
}
////////////////////////////////////////////////////////////////////////////////
const geometry_msgs::msg::Point32 & Vlp16Translator::Config::get_offset() const
{
  return m_offset_m;
}
////////////////////////////////////////////////////////////////////////////////
const geometry_msgs::msg::Point32 & Vlp16Translator::Config::get_rotation() const
{
  return m_rotation_rad;
}
////////////////////////////////////////////////////////////////////////////////
float Vlp16Translator::Config::get_min_distance() const
{
  return m_min_distance_m;
}
////////////////////////////////////////////////////////////////////////////////
float Vlp16Translator::Config::get_max_distance() const
{
  return m_max_distance_m;
}
////////////////////////////////////////////////////////////////////////////////
float Vlp16Translator::Config::get_min_angle() const
{
  return m_min_angle_deg;
}
////////////////////////////////////////////////////////////////////////////////
float Vlp16Translator::Config::get_max_angle() const
{
  return m_max_angle_deg;
}

}  // namespace velodyne_driver
}  // namespace drivers
}  // namespace autoware
