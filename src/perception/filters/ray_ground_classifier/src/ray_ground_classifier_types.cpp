// Copyright 2017-2019 Apex.AI, Inc.
// Co-developed by Tier IV, Inc. and Apex.AI, Inc.
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

#include <cmath>
#include <cstdint>
#include <stdexcept>

#include "ray_ground_classifier/ray_ground_point_classifier.hpp"

namespace autoware
{
namespace perception
{
namespace filters
{
namespace ray_ground_classifier
{

using autoware::perception::filters::ray_ground_classifier::FEPS;
using autoware::perception::filters::ray_ground_classifier::PointXYZIF;

Config::Config(
  const float sensor_height_m,
  const float max_local_slope_deg,
  const float max_global_slope_deg,
  const float nonground_retro_thresh_deg,
  const float min_height_thresh_m,
  const float max_global_height_thresh_m,
  const float max_last_local_ground_thresh_m,
  const float max_provisional_ground_distance_m,
  const float min_height_m,
  const float max_height_m)
: m_ground_z_m(-sensor_height_m),
  m_max_local_slope(tanf(static_cast<float>(deg2rad(max_local_slope_deg)))),
  m_max_global_slope(tanf(static_cast<float>(deg2rad(max_global_slope_deg)))),
  m_nonground_retro_thresh(tanf(static_cast<float>(deg2rad(nonground_retro_thresh_deg)))),
  m_min_height_thresh_m(min_height_thresh_m),
  m_max_global_height_thresh_m(max_global_height_thresh_m),
  m_max_last_local_ground_thresh_m(max_last_local_ground_thresh_m),
  m_max_provisional_ground_distance_m(max_provisional_ground_distance_m),
  m_min_height_m(min_height_m),
  m_max_height_m(max_height_m)
{
  // TODO(c.ho) nan check
  if ((max_local_slope_deg <= 0.0F) ||
    (max_global_slope_deg <= 0.0F) ||
    (nonground_retro_thresh_deg <= 0.0F))
  {
    throw std::runtime_error("ray ground classifier: config angles must be positive");
  }
  if ((max_local_slope_deg > 90.0F) ||
    (max_global_slope_deg > 90.0F) ||
    (nonground_retro_thresh_deg > 90.0F))
  {
    throw std::runtime_error("ray ground classifier: config angles must be < 90");
  }

  if ((m_min_height_thresh_m <= 0.0F) ||
    (m_max_global_height_thresh_m <= 0.0F))
  {
    throw std::runtime_error("ray ground classifier: config distances must be positive");
  }
  if ((m_min_height_m >= m_max_height_m)) {
    throw std::runtime_error("ray ground classifier: inconsistent max/min heights");
  }
  if ((max_local_slope_deg >= nonground_retro_thresh_deg) ||
    (max_global_slope_deg >= nonground_retro_thresh_deg))
  {
    throw std::runtime_error("ray ground classifier: retro nonground classification must "
            "be greater than other angle thresholds");
  }
  if ((m_max_last_local_ground_thresh_m < m_max_global_height_thresh_m)) {
    throw std::runtime_error("ray ground classifier: max local last ground thresh must "
            "be greater than max_global_height_m");
  }
}
////////////////////////////////////////////////////////////////////////////////
float Config::get_sensor_height() const
{
  return -m_ground_z_m;
}
////////////////////////////////////////////////////////////////////////////////
float Config::get_ground_z() const
{
  return m_ground_z_m;
}
////////////////////////////////////////////////////////////////////////////////
float Config::get_max_local_slope() const
{
  return m_max_local_slope;
}
////////////////////////////////////////////////////////////////////////////////
float Config::get_max_global_slope() const
{
  return m_max_global_slope;
}
////////////////////////////////////////////////////////////////////////////////
float Config::get_nonground_retro_thresh() const
{
  return m_nonground_retro_thresh;
}
////////////////////////////////////////////////////////////////////////////////
float Config::get_min_height_thresh() const
{
  return m_min_height_thresh_m;
}
////////////////////////////////////////////////////////////////////////////////
float Config::get_max_global_height_thresh() const
{
  return m_max_global_height_thresh_m;
}
////////////////////////////////////////////////////////////////////////////////
float Config::get_max_last_local_ground_thresh() const
{
  return m_max_last_local_ground_thresh_m;
}
////////////////////////////////////////////////////////////////////////////////
float Config::get_max_provisional_ground_distance() const
{
  return m_max_provisional_ground_distance_m;
}
////////////////////////////////////////////////////////////////////////////////
float Config::get_min_height() const
{
  return m_min_height_m;
}
////////////////////////////////////////////////////////////////////////////////
float Config::get_max_height() const
{
  return m_max_height_m;
}
////////////////////////////////////////////////////////////////////////////////
PointXYZIFR::PointXYZIFR(const PointXYZIF & pt)
: m_point(pt),
  m_r_xy(sqrtf((pt.x * pt.x) + (pt.y * pt.y)))
{
}
////////////////////////////////////////////////////////////////////////////////
float PointXYZIFR::get_r() const
{
  return m_r_xy;
}
////////////////////////////////////////////////////////////////////////////////
float PointXYZIFR::get_z() const
{
  return m_point.z;
}
////////////////////////////////////////////////////////////////////////////////
bool PointXYZIFR::operator<(const PointXYZIFR & rhs) const
{
  return (fabsf(get_r() - rhs.get_r()) > FEPS) ?
         (get_r() < rhs.get_r()) : (get_z() < rhs.get_z());
}
////////////////////////////////////////////////////////////////////////////////
const PointXYZIF * PointXYZIFR::get_point_pointer() const
{
  return &m_point;
}

}  // namespace ray_ground_classifier
}  // namespace filters
}  // namespace perception
}  // namespace autoware
