// Copyright 2017-2020 Apex.AI, Inc., Arm Limited
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
//
// Co-developed by Tier IV, Inc. and Apex.AI, Inc.

//lint -e537 NOLINT  // cpplint vs pclint
#include <algorithm>
#include <cmath>
#include <stdexcept>

#include "common/types.hpp"
#include "geometry/interval.hpp"
#include "ray_ground_classifier/ray_ground_point_classifier.hpp"

using autoware::common::types::bool8_t;
using autoware::common::types::float32_t;
using autoware::common::geometry::Interval_f;

namespace autoware
{
namespace perception
{
namespace filters
{
namespace ray_ground_classifier
{

////////////////////////////////////////////////////////////////////////////////
RayGroundPointClassifier::RayGroundPointClassifier(
  const Config & config)
: m_config(config)
{
  reset();
}

////////////////////////////////////////////////////////////////////////////////
RayGroundPointClassifier::RayGroundPointClassifier(
  const RayGroundPointClassifier & original)
: m_config(original.m_config)
{
  reset();
}

////////////////////////////////////////////////////////////////////////////////
void RayGroundPointClassifier::reset()
{
  m_prev_radius_m = 0.0F;
  m_prev_height_m = m_config.m_ground_z_m;
  m_last_was_ground = true;
  m_prev_ground_radius_m = 0.0F;
  m_prev_ground_height_m = m_config.m_ground_z_m;
}

////////////////////////////////////////////////////////////////////////////////
RayGroundPointClassifier::PointLabel RayGroundPointClassifier::is_ground(const PointXYZIFR & pt)
{
  // consider inline if benchmarkings shows that this is slow
  PointLabel ret;
  const float32_t height_m = pt.get_z();
  const float32_t radius_m = pt.get_r();

  // a small fudge factor is added because we check in the sorting process for "almost zero"
  // This is because points which are almost collinear are sorted by height
  const float32_t dr_m = (radius_m - m_prev_radius_m) + autoware::common::types::FEPS;
  if (dr_m < 0.0F) {
    throw std::runtime_error("Ray Ground filter must receive points in increasing radius");
  }

  const float32_t dh_m = fabsf(height_m - m_prev_height_m);
  const Interval_f range(m_config.m_min_height_thresh_m, m_config.m_max_global_height_thresh_m);
  const auto dr_m_clamped = Interval_f::clamp_to(range, m_config.m_max_local_slope * dr_m);
  const bool8_t is_local = (dh_m < dr_m_clamped);
  const float32_t global_height_thresh_m =
    std::min(m_config.m_max_global_slope * radius_m, m_config.m_max_global_height_thresh_m);
  const bool8_t has_vertical_structure = (dh_m > (dr_m * m_config.m_nonground_retro_thresh));
  if (m_last_was_ground) {
    if (is_local) {
      // local in height, so ground
      ret = PointLabel::GROUND;
    } else if (has_vertical_structure) {
      // vertical structure, so nonground, need to retroactively annotate provisional gorund
      ret = PointLabel::RETRO_NONGROUND;
    } else {
      // check global cone
      ret = (fabsf(height_m - m_config.m_ground_z_m) < global_height_thresh_m) ?
        PointLabel::GROUND :
        (dr_m < m_config.m_max_provisional_ground_distance_m) ?
        PointLabel::NONGROUND :
        PointLabel::NONLOCAL_NONGROUND;
    }
  } else {
    const float32_t drg_m = (radius_m - m_prev_ground_radius_m);
    const float32_t dhg_m = fabsf(height_m - m_prev_ground_height_m);
    const Interval_f range(
      m_config.m_min_height_thresh_m, m_config.m_max_last_local_ground_thresh_m);
    const auto drg_m_clamped =
      Interval_f::clamp_to(range, m_config.m_max_local_slope * drg_m);
    const bool8_t is_local_to_last_ground = (dhg_m <= drg_m_clamped);
    if (is_local_to_last_ground) {
      // local to last ground: provisional ground
      ret = PointLabel::PROVISIONAL_GROUND;
    } else if (is_local) {
      // local in height, so nonground
      ret = PointLabel::NONGROUND;
    } else {
      // global ground: provisionally ground
      ret = ((fabsf(height_m - m_config.m_ground_z_m) < global_height_thresh_m)) ?
        PointLabel::PROVISIONAL_GROUND :
        PointLabel::NONGROUND;
    }
  }
  // update state
  m_last_was_ground = label_is_ground(ret);
  m_prev_radius_m = radius_m;
  m_prev_height_m = height_m;
  if (m_last_was_ground) {
    m_prev_ground_radius_m = radius_m;
    m_prev_ground_height_m = height_m;
  }

  return ret;
}

////////////////////////////////////////////////////////////////////////////////
bool8_t RayGroundPointClassifier::label_is_ground(const RayGroundPointClassifier::PointLabel label)
{
  return static_cast<int8_t>(label) <= static_cast<int8_t>(0);
}

}  // namespace ray_ground_classifier
}  // namespace filters
}  // namespace perception
}  // namespace autoware
