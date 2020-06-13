// Copyright 2017-2019 Apex.AI, Inc.
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

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <stdexcept>

#include "common/types.hpp"
#include "lidar_utils/lidar_utils.hpp"
#include "ray_ground_classifier/ray_aggregator.hpp"
#include "ray_ground_classifier/ray_ground_point_classifier.hpp"

namespace autoware
{
namespace perception
{
namespace filters
{
namespace ray_ground_classifier
{

using autoware::common::types::FEPS;
using autoware::common::types::PI;
using autoware::common::types::POINT_BLOCK_CAPACITY;
using autoware::common::types::bool8_t;
using autoware::common::types::float32_t;

////////////////////////////////////////////////////////////////////////////////
RayAggregator::Config::Config(
  const float32_t min_ray_angle_rad,
  const float32_t max_ray_angle_rad,
  const float32_t ray_width_rad,
  const std::size_t min_ray_points)
: m_min_ray_points(min_ray_points),
  m_num_rays(),
  m_ray_width_rad(ray_width_rad),
  m_min_angle_rad(min_ray_angle_rad),
  m_domain_crosses_180(max_ray_angle_rad < min_ray_angle_rad)
{
  if (m_domain_crosses_180) {
    const float32_t angle_range = (PI - min_ray_angle_rad) +
      (PI + max_ray_angle_rad);
    m_num_rays = static_cast<std::size_t>(std::ceil(angle_range / ray_width_rad));
  } else {
    m_num_rays = static_cast<std::size_t>(std::ceil(
        (max_ray_angle_rad - min_ray_angle_rad) / ray_width_rad));
  }
  if (ray_width_rad < FEPS) {
    throw std::runtime_error("Ray width negative or infinitesimally small");
  }
  if (min_ray_points > static_cast<std::size_t>(POINT_BLOCK_CAPACITY)) {
    throw std::runtime_error("Min ray points larger than point block capacity, consider reducing");
  }
  // TODO(c.ho) upper limit on number of rays?
}
////////////////////////////////////////////////////////////////////////////////
std::size_t RayAggregator::Config::get_min_ray_points() const
{
  return m_min_ray_points;
}
////////////////////////////////////////////////////////////////////////////////
std::size_t RayAggregator::Config::get_num_rays() const
{
  return m_num_rays;
}
////////////////////////////////////////////////////////////////////////////////
float32_t RayAggregator::Config::get_min_angle() const
{
  return m_min_angle_rad;
}
////////////////////////////////////////////////////////////////////////////////
float32_t RayAggregator::Config::get_ray_width() const
{
  return m_ray_width_rad;
}
////////////////////////////////////////////////////////////////////////////////
bool8_t RayAggregator::Config::domain_crosses_180() const
{
  return m_domain_crosses_180;
}
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
RayAggregator::RayAggregator(const Config & cfg)
: m_cfg(cfg),
  m_rays(m_cfg.get_num_rays()),
  m_ready_indices(m_cfg.get_num_rays()),
  m_ready_start_idx{},  // zero initialization
  m_num_ready{},  // zero initialization
  m_ray_state(m_cfg.get_num_rays())
{
  m_rays.clear();  // capacity unchanged
  const std::size_t ray_size =
    std::max(m_cfg.get_min_ray_points(), static_cast<std::size_t>(POINT_BLOCK_CAPACITY));
  m_ray_sorter.reserve(ray_size);
  for (std::size_t idx = 0U; idx < m_cfg.get_num_rays(); ++idx) {
    m_rays.emplace_back(ray_size);
    m_rays.back().clear();
    m_ray_state.push_back(RayState::NOT_READY);
  }
  m_ready_indices.resize(m_ready_indices.capacity());
}
////////////////////////////////////////////////////////////////////////////////
void RayAggregator::insert(const PointXYZIFR & pt)
{
  if (static_cast<uint16_t>(PointXYZIF::END_OF_SCAN_ID) == pt.get_point_pointer()->id) {
    // all rays ready
    m_ready_start_idx = 0U;
    m_num_ready = 0U;
    for (std::size_t idx = 0U; idx < m_rays.size(); ++idx) {
      const std::size_t jdx = idx;  // Fix for MISRA Fp, idx modified in loop
      // Add all non empty "NOT_READY" rays to the ready list since the end of scan in reached.
      if (RayState::RESET != m_ray_state[jdx]) {
        if (!m_rays[jdx].empty()) {
          m_ready_indices[m_num_ready] = idx;
          ++m_num_ready;
        }
      }
    }
  } else {
    const std::size_t idx = bin(pt);
    Ray & ray = m_rays[idx];
    if (RayState::RESET == m_ray_state[idx]) {
      ray.clear();  // capacity unchanged
      m_ray_state[idx] = RayState::NOT_READY;
    }
    if (ray.size() >= ray.capacity()) {
      throw std::runtime_error("RayAggregator: Ray capacity overrun! Use smaller bins");
    }
    // insert point to ray, do some presorting
    ray.push_back(pt);
    // TODO(c.ho) get push_heap working to amortize sorting burden
    // check if ray is ready
    if ((RayState::READY != m_ray_state[idx]) && (m_cfg.get_min_ray_points() <= ray.size())) {
      m_ray_state[idx] = RayState::READY;
      // "push" to ring buffer
      const std::size_t jdx = (m_ready_start_idx + m_num_ready) % m_ready_indices.size();
      m_ready_indices[jdx] = idx;
      ++m_num_ready;
      // TODO(c.ho) bounds check?
    }
  }
}
////////////////////////////////////////////////////////////////////////////////
void RayAggregator::insert(const PointXYZIF & pt)
{
  insert(PointXYZIFR{pt});
}
////////////////////////////////////////////////////////////////////////////////
void RayAggregator::insert(const PointBlock & blk)
{
  for (const PointXYZIF & pt : blk) {
    insert(pt);
    if (static_cast<uint16_t>(PointXYZIF::END_OF_SCAN_ID) == pt.id) {
      break;
    }
  }
}
////////////////////////////////////////////////////////////////////////////////
bool8_t RayAggregator::is_ray_ready() const
{
  return m_num_ready != 0U;
}
////////////////////////////////////////////////////////////////////////////////
const Ray & RayAggregator::get_next_ray()
{
  if (0U == m_num_ready) {
    throw std::runtime_error("RayAggregator: no rays ready");
  }
  const std::size_t idx = m_ready_indices[m_ready_start_idx];
  Ray & ret = m_rays[idx];
  // Sort ray
  m_ray_sorter.sort(ret.begin(), ret.end());
  // ready to be reset on next insertion to this item
  m_ray_state[idx] = RayState::RESET;
  // "pop" from ring buffer
  m_ready_start_idx = (m_ready_start_idx + 1U) % m_ready_indices.size();
  --m_num_ready;
  return ret;
}

////////////////////////////////////////////////////////////////////////////////
std::size_t RayAggregator::bin(const PointXYZIFR & pt) const
{
  const float32_t x = pt.get_point_pointer()->x;
  const float32_t y = pt.get_point_pointer()->y;
  // (0, 0) is always bin 0
  float32_t idx = 0.0F;
  if ((fabsf(x) > autoware::common::types::FEPS) ||
    (fabsf(y) > autoware::common::types::FEPS))
  {
    const float32_t th = autoware::common::lidar_utils::fast_atan2(y, x);
    idx = th - m_cfg.get_min_angle();
    if (m_cfg.domain_crosses_180() && (idx < 0.0F)) {
      // Case where receptive field crosses the +PI/-PI singularity
      // [-PI, max_angle) domain
      idx = idx + autoware::common::types::TAU;
    }
    // Avoid underflow
    idx = std::max(0.0F, idx);
  }
  // [min_angle, +PI) domain: normal calculation
  // normal case, no wraparound
  idx = std::floor(idx / m_cfg.get_ray_width());
  return static_cast<std::size_t>(idx);
}
}  // namespace ray_ground_classifier
}  // namespace filters
}  // namespace perception
}  // namespace autoware
