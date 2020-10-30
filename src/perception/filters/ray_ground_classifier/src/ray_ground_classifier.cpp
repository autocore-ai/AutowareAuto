// Copyright 2017-2020 the Autoware Foundation, Arm Limited
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
#include <utility>
//lint -e537 NOLINT  // cpplint vs pclint
#include <algorithm>
#include <cstdint>
#include <stdexcept>

#include "common/types.hpp"
#include "ray_ground_classifier/ray_ground_classifier.hpp"
#include "ray_ground_classifier/ray_ground_point_classifier.hpp"

using autoware::common::types::bool8_t;
using autoware::common::types::float32_t;

namespace autoware
{
namespace perception
{
namespace filters
{
namespace ray_ground_classifier
{

////////////////////////////////////////////////////////////////////////////////
RayGroundClassifier::RayGroundClassifier(const Config & cfg)
: m_sort_array(autoware::common::types::POINT_BLOCK_CAPACITY),
  m_ray_sorter(autoware::common::types::POINT_BLOCK_CAPACITY),
  m_point_classifier(cfg)
{
  m_sort_array.clear();
}
////////////////////////////////////////////////////////////////////////////////
RayGroundClassifier::RayGroundClassifier(const RayGroundClassifier & original)
: m_sort_array(original.m_sort_array),
  m_ray_sorter(original.m_ray_sorter.capacity()),
  m_point_classifier(original.m_point_classifier)
{
  m_sort_array.clear();
}
////////////////////////////////////////////////////////////////////////////////
void RayGroundClassifier::insert(const PointXYZIF * pt)
{
  insert(PointXYZIFR{pt});
}
////////////////////////////////////////////////////////////////////////////////
void RayGroundClassifier::insert(const PointXYZIFR & pt)
{
  if (m_sort_array.size() >= m_sort_array.capacity()) {
    throw std::runtime_error("RayGroundClassifier: cannot insert into full array");
  }
  m_sort_array.push_back(pt);
}
////////////////////////////////////////////////////////////////////////////////
bool8_t RayGroundClassifier::can_fit_result(
  const Ray & ray,
  const PointPtrBlock & ground_block,
  const PointPtrBlock & nonground_block) const
{
  return (ray.size() + std::max(ground_block.size(), nonground_block.size())) <=
         autoware::common::types::POINT_BLOCK_CAPACITY;
}
////////////////////////////////////////////////////////////////////////////////
bool8_t RayGroundClassifier::can_fit_result(
  const PointPtrBlock & ground_block,
  const PointPtrBlock & nonground_block) const
{
  return can_fit_result(m_sort_array, ground_block, nonground_block);
}
////////////////////////////////////////////////////////////////////////////////
void RayGroundClassifier::partition(
  PointPtrBlock & ground_block,
  PointPtrBlock & nonground_block,
  const bool8_t presorted)
{
  if (!presorted) {
    sort_ray();
  }
  segment_ray(ground_block, nonground_block);
}
////////////////////////////////////////////////////////////////////////////////
void RayGroundClassifier::structured_partition(
  const PointPtrBlock & raw_block,
  PointPtrBlock & ground_block,
  PointPtrBlock & nonground_block)
{
  // reset blocks
  ground_block.clear();
  nonground_block.clear();
  // insert points and segment rays
  uint16_t last_id = raw_block[0U]->id;
  for (const PointXYZIF * pt : raw_block) {
    const uint16_t id = pt->id;
    // terminal scan, add to both
    if (static_cast<uint16_t>(autoware::common::types::PointXYZIF::END_OF_SCAN_ID) == id) {
      insert(ground_block, pt);
      insert(nonground_block, pt);
      break;
    }
    if (id != last_id) {
      // we have started a new ray; last ray is complete
      sort_ray();
      segment_ray(ground_block, nonground_block);
      // reset stuff
      last_id = id;
    }
    insert(pt);
  }
  // segment last ray
  sort_ray();
  segment_ray(ground_block, nonground_block);
}

////////////////////////////////////////////////////////////////////////////////
void RayGroundClassifier::sort_ray()
{
  // sort by radial distance
  m_ray_sorter.sort(m_sort_array.begin(), m_sort_array.end());
}
////////////////////////////////////////////////////////////////////////////////
void RayGroundClassifier::insert(PointPtrBlock & block, const PointXYZIF * pt)
{
  block.push_back(pt);
}
////////////////////////////////////////////////////////////////////////////////
void RayGroundClassifier::segment_ray(PointPtrBlock & ground_block, PointPtrBlock & nonground_block)
{
  partition(m_sort_array, ground_block, nonground_block);
  // reset internal array
  m_sort_array.clear();  // capacity unchanged
}
////////////////////////////////////////////////////////////////////////////////
void RayGroundClassifier::partition(
  const Ray & ray,
  PointPtrBlock & ground_block,
  PointPtrBlock & nonground_block)
{
  // Make sure result can fit
  if (!can_fit_result(ray, ground_block, nonground_block)) {
    throw std::runtime_error("RayGroundClassifier: Blocks cannot fit partition result");
  }
  // reset classifier
  m_point_classifier.reset();
  // filter and push to appropriate queues
  const PointXYZIF * last_point_ptr = nullptr;
  RayGroundPointClassifier::PointLabel last_label =
    RayGroundPointClassifier::PointLabel::NONGROUND;
  for (std::size_t idx = 0U; idx < ray.size(); ++idx) {
    const std::size_t jdx = idx;  // fixes PCLint FP: idx modified in loop
    const PointXYZIFR & pt = ray[jdx];
    const RayGroundPointClassifier::PointLabel label = m_point_classifier.is_ground(pt);
    // modify label of last point
    if (((label == RayGroundPointClassifier::PointLabel::NONGROUND) &&
      (last_label == RayGroundPointClassifier::PointLabel::PROVISIONAL_GROUND)) ||
      (label == RayGroundPointClassifier::PointLabel::RETRO_NONGROUND))
    {
      last_label = RayGroundPointClassifier::PointLabel::NONGROUND;
    }
    // push last point accordingly
    if (last_point_ptr != nullptr) {
      if (RayGroundPointClassifier::label_is_ground(last_label)) {
        insert(ground_block, last_point_ptr);
      } else {
        insert(nonground_block, last_point_ptr);
      }
    }
    // update state
    last_point_ptr = pt.get_point_pointer();
    last_label = label;
  }
  // push trailing point
  if (last_point_ptr != nullptr) {
    if (RayGroundPointClassifier::label_is_ground(last_label)) {
      insert(ground_block, last_point_ptr);
    } else {
      insert(nonground_block, last_point_ptr);
    }
  }
}

}  // namespace ray_ground_classifier
}  // namespace filters
}  // namespace perception
}  // namespace autoware
