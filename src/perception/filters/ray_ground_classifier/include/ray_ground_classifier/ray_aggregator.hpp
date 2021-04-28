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

/// \file
/// \brief This file defines the ray aggregator for generic point cloud support

#ifndef RAY_GROUND_CLASSIFIER__RAY_AGGREGATOR_HPP_
#define RAY_GROUND_CLASSIFIER__RAY_AGGREGATOR_HPP_


#include <algorithm>
#include <cmath>
#include <complex>
#include <cstdint>
#include <vector>

#include "autoware_auto_algorithm/algorithm.hpp"
#include "common/types.hpp"
#include "ray_ground_classifier/ray_ground_point_classifier.hpp"

namespace autoware
{
namespace perception
{
namespace filters
{
namespace ray_ground_classifier
{

using autoware::common::types::PointPtrBlock;
using autoware::common::types::bool8_t;
using autoware::common::types::float32_t;

/// \brief Used as a prefiltering step for RayGroundClassifier. Aggregates unstructured
///        blobs into rays of points that the RayGroundClassifier can partition.
class RAY_GROUND_CLASSIFIER_PUBLIC RayAggregator
{
public:
  /// \brief Configuration object for RayAggregator
  class RAY_GROUND_CLASSIFIER_PUBLIC Config
  {
public:
    /// \brief Constructor
    /// \param[in] min_ray_angle_rad Minimum ray angle
    /// \param[in] max_ray_angle_rad Maximum ray angle
    /// \param[in] ray_width_rad Width of ray, defines number of rays
    /// \param[in] min_ray_points Number of points needed in a ray before it's ready for
    ///                           partitioning
    Config(
      const float32_t min_ray_angle_rad,
      const float32_t max_ray_angle_rad,
      const float32_t ray_width_rad,
      const std::size_t min_ray_points);

    /// \brief Get number of rays
    /// \return Value
    std::size_t get_num_rays() const;
    /// \brief Get number of points needed for a ray to be ready for partitioning
    /// \return Value
    std::size_t get_min_ray_points() const;
    /// \brief Get minimum ray angle
    /// \return Value
    float32_t get_min_angle() const;
    /// \brief Get maximum ray angle
    /// \return Value
    float32_t get_ray_width() const;
    /// \brief Whether domain crosses the -PI/+PI singularity, e.g. min_ray_angle=300,
    ///        max_ray_angle = -300
    /// \return Value
    bool8_t domain_crosses_180() const;

private:
    const std::size_t m_min_ray_points;
    std::size_t m_num_rays;
    const float32_t m_ray_width_rad;
    const float32_t m_min_angle_rad;
    const bool8_t m_domain_crosses_180;
  };  // class Config

  /// \brief Constructor
  /// \param[in] cfg Configuration class
  explicit RayAggregator(const Config & cfg);
  /// \brief Ready all the non empty rays. To be called once all insertions are done.
  /// not thread safe
  void end_of_scan();
  /// \brief Insert point into set of rays. Concurrent inserts are safe
  /// \param[in] pt Point to be inserted
  /// \return true if the insertion suceeded and false if an end of scan is detected
  bool8_t insert(const PointXYZIFR & pt);
  /// \brief Insert point into set of rays. Concurrent inserts are safe
  /// \param[in] pt Point to be inserted
  /// \return true if the insertion suceeded and false if an end of scan is detected
  bool8_t insert(const PointXYZIF * pt);
  /// \brief Insert points associated with blk into the ray set. Concurrent inserts are safe
  /// \param[in] blk Block of points to be added
  /// \return true if the insertion suceeded and false if an end of scan is detected
  bool8_t insert(const PointPtrBlock & blk);
  /// \brief Insert points from an iterator. Concurrent inserts are safe
  /// \param[in] first Beginning of iterator
  /// \param[in] last One past the last element of the iterator
  /// \return true if the insertion suceeded and false if an end of scan is detected
  bool8_t insert(const PointXYZIF * first, const PointXYZIF * last)
  {
    bool8_t ret = true;
    for (const PointXYZIF * it = first; it != last; ++it) {
      ret = ret && insert(it);
      if (!ret) {
        break;
      }
    }
    return ret;
  }
  bool8_t insert(PointXYZIF * first, PointXYZIF * last)
  {
    bool8_t ret = true;
    for (PointXYZIF * it = first; it != last; ++it) {
      ret = ret && insert(it);
      if (!ret) {
        break;
      }
    }
    return ret;
  }
  template<typename InputIT>
  bool8_t insert(InputIT first, InputIT last)
  {
    static_assert(
      std::is_pointer<InputIT>::value,
      "insert(first,last) must be called on an iterable pointer");
    bool8_t ret = true;
    for (InputIT it = first; it != last; ++it) {
      ret = ret && insert(*it);
      if (!ret) {
        break;
      }
    }
    return ret;
  }

  /// \brief Whether a ray is ready for processing
  /// \return Value
  bool8_t is_ray_ready() const;
  /// \brief How many rays are ready for processing
  /// \return Value
  std::size_t get_ready_ray_count() const;
  /// \brief Get next ray that is ready for partitioning
  /// Concurrent calls are thread safe
  /// \return Const reference to next ray ready for processing
  /// \throw std::runtime_error If no ray is ready
  const Ray & get_next_ray();
  /// \brief Clear all the ready rays so that is_ray_ready() return false
  void reset();

private:
  enum class RayState : uint8_t
  {
    /// \brief Ray does not have enough points to be classified
    NOT_READY = 0U,
    /// \brief Ray has enough points to be classified
    READY,
    /// \brief Ray has been read via get_next_ray(), and should be reset on next insert
    RESET
  };  // enum class RayState

  /// \brief Compute which bin a point belongs to
  inline std::size_t RAY_GROUND_CLASSIFIER_LOCAL bin(const PointXYZIFR & pt) const;
  const Config m_cfg;
  std::vector<Ray> m_rays;
  // simple index ring buffer
  std::vector<std::size_t> m_ready_indices;
  std::size_t m_ready_start_idx;
  std::size_t m_num_ready;

  // which rays are ready to be reset etc. TODO(c.ho) fold this into an internal ray class
  std::vector<RayState> m_ray_state;
};  // class RayAggregator
}  // namespace ray_ground_classifier
}  // namespace filters
}  // namespace perception
}  // namespace autoware

#endif  // RAY_GROUND_CLASSIFIER__RAY_AGGREGATOR_HPP_
