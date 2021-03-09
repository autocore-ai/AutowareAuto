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
/// \brief This file defines the computational core of the ray ground filter algorithm

#ifndef RAY_GROUND_CLASSIFIER__RAY_GROUND_POINT_CLASSIFIER_HPP_
#define RAY_GROUND_CLASSIFIER__RAY_GROUND_POINT_CLASSIFIER_HPP_

#include <cmath>
#include <limits>
#include <vector>

#include "common/types.hpp"
#include "helper_functions/float_comparisons.hpp"
#include "ray_ground_classifier/visibility_control.hpp"

namespace autoware
{
/// \brief Perception related algorithms and functionality, such as those
///        acting on 3D lidar data, camera data, radar, or ultrasonic information
namespace perception
{
/// \brief Classifiers and operations that act to reduce or organize data. Currently
///        this namespace is strictly for point cloud filters, e.g. voxelgrid and ground filtering,
///        but in the future it may include filtering for images and other functionality
namespace filters
{
using autoware::common::types::PI;
using autoware::common::types::TAU;
using autoware::common::types::PointXYZIF;
using autoware::common::types::bool8_t;
using autoware::common::types::float32_t;
using autoware::common::types::float64_t;
using autoware::common::helper_functions::comparisons::abs_eq;

/// \brief Resources for the ray ground filter algorithm used for
///        ground filtering in point clouds
namespace ray_ground_classifier
{
/// \brief th_rad - phi_rad, normalized to +/- pi
/// \param[in] th_rad the reference angle
/// \param[in] phi_rad the test angle
/// \return angle from reference angle to test angle
inline float32_t angle_distance_rad(const float32_t th_rad, const float32_t phi_rad)
{
  return fmodf((th_rad - phi_rad) + PI + TAU, TAU) - PI;
}

inline float64_t deg2rad(float64_t degrees)
{
  return degrees * 4.0 * atan(1.0) / 180.0;
}

/// \brief A struct that holds configuration parameters for the ground filter
struct RAY_GROUND_CLASSIFIER_PUBLIC Config
{
  /// \brief Constructor
  /// \param[in] sensor_height_m How high the sensor is off the ground
  /// \param[in] max_local_slope_deg Maximum permissible slope for two ground points within
  ///                                reclass_threshold
  /// \param[in] max_global_slope_deg Maximum permissible slope from base footprint of sensor
  /// \param[in] nonground_retro_thresh_deg How steep consecutive points need to be to retroactively
  ///                                       annotate a point as nonground
  /// \param[in] min_height_thresh_m Local height threshold can be no less than this
  /// \param[in] max_global_height_thresh_m Global height threshold can be no more than this
  /// \param[in] max_last_local_ground_thresh_m Saturation threshold for locality wrt last ground
  ///                                           point (for classifying as ground from nonground)
  /// \param[in] max_provisional_ground_distance_m Max radial distance until provisional ground is
  ///                                              not influenced by next points
  /// throw std::runtime_error If any of the configuration parameters are inconsistent (e.g. angles
  ///                          are outside of (0, 90), min_* > max_*, etc.)
  Config(
    const float32_t sensor_height_m,
    const float32_t max_local_slope_deg,
    const float32_t max_global_slope_deg,
    const float32_t nonground_retro_thresh_deg,
    const float32_t min_height_thresh_m,
    const float32_t max_global_height_thresh_m,
    const float32_t max_last_local_ground_thresh_m,
    const float32_t max_provisional_ground_distance_m);
  /// \brief Get height of sensor off of ground, in meters
  /// \return The value in meters
  float32_t get_sensor_height() const;

  /// \brief Get z value (meters) of the ground in sensor frame, -sensor_height
  const float32_t m_ground_z_m;

  /// \brief Get maximum allowed slope between two nearby points that are both ground,
  /// the value is a nondimensionalized ratio, rise / run
  const float32_t m_max_local_slope;

  /// \brief Get maximum allowed slope between a ground point  and the sensor,
  /// the value is a nondimensionalized ratio, rise / run
  const float32_t m_max_global_slope;

  /// \brief Get minimum slope at which vertical structure is assumed,
  /// the value is a nondimensionalized ratio, rise / run
  const float32_t m_nonground_retro_thresh;

  /// \brief Get minimum value for local height threshold
  const float32_t m_min_height_thresh_m;

  /// \brief The maximum value for global height threshold
  const float32_t m_max_global_height_thresh_m;

  /// \brief The maximum value for local heigh threshold
  const float32_t m_max_last_local_ground_thresh_m;

  /// \brief The maximum influence distance for provisional ground point label
  const float32_t m_max_provisional_ground_distance_m;
};  // class Config

/// \brief This is a simplified point view of a ray. The only information needed is height and
///        projected radial distance from the sensor
class RAY_GROUND_CLASSIFIER_PUBLIC PointXYZIFR
{
  friend bool8_t operator<(const PointXYZIFR & lhs, const PointXYZIFR & rhs) noexcept;

public:
  /// \brief Default constructor
  PointXYZIFR() = default;
  /// \brief Conversion constructor
  /// \param[in] pt The point to convert into a 2D view
  explicit PointXYZIFR(const PointXYZIF * pt);
  /// \brief Getter for radius
  /// \return The projected radial distance
  float32_t get_r() const;
  /// \brief Getter for height
  /// \return The height of the point
  float32_t get_z() const;

  /// \brief Get address-of core point
  /// \return Pointer to internally stored point
  const PointXYZIF * get_point_pointer() const;

private:
  const PointXYZIF * m_point;
  float32_t m_r_xy;
};  // class PointXYZIFR

/// \brief Comparison operator for default sorting
/// \param[in] lhs Left hand side of comparison
/// \param[in] rhs Right hand side of comparison
/// \return True if lhs < rhs: if lhs.r < rhs.r, if nearly same radius then lhs.z < rhs.z
inline bool8_t operator<(const PointXYZIFR & lhs, const PointXYZIFR & rhs) noexcept
{
  const auto same_radius = abs_eq(
    lhs.m_r_xy, rhs.m_r_xy,
    std::numeric_limits<decltype(lhs.m_r_xy)>::epsilon());
  return same_radius ? (lhs.m_point->z < rhs.m_point->z) : (lhs.m_r_xy < rhs.m_r_xy);
}

using Ray = std::vector<PointXYZIFR>;

/// \brief Simple stateful implementation of ray ground filter:
///        https://github.com/CPFL/Autoware/blob/develop/ros/src/sensing/filters/packages/
///        points_preprocessor/nodes/ray_ground_ray_ground_filter/ray_ground_filter.cpp\#L187
class RAY_GROUND_CLASSIFIER_PUBLIC RayGroundPointClassifier
{
public:
  /// \brief Return codes from is_ground()
  enum class PointLabel : int8_t
  {
    /// \brief Point is ground
    GROUND = 0,
    /// \brief Point is nonground
    NONGROUND = 1,
    /// \brief Last point was nonground. This one is maybe ground. Label is
    /// solidified if terminal or next label is ground, otherwise nonground
    PROVISIONAL_GROUND = -1,
    /// \brief point is so vertical wrt last that last point is also nonground
    RETRO_NONGROUND = 2,
    /// \brief last was provisional ground, but this point is distant
    NONLOCAL_NONGROUND = 3
  };

  /// \brief Constructor
  /// \param[in] config The configuration struct for the filter
  /// \throw std::runtime_error if the configuration is invalid
  explicit RayGroundPointClassifier(const Config & config);

  /// \brief Copy constructor
  explicit RayGroundPointClassifier(const RayGroundPointClassifier & original);

  /// \brief Reinitializes state of filter, should be run before scanning through a ray
  void reset();

  /// \brief Decides if point is ground or not based on locality to last point, max local and
  ///        global slopes, dependent on and updates state
  /// \param[in] pt The point to be classified as ground or not ground
  /// \return PointLabel::GROUND if the point is classified as ground
  ///         PointLabel::NONGROUND if the point is classified as not ground
  ///         PointLabel::RETRO_NONGROUND if the point is classified as not ground and
  ///           the point is so vertical that the last point should also be ground
  /// \throw std::runtime_error if points are not received in order of increasing radius
  PointLabel is_ground(const PointXYZIFR & pt);

  /// \brief Whether a points label is abstractly ground or nonground
  /// \param[in] label the label to check whether or not its ground
  /// \return True if ground or provisionally ground, false if nonground or retro nonground
  static bool8_t label_is_ground(const PointLabel label);

private:
  /// 'state' member variables
  float32_t m_prev_radius_m;
  float32_t m_prev_height_m;
  float32_t m_prev_ground_radius_m;
  float32_t m_prev_ground_height_m;
  bool8_t m_last_was_ground;
  /// complete config struct
  const Config m_config;
};  // RayGroundPointClassifier
}  // namespace ray_ground_classifier
}  // namespace filters
}  // namespace perception
}  // namespace autoware

#endif  // RAY_GROUND_CLASSIFIER__RAY_GROUND_POINT_CLASSIFIER_HPP_
