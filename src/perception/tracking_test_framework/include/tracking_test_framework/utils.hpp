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

#ifndef TRACKING_TEST_FRAMEWORK__UTILS_HPP_
#define TRACKING_TEST_FRAMEWORK__UTILS_HPP_

#include <helper_functions/float_comparisons.hpp>
#include <vector>

namespace comparison = autoware::common::helper_functions::comparisons;

namespace autoware
{
namespace tracking_test_framework
{
namespace utils
{
/// \brief Method to convert angle in degrees to radians
/// \param[in] orientation_deg in degrees
/// \return returns the angle converted to radians
inline autoware::common::types::float32_t to_radians(
  const autoware::common::types::float32_t orientation_deg)
{
  constexpr auto deg_in_PI = 180.0F;
  return orientation_deg * (autoware::common::types::PI / deg_in_PI);
}
/// \brief Method to convert angle in radians to degrees
/// \param[in] orientation_rad in radians
/// \return returns the angle converted to degrees
inline autoware::common::types::float32_t to_degrees(
  const autoware::common::types::float32_t orientation_rad)
{
  constexpr auto deg_in_PI = 180.0F;
  return orientation_rad * (deg_in_PI / autoware::common::types::PI);
}
/// \brief Method to compute cross product of two Eigen::Vector2fs
/// \param[in] vec1 first 2D Eigen::Vector2f
/// \param[in] vec2 second 2D Eigen::Vector2f
/// \return returns the cross product
inline autoware::common::types::float32_t cross_2d(
  const Eigen::Vector2f & vec1, const Eigen::Vector2f & vec2)
{
  return (vec1.x() * vec2.y()) - (vec1.y() * vec2.x());
}
/// \brief Method to compute linearly spaced intervals between a given range
/// \param[in] start start number of the range
/// \param[in] end end number of the range
/// \param[in] num number of intervals needed
/// \return returns the std::vector<T> of linearly spaced intervals including start and end numbers
template<typename T>
std::vector<T> linspace(const T start, const T end, const size_t num)
{
  if (num == 0) {
    return std::vector<T>{};
  } else if (num == 1) {
    return std::vector<T>{start};
  }
  std::vector<T> linspaced(num);
  size_t i = 0;
  T delta = (end - start) / static_cast<T>(num - 1);
  std::generate(
    linspaced.begin(), linspaced.end(), [&]() {
      T result{};
      result = start + delta * static_cast<T>(i);
      i++;
      return result;
    });
  return linspaced;
}

/// \brief Method to wrap angle between 0 to 2*PI
/// \param[in] angle_rad angle in radians
/// \return returns the angle scaled between 0 to 2*PI in radians
inline autoware::common::types::float32_t wrap_to_2pi(
  const autoware::common::types::float32_t angle_rad)
{
  autoware::common::types::float32_t angle_deg = to_degrees(angle_rad);
  angle_deg = static_cast<autoware::common::types::float32_t>(fmod(angle_deg, 360.F));
  if (comparison::abs_lt<autoware::common::types::float32_t>(angle_deg, 0.0F, 0.0001F)) {
    angle_deg += 360.0F;
  }
  return to_radians(angle_deg);
}

/// \brief Method to convert point in Eigen::Vector2f to autoware::common::types::PointXYZIF
/// \param[in] v point(x,y) in Eigen::Vector2f format
/// \return returns the autoware::common::types::PointXYZIF representation of point(x,y)
inline autoware::common::types::PointXYZIF get_point_from_vector_2d(const Eigen::Vector2f & v)
{
  autoware::common::types::PointXYZIF point{};
  point.x = static_cast<autoware::common::types::float32_t>(v(0));
  point.y = static_cast<autoware::common::types::float32_t>(v(1));
  return point;
}

}  // namespace utils
}  // namespace tracking_test_framework
}  // namespace autoware

#endif  // TRACKING_TEST_FRAMEWORK__UTILS_HPP_
