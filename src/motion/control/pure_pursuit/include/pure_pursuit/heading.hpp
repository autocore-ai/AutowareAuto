// Copyright 2019 Christopher Ho
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

/// Note this code is copy-pasted directly from christopher.ho's personal repository:
/// https://gitlab.com/aninnymouse/mpc/blob/master/common/motion_common/src/motion_common/
#ifndef PURE_PURSUIT__HEADING_HPP_
#define PURE_PURSUIT__HEADING_HPP_

#include <pure_pursuit/visibility_control.hpp>
#include <autoware_auto_msgs/msg/complex32.hpp>
#include <builtin_interfaces/msg/duration.hpp>
#include <builtin_interfaces/msg/time.hpp>

#include <chrono>

namespace autoware
{
namespace motion
{
namespace control
{
/// \brief Resources relating to the pure pursuit package
namespace pure_pursuit
{
/// Addition for complex types with quaternion semantics
/// \param[in] a The first element to add
/// \param[in] b The second element to add
/// \return a + b with quaternion semantics
PURE_PURSUIT_PUBLIC autoware_auto_msgs::msg::Complex32 angle_addition(
  const autoware_auto_msgs::msg::Complex32 a,
  const autoware_auto_msgs::msg::Complex32 b) noexcept;

/// Different between two 2D quaternions
/// \param[in] a The first element to be subtracted from
/// \param[in] b The second element to subtract
/// \return a - b with quaternion semantics
PURE_PURSUIT_PUBLIC autoware_auto_msgs::msg::Complex32 angle_difference(
  const autoware_auto_msgs::msg::Complex32 a,
  autoware_auto_msgs::msg::Complex32 b) noexcept;

/// Convert 2D quaternion to a single scalar value
/// \param[in] heading The complex/2D quaternion to convert to an angle
/// \return An angle in range [-Pi, Pi]
PURE_PURSUIT_PUBLIC decltype(autoware_auto_msgs::msg::Complex32::real)
to_angle(autoware_auto_msgs::msg::Complex32 heading) noexcept;

/// Convert single scalar angle to 2D quaternion
/// \param[in] angle An angle in radians to convert to 2D quaternion
/// \return A complex number with 2D quaternion semantics
PURE_PURSUIT_PUBLIC autoware_auto_msgs::msg::Complex32 from_angle(const float angle) noexcept;

/// A simple struct to hold the result of efficient sin/cos calculations from a 2D quaternion
struct PURE_PURSUIT_PUBLIC TrigValue
{
  /// The sine value of the angle
  decltype(autoware_auto_msgs::msg::Complex32::real) sin_value;
  /// The cosine value of the angle
  decltype(autoware_auto_msgs::msg::Complex32::real) cos_value;
};

/// Get sine and cosine from 2D quaternion efficiently
/// \param[in] heading A 2D quaternion to from which to compute sine and cosine
/// \return Packed sine and cosine values
PURE_PURSUIT_PUBLIC TrigValue sin_cos(const autoware_auto_msgs::msg::Complex32 heading) noexcept;
}  // namespace pure_pursuit
}  // namespace control
}  // namespace motion
}  // namespace autoware

#endif  // PURE_PURSUIT__HEADING_HPP_
