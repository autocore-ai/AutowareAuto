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

#include "pure_pursuit/heading.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace autoware
{
namespace motion
{
namespace control
{
/// \brief Resources relating to the pure pursuit package
namespace pure_pursuit
{

using Real = decltype(autoware_auto_msgs::msg::Complex32::real);

/// Addition for complex types with quaternion semantics
autoware_auto_msgs::msg::Complex32 angle_addition(
  const autoware_auto_msgs::msg::Complex32 a,
  const autoware_auto_msgs::msg::Complex32 b) noexcept
{
  // Could technically use none, but I get basically nothing from that
  autoware_auto_msgs::msg::Complex32 ret{rosidl_generator_cpp::MessageInitialization::ALL};
  ret.real = (a.real * b.real) - (a.imag * b.imag);
  ret.imag = (a.real * b.imag) + (a.imag * b.real);
  return ret;
}

/// Different between two 2D quaternions
autoware_auto_msgs::msg::Complex32 angle_difference(
  const autoware_auto_msgs::msg::Complex32 a,
  autoware_auto_msgs::msg::Complex32 b) noexcept
{
  b.real = -b.real;
  return angle_addition(a, b);
}

/// Convert 2D quaternion to a single scalar value
float
to_angle(autoware_auto_msgs::msg::Complex32 heading) noexcept
{
  const auto mag2 = (heading.real * heading.real) + (heading.imag * heading.imag);
  if (std::abs(mag2 - 1.0F) > std::numeric_limits<Real>::epsilon()) {
    const auto imag = Real{1.0F} / sqrtf(mag2);
    heading.real *= imag;
    heading.imag *= imag;
    // Don't need to touch imaginary/z part
  }
  // See:
  // https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles
  const auto y = Real{2.0F} *heading.real * heading.imag;
  const auto x = Real{1.0F} - (Real{2.0F} *heading.imag * heading.imag);
  // TODO(c.ho) fast atan2
  return atan2f(y, x);
}

autoware_auto_msgs::msg::Complex32 from_angle(const float angle) noexcept
{
  const auto th = 0.5F * angle;
  autoware_auto_msgs::msg::Complex32 ret{};
  ret.real = cosf(th);
  ret.imag = sinf(th);
  return ret;
}

/// Get sine and cosine from 2D quaternion efficiently
TrigValue sin_cos(const autoware_auto_msgs::msg::Complex32 heading) noexcept
{
  return TrigValue{Real{2.0F} *heading.real * heading.imag,
    (heading.real + heading.imag) * (heading.real - heading.imag)};
}

}  // namespace pure_pursuit
}  // namespace control
}  // namespace motion
}  // namespace autoware
