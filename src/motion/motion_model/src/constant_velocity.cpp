// Copyright 2018 Apex.AI, Inc.
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

/// \copyright Copyright 2018 Apex.AI, Inc.
/// All rights reserved.

#include <common/types.hpp>
#include "motion_model/constant_velocity.hpp"

using autoware::common::types::float32_t;

namespace autoware
{
namespace motion
{
namespace motion_model
{
///
ConstantVelocity & ConstantVelocity::operator=(const ConstantVelocity & rhs)
{
  if (this != &rhs) {
    m_state = rhs.m_state;
  }
  return *this;
}
///
void ConstantVelocity::predict(
  Eigen::Matrix<float32_t, 4U, 1U> & x,
  const std::chrono::nanoseconds & dt) const
{
  const float32_t dt_s = static_cast<float32_t>(dt.count()) / 1000000000LL;
  x(States::POSE_X) = m_state(States::POSE_X) + (dt_s * m_state(States::VELOCITY_X));
  x(States::POSE_Y) = m_state(States::POSE_Y) + (dt_s * m_state(States::VELOCITY_Y));
  x(States::VELOCITY_X) = m_state(States::VELOCITY_X);
  x(States::VELOCITY_Y) = m_state(States::VELOCITY_Y);
}

///
void ConstantVelocity::predict(const std::chrono::nanoseconds & dt)
{
  predict(m_state, dt);
}

///
void ConstantVelocity::compute_jacobian(
  Eigen::Matrix<float32_t, 4U, 4U> & F,
  const std::chrono::nanoseconds & dt)
{
  const float32_t dt_s = static_cast<float32_t>(dt.count()) / 1000000000LL;
  // identity matrix
  F.setIdentity();
  // only nonzero elements are ones along diagonal + constant terms for velocity
  F(States::POSE_X, States::VELOCITY_X) = dt_s;
  F(States::POSE_Y, States::VELOCITY_Y) = dt_s;
}

///
void ConstantVelocity::compute_jacobian_and_predict(
  Eigen::Matrix<float32_t, 4U, 4U> & F,
  const std::chrono::nanoseconds & dt)
{
  compute_jacobian(F, dt);
  predict(dt);
}
///
float ConstantVelocity::operator[](const index_t idx) const {return m_state(idx);}
///
void ConstantVelocity::reset(const Eigen::Matrix<float32_t, 4U, 1U> & x)
{
  m_state = x;
}
///
const Eigen::Matrix<float32_t, 4U, 1U> & ConstantVelocity::get_state() const
{
  return m_state;
}
}  // namespace motion_model
}  // namespace motion
}  // namespace autoware
