// Copyright 2018 Apex.AI, Inc.
// Co-developed by Tier IV, Inc. and Apex.AI, Inc.
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

/// \copyright Copyright 2018 Apex.AI, Inc.
/// All rights reserved.
#include <common/types.hpp>
#include "motion_model/catr_model.hpp"

using autoware::common::types::float32_t;

namespace autoware
{
namespace motion
{
namespace motion_model
{

///
CatrModel & CatrModel::operator=(const CatrModel & rhs)
{
  if (this != &rhs) {
    m_state = rhs.m_state;
  }
  return *this;
}
///
void CatrModel::predict(
  Eigen::Matrix<float32_t, 6U, 1U> & x,
  const std::chrono::nanoseconds & dt) const
{
  const float32_t Dt = static_cast<float32_t>(dt.count()) / 1000000000LL;
  catr_workspace_init_variant(
    m_state,
    Dt,
    m_invariants,
    m_variants);
  catr_predict(m_state, m_invariants, m_variants, x);
}

///
void CatrModel::predict(const std::chrono::nanoseconds & dt)
{
  predict(m_state, dt);
  // m_state has been updated, recompute invariants
  catr_workspace_init_invariant(m_state, m_invariants);
}

///
void CatrModel::compute_jacobian(
  Eigen::Matrix<float32_t, 6U, 6U> & F,
  const std::chrono::nanoseconds & dt)
{
  const float32_t Dt = static_cast<float32_t>(dt.count()) / 1000000000LL;
  catr_workspace_init_variant(
    m_state,
    Dt,
    m_invariants,
    m_variants);
  catr_compute_jacobian(m_invariants, m_variants, F);
}

///
void CatrModel::compute_jacobian_and_predict(
  Eigen::Matrix<float32_t, 6U, 6U> & F,
  const std::chrono::nanoseconds & dt)
{
  const float32_t Dt = static_cast<float32_t>(dt.count()) / 1000000000LL;
  catr_workspace_init_variant(
    m_state,
    Dt,
    m_invariants,
    m_variants);
  catr_compute_jacobian(m_invariants, m_variants, F);
  catr_predict(m_state, m_invariants, m_variants, m_state);
  // m_state has been updated, recompute invariants
  catr_workspace_init_invariant(m_state, m_invariants);
}

///
float CatrModel::operator[](const index_t idx) const {return m_state(idx);}

///
void CatrModel::reset(const Eigen::Matrix<float32_t, 6U, 1U> & x)
{
  m_state = x;
  // m_state has been updated, recompute invariants
  catr_workspace_init_invariant(m_state, m_invariants);
}
///
const Eigen::Matrix<float32_t, 6U, 1U> & CatrModel::get_state() const
{
  return m_state;
}
}  // namespace motion_model
}  // namespace motion
}  // namespace autoware
