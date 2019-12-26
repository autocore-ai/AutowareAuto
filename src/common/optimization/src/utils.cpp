// Copyright 2019 Apex.AI, Inc.
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

#include <optimization/utils.hpp>

namespace autoware
{
namespace common
{
namespace optimization
{
ComputeMode::ComputeMode(bool score, bool jacobian, bool hessian)
: m_score(score), m_jacobian(jacobian), m_hessian(hessian) {}

ComputeMode & ComputeMode::set_score() noexcept
{
  m_score = true;
  return *this;
}
ComputeMode & ComputeMode::set_jacobian() noexcept
{
  m_jacobian = true;
  return *this;
}
ComputeMode & ComputeMode::set_hessian() noexcept
{
  m_hessian = true;
  return *this;
}

bool ComputeMode::score() const noexcept {return m_score;}
bool ComputeMode::jacobian() const noexcept {return m_jacobian;}
bool ComputeMode::hessian() const noexcept {return m_hessian;}
bool ComputeMode::operator==(const ComputeMode & other) const noexcept
{
  return (m_score == other.score()) &&
         (m_jacobian == other.jacobian()) &&
         (m_hessian == other.hessian());
}
bool ComputeMode::operator!=(const ComputeMode & other) const noexcept
{
  return !(*this == other);
}

}  // namespace optimization
}  // namespace common
}  // namespace autoware
