// Copyright 2021 the Autoware Foundation
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
// Developed by Apex.AI, Inc.

/// \copyright Copyright 2021 the Autoware Foundation
/// All rights reserved.
/// \file
/// \brief Implements the linear motion model.

#include <motion_model/linear_motion_model.hpp>

namespace
{
Eigen::Matrix3f create_single_variable_block(const std::chrono::nanoseconds & dt)
{
  const auto t = std::chrono::duration<float>{dt}.count();
  const auto t2 = t * t;
  return (Eigen::Matrix3f{} <<
         1.0F, t, 0.5F * t2,
         0.0F, 1.0F, t,
         0.0F, 0.0F, 1.0F).finished();
}
}  // namespace

namespace autoware
{
namespace prediction
{

template<>
state::ConstAccelerationXYYaw::Matrix
LinearMotionModel<state::ConstAccelerationXYYaw>::crtp_jacobian(
  const State &, const std::chrono::nanoseconds & dt) const
{
  const Eigen::Matrix3f single_variable_block{create_single_variable_block(dt)};
  State::Matrix m{State::Matrix::Zero()};
  m.block<3, 3>(0, 0) = single_variable_block;
  m.block<3, 3>(3, 3) = single_variable_block;
  m.block<3, 3>(6, 6) = single_variable_block;
  return m;
}

template<>
state::ConstAccelerationXY::Matrix
LinearMotionModel<state::ConstAccelerationXY>::crtp_jacobian(
  const State &, const std::chrono::nanoseconds & dt) const
{
  const Eigen::Matrix3f single_variable_block{create_single_variable_block(dt)};
  State::Matrix m{State::Matrix::Zero()};
  m.block<3, 3>(0, 0) = single_variable_block;
  m.block<3, 3>(3, 3) = single_variable_block;
  return m;
}

}  // namespace prediction
}  // namespace autoware
