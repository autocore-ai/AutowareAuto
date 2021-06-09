// Copyright 2021 Apex.AI, Inc.
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

#include <motion_model/linear_motion_model.hpp>

#include <common/types.hpp>

namespace
{
using autoware::common::types::float32_t;
using autoware::common::types::float64_t;

template<typename ScalarT>
Eigen::Matrix<ScalarT, 3, 3> create_single_variable_block(const std::chrono::nanoseconds & dt)
{
  const auto t = std::chrono::duration<float64_t>{dt}.count();
  const auto t2 = t * t;
  return (Eigen::Matrix3d{} <<
         1.0, t, 0.5 * t2,
         0.0, 1.0, t,
         0.0, 0.0, 1.0).finished().cast<ScalarT>();
}

template<typename ScalarT, int size>
Eigen::Matrix<ScalarT, size, size> create_jacobian(const std::chrono::nanoseconds & dt)
{
  // For now we just create a new matrix on every call. If this is too slow, we can create a new
  // class that caches a certain number of results.
  const Eigen::Matrix<ScalarT, 3, 3> single_variable_block{
    create_single_variable_block<ScalarT>(dt)};
  Eigen::Matrix<ScalarT, size, size> m{Eigen::Matrix<ScalarT, size, size>::Zero()};
  for (int i = 0; i < size; i += 3) {
    m.template block<3, 3>(i, i) = single_variable_block;
  }
  return m;
}

}  // namespace

namespace autoware
{
namespace common
{
namespace motion_model
{

template<typename StateT>
typename StateT::Matrix
LinearMotionModel<StateT>::crtp_jacobian(
  const State &, const std::chrono::nanoseconds & dt) const
{
  return create_jacobian<typename State::Scalar, State::size()>(dt);
}

/// \cond DO_NOT_DOCUMENT

template class MOTION_MODEL_PUBLIC LinearMotionModel<state_vector::ConstAccelerationXY32>;
template class MOTION_MODEL_PUBLIC LinearMotionModel<state_vector::ConstAccelerationXY64>;

template class MOTION_MODEL_PUBLIC LinearMotionModel<state_vector::ConstAccelerationXYZ32>;
template class MOTION_MODEL_PUBLIC LinearMotionModel<state_vector::ConstAccelerationXYZ64>;

template class MOTION_MODEL_PUBLIC LinearMotionModel<state_vector::ConstAccelerationXYYaw32>;
template class MOTION_MODEL_PUBLIC LinearMotionModel<state_vector::ConstAccelerationXYYaw64>;

template class MOTION_MODEL_PUBLIC LinearMotionModel<state_vector::ConstAccelerationXYZYaw32>;
template class MOTION_MODEL_PUBLIC LinearMotionModel<state_vector::ConstAccelerationXYZYaw64>;

template class MOTION_MODEL_PUBLIC LinearMotionModel<state_vector::ConstAccelerationXYZRPY32>;
template class MOTION_MODEL_PUBLIC LinearMotionModel<state_vector::ConstAccelerationXYZRPY64>;

/// \endcond

}  // namespace motion_model
}  // namespace common
}  // namespace autoware
