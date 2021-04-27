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

#include <state_estimation/noise_model/wiener_noise.hpp>

namespace
{
Eigen::Matrix3f create_single_variable_block(const std::chrono::nanoseconds & dt)
{
  const auto float_seconds = std::chrono::duration<float>{dt}.count();
  const auto float_seconds_2 = float_seconds * float_seconds;
  const Eigen::Vector3f noise_gain{0.5F * float_seconds_2, float_seconds, 1.0F};
  return noise_gain * noise_gain.transpose();
}
}  // namespace

namespace autoware
{
namespace common
{
namespace state_estimation
{


template<>
common::state_vector::ConstAccelerationXYYaw::Matrix
WienerNoise<common::state_vector::ConstAccelerationXYYaw>::crtp_covariance(
  const std::chrono::nanoseconds & dt) const
{
  if (m_acceleration_variances.size() != 3U) {
    std::runtime_error(
      "Wrong initialization of noise model. Check the number of provided variances.");
  }
  const auto block_matrix = create_single_variable_block(dt);
  common::state_vector::ConstAccelerationXYYaw::Matrix m{
    common::state_vector::ConstAccelerationXYYaw::Matrix::Zero()};
  m.block<3, 3>(0, 0) = block_matrix * m_acceleration_variances[0U] * m_acceleration_variances[0U];
  m.block<3, 3>(3, 3) = block_matrix * m_acceleration_variances[1U] * m_acceleration_variances[1U];
  m.block<3, 3>(6, 6) = block_matrix * m_acceleration_variances[2U] * m_acceleration_variances[2U];
  return m;
}

template<>
common::state_vector::ConstAccelerationXY::Matrix
WienerNoise<common::state_vector::ConstAccelerationXY>::crtp_covariance(
  const std::chrono::nanoseconds & dt) const
{
  if (m_acceleration_variances.size() != 2U) {
    std::runtime_error(
      "Wrong initialization of noise model. Check the number of provided variances.");
  }
  const auto block_matrix = create_single_variable_block(dt);
  common::state_vector::ConstAccelerationXY::Matrix m{
    common::state_vector::ConstAccelerationXY::Matrix::Zero()};
  m.block<3, 3>(0, 0) = block_matrix * m_acceleration_variances[0U] * m_acceleration_variances[0U];
  m.block<3, 3>(3, 3) = block_matrix * m_acceleration_variances[1U] * m_acceleration_variances[1U];
  return m;
}

}  // namespace state_estimation
}  // namespace common
}  // namespace autoware
