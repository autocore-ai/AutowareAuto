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

#include <common/types.hpp>
#include <cstddef>

using autoware::common::types::float32_t;
using autoware::common::types::float64_t;

namespace
{
template<typename ScalarT>
Eigen::Matrix<ScalarT, 3, 3> create_single_variable_block(const std::chrono::nanoseconds & dt)
{
  const auto seconds = std::chrono::duration<double>{dt}.count();
  const auto seconds_2 = seconds * seconds;
  const Eigen::Vector3d noise_gain{0.5 * seconds_2, seconds, 1.0};
  const Eigen::Matrix3d block = noise_gain * noise_gain.transpose();
  return block.cast<ScalarT>();
}

template<typename ScalarT, int size, std::size_t stddev_size>
Eigen::Matrix<ScalarT, size, size> create_covariance(
  const std::chrono::nanoseconds & dt,
  const typename std::array<ScalarT, stddev_size> acceleration_variances)
{
  const Eigen::Matrix<ScalarT, 3, 3> block_matrix =
    create_single_variable_block<ScalarT>(dt);
  Eigen::Matrix<ScalarT, size, size> m = Eigen::Matrix<ScalarT, size, size>::Zero();
  for (std::size_t i = 0; i < stddev_size; ++i) {
    const int offs = static_cast<int>(3 * i);
    m.template block<3, 3>(offs, offs) =
      block_matrix * acceleration_variances[i] * acceleration_variances[i];
  }
  return m;
}
}  // namespace

namespace autoware
{
namespace common
{
namespace state_estimation
{


template<>
common::state_vector::ConstAccelerationXYYaw32::Matrix
WienerNoise<common::state_vector::ConstAccelerationXYYaw32>::crtp_covariance(
  const std::chrono::nanoseconds & dt) const
{
  return create_covariance<float32_t, 9, 3>(dt, m_acceleration_variances);
}

template<>
common::state_vector::ConstAccelerationXYYaw64::Matrix
WienerNoise<common::state_vector::ConstAccelerationXYYaw64>::crtp_covariance(
  const std::chrono::nanoseconds & dt) const
{
  return create_covariance<float64_t, 9, 3>(dt, m_acceleration_variances);
}

template<>
common::state_vector::ConstAccelerationXY32::Matrix
WienerNoise<common::state_vector::ConstAccelerationXY32>::crtp_covariance(
  const std::chrono::nanoseconds & dt) const
{
  return create_covariance<float32_t, 6, 2>(dt, m_acceleration_variances);
}

template<>
common::state_vector::ConstAccelerationXY64::Matrix
WienerNoise<common::state_vector::ConstAccelerationXY64>::crtp_covariance(
  const std::chrono::nanoseconds & dt) const
{
  return create_covariance<float64_t, 6, 2>(dt, m_acceleration_variances);
}

}  // namespace state_estimation
}  // namespace common
}  // namespace autoware
