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
// Co-developed by Tier IV, Inc. and Apex.AI, Inc.

#ifndef STATE_ESTIMATION__NOISE_MODEL__WIENER_NOISE_HPP_
#define STATE_ESTIMATION__NOISE_MODEL__WIENER_NOISE_HPP_

#include <state_estimation/noise_model/noise_interface.hpp>
#include <state_estimation/visibility_control.hpp>
#include <state_vector/common_states.hpp>

#include <algorithm>
#include <array>
#include <vector>

namespace autoware
{
namespace common
{
namespace state_estimation
{

///
/// @brief      A trait that defines the number of acceleration components.
///
/// @tparam     StateT  A state vector type.
///
template<typename StateT>
struct number_of_acceleration_components : public std::integral_constant<std::size_t, 0UL>
{
  static_assert(sizeof(StateT) == 0, "This class must be specialized to a specific state type.");
};

///
/// @brief      A class that describes the Wiener process noise.
///
/// @details    For more details see notebook here: @ref noise-model-design
///
/// @tparam     StateT  A given state type.
///
template<typename StateT>
class STATE_ESTIMATION_PUBLIC WienerNoise : public NoiseInterface<WienerNoise<StateT>>
{
  using AccelerationArray = std::array<
    typename StateT::Scalar, number_of_acceleration_components<StateT>::value>;

public:
  using State = StateT;

  ///
  /// @brief      Constructor from acceleration variances.
  ///
  /// @param[in]  acceleration_variances  The acceleration variances, note that these are sigmas,
  ///                                     not sigmas squared. Note that while this array has place
  ///                                     for all the variables, it should only hold those
  ///                                     representing acceleration values. The positions of these
  ///                                     variables in the array do not represent their position in
  ///                                     the actual state vector and should start from the start of
  ///                                     this array.
  ///
  explicit WienerNoise(const AccelerationArray & acceleration_variances)
  : m_acceleration_variances{acceleration_variances} {}

protected:
  // Required to allow the crtp interface call the following functions.
  friend NoiseInterface<WienerNoise<StateT>>;

  ///
  /// @brief      A CRTP-called covariance getter.
  ///
  /// @return     A covariance of the noise process over given time.
  ///
  typename State::Matrix crtp_covariance(const std::chrono::nanoseconds &) const;

private:
  AccelerationArray m_acceleration_variances{};
};

template<typename StateT, typename OtherScalarT>
auto make_wiener_noise(const std::vector<OtherScalarT> & acceleration_variances)
{
  std::array<typename StateT::Scalar, number_of_acceleration_components<StateT>::value> variances;
  if (acceleration_variances.size() != variances.size()) {
    throw std::runtime_error(
            "There must be " + std::to_string(variances.size()) + " acceleration variances");
  }
  std::copy(acceleration_variances.begin(), acceleration_variances.end(), variances.begin());
  return WienerNoise<StateT>{variances};
}

///
/// @brief      A specialization of the number_of_acceleration_components trait for
///             common::state_vector::ConstAccelerationXY.
///
template<typename ScalarT>
struct number_of_acceleration_components<common::state_vector::ConstAccelerationXY<ScalarT>>
  : public std::integral_constant<std::size_t, 2UL> {};

///
/// @brief      A specialization of the number_of_acceleration_components trait for
///             common::state_vector::ConstAccelerationXYZ.
///
template<typename ScalarT>
struct number_of_acceleration_components<common::state_vector::ConstAccelerationXYZ<ScalarT>>
  : public std::integral_constant<std::size_t, 3UL> {};

///
/// @brief      A specialization of the number_of_acceleration_components trait for
///             common::state_vector::ConstAccelerationXYYaw.
///
template<typename ScalarT>
struct number_of_acceleration_components<common::state_vector::ConstAccelerationXYYaw<ScalarT>>
  : public std::integral_constant<std::size_t, 3UL> {};

///
/// @brief      A specialization of the number_of_acceleration_components trait for
///             common::state_vector::ConstAccelerationXYZYaw.
///
template<typename ScalarT>
struct number_of_acceleration_components<common::state_vector::ConstAccelerationXYZYaw<ScalarT>>
  : public std::integral_constant<std::size_t, 4UL> {};

///
/// @brief      A specialization of the number_of_acceleration_components trait for
///             common::state_vector::ConstAccelerationXYZRPY.
///
template<typename ScalarT>
struct number_of_acceleration_components<common::state_vector::ConstAccelerationXYZRPY<ScalarT>>
  : public std::integral_constant<std::size_t, 6UL> {};


}  // namespace state_estimation
}  // namespace common
}  // namespace autoware

#endif  // STATE_ESTIMATION__NOISE_MODEL__WIENER_NOISE_HPP_
