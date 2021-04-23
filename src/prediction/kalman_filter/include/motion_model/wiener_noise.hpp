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
/// \brief This file contains the Wiener noise model classes.

#ifndef MOTION_MODEL__WIENER_NOISE_HPP_
#define MOTION_MODEL__WIENER_NOISE_HPP_

#include <kalman_filter/common_states.hpp>
#include <kalman_filter/visibility_control.hpp>
#include <motion_model/noise_interface.hpp>

#include <algorithm>
#include <array>
#include <vector>

namespace autoware
{
namespace prediction
{

///
/// @brief      A trait that defines the number of acceleration components.
///
/// @tparam     StateT  A state vector type.
///
template<typename StateT>
struct number_of_acceleration_components : public std::integral_constant<std::size_t, 0UL> {};

///
/// @brief      A class that describes the Wiener process noise.
///
///             For more details see notebook here:
///             https://nbviewer.jupyter.org/github/rlabbe/Kalman-and-Bayesian-Filters-in-Python/
///             blob/master/07-Kalman-Filter-Math.ipynb#Piecewise-White-Noise-Model (combine into
///             one line)
///
/// @tparam     StateT  A given state type.
///
template<typename StateT>
class KALMAN_FILTER_PUBLIC WienerNoise : public NoiseInterface<WienerNoise<StateT>>
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
  typename State::Matrix crtp_covariance(const std::chrono::nanoseconds &) const
  {
    static_assert(
      sizeof(StateT) == 0U,
      "\n\nThis function must be specialized for specific states.\n\n");
  }

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
///             state::ConstAccelerationXY.
///
template<>
struct number_of_acceleration_components<state::ConstAccelerationXY>
  : public std::integral_constant<std::size_t, 2UL> {};

///
/// @brief      A specialization of covariance matrix computation for ConstAccelerationXY state.
///
/// @param[in]  dt    Time step.
///
/// @return     Covariance matrix.
///
template<>
KALMAN_FILTER_PUBLIC state::ConstAccelerationXY::Matrix
WienerNoise<state::ConstAccelerationXY>::crtp_covariance(
  const std::chrono::nanoseconds & dt) const;


///
/// @brief      A specialization of the number_of_acceleration_components trait for
///             state::ConstAccelerationXYYaw.
///
template<>
struct number_of_acceleration_components<state::ConstAccelerationXYYaw>
  : public std::integral_constant<std::size_t, 3UL> {};

///
/// @brief      A specialization of covariance matrix computation for ConstAccelerationXYYaw state.
///
/// @param[in]  dt    Time step.
///
/// @return     Covariance matrix.
///
template<>
KALMAN_FILTER_PUBLIC state::ConstAccelerationXYYaw::Matrix
WienerNoise<state::ConstAccelerationXYYaw>::crtp_covariance(
  const std::chrono::nanoseconds & dt) const;

}  // namespace prediction
}  // namespace autoware

#endif  // MOTION_MODEL__WIENER_NOISE_HPP_
