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

#ifndef STATE_ESTIMATION__NOISE_MODEL__NOISE_INTERFACE_HPP_
#define STATE_ESTIMATION__NOISE_MODEL__NOISE_INTERFACE_HPP_

#include <state_estimation/visibility_control.hpp>

#include <chrono>

namespace autoware
{
namespace common
{
namespace state_estimation
{

///
/// @brief      A CRTP interface for implementing noise models used for providing motion model noise
///             covariance.
///
/// @tparam     Derived  A derived class that holds the actual implementation.
///
template<typename Derived>
class STATE_ESTIMATION_PUBLIC NoiseInterface
{
public:
  ///
  /// @brief      Get a covariance matrix for this noise model.
  ///
  /// @param[in]  dt    Time difference.
  ///
  /// @return     A covariance matrix for the noise gain during the dt time difference.
  ///
  inline auto covariance(const std::chrono::nanoseconds & dt) const
  {
    return static_cast<const Derived &>(*this).crtp_covariance(dt);
  }
};

}  // namespace state_estimation
}  // namespace common
}  // namespace autoware

#endif  // STATE_ESTIMATION__NOISE_MODEL__NOISE_INTERFACE_HPP_
