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
/// \brief This file defines an interface for the motion model.

#ifndef MOTION_MODEL__MOTION_MODEL_INTERFACE_HPP_
#define MOTION_MODEL__MOTION_MODEL_INTERFACE_HPP_

#include <helper_functions/crtp.hpp>
#include <kalman_filter/generic_state.hpp>

#include <chrono>

namespace autoware
{
namespace prediction
{

///
/// @brief      A CRTP interface for any motion model.
///
/// @tparam     Derived  Motion model implementation class.
///
template<typename Derived>
class KALMAN_FILTER_PUBLIC MotionModelInterface : public common::helper_functions::crtp<Derived>
{
public:
  ///
  /// @brief      Get the next predicted state.
  ///
  /// @param[in]  state   The initial state
  /// @param[in]  dt      Length of prediction into the future
  ///
  /// @tparam     StateT  Type of the state.
  ///
  /// @return     Predicted state after the time has passed.
  ///
  template<typename StateT>
  inline auto predict(const StateT & state, const std::chrono::nanoseconds & dt) const
  {
    static_assert(is_state<StateT>::value, "\n\nStateT must be a GenericState\n\n");
    return this->impl().crtp_predict(state, dt);
  }
  ///
  /// @brief      Get the Jacobian of this motion model.
  ///
  /// @param[in]  state   The current state.
  /// @param[in]  dt      Time span.
  ///
  /// @tparam     StateT  Type of the state.
  ///
  /// @return     A Jacobian of the motion model. In the linear case - a transition matrix.
  ///
  template<typename StateT>
  inline auto jacobian(const StateT & state, const std::chrono::nanoseconds & dt) const
  {
    return this->impl().crtp_jacobian(state, dt);
  }
};

}  // namespace prediction
}  // namespace autoware

#endif  // MOTION_MODEL__MOTION_MODEL_INTERFACE_HPP_
