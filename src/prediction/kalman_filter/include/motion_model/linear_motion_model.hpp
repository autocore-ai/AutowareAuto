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
/// \brief This file defines the linear motion model.

#ifndef MOTION_MODEL__LINEAR_MOTION_MODEL_HPP_
#define MOTION_MODEL__LINEAR_MOTION_MODEL_HPP_

#include <kalman_filter/common_states.hpp>
#include <kalman_filter/generic_state.hpp>
#include <kalman_filter/visibility_control.hpp>
#include <motion_model/motion_model_interface.hpp>

namespace autoware
{
namespace prediction
{

///
/// @brief      A generic linear motion model class.
///
///             This class is designed to handle all the linear motion models, i.e., those that only
///             have independent variables in them. In this case, the Jacobian is just a transition
///             matrix.
///
/// @tparam     StateT  State type.
///
template<typename StateT>
class KALMAN_FILTER_PUBLIC LinearMotionModel
  : public MotionModelInterface<LinearMotionModel<StateT>>
{
public:
  using State = StateT;

protected:
  // Allow the CRTP interface to call private functions.
  friend MotionModelInterface<LinearMotionModel<StateT>>;

  ///
  /// @brief      A crtp-called function that predicts the state forward.
  ///
  /// @param[in]  state  The current state vector
  /// @param[in]  dt     Time difference
  ///
  /// @return     New state after prediction.
  ///
  inline State crtp_predict(
    const State & state,
    const std::chrono::nanoseconds & dt) const
  {
    return State{crtp_jacobian(state, dt) * state.vector()};
  }

  /// @brief      A crtp-called function that computes a Jacobian.
  typename State::Matrix crtp_jacobian(const State &, const std::chrono::nanoseconds &) const
  {
    static_assert(
      sizeof(StateT) == 0U,
      "\n\nThis function must be specialized for specific states.\n\n");
  }
};


///
/// @brief      A specialization of this function for a state::ConstAccelerationXYYaw state.
///
/// @param[in]  state  The current state. Unused in this function.
/// @param[in]  dt     Time difference.
///
/// @return     The Jacobian matrix.
///
template<>
KALMAN_FILTER_PUBLIC state::ConstAccelerationXYYaw::Matrix
LinearMotionModel<state::ConstAccelerationXYYaw>::crtp_jacobian(
  const State & state, const std::chrono::nanoseconds & dt) const;

///
/// @brief      A specialization of this function for a state::ConstAccelerationXY state.
///
/// @param[in]  state  The current state. Unused in this function.
/// @param[in]  dt    Time difference.
///
/// @return     The Jacobian matrix.
///
template<>
KALMAN_FILTER_PUBLIC state::ConstAccelerationXY::Matrix
LinearMotionModel<state::ConstAccelerationXY>::crtp_jacobian(
  const State & state, const std::chrono::nanoseconds & dt) const;

}  // namespace prediction
}  // namespace autoware

#endif  // MOTION_MODEL__LINEAR_MOTION_MODEL_HPP_
