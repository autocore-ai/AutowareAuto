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

#ifndef MOTION_MODEL__DIFFERENTIAL_DRIVE_MOTION_MODEL_HPP_
#define MOTION_MODEL__DIFFERENTIAL_DRIVE_MOTION_MODEL_HPP_

#include <kalman_filter/common_states.hpp>
#include <kalman_filter/generic_state.hpp>
#include <kalman_filter/visibility_control.hpp>
#include <motion_model/motion_model_interface.hpp>

namespace autoware
{
namespace prediction
{

/// @brief      A generic differential motion model. This class only exists to be specialized for
///             specific motion model implementations.
template<typename StateT>
class KALMAN_FILTER_PUBLIC DifferentialDriveMotionModel
  : public MotionModelInterface<DifferentialDriveMotionModel<StateT>>
{
public:
  using State = StateT;

protected:
  // Allow the CRTP interface to call private functions.
  friend MotionModelInterface<DifferentialDriveMotionModel<StateT>>;

  /// @brief      A crtp-called function that predicts the state forward.
  State crtp_predict(const State &, const std::chrono::nanoseconds &) const
  {
    static_assert(
      sizeof(StateT) == 0,
      "Function crtp_predict is expected to be specialized for every state it is used with.");
  }

  /// @brief      A crtp-called function that computes a Jacobian.
  typename State::Matrix crtp_jacobian(const State &, const std::chrono::nanoseconds &) const
  {
    static_assert(
      sizeof(StateT) == 0,
      "Function crtp_jacobian is expected to be specialized for every state it is used with.");
  }
};

/// @brief      An alias of the differential drive motion model for the
///             state::ConstantVelocityAndTurnRate state.
using CvtrMotionModel = DifferentialDriveMotionModel<state::ConstantVelocityAndTurnRate>;
/// @brief      An alias of the differential drive motion model for the
///             state::ConstantAccelerationAndTurnRate state.
using CatrMotionModel = DifferentialDriveMotionModel<state::ConstantAccelerationAndTurnRate>;


/// @brief      A crtp-called function that predicts the state forward.
template<>
KALMAN_FILTER_PUBLIC CvtrMotionModel::State CvtrMotionModel::crtp_predict(
  const CvtrMotionModel::State & state,
  const std::chrono::nanoseconds & dt) const;

/// @brief      A crtp-called function that computes a Jacobian.
template<>
KALMAN_FILTER_PUBLIC CvtrMotionModel::State::Matrix CvtrMotionModel::crtp_jacobian(
  const CvtrMotionModel::State & state,
  const std::chrono::nanoseconds & dt) const;


/// @brief      A crtp-called function that predicts the state forward.
template<>
KALMAN_FILTER_PUBLIC CatrMotionModel::State CatrMotionModel::crtp_predict(
  const CatrMotionModel::State & state,
  const std::chrono::nanoseconds & dt) const;

/// @brief      A crtp-called function that computes a Jacobian.
template<>
KALMAN_FILTER_PUBLIC CatrMotionModel::State::Matrix CatrMotionModel::crtp_jacobian(
  const CatrMotionModel::State & state,
  const std::chrono::nanoseconds & dt) const;


}  // namespace prediction
}  // namespace autoware

#endif  // MOTION_MODEL__DIFFERENTIAL_DRIVE_MOTION_MODEL_HPP_
