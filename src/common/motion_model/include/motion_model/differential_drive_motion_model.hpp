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

#ifndef MOTION_MODEL__DIFFERENTIAL_DRIVE_MOTION_MODEL_HPP_
#define MOTION_MODEL__DIFFERENTIAL_DRIVE_MOTION_MODEL_HPP_

#include <motion_model/visibility_control.hpp>
#include <motion_model/motion_model_interface.hpp>
#include <state_vector/common_states.hpp>
#include <state_vector/generic_state.hpp>

namespace autoware
{
namespace common
{
namespace motion_model
{

/// @brief      A generic differential motion model. This class only exists to be specialized for
///             specific motion model implementations.
template<typename StateT>
class MOTION_MODEL_PUBLIC DifferentialDriveMotionModel
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
///             common::state_vector::ConstantVelocityAndTurnRate state.
template<typename ScalarT>
using CvtrMotionModel =
  DifferentialDriveMotionModel<common::state_vector::ConstantVelocityAndTurnRate<ScalarT>>;
using CvtrMotionModel32 = CvtrMotionModel<common::types::float32_t>;
using CvtrMotionModel64 = CvtrMotionModel<common::types::float64_t>;

/// @brief      An alias of the differential drive motion model for the
///             common::state_vector::ConstantAccelerationAndTurnRate state.
template<typename ScalarT>
using CatrMotionModel =
  DifferentialDriveMotionModel<common::state_vector::ConstantAccelerationAndTurnRate<ScalarT>>;
using CatrMotionModel32 = CatrMotionModel<common::types::float32_t>;
using CatrMotionModel64 = CatrMotionModel<common::types::float64_t>;


/// @brief      A crtp-called function that predicts the state forward.
template<>
MOTION_MODEL_PUBLIC CvtrMotionModel32::State CvtrMotionModel32::crtp_predict(
  const CvtrMotionModel32::State & state,
  const std::chrono::nanoseconds & dt) const;

/// @brief      A crtp-called function that predicts the state forward.
template<>
MOTION_MODEL_PUBLIC CvtrMotionModel64::State CvtrMotionModel64::crtp_predict(
  const CvtrMotionModel64::State & state,
  const std::chrono::nanoseconds & dt) const;

/// @brief      A crtp-called function that computes a Jacobian.
template<>
MOTION_MODEL_PUBLIC CvtrMotionModel32::State::Matrix CvtrMotionModel32::crtp_jacobian(
  const CvtrMotionModel32::State & state,
  const std::chrono::nanoseconds & dt) const;

/// @brief      A crtp-called function that computes a Jacobian.
template<>
MOTION_MODEL_PUBLIC CvtrMotionModel64::State::Matrix CvtrMotionModel64::crtp_jacobian(
  const CvtrMotionModel64::State & state,
  const std::chrono::nanoseconds & dt) const;

/// @brief      A crtp-called function that predicts the state forward.
template<>
MOTION_MODEL_PUBLIC CatrMotionModel32::State CatrMotionModel32::crtp_predict(
  const CatrMotionModel32::State & state,
  const std::chrono::nanoseconds & dt) const;

/// @brief      A crtp-called function that predicts the state forward.
template<>
MOTION_MODEL_PUBLIC CatrMotionModel64::State CatrMotionModel64::crtp_predict(
  const CatrMotionModel64::State & state,
  const std::chrono::nanoseconds & dt) const;

/// @brief      A crtp-called function that computes a Jacobian.
template<>
MOTION_MODEL_PUBLIC CatrMotionModel32::State::Matrix CatrMotionModel32::crtp_jacobian(
  const CatrMotionModel32::State & state,
  const std::chrono::nanoseconds & dt) const;

/// @brief      A crtp-called function that computes a Jacobian.
template<>
MOTION_MODEL_PUBLIC CatrMotionModel64::State::Matrix CatrMotionModel64::crtp_jacobian(
  const CatrMotionModel64::State & state,
  const std::chrono::nanoseconds & dt) const;

}  // namespace motion_model
}  // namespace common
}  // namespace autoware

#endif  // MOTION_MODEL__DIFFERENTIAL_DRIVE_MOTION_MODEL_HPP_
