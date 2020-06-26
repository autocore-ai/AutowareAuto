// Copyright 2018 Apex.AI, Inc.
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

/// \copyright Copyright 2018 Apex.AI, Inc.
/// All rights reserved.
/// \file
/// \brief This file defines the constant velocity motion model
#ifndef MOTION_MODEL__CATR_CORE_HPP_
#define MOTION_MODEL__CATR_CORE_HPP_

#include <common/types.hpp>
#include "motion_model/visibility_control.hpp"
#include "motion_model/motion_model.hpp"

using autoware::common::types::bool8_t;
using autoware::common::types::float32_t;

namespace autoware
{
namespace motion
{
namespace motion_model
{

/// \brief This struct holds some common worker variables for CATR model's
///        jacobian and prediction computation, specifically those that are time invariant
struct MOTION_MODEL_PUBLIC CatrInvariantWorkspace
{
  bool8_t is_w_nonzero;  ///< if heading rate is above some small number threshold
  float32_t w;  ///< current state heading rate
  float32_t a;  ///< current state acceleration
  float32_t s;  ///< sine of current state's heading
  float32_t c;  ///< cosine of current state's heading
  float32_t vw;  ///< product of velocity times heading rate, not calculated if w is 0
  float32_t w_inv;  ///< inverse of heading rate, not calculated if w is 0
  float32_t w2_inv;  ///< inverse square of heading rate, not calculated if w is 0
};

/// \brief This struct holds some common worker variables for CATR model's
///        jacobian and prediction computation, specifically those that are time varying
struct MOTION_MODEL_PUBLIC CatrVariantWorkspace
{
  float32_t dt;  ///< time step
  float32_t vp;  ///< next velocity after time step
  float32_t wT;  ///< product of heading rate and time step, not calculated if w is 0
  float32_t thp;  ///< next heading after time step
  float32_t sp;  ///< sine of next heading, not calculated if w is 0
  float32_t cp;  ///< cosine of next heading, not calculated if w is 0
  float32_t awT;  ///< product of acceleration, heading rate, and time step. If w is 0, path length
};


/// \brief This is an enum-like struct for convenience
struct MOTION_MODEL_PUBLIC CatrState
{
  static const index_t POSE_X = 0U;  ///< Index of x position
  static const index_t POSE_Y = 1U;  ///< Index of y position
  static const index_t VELOCITY = 2U;  ///< index of velocity
  static const index_t ACCELERATION = 3U;  ///< index of acceleration
  static const index_t HEADING = 4U;  ///< index of heading
  static const index_t TURN_RATE = 5U;  ///< index of turn rate (heading derivative)
};

/// \brief Initalize invariant values in catr workspace
/// \param[in] x state vector to initialize invariants with
/// \param[out] ws gets filled with invariants
/// \tparam NumStates dimensionality of model, CATR model is assumed to take the first 6 slots
template<int32_t NumStates>
MOTION_MODEL_LOCAL void catr_workspace_init_invariant(
  const Eigen::Matrix<float32_t, NumStates, 1U> & x,
  CatrInvariantWorkspace & ws);

/// \brief Initialize dt varying values for CATR model
/// \param[in] x state vector to use as refrence
/// \param[in] dt_s lookahead time to use for computation
/// \param[in] iws workspace variable filled with invariants, assumed to have been initialized with
///                the same x in catr_workspace_init_invariant()
/// \param[inout] ws get filled with varying stuff
/// \tparam NumStates dimensionality of model, CATR model is assumed to take the first 6 slots
template<int32_t NumStates>
MOTION_MODEL_LOCAL void catr_workspace_init_variant(
  const Eigen::Matrix<float32_t, NumStates, 1U> & x,
  const float32_t dt_s,
  const CatrInvariantWorkspace & iws,
  CatrVariantWorkspace & ws);

/// \brief Compute jacobian for CATR model, assumes ws is fully initialized
/// \param[in] iws precompute worker variables for a given state
/// \param[in] vws precompute worker variables for a given state and dt
/// \param[out] F gets filled with jacobian
/// \tparam NumStates dimensionality of model, CATR model is assumed to take the first 6 slots
template<int32_t NumStates>
MOTION_MODEL_LOCAL void catr_compute_jacobian(
  const CatrInvariantWorkspace & iws,
  const CatrVariantWorkspace & vws,
  Eigen::Matrix<float32_t, NumStates, NumStates> & F);

/// \brief Propagate CATR model forward in time
/// \param[in] ref reference state to propagate forward
/// \param[in] iws precomputed worker variables for ref
/// \param[in] vws precomputed worker variables for ref and a given dt
/// \param[out] x gets filled with reference state propagated forward
/// \tparam NumStates dimensionality of model, CATR model is assumed to take the first 6 slots
template<int32_t NumStates>
MOTION_MODEL_LOCAL void catr_predict(
  const Eigen::Matrix<float32_t, NumStates, 1U> & ref,
  const CatrInvariantWorkspace & iws,
  const CatrVariantWorkspace & vws,
  Eigen::Matrix<float32_t, NumStates, 1U> & x);

}  // namespace motion_model
}  // namespace motion
}  // namespace autoware

#endif  // MOTION_MODEL__CATR_CORE_HPP_
