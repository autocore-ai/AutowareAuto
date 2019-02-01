// Copyright 2018 Apex.AI, Inc.
// Co-developed by Tier IV, Inc. and Apex.AI, Inc.
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

/// \copyright Copyright 2018 Apex.AI, Inc.
/// All rights reserved.
/// \file
/// \brief This file defines the constant velocity motion model
#ifndef MOTION_MODEL__CATR_MODEL_HPP_
#define MOTION_MODEL__CATR_MODEL_HPP_

#include <chrono>
#include "motion_model/motion_model.hpp"
#include "motion_model/catr_core.hpp"
#include "motion_model/visibility_control.hpp"

namespace autoware
{
namespace motion
{
namespace motion_model
{
/// \brief This is a constant acceleration, constant turn-rate motion model. This
///        model produces clothoidal trajectories and corresponds to a zero order
///        hold on vehicle controls (throttle and steer angle)
class MOTION_MODEL_PUBLIC CatrModel : public MotionModel<6U>
{
public:
  /// \brief Default assignment operator
  /// \param[in] rhs Object to copy
  /// \return reference to this object
  CatrModel & operator=(const CatrModel & rhs);
  /// \brief Do motion based on current state, store result somewhere else.
  ///        This is intended to be used with motion planning/collision avoidance
  ///        i.e. to make multiple calls to predict with different time deltas for a
  ///        rollout to get a concrete representation of predicted trajectories.
  ///        This function does not mutate the core state of the motion model, and should
  ///        only mutate some cached worker variables at most.
  /// \param[out] x vector to store result into
  /// \param[in] dt prediction horizon based on current state
  void predict(
    Eigen::Matrix<float, 6U, 1U> & x,
    const std::chrono::nanoseconds & dt) const override;

  /// \brief Update current state with a given motion. Note that this should be called
  ///        after compute_jacobian() as it will change the object's state. This is meant
  ///        to be called before doing assignment and observation updating. This is the
  ///        equivalent of temporal update for the state.
  ///        This function mutates the core state of the motion model.
  /// \param[in] dt prediction horizon based on current state
  void predict(const std::chrono::nanoseconds & dt) override;

  /// \brief Compute the jacobian based on the current state and store the result somewhere else
  /// \param[out] F matrix to store jacobian into
  /// \param[in] dt prediction horizon to build jacobian off of
  void compute_jacobian(
    Eigen::Matrix<float, 6U, 6U> & F,
    const std::chrono::nanoseconds & dt) override;

  /// \brief This is called by Esrcf. This should be first a computation of the jacobian, and
  ///        then a motion to update the state. This is a distinct function because depending
  ///        on the motion model, there is some caching and optimization that can be done computing
  ///        both the motion and jacobian together.
  /// \param[out] F matrix to store jacobian into
  /// \param[in] dt prediction horizon to build jacobian off of
  void compute_jacobian_and_predict(
    Eigen::Matrix<float, 6U, 6U> & F,
    const std::chrono::nanoseconds & dt) override;

  /// \brief Get elements of the model's state.
  /// \param[in] idx index of state variable to get
  /// \return copy of state variable
  float operator[](const index_t idx) const override;

  /// \brief Set the state
  /// \param[in] x the state to store internally
  void reset(const Eigen::Matrix<float, 6U, 1U> & x) override;

  /// \brief const access to internal state
  /// \return const reference to internal state vector
  const Eigen::Matrix<float, 6U, 1U> & get_state() const override;

private:
  Eigen::Matrix<float, 6U, 1U> m_state;
  CatrInvariantWorkspace m_invariants;
  mutable CatrVariantWorkspace m_variants;
};  // class ConstantVelocity
}  // namespace motion_model
}  // namespace motion
}  // namespace autoware

#endif  // MOTION_MODEL__CATR_MODEL_HPP_
