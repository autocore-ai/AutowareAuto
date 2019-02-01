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
/// \brief This file defines the motion model interface used by kalman filters
#ifndef MOTION_MODEL__MOTION_MODEL_HPP_
#define MOTION_MODEL__MOTION_MODEL_HPP_

#include <chrono>
#include "Eigen/Core"
#include "motion_model/visibility_control.hpp"

namespace autoware
{
/// \brief This namespace is for motion planning, motion models, and related functionality
namespace motion
{
/// \brief This namespace is for the motion_model package
namespace motion_model
{

/// \brief indexing matches what matrices use
using index_t = Eigen::Index;

/// \brief Virtual interface for all motion models for use with prediction
/// \tparam NumStates dimensionality of this model's state space
template<int NumStates>
class MotionModel
{
public:
  /// \brief Do motion based on current state, store result somewhere else.
  ///        This is intended to be used with motion planning/collision avoidance,
  ///        i.e. to make multiple calls to predict with different time deltas for a
  ///        rollout to get a concrete representation of predicted trajectories.
  ///        This function does not mutate the core state of the motion model, and should
  ///        only mutate some cached worker variables at most.
  /// \param[out] x vector to store result into
  /// \param[in] dt prediction horizon based on current state
  virtual void predict(
    Eigen::Matrix<float, NumStates, 1U> & x,
    const std::chrono::nanoseconds & dt) const = 0;
  /// \brief Update current state with a given motion. Note that this should be called
  ///        after compute_jacobian() as it will change the object's state. This is meant
  ///        to be called before doing assignment and observation updating. This is the
  ///        equivalent of temporal update for the state.
  ///        This function mutates the core state of the motion model.
  /// \param[in] dt prediction horizon based on current state
  virtual void predict(const std::chrono::nanoseconds & dt) = 0;
  /// \brief Compute the jacobian based on the current state and store the result somewhere else
  /// \param[out] F matrix to store jacobian into
  /// \param[in] dt prediction horizon to build jacobian off of
  virtual void compute_jacobian(
    Eigen::Matrix<float, NumStates, NumStates> & F,
    const std::chrono::nanoseconds & dt) = 0;
  /// \brief This is called by Esrcf. This should be first a computation of the jacobian, and
  ///        then a motion to update the state. This is a distinct function because depending
  ///        on the motion model, there is some caching and optimization that can be done computing
  ///        both the motion and jacobian together.
  /// \param[out] F matrix to store jacobian into
  /// \param[in] dt prediction horizon to build jacobian off of
  virtual void compute_jacobian_and_predict(
    Eigen::Matrix<float, NumStates, NumStates> & F,
    const std::chrono::nanoseconds & dt) = 0;
  /// \brief Get elements of the model's state.
  /// \param[in] idx index of state variable to get
  /// \return copy of state variable
  virtual float operator[](const index_t idx) const = 0;
  /// \brief Set the state
  /// \param[in] x the state to store internally
  virtual void reset(const Eigen::Matrix<float, NumStates, 1U> & x) = 0;
  /// \brief const access to internal state
  /// \return const reference to internal state vector
  virtual const Eigen::Matrix<float, NumStates, 1U> & get_state() const = 0;
  /// \brief get dimensionality of this model
  /// \return number of dimensions in this model
  constexpr index_t get_num_states() {return NumStates;}

private:
  MotionModel<NumStates> & operator=(const MotionModel<NumStates> & rhs) = delete;
};  // class MotionModel
}  // namespace motion_model
}  // namespace motion
}  // namespace autoware

#endif  // MOTION_MODEL__MOTION_MODEL_HPP_
