// Copyright 2021 The Autoware Foundation
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

/// \copyright Copyright 2021 The Autoware Foundation
/// \file
/// \brief This file defines the lonely_world_prediction class.

#ifndef LONELY_WORLD_PREDICTION__LONELY_WORLD_PREDICTION_HPP_
#define LONELY_WORLD_PREDICTION__LONELY_WORLD_PREDICTION_HPP_

#include "lonely_world_prediction/parameters.hpp"
#include "lonely_world_prediction/visibility_control.hpp"

#include "autoware_auto_msgs/msg/predicted_objects.hpp"
#include "autoware_auto_msgs/msg/tracked_objects.hpp"

namespace autoware
{
namespace prediction
{
/**
 * \brief Predict object to remain stationary
 *
 * Stationary means that all predicted states have:
 * - pose = initial pose
 *
 * The initial velocity and acceleration are unmodified and may be nonzero
 *
 * \param[in,out] predicted_object Contains the path on return
 * \param[in] parameters Define time step and time horizon of path. The last predicted state is
 * after the `time_horizon` if `time_step` divides `time_horizon` with a remainder.
 */
void LONELY_WORLD_PREDICTION_PUBLIC predict_stationary(
  autoware_auto_msgs::msg::PredictedObject & predicted_object,
  const autoware::prediction::Parameters & parameters);

/**
 * \brief Predict all input objects to remain stationary
 * \sa predict_stationary() for details how each object is predicted
 * \param[in,out] predicted_objects
 * \param[in] parameters
 */
void LONELY_WORLD_PREDICTION_PUBLIC predict_stationary(
  autoware_auto_msgs::msg::PredictedObjects & predicted_objects,
  const autoware::prediction::Parameters & parameters);

}  // namespace prediction
}  // namespace autoware

#endif  // LONELY_WORLD_PREDICTION__LONELY_WORLD_PREDICTION_HPP_
