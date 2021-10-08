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

// Co-developed by Tier IV, Inc. and Apex.AI, Inc.

/// \copyright Copyright 2021 The Autoware Foundation
/// \file
/// \brief Define initialization functions from tracked objects

#ifndef LONELY_WORLD_PREDICTION__INIT_FROM_TRACKED_HPP_
#define LONELY_WORLD_PREDICTION__INIT_FROM_TRACKED_HPP_

#include "lonely_world_prediction/visibility_control.hpp"

#include "autoware_auto_msgs/msg/predicted_objects.hpp"
#include "autoware_auto_msgs/msg/tracked_objects.hpp"

namespace autoware
{
namespace prediction
{
geometry_msgs::msg::PoseWithCovariance LONELY_WORLD_PREDICTION_PUBLIC
make_pose(const autoware_auto_msgs::msg::TrackedObjectKinematics & tracked);

/**
 * @brief Create one `PredictedObject` from each `TrackedObject` and copy over all members that are
 * common, leaving the `predicted_paths` member empty.
 *
 * @param[in] objects Input tracked objects
 * @return predicted objects
 */
autoware_auto_msgs::msg::PredictedObjects LONELY_WORLD_PREDICTION_PUBLIC
from_tracked(const autoware_auto_msgs::msg::TrackedObjects & objects);

/**
 * @brief Create a `PredictedObject` from a `TrackedObject` and copy over all members that are
 * common, leaving the `predicted_paths` member empty.
 *
 * @param[in] object Input tracked object
 * @return predicted object
 */
autoware_auto_msgs::msg::PredictedObject LONELY_WORLD_PREDICTION_PUBLIC
from_tracked(const autoware_auto_msgs::msg::TrackedObject & object);

/**
 * @brief Create `PredictedObjectKinematics` from `TrackedObjectKinematics` and copy over all
 * members that are common, leaving the `predicted_paths` member empty.
 *
 * @param[in] kinematics Input kinematics
 * @return predicted objects
 */
autoware_auto_msgs::msg::PredictedObjectKinematics LONELY_WORLD_PREDICTION_PUBLIC
from_tracked(const autoware_auto_msgs::msg::TrackedObjectKinematics & kinematics);

}  // namespace prediction
}  // namespace autoware

#endif  // LONELY_WORLD_PREDICTION__INIT_FROM_TRACKED_HPP_
