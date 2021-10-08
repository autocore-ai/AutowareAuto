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

#ifndef MAKE_INPUTS_HPP_
#define MAKE_INPUTS_HPP_

#include "lonely_world_prediction/visibility_control.hpp"

#include "autoware_auto_msgs/msg/predicted_objects.hpp"
#include "autoware_auto_msgs/msg/tracked_object_kinematics.hpp"
#include "autoware_auto_msgs/msg/tracked_objects.hpp"
#include "geometry_msgs/msg/polygon.hpp"

namespace autoware
{
namespace prediction
{
namespace test
{
/**
 * A factory function to create a test object of type `T`
 */
template<typename T>
T LONELY_WORLD_PREDICTION_PUBLIC make();

template<>
autoware_auto_msgs::msg::PredictedObjects LONELY_WORLD_PREDICTION_PUBLIC make();

template<>
autoware_auto_msgs::msg::TrackedObjectKinematics LONELY_WORLD_PREDICTION_PUBLIC make();

template<>
autoware_auto_msgs::msg::TrackedObjects LONELY_WORLD_PREDICTION_PUBLIC make();

template<>
geometry_msgs::msg::Polygon LONELY_WORLD_PREDICTION_PUBLIC make();

}  // namespace test
}  // namespace prediction
}  // namespace autoware
#endif  // MAKE_INPUTS_HPP_
