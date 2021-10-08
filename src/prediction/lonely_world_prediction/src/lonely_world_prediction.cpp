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

#include "lonely_world_prediction/lonely_world_prediction.hpp"

#include <utility>

#include "time_utils/time_utils.hpp"

namespace autoware
{
namespace prediction
{
void predict_stationary(
  autoware_auto_msgs::msg::PredictedObject & predicted_object,
  const autoware::prediction::Parameters & parameters)
{
  // need an extra state in the end if the division has remainder
  auto n_steps =
    static_cast<std::size_t>(parameters.time_horizon().count() / parameters.time_step().count());
  if (parameters.time_horizon().count() % parameters.time_step().count()) {
    ++n_steps;
  }

  // TODO(frederik.beaujean) predicted path only has one pose, not multiple for each shape
  autoware_auto_msgs::msg::PredictedPath predicted_path;
  predicted_path.path =
    decltype(predicted_path.path) {n_steps, predicted_object.kinematics.initial_pose.pose};
  predicted_path.confidence = 1.0;
  predicted_path.time_step = time_utils::to_message(parameters.time_step());

  predicted_object.kinematics.predicted_paths.emplace_back(std::move(predicted_path));
}

void predict_stationary(
  autoware_auto_msgs::msg::PredictedObjects & predicted_objects,
  const autoware::prediction::Parameters & parameters)
{
  std::for_each(
    predicted_objects.objects.begin(),
    predicted_objects.objects.end(),
    [&](autoware_auto_msgs::msg::PredictedObject & object) {
      predict_stationary(object, parameters);
    });
}

}  // namespace prediction
}  // namespace autoware
