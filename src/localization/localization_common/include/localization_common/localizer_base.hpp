// Copyright 2019 Apex.AI, Inc.
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

#ifndef LOCALIZATION_COMMON__LOCALIZER_BASE_HPP_
#define LOCALIZATION_COMMON__LOCALIZER_BASE_HPP_

#include <localization_common/visibility_control.hpp>
#include <tf2/buffer_core.h>
#include <geometry_msgs/msg/transform.hpp>
#include <localization_common/initialization.hpp>

namespace autoware
{
namespace localization
{
namespace localization_common
{
// The class could be templated on the optimizationproblem/solver as well.
template<typename MsgT, typename MapT, typename PoseInitializationT = VehicleOdometryPoseInitialization>
class RelativeLocalizerBase
{
    using PoseT = geometry_msgs::msg::Transform;
  using ScoreT = double;

  // Use the observation message to localize base_link in map.
  virtual PoseT register_measurement(const MsgT & msg, const PoseT & initial_guess) = 0;

  virtual void set_map(const MapT & msg) = 0;

  // The function to calculate/return a fitness score indicating the confidence in the produced result.
  virtual ScoreT get_fitness_score();

  // Function to evaluate the estimated relative transform and decide if it's an outlier/faulty or a valid transform
  // i.e. A relative transform that is too far away from the initial guess could be caused by a faulty measurement/computation
  virtual bool validate_result();
};

}          // namespace autoware
}      // namespace localization
}  // namespace localization_common

#endif