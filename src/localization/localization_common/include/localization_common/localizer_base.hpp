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
using Real = double;
// The class could be templated on the optimizationproblem/solver as well.
template<typename InputMsgT, typename MapMsgT>
class RelativeLocalizerBase
{
public:
  // TODO(yunus.caliskan): Use a transform message with covariance
  using PoseT = geometry_msgs::msg::TransformStamped;
  /// Registers a measurement to the current map and returns the
  /// estimated pose of the vehicle and its validity.
  /// \param[in] msg Measurement message to register.
  /// \param[in] initial_guess Initial guess of the pose to initialize the localizer with
  /// in iterative processes like solving optimization problems.
  /// \param[out] pose_out Resulting pose estimate after registration.
  /// \return If the returning pose estimate is deemed valid or not.
  /// Details are implementation defined.
  virtual bool register_measurement(
    const InputMsgT & msg, const PoseT & initial_guess, PoseT & pose_out) = 0;

  /// Set map.
  /// \param msg Map message.
  virtual void set_map(const MapMsgT & msg) = 0;
};
}  // namespace localization_common
}  // namespace localization
}  // namespace autoware

#endif  // LOCALIZATION_COMMON__LOCALIZER_BASE_HPP_
