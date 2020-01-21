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
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>

namespace autoware
{
namespace localization
{
namespace localization_common
{
using Real = double;
/// The base class for relative localizers.
/// \tparam InputMsgT Message type that will be registered against a map.
/// \tparam MapMsgT Map type.
template<typename InputMsgT, typename MapMsgT>
class RelativeLocalizerBase
{
public:
  using PoseWithCovarianceStamped = geometry_msgs::msg::PoseWithCovarianceStamped;
  using Transform = geometry_msgs::msg::TransformStamped;

  /// Registers a measurement to the current map and returns the
  /// estimated pose of the vehicle and its validity.
  /// \param[in] msg Measurement message to register.
  /// \param[in] transform_initial Initial guess of the pose to initialize the localizer with
  /// in iterative processes like solving optimization problems.
  /// \return Resulting pose estimate after registration.
  /// \throws If the result is invalid and cannot be used. Defined in the implementation.
  virtual PoseWithCovarianceStamped register_measurement(
    const InputMsgT & msg, const Transform & transform_initial) = 0;

  /// Set map.
  /// \param msg Map message.
  virtual void set_map(const MapMsgT & msg) = 0;

  virtual ~RelativeLocalizerBase() = default;
};
}  // namespace localization_common
}  // namespace localization
}  // namespace autoware

#endif  // LOCALIZATION_COMMON__LOCALIZER_BASE_HPP_
