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

#include "lonely_world_prediction/init_from_tracked.hpp"

#include <algorithm>

#include "autoware_auto_msgs/msg/predicted_object.hpp"
#include "autoware_auto_msgs/msg/predicted_object_kinematics.hpp"

namespace autoware
{
namespace prediction
{
geometry_msgs::msg::PoseWithCovariance make_pose(
  const autoware_auto_msgs::msg::TrackedObjectKinematics & tracked)
{
  using autoware_auto_msgs::msg::TrackedObjectKinematics;

  geometry_msgs::msg::PoseWithCovariance pose;
  pose.pose.position = tracked.centroid_position;
  switch (tracked.orientation_availability) {
    case TrackedObjectKinematics::UNAVAILABLE:
      break;
    case TrackedObjectKinematics::SIGN_UNKNOWN:
      // TODO(frederik.beaujean) Uncertainty due to sign is lost. Modify tracker or message type to
      // preserve this information
      pose.pose.orientation = tracked.orientation;
      break;
    case TrackedObjectKinematics::AVAILABLE:
      pose.pose.orientation = tracked.orientation;
      break;
    default:
      throw std::out_of_range("invalid orientation availability");
  }
  // copy over rows and leave covariance with orientation at zero. Since orientation is not part of
  // the tracker state, no input covariance is available

  // (a b c) -> (a b c 0 0 0)
  // (b d e) -> (b d e 0 0 0)
  // (c e f) -> (c e f 0 0 0)
  // (     ) -> (0 0 0 0 0 0)
  // (     ) -> (0 0 0 0 0 0)
  // (     ) -> (0 0 0 0 0 0)
  const int dim = 3;
  for (int i = 0; i < dim; ++i) {
    std::copy(
      tracked.position_covariance.begin() + i * dim,
      tracked.position_covariance.begin() + (i + 1) * dim,
      pose.covariance.begin() + 2 * i * dim);
  }

  return pose;
}

autoware_auto_msgs::msg::PredictedObjects from_tracked(
  const autoware_auto_msgs::msg::TrackedObjects & tracked)
{
  // safest initialization in cases new members are added to TrackedMsgT
  autoware_auto_msgs::msg::PredictedObjects predicted{
    rosidl_runtime_cpp::MessageInitialization::ZERO};
  predicted.header = tracked.header;

  predicted.objects.reserve(tracked.objects.size());
  std::transform(
    tracked.objects.begin(),
    tracked.objects.end(),
    std::back_inserter(predicted.objects),
    [](const autoware_auto_msgs::msg::TrackedObject & t) {return from_tracked(t);});

  return predicted;
}

autoware_auto_msgs::msg::PredictedObject from_tracked(
  const autoware_auto_msgs::msg::TrackedObject & tracked)
{
  return autoware_auto_msgs::build<autoware_auto_msgs::msg::PredictedObject>()
         .object_id(tracked.object_id)
         .existence_probability(tracked.existence_probability)
         .classification(tracked.classification)
         .kinematics(from_tracked(tracked.kinematics))
         .shape(tracked.shape);
}

autoware_auto_msgs::msg::PredictedObjectKinematics from_tracked(
  const autoware_auto_msgs::msg::TrackedObjectKinematics & tracked)
{
  return autoware_auto_msgs::build<autoware_auto_msgs::msg::PredictedObjectKinematics>()
         .initial_pose(make_pose(tracked))
         .initial_twist(tracked.twist)
         .initial_acceleration(tracked.acceleration)
         .predicted_paths({});
}

}  // namespace prediction
}  // namespace autoware
