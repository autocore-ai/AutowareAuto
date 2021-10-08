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

#include "lonely_world_prediction/test/make_inputs.hpp"

#include "lonely_world_prediction/init_from_tracked.hpp"

#include "autoware_auto_msgs/msg/object_classification.hpp"
#include "autoware_auto_msgs/msg/shape.hpp"
#include "autoware_auto_msgs/msg/tracked_object.hpp"

namespace autoware
{
namespace prediction
{
namespace test
{
template<>
autoware_auto_msgs::msg::PredictedObjects make()
{
  return from_tracked(make<autoware_auto_msgs::msg::TrackedObjects>());
}

template<>
autoware_auto_msgs::msg::TrackedObjectKinematics make()
{
  // ignore covariance and angular part
  geometry_msgs::msg::TwistWithCovariance twist;
  twist.twist.linear.x = 1.1;

  // acceleration
  geometry_msgs::msg::AccelWithCovariance acceleration;
  acceleration.accel.linear.x = 0.2;

  return autoware_auto_msgs::build<autoware_auto_msgs::msg::TrackedObjectKinematics>()
         .centroid_position(geometry_msgs::msg::Point())
         .position_covariance({})
         .orientation(geometry_msgs::msg::Quaternion())
         .orientation_availability(autoware_auto_msgs::msg::TrackedObjectKinematics::AVAILABLE)
         .twist(twist)
         .acceleration(acceleration)
         .is_stationary(false);
}

template<>
autoware_auto_msgs::msg::TrackedObjects make()
{
  const auto classification =
    autoware_auto_msgs::build<autoware_auto_msgs::msg::ObjectClassification>()
    .classification(autoware_auto_msgs::msg::ObjectClassification::UNKNOWN)
    .probability(1.0);

  const auto shape = autoware_auto_msgs::build<autoware_auto_msgs::msg::Shape>()
    .polygon(make<geometry_msgs::msg::Polygon>())
    .height(2.3F);
  auto object = autoware_auto_msgs::build<autoware_auto_msgs::msg::TrackedObject>()
    .object_id(134)
    .existence_probability(1.0F)
    .classification({classification})
    .kinematics(make<autoware_auto_msgs::msg::TrackedObjectKinematics>())
    .shape({shape});

  autoware_auto_msgs::msg::TrackedObjects tracked;
  tracked.objects.push_back(object);
  return tracked;
}

template<>
geometry_msgs::msg::Polygon make()
{
  geometry_msgs::msg::Polygon polygon;
  auto & points = polygon.points;

  // implicitly assign 0 to z coordinate of all points
  points.resize(4);

  points[0].x = -2.0F;
  points[0].y = -1.0F;

  points[1].x = -2.0F;
  points[1].y = +1.0F;

  points[2].x = +2.0F;
  points[2].y = +1.0F;

  points[3].x = +2.0F;
  points[3].y = -1.0F;

  return polygon;
}

}  // namespace test
}  // namespace prediction
}  // namespace autoware
