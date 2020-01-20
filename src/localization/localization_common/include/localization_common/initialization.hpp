// Copyright 2017-2019 Apex.AI, Inc.
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

#ifndef LOCALIZATION_COMMON__INITIALIZATION_HPP_
#define LOCALIZATION_COMMON__INITIALIZATION_HPP_

#include <geometry_msgs/msg/transform.hpp>
#include <tf2/buffer_core.h>
// probably include the motion model

namespace autoware
{
namespace localization
{
namespace localization_common
{

template <typename Derived>
class PoseInitializationBase{
    using PoseT = geometry_msgs::msg::Transform;

    PoseT guess_initial(const tf2::BufferCore &);
};

class VehicleOdometryPoseInitialization : public PoseInitializationBase<VehicleOdometryPoseInitialization>{
    using PoseT = geometry_msgs::msg::Transform;

    PoseT guess_initial(const tf2::BufferCore &);
};

}          // namespace autoware
}      // namespace localization
}  // namespace localization_common

#endif