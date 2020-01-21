// Copyright 2020 Apex.AI, Inc.
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

#ifndef TEST_INITIALIZATION_HPP_
#define TEST_INITIALIZATION_HPP_
#include <gtest/gtest.h>
#include <localization_common/initialization.hpp>

namespace autoware
{
namespace localization
{
namespace localization_common
{
geometry_msgs::msg::TransformStamped make_transform(
  float ang_x, float ang_y, float ang_z,
  float x, float y, float z);

void check_transform_eq(
  const geometry_msgs::msg::TransformStamped & t1,
  const geometry_msgs::msg::TransformStamped & t2);

geometry_msgs::msg::Transform get_interpolation(
  const geometry_msgs::msg::Transform & tf1, const geometry_msgs::msg::Transform & tf2,
  float ratio);

struct BestEffortInitializerTestParams
{
  geometry_msgs::msg::TransformStamped tf0;
  geometry_msgs::msg::TransformStamped tf1;
  std::chrono::milliseconds dt;    // time diff between two set transforms
  std::chrono::milliseconds dt_small;    // time diff less than dt (for interpolation)
};

}          // namespace autoware
}      // namespace localization
}  // namespace localization_common

#endif  // TEST_INITIALIZATION_HPP_
