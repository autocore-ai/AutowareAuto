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
//
// Co-developed by Tier IV, Inc. and Apex.AI, Inc.

#include "gmock/gmock.h"
#include "gtest/gtest.h"
#include "lonely_world_prediction/lonely_world_prediction.hpp"
#include "lonely_world_prediction/test/make_inputs.hpp"
#include "time_utils/time_utils.hpp"

#include "autoware_auto_msgs/msg/predicted_objects.hpp"

namespace
{
using namespace autoware::prediction;        // NOLINT : this test specific to the namespace
using namespace autoware::prediction::test;  // NOLINT : this test specific to the namespace
using namespace std::chrono_literals;
using testing::Each;
using testing::Eq;

// cppcheck-suppress syntaxError
TEST(parameters, negative_time_step)
{
  EXPECT_THROW(Parameters(-1ms, 1800ms), std::invalid_argument);
}

TEST(parameters, zero_time_step) {
  EXPECT_THROW(Parameters(0ms, 1800ms), std::invalid_argument);
}

TEST(parameters, negative_time_horizon)
{
  EXPECT_THROW(Parameters(27ms, -1ms), std::invalid_argument);
}

TEST(parameters, zero_time_horizon) {
  EXPECT_THROW(Parameters(27ms, 0ms), std::invalid_argument);
}

TEST(parameters, zero_time_step_and_horizon)
{
  EXPECT_THROW(Parameters(0ms, 0ms), std::invalid_argument);
}

TEST(parameters, time_step_larger_than_horizon)
{
  EXPECT_THROW(Parameters(100ms, 50ms), std::invalid_argument);
}

TEST(parameters, time_step_equals_horizon) {
  EXPECT_NO_THROW(Parameters(100ms, 100ms));
}

TEST(parameters, time_step_smaller_than_horizon) {
  EXPECT_NO_THROW(Parameters(100ms, 500ms));
}

TEST(rule_based, stationary)
{
  const auto input = make<autoware_auto_msgs::msg::PredictedObjects>();
  ASSERT_EQ(input.objects.size(), 1UL);

  Parameters parameters(27ms, 1800ms);

  auto output = input;
  predict_stationary(output, parameters);

  ASSERT_EQ(output.objects.size(), input.objects.size());
  const autoware_auto_msgs::msg::PredictedObject & obj = output.objects.front();
  EXPECT_THAT(input.objects.front().kinematics.initial_pose, Eq(obj.kinematics.initial_pose));

  ASSERT_EQ(obj.kinematics.predicted_paths.size(), 1UL);
  const auto & path = obj.kinematics.predicted_paths.front();
  EXPECT_THAT(path.path, Each(Eq(obj.kinematics.initial_pose.pose)));
  EXPECT_THAT(path.time_step, Eq(time_utils::to_message(parameters.time_step())));
  // 66 * 27 ms = 1782 ms, so need one extra to step cover at least the time horizon of 1800 ms
  EXPECT_THAT(path.path.size(), 67UL);
  EXPECT_THAT(path.confidence, Eq(1.0F));
}

}  // namespace
