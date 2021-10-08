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

#include <memory>

#include "fake_test_node/fake_test_node.hpp"
#include "gtest/gtest.h"
#include "lonely_world_prediction/test/make_inputs.hpp"
#include "prediction_nodes/prediction_node.hpp"

#include "autoware_auto_msgs/msg/object_classification.hpp"
#include "autoware_auto_msgs/msg/shape.hpp"
#include "autoware_auto_msgs/msg/tracked_object.hpp"
#include "autoware_auto_msgs/msg/tracked_object_kinematics.hpp"

namespace
{
using PredictionNodeTest = autoware::tools::testing::FakeTestNode;

using namespace autoware::prediction;        // NOLINT : this test specific to the namespace
using namespace autoware::prediction::test;  // NOLINT : this test specific to the namespace
using namespace std::chrono_literals;


// cppcheck-suppress syntaxError
TEST_F(PredictionNodeTest, BasicInputOutput)
{
  rclcpp::NodeOptions options{};
  const auto node = std::make_shared<PredictionNode>(options);

  PredictionNode::PredictedMsgT::SharedPtr last_received_msg{};
  const auto input_topic = "/tracked_objects";
  const auto output_topic = "/prediction/predicted_objects";

  auto fake_publisher = create_publisher<PredictionNode::TrackedMsgT>(input_topic, 1s);

  auto result_subscription = create_subscription<PredictionNode::PredictedMsgT>(
    output_topic,
    *node,  //
    [&last_received_msg](const decltype(last_received_msg) msg) {last_received_msg = msg;});

  const auto input_msg = make<PredictionNode::TrackedMsgT>();

  const auto dt{100ms};
  const auto max_wait_time{std::chrono::seconds{1LL}};
  auto time_passed{std::chrono::milliseconds{0LL}};
  while (!last_received_msg) {
    fake_publisher->publish(input_msg);
    rclcpp::spin_some(node);
    rclcpp::spin_some(get_fake_node());
    std::this_thread::sleep_for(dt);
    time_passed += dt;
    if (time_passed > max_wait_time) {
      FAIL() << "Did not receive a message soon enough.";
    }
  }

  ASSERT_TRUE(last_received_msg);
  ASSERT_EQ(last_received_msg->objects.size(), 1UL);
}

}  // namespace
