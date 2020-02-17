// Copyright 2020 Embotech AG, Zurich, Switzerland
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <gtest/gtest.h>
#include <recordreplay_planner_node/recordreplay_planner_node.hpp>
#include <autoware_auto_msgs/msg/trajectory.hpp>
#include <motion_testing/motion_testing.hpp>
#include <rclcpp/rclcpp.hpp>

#include <chrono>
#include <algorithm>
#include <memory>

using motion::planning::recordreplay_planner_node::RecordReplayPlannerNode;
using motion::motion_testing::make_state;
using std::chrono::system_clock;
using Trajectory = autoware_auto_msgs::msg::Trajectory;
using State = autoware_auto_msgs::msg::VehicleKinematicState;


TEST(mytest_base, basic)
{
  const auto ego_topic = "ego_topic";
  const auto trajectory_topic = "trajectory_topic";
  rclcpp::init(0, nullptr);

  auto plannernode = std::make_shared<RecordReplayPlannerNode>(
    "some_name",
    "",
    ego_topic,
    trajectory_topic,
    0.1
  );

  using PubAllocT = rclcpp::PublisherOptionsWithAllocator<std::allocator<void>>;
  const auto publisher =
    std::make_shared<rclcpp::Node>("recordreplay_node_testpublisher");
  const auto pub =
    publisher->create_publisher<State>(ego_topic, rclcpp::QoS{10}.transient_local(), PubAllocT{});

  rclcpp::executors::SingleThreadedExecutor exec;
  exec.add_node(plannernode);
  exec.add_node(publisher);

  auto dummy_state = std::make_shared<State>();
  pub->publish(*dummy_state);
  EXPECT_NO_THROW(exec.spin_some(std::chrono::milliseconds(100LL)));

  // TODO(s.me) actually do what I planned on doing in the launch_testing file here.
  // This is tracked by https://gitlab.com/autowarefoundation/autoware.auto/AutowareAuto/issues/273.

  rclcpp::shutdown();
}
