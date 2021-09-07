// Copyright 2021 Apex.AI, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//    http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//
// Co-developed by Tier IV, Inc. and Apex.AI, Inc.

/// \copyright Copyright 2021 Apex.AI, Inc.
/// All rights reserved.

#include <fake_test_node/fake_test_node.hpp>
#include <gtest/gtest.h>
#include <state_estimation_nodes/state_estimation_node.hpp>

#include <tf2_eigen/tf2_eigen.h>

#include <memory>
#include <string>
#include <vector>

using nav_msgs::msg::Odometry;
using geometry_msgs::msg::PoseWithCovarianceStamped;
using autoware::common::state_estimation::StateEstimationNode;
using autoware_auto_msgs::msg::RelativePositionWithCovarianceStamped;
using autoware::common::types::float64_t;
using autoware::common::types::bool8_t;

namespace
{
rclcpp::Time to_ros_time(const std::chrono::system_clock::time_point & time_point)
{
  using std::chrono::duration_cast;
  using std::chrono::nanoseconds;
  return rclcpp::Time{duration_cast<nanoseconds>(time_point.time_since_epoch()).count()};
}

tf2::TimePoint to_tf_time_point(const nav_msgs::msg::Odometry::_header_type::_stamp_type & stamp)
{
  using std::chrono::seconds;
  using std::chrono::nanoseconds;
  return tf2::TimePoint{seconds{stamp.sec} + nanoseconds{stamp.nanosec}};
}

template<typename TfBufferT, typename MsgT, typename NodeT, typename FakeNodeT>
void wait_for_tf(
  TfBufferT & buffer,
  const NodeT & node_under_test,
  const FakeNodeT & fake_node,
  const MsgT & msg,
  const std_msgs::msg::Header::_frame_id_type & target_frame,
  const std_msgs::msg::Header::_frame_id_type & source_frame,
  const std::chrono::milliseconds & max_wait_time,
  const std::chrono::milliseconds & dt)
{
  std::chrono::milliseconds wait_for_tf_time{};
  while (
    !buffer.canTransform(
      target_frame, source_frame,
      to_tf_time_point(msg.header.stamp)) &&
    (wait_for_tf_time < max_wait_time))
  {
    rclcpp::spin_some(node_under_test);
    rclcpp::spin_some(fake_node);
    std::this_thread::sleep_for(dt / 10);
    wait_for_tf_time += dt / 10;
  }
  if (wait_for_tf_time >= max_wait_time) {
    FAIL() << "Could not get tf in time.";
  }
}

PoseWithCovarianceStamped create_empty_pose(const std::string & frame_id)
{
  PoseWithCovarianceStamped msg{};
  msg.header.frame_id = frame_id;
  msg.pose.covariance[0] = 0.1;
  msg.pose.covariance[7] = 0.1;
  msg.pose.covariance[14] = 0.1;
  msg.pose.covariance[21] = 0.1;
  msg.pose.covariance[28] = 0.1;
  msg.pose.covariance[35] = 0.1;
  return msg;
}

RelativePositionWithCovarianceStamped create_empty_relative_pose(
  const std::string & frame_id,
  const std::string & child_frame_id)
{
  RelativePositionWithCovarianceStamped msg{};
  msg.header.frame_id = frame_id;
  msg.child_frame_id = child_frame_id;
  msg.covariance[0] = 0.1;
  msg.covariance[4] = 0.1;
  msg.covariance[8] = 0.1;
  return msg;
}

rclcpp::NodeOptions get_default_options(const bool8_t data_driven, const bool8_t publish_tf)
{
  rclcpp::NodeOptions node_options{};
  node_options.append_parameter_override(
    "topics.input_pose", std::vector<std::string>{"/pose_topic_1"});
  node_options.append_parameter_override(
    "topics.input_relative_pos", std::vector<std::string>{"/relative_pos_topic_1"});
  node_options.append_parameter_override("data_driven", data_driven);
  node_options.append_parameter_override("publish_tf", publish_tf);
  node_options.append_parameter_override("frame_id", "map");
  node_options.append_parameter_override("child_frame_id", "base_link");
  node_options.append_parameter_override("mahalanobis_threshold", 10.0);
  node_options.append_parameter_override(
    "state_variances", std::vector<float64_t>{
      1.0, 1.0, 1.0, 1.0, 1.0, 1.0,  // Position variances.
      1.0, 1.0, 1.0, 1.0, 1.0, 1.0,  // Velocity variances.
      1.0, 1.0, 1.0, 1.0, 1.0, 1.0   // Acceleration variances.
    });
  node_options.append_parameter_override(
    "process_noise_variances.acceleration", std::vector<float64_t>{1.0, 1.0, 1.0, 1.0, 1.0, 1.0});
  return node_options;
}

}  // namespace


struct ParameterBundle
{
  bool8_t publish_tf{};
};

constexpr ParameterBundle kPublish{true};
constexpr ParameterBundle kNoPublish{false};

using StateEstimationNodeTest =
  autoware::tools::testing::FakeTestNodeParametrized<ParameterBundle>;

INSTANTIATE_TEST_CASE_P(
  StateEstimationNodeTests,
  StateEstimationNodeTest,
  ::testing::Values(kPublish, kNoPublish), /*This comment needed to shut off a warning*/);

/// @test Test that if we publish one message, it generates a state estimate which is sent out.
TEST_P(StateEstimationNodeTest, PublishAndReceivePoseMessage) {
  auto msg = create_empty_pose("map");
  const bool8_t data_driven = true;
  rclcpp::NodeOptions node_options = get_default_options(data_driven, GetParam().publish_tf);
  const auto node{std::make_shared<StateEstimationNode>(node_options)};

  auto count_received_msgs{0};
  auto fake_pose_publisher = create_publisher<PoseWithCovarianceStamped>("/pose_topic_1");
  auto result_odom_subscription = create_subscription<Odometry>(
    "/filtered_state", *node,
    [&count_received_msgs](
      const Odometry::SharedPtr) {
      count_received_msgs++;
    });

  const auto dt{std::chrono::milliseconds{100LL}};
  const auto max_wait_time{std::chrono::seconds{10LL}};
  auto time_passed{std::chrono::milliseconds{0LL}};
  while (count_received_msgs < 1) {
    msg.header.stamp = to_ros_time(std::chrono::system_clock::now());
    fake_pose_publisher->publish(msg);
    rclcpp::spin_some(node);
    rclcpp::spin_some(get_fake_node());
    std::this_thread::sleep_for(dt / 10);
    time_passed += dt;
    if (time_passed > max_wait_time) {
      FAIL() << "Did not receive a message soon enough.";
    }
  }
  if (GetParam().publish_tf) {
    wait_for_tf(get_tf_buffer(), node, get_fake_node(), msg, "map", "base_link", max_wait_time, dt);
    const auto transform{
      get_tf_buffer().lookupTransform("map", "base_link", to_tf_time_point(msg.header.stamp))};
    EXPECT_EQ(transform.header.frame_id, "map");
    EXPECT_EQ(transform.child_frame_id, "base_link");
    EXPECT_EQ(transform.header.stamp.sec, msg.header.stamp.sec);
    EXPECT_EQ(transform.header.stamp.nanosec, msg.header.stamp.nanosec);
    EXPECT_DOUBLE_EQ(transform.transform.translation.x, 0.0);
    EXPECT_DOUBLE_EQ(transform.transform.translation.y, 0.0);
    EXPECT_DOUBLE_EQ(transform.transform.translation.z, 0.0);
  } else {
    EXPECT_FALSE(
      get_tf_buffer().canTransform("map", "base_link", to_tf_time_point(msg.header.stamp)));
  }
  SUCCEED();
}

/// @test Test that we can track an object moving in a straight line.
TEST_P(StateEstimationNodeTest, TrackObjectStraightLine) {
  geometry_msgs::msg::TransformStamped transform;
  auto expected_child_frame_id = "base_link";

  auto pose_msg = create_empty_pose("map");
  // Orient the pose along the {1, 1, 1} vector.
  pose_msg.pose.pose.orientation.w = 0.854;
  pose_msg.pose.pose.orientation.x = -0.354;
  pose_msg.pose.pose.orientation.y = -0.354;
  pose_msg.pose.pose.orientation.z = 0.146;
  auto relative_pose_msg = create_empty_relative_pose("map", "base_link");

  const bool8_t data_driven = true;
  rclcpp::NodeOptions node_options = get_default_options(data_driven, GetParam().publish_tf);
  node_options.append_parameter_override("child_frame_id", expected_child_frame_id);
  const auto node{std::make_shared<StateEstimationNode>(node_options)};

  std::vector<Odometry::SharedPtr> received_msgs{0};
  auto fake_pose_publisher = create_publisher<PoseWithCovarianceStamped>("/pose_topic_1");
  auto fake_relative_pose_publisher = create_publisher<RelativePositionWithCovarianceStamped>(
    "/relative_pos_topic_1");
  auto result_odom_subscription = create_subscription<Odometry>(
    "/filtered_state", *node,
    [&received_msgs](
      const Odometry::SharedPtr msg) {
      received_msgs.push_back(msg);
    });

  const auto dt{std::chrono::milliseconds{100LL}};
  const auto speed = 2.0;
  const auto starting_time_point = std::chrono::system_clock::now();
  std::chrono::seconds total_travel_time{2LL};
  size_t messages_sent{};

  std::vector<std::string> msg_types{"pose", "relative_pose"};

  // cppcheck-suppress syntaxError // Trust me, this is valid C++ syntax.
  for (struct { std::chrono::milliseconds time_passed{}; std::size_t msg_idx{}; } iter;
    iter.time_passed <= total_travel_time;
    iter.time_passed += dt, iter.msg_idx = (iter.msg_idx + 1U) % msg_types.size())
  {
    const auto stamp = to_ros_time(starting_time_point + iter.time_passed);
    std::chrono::duration<float64_t> seconds_passed{iter.time_passed};
    if (msg_types[iter.msg_idx] == "pose") {
      pose_msg.header.stamp = stamp;
      pose_msg.pose.pose.position.x = seconds_passed.count() * speed;
      pose_msg.pose.pose.position.y = seconds_passed.count() * speed;
      pose_msg.pose.pose.position.z = seconds_passed.count() * speed;
      fake_pose_publisher->publish(pose_msg);
    } else if (msg_types[iter.msg_idx] == "relative_pose") {
      relative_pose_msg.header.stamp = stamp;
      relative_pose_msg.position.x = seconds_passed.count() * speed;
      relative_pose_msg.position.y = seconds_passed.count() * speed;
      relative_pose_msg.position.z = seconds_passed.count() * speed;
      fake_relative_pose_publisher->publish(relative_pose_msg);
    }
    rclcpp::spin_some(node);
    rclcpp::spin_some(get_fake_node());
    // We don't have to wait in real time.
    std::this_thread::sleep_for(dt / 100);
    messages_sent++;
  }
  const auto distance_travelled = speed * total_travel_time.count();

  const auto max_wait_time{std::chrono::seconds{10LL}};
  std::chrono::milliseconds time_passed{0LL};
  while (received_msgs.size() < messages_sent) {
    time_passed += dt;
    rclcpp::spin_some(node);
    rclcpp::spin_some(get_fake_node());
    std::this_thread::sleep_for(dt);
    ASSERT_LT(time_passed, max_wait_time) <<
      "Some messages were dropped. Received: " << received_msgs.size();
  }
  if (GetParam().publish_tf) {
    auto & msg = *received_msgs.back();
    wait_for_tf(
      get_tf_buffer(),
      node, get_fake_node(),
      msg, "map", expected_child_frame_id, max_wait_time, dt);
    const auto transform{get_tf_buffer().lookupTransform(
        "map", expected_child_frame_id, to_tf_time_point(msg.header.stamp))};
    EXPECT_EQ(transform.header.frame_id, "map");
    EXPECT_EQ(transform.child_frame_id, expected_child_frame_id);
    EXPECT_EQ(transform.header.stamp.sec, msg.header.stamp.sec);
    EXPECT_EQ(transform.header.stamp.nanosec, msg.header.stamp.nanosec);
    EXPECT_DOUBLE_EQ(transform.transform.translation.x, msg.pose.pose.position.x);
    EXPECT_DOUBLE_EQ(transform.transform.translation.y, msg.pose.pose.position.y);
    EXPECT_DOUBLE_EQ(transform.transform.translation.z, msg.pose.pose.position.z);
  } else {
    EXPECT_FALSE(
      get_tf_buffer().canTransform(
        "map", expected_child_frame_id, to_tf_time_point(pose_msg.header.stamp)));
  }
  const float64_t epsilon = 0.2;
  ASSERT_FALSE(received_msgs.empty());
  EXPECT_EQ(received_msgs.back()->child_frame_id, expected_child_frame_id);

  EXPECT_NEAR(distance_travelled, received_msgs.back()->pose.pose.position.x, epsilon);
  EXPECT_NEAR(distance_travelled, received_msgs.back()->pose.pose.position.y, epsilon);
  EXPECT_NEAR(distance_travelled, received_msgs.back()->pose.pose.position.z, epsilon);
  EXPECT_NEAR(
    std::sqrt(3.0 * speed * speed), received_msgs.back()->twist.twist.linear.x, epsilon);
  EXPECT_NEAR(0.0, received_msgs.back()->twist.twist.linear.y, epsilon);
}

/// @test Test for the case when we publish on a timer.
TEST_F(StateEstimationNodeTest, PublishOnTimer) {
  auto msg = create_empty_pose("map");
  msg.header.stamp.sec = 5;
  msg.header.stamp.nanosec = 12345U;

  const bool8_t data_driven = false;
  const bool8_t publish_tf = false;
  rclcpp::NodeOptions node_options = get_default_options(data_driven, publish_tf);
  node_options.append_parameter_override("output_frequency", 10.0);
  const auto node{std::make_shared<StateEstimationNode>(node_options)};

  auto count_received_msgs{0};
  auto fake_pose_publisher = create_publisher<PoseWithCovarianceStamped>("/pose_topic_1");
  auto result_odom_subscription = create_subscription<Odometry>(
    "/filtered_state", *node,
    [&count_received_msgs](
      const Odometry::SharedPtr) {
      count_received_msgs++;
    });

  // Check that before the node receives the first odometry message it is not publishing.
  const auto dt{std::chrono::milliseconds{100LL}};
  auto max_wait_time{std::chrono::seconds{1LL}};
  auto time_passed{std::chrono::milliseconds{0LL}};
  while (time_passed < max_wait_time) {
    rclcpp::spin_some(node);
    rclcpp::spin_some(get_fake_node());
    std::this_thread::sleep_for(dt / 10);
    time_passed += dt;
    if (count_received_msgs > 0) {
      FAIL() << "The node should not have published before receiving an odometry message.";
    }
  }

  // Check that after the node receives the first odometry message it continuously publishes.
  max_wait_time = std::chrono::seconds{10LL};
  time_passed = std::chrono::milliseconds{0LL};
  const auto minimum_number_of_messages{5};
  while (count_received_msgs < minimum_number_of_messages) {
    if (count_received_msgs < 1) {
      // We want to stop publishing after receiving the first message as publishing is only needed
      // here to enable timer-based publishing of the node under test.
      msg.header.stamp = to_ros_time(std::chrono::system_clock::now());
      fake_pose_publisher->publish(msg);
    }
    rclcpp::spin_some(node);
    rclcpp::spin_some(get_fake_node());
    std::this_thread::sleep_for(dt);
    time_passed += dt;
    if (time_passed > max_wait_time) {
      FAIL() << "Did not receive enough messages.";
    }
  }
  SUCCEED();
}
