// Copyright 2020 Apex.AI, Inc.
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

/// \copyright Copyright 2020 Apex.AI, Inc.
/// All rights reserved.

#include <gtest/gtest.h>
#include <state_estimation_node/state_estimation_node.hpp>

#include <memory>
#include <string>
#include <vector>

using nav_msgs::msg::Odometry;
using autoware::prediction::state_estimation_node::StateEstimationNode;

namespace
{
rclcpp::Time to_ros_time(const std::chrono::system_clock::time_point & time_point)
{
  using std::chrono::duration_cast;
  using std::chrono::nanoseconds;
  return rclcpp::Time{duration_cast<nanoseconds>(time_point.time_since_epoch()).count()};
}
}  // namespace

class StateEstimationNodeTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    ASSERT_FALSE(rclcpp::is_initialized());
    rclcpp::init(0, nullptr);
    ASSERT_TRUE(rclcpp::is_initialized());
    m_fake_odometry_node = std::make_shared<rclcpp::Node>("fake_odometry_node");
  }

  void TearDown() override
  {
    m_fake_odometry_subscription.reset();
    m_fake_odometry_publisher.reset();
    m_fake_odometry_node.reset();
    (void)rclcpp::shutdown();
  }

  void create_fake_odom_publisher(
    const std::string & topic,
    const std::chrono::milliseconds & timeout = std::chrono::seconds{10LL})
  {
    m_fake_odometry_publisher = m_fake_odometry_node->create_publisher<Odometry>(
      topic, 10);

    std::chrono::milliseconds spent_time{0LL};
    std::chrono::milliseconds dt{100LL};
    while (m_fake_odometry_node->count_subscribers(topic) < 1) {
      spent_time += dt;
      ASSERT_LT(spent_time, timeout) << "Nobody is listening to the fake topic we publish.";
      std::this_thread::sleep_for(dt);
    }
  }

  template<typename NodeT>
  void create_result_odom_subscription(
    const std::string & topic,
    NodeT * node_under_test,
    std::function<void(const Odometry::SharedPtr msg)> callback,
    const std::chrono::milliseconds & timeout = std::chrono::seconds{10LL})
  {
    ASSERT_NE(node_under_test, nullptr);
    m_fake_odometry_subscription = m_fake_odometry_node->create_subscription<Odometry>(
      topic, 10, callback);

    std::chrono::milliseconds spent_time{0LL};
    std::chrono::milliseconds dt{100LL};
    while (node_under_test->count_publishers(topic) < 1) {
      spent_time += dt;
      ASSERT_LT(spent_time, timeout) << "The node under test is not publishing what we listen to.";
      std::this_thread::sleep_for(dt);
    }
  }

  rclcpp::Node::SharedPtr get_fake_odometry_node() {return m_fake_odometry_node;}
  rclcpp::Publisher<Odometry> & get_fake_odometry_publisher() {return *m_fake_odometry_publisher;}

private:
  rclcpp::Node::SharedPtr m_fake_odometry_node{nullptr};
  rclcpp::Publisher<Odometry>::SharedPtr m_fake_odometry_publisher{nullptr};
  rclcpp::Subscription<Odometry>::SharedPtr m_fake_odometry_subscription{nullptr};
};

/// @test Test that if we publish one message, it generates a state estimate which is sent out.
TEST_F(StateEstimationNodeTest, publish_and_receive_odom_message) {
  nav_msgs::msg::Odometry msg{};
  msg.header.frame_id = "map";
  msg.header.stamp.sec = 5;
  msg.header.stamp.nanosec = 12345U;
  msg.pose.covariance[0] = 1.0;
  msg.pose.covariance[7] = 1.0;
  msg.twist.covariance[0] = 1.0;
  msg.twist.covariance[7] = 1.0;

  rclcpp::NodeOptions node_options{};
  const std::string param_file_name{"state_estimation_node_test.param.yaml"};
  node_options.arguments({"__params:=" + param_file_name});
  const auto node{std::make_shared<StateEstimationNode>(
      "state_estimation_node", "state_estimation_namespace", node_options)};

  auto count_received_msgs{0};
  create_fake_odom_publisher("/odom_topic_1");
  create_result_odom_subscription("/state_estimation_namespace/filtered_state", node.get(),
    [&count_received_msgs](
      const Odometry::SharedPtr) {
      count_received_msgs++;
    });

  const auto dt{std::chrono::milliseconds{100LL}};
  const auto max_wait_time{std::chrono::seconds{10LL}};
  auto time_passed{std::chrono::milliseconds{0LL}};
  while (count_received_msgs < 1) {
    get_fake_odometry_publisher().publish(msg);
    rclcpp::spin_some(node);
    rclcpp::spin_some(get_fake_odometry_node());
    std::this_thread::sleep_for(dt);
    time_passed += dt;
    if (time_passed > max_wait_time) {
      FAIL() << "Did not receive a message soon enough.";
    }
  }
  SUCCEED();
}

/// @test Test that we can track an object moving in a straight line.
TEST_F(StateEstimationNodeTest, track_object_straight_line) {
  nav_msgs::msg::Odometry msg{};
  msg.header.frame_id = "map";
  msg.pose.covariance[0] = 1.0;
  msg.pose.covariance[7] = 1.0;
  msg.twist.covariance[0] = 1.0;
  msg.twist.covariance[7] = 1.0;

  rclcpp::NodeOptions node_options{};
  const std::string param_file_name{"state_estimation_node_test.param.yaml"};
  node_options.arguments({"__params:=" + param_file_name});
  const auto node{std::make_shared<StateEstimationNode>(
      "state_estimation_node", "state_estimation_namespace", node_options)};

  std::vector<Odometry::SharedPtr> received_msgs{0};
  create_fake_odom_publisher("/odom_topic_1");
  create_result_odom_subscription("/state_estimation_namespace/filtered_state", node.get(),
    [&received_msgs](
      const Odometry::SharedPtr msg) {
      received_msgs.push_back(msg);
    });

  const auto dt{std::chrono::milliseconds{100LL}};
  const auto speed = 1.0;
  const auto starting_time_point = std::chrono::system_clock::now();
  std::chrono::seconds total_travel_time{2LL};
  size_t messages_sent{};
  msg.header.stamp = to_ros_time(starting_time_point);
  for (std::chrono::milliseconds time_passed{};
    time_passed <= total_travel_time;
    time_passed += dt)
  {
    msg.header.stamp = to_ros_time(starting_time_point + time_passed);
    std::chrono::duration<double> seconds_passed{time_passed};
    msg.pose.pose.position.x = seconds_passed.count() * speed;
    msg.pose.pose.position.y = seconds_passed.count() * speed;
    msg.twist.twist.linear.x = speed;
    msg.twist.twist.linear.y = speed;
    get_fake_odometry_publisher().publish(msg);
    rclcpp::spin_some(node);
    rclcpp::spin_some(get_fake_odometry_node());
    std::this_thread::sleep_for(dt);
    messages_sent++;
  }
  const auto distance_travelled = speed * total_travel_time.count();

  const auto max_wait_time{std::chrono::seconds{10LL}};
  std::chrono::milliseconds time_passed{0LL};
  while (received_msgs.size() < messages_sent) {
    time_passed += dt;
    rclcpp::spin_some(node);
    rclcpp::spin_some(get_fake_odometry_node());
    std::this_thread::sleep_for(dt);
    ASSERT_LT(time_passed, max_wait_time) <<
      "Some messages were dropped. Received: " << received_msgs.size();
  }
  const double epsilon = 0.2;
  EXPECT_NEAR(distance_travelled, received_msgs.back()->pose.pose.position.x, epsilon);
  EXPECT_NEAR(distance_travelled, received_msgs.back()->pose.pose.position.y, epsilon);
}

// TODO(igor): Add test for the case when we publish on a timer.
