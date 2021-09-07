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
//
// Co-developed by Tier IV, Inc. and Apex.AI, Inc.

/// \copyright Copyright 2020 Apex.AI, Inc.
/// All rights reserved.

#include <gtest/gtest.h>
#include <covariance_insertion_nodes/covariance_insertion_node.hpp>

#include <memory>
#include <string>
#include <vector>

using autoware::covariance_insertion_nodes::CovarianceInsertionNode;

class CovarianceInsertionNodeTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    ASSERT_FALSE(rclcpp::ok());
    rclcpp::init(0, nullptr);
    ASSERT_TRUE(rclcpp::ok());
    m_fake_node = std::make_shared<rclcpp::Node>("fake_node");
  }

  void TearDown() override
  {
    m_fake_subscription.reset();
    m_fake_publisher.reset();
    m_fake_node.reset();
    (void)rclcpp::shutdown();
  }

  template<typename MessageT>
  std::shared_ptr<rclcpp::Publisher<MessageT>> create_fake_publisher(
    const std::string & topic,
    const std::chrono::milliseconds & timeout = std::chrono::seconds{10LL})
  {
    auto publisher{m_fake_node->create_publisher<MessageT>(topic, 10)};
    m_fake_publisher = publisher;

    std::chrono::milliseconds spent_time{0LL};
    std::chrono::milliseconds dt{100LL};
    while (m_fake_node->count_subscribers(topic) < 1) {
      spent_time += dt;
      if (spent_time > timeout) {
        throw std::runtime_error("Nobody is listening to the fake topic we publish.");
      }
      std::this_thread::sleep_for(dt);
    }
    return publisher;
  }

  template<typename NodeT, typename MessageT>
  void create_result_subscription(
    const std::string & topic,
    NodeT * node_under_test,
    std::function<void(const std::shared_ptr<MessageT> msg)> callback,
    const std::chrono::milliseconds & timeout = std::chrono::seconds{10LL})
  {
    ASSERT_NE(node_under_test, nullptr);
    m_fake_subscription = m_fake_node->create_subscription<MessageT>(
      topic, 10, callback);

    std::chrono::milliseconds spent_time{0LL};
    std::chrono::milliseconds dt{100LL};
    while (node_under_test->count_publishers(topic) < 1) {
      spent_time += dt;
      ASSERT_LT(spent_time, timeout) << "The node under test is not publishing what we listen to.";
      std::this_thread::sleep_for(dt);
    }
  }

  rclcpp::Node::SharedPtr get_fake_node() {return m_fake_node;}

private:
  rclcpp::Node::SharedPtr m_fake_node{nullptr};
  rclcpp::PublisherBase::SharedPtr m_fake_publisher{nullptr};
  rclcpp::SubscriptionBase::SharedPtr m_fake_subscription{nullptr};
};

/// @test Test that we can add covariance to Odometry message.
TEST_F(CovarianceInsertionNodeTest, PublishAndReceiveOdomMessage) {
  nav_msgs::msg::Odometry msg{};

  rclcpp::NodeOptions node_options{};
  node_options.append_parameter_override("input_msg_type", "Odometry");
  node_options.append_parameter_override(
    "override_covariance.pose",
    std::vector<autoware::common::types::float64_t>(36UL, 42.0));
  node_options.append_parameter_override(
    "override_covariance.twist",
    std::vector<autoware::common::types::float64_t>(36UL, 23.0));
  const auto node{std::make_shared<CovarianceInsertionNode>(node_options)};

  nav_msgs::msg::Odometry::SharedPtr last_msg{};
  auto publisher = create_fake_publisher<nav_msgs::msg::Odometry>("messages");
  create_result_subscription<CovarianceInsertionNode, nav_msgs::msg::Odometry>(
    "messages_with_overriden_covariance", node.get(),
    [&last_msg](
      const nav_msgs::msg::Odometry::SharedPtr received_msg) {
      last_msg = received_msg;
    });

  const auto dt{std::chrono::milliseconds{100LL}};
  const auto max_wait_time{std::chrono::seconds{10LL}};
  auto time_passed{std::chrono::milliseconds{0LL}};
  while (!last_msg) {
    publisher->publish(msg);
    rclcpp::spin_some(node);
    rclcpp::spin_some(get_fake_node());
    std::this_thread::sleep_for(dt);
    time_passed += dt;
    if (time_passed > max_wait_time) {
      FAIL() << "Did not receive a message soon enough.";
    }
  }
  for (const auto & val : last_msg->pose.covariance) {
    EXPECT_DOUBLE_EQ(42.0, val);
  }
  for (const auto & val : last_msg->twist.covariance) {
    EXPECT_DOUBLE_EQ(23.0, val);
  }
  SUCCEED();
}

/// @test Test that we can add covariance to Pose message making it a PoseWithCovariance one.
TEST_F(CovarianceInsertionNodeTest, AddCovariancesToPose) {
  geometry_msgs::msg::Pose msg{};

  rclcpp::NodeOptions node_options{};
  node_options.append_parameter_override("input_topic", "messages");
  node_options.append_parameter_override("input_msg_type", "Pose");
  node_options.append_parameter_override(
    "override_covariance.directly",
    std::vector<autoware::common::types::float64_t>(36UL, 42.0));
  const auto node{std::make_shared<CovarianceInsertionNode>(node_options)};

  geometry_msgs::msg::PoseWithCovariance::SharedPtr last_msg{};
  auto publisher = create_fake_publisher<geometry_msgs::msg::Pose>("messages");
  create_result_subscription<CovarianceInsertionNode, geometry_msgs::msg::PoseWithCovariance>(
    "messages_with_overriden_covariance", node.get(),
    [&last_msg](
      const geometry_msgs::msg::PoseWithCovariance::SharedPtr received_msg) {
      last_msg = received_msg;
    });

  const auto dt{std::chrono::milliseconds{100LL}};
  const auto max_wait_time{std::chrono::seconds{10LL}};
  auto time_passed{std::chrono::milliseconds{0LL}};
  while (!last_msg) {
    publisher->publish(msg);
    rclcpp::spin_some(node);
    rclcpp::spin_some(get_fake_node());
    std::this_thread::sleep_for(dt);
    time_passed += dt;
    if (time_passed > max_wait_time) {
      FAIL() << "Did not receive a message soon enough.";
    }
  }
  for (const auto & val : last_msg->covariance) {
    EXPECT_DOUBLE_EQ(42.0, val);
  }
  SUCCEED();
}

/// @test Test that we detect setting covariances to wrong entries.
TEST_F(CovarianceInsertionNodeTest, FailSettingTwistToPose) {
  rclcpp::NodeOptions node_options{};
  node_options.append_parameter_override("input_topic", "messages");
  node_options.append_parameter_override("input_msg_type", "Pose");
  node_options.append_parameter_override(
    "override_covariance.twist",
    std::vector<autoware::common::types::float64_t>(36UL, 42.0));
  // Throw because a pose has no ".twist" field.
  EXPECT_THROW(CovarianceInsertionNode{node_options}, std::runtime_error);
}

/// @test Test that we detect setting covariance of a wrong size.
TEST_F(CovarianceInsertionNodeTest, FailOnWrongNumberOfEntries) {
  const auto wrong_covariane_size{42UL};
  rclcpp::NodeOptions node_options{};
  node_options.append_parameter_override("input_topic", "messages");
  node_options.append_parameter_override("input_msg_type", "geometry_msgs/msg/Pose");
  node_options.append_parameter_override(
    "override_covariance.twist",
    std::vector<autoware::common::types::float64_t>(wrong_covariane_size, 42.0));
  EXPECT_THROW(CovarianceInsertionNode{node_options}, std::runtime_error);
}
