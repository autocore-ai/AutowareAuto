// Copyright 2021 Apex.AI, Inc.
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

#include <gnss_conversion_nodes/gnss_conversion_node.hpp>
#include <rclcpp/rclcpp.hpp>

#include <gtest/gtest.h>

#include <memory>
#include <string>
#include <vector>

using autoware::gnss_conversion_nodes::GnssConversionNode;
using autoware_auto_msgs::msg::RelativePositionWithCovarianceStamped;

class TestGnssConversionNode : public ::testing::Test
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

/// @test Test that we can convert a message.
TEST_F(TestGnssConversionNode, PublishAndReceiveMsg) {
  sensor_msgs::msg::NavSatFix msg{};
  msg.header.stamp.set__sec(42).set__nanosec(42);
  msg.header.frame_id = "fix";
  msg.position_covariance_type = msg.COVARIANCE_TYPE_DIAGONAL_KNOWN;
  msg.status.status = msg.status.STATUS_FIX;
  msg.position_covariance = std::array<autoware::common::types::float64_t, 9UL>{};
  // Coordinate of the Hofbraeuhaus in Munich.
  msg.longitude = 11.5777366;
  msg.latitude = 48.1376098;

  rclcpp::NodeOptions node_options{};
  const std::vector<autoware::common::types::float64_t> override_variances{1.0, 2.0, 3.0};
  node_options.append_parameter_override("override_variances", override_variances);
  const auto node{std::make_shared<GnssConversionNode>(node_options)};

  RelativePositionWithCovarianceStamped::SharedPtr last_msg{};
  auto publisher = create_fake_publisher<sensor_msgs::msg::NavSatFix>("wgs84_position");
  create_result_subscription<GnssConversionNode, RelativePositionWithCovarianceStamped>(
    "ecef_position", node.get(),
    [&last_msg](
      const RelativePositionWithCovarianceStamped::SharedPtr received_msg) {
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
  EXPECT_EQ("earth", last_msg->header.frame_id);
  EXPECT_NEAR(last_msg->position.x, 4177570.0, 1.0);
  EXPECT_NEAR(last_msg->position.y, 855840.0, 1.0);
  EXPECT_NEAR(last_msg->position.z, 4727101.0, 1.0);
  // Check the diagonal values.
  ASSERT_EQ(last_msg->covariance.size(), 9UL);
  EXPECT_DOUBLE_EQ(override_variances[0] * override_variances[0], last_msg->covariance[0]);
  EXPECT_DOUBLE_EQ(override_variances[1] * override_variances[1], last_msg->covariance[4]);
  EXPECT_DOUBLE_EQ(override_variances[2] * override_variances[2], last_msg->covariance[8]);

  for (auto i = 0U; i < last_msg->covariance.size(); ++i) {
    if ((i == 0) || (i == 4) || (i == 8)) {continue;}
    EXPECT_DOUBLE_EQ(last_msg->covariance[i], 0.0) << "Off-center value is not 0.0 for i = " << i;
  }

  SUCCEED();
}

/// @test Test that when there is no fix the message is not converted.
TEST_F(TestGnssConversionNode, NoConversionWhenNoGnssFix) {
  sensor_msgs::msg::NavSatFix msg{};
  msg.status.status = msg.status.STATUS_NO_FIX;

  rclcpp::NodeOptions node_options{};
  const std::vector<autoware::common::types::float64_t> override_variances{1.0, 2.0, 3.0};
  node_options.append_parameter_override("override_variances", override_variances);
  const auto node{std::make_shared<GnssConversionNode>(node_options)};

  std::int32_t number_of_received_msgs{};
  auto publisher = create_fake_publisher<sensor_msgs::msg::NavSatFix>("wgs84_position");
  create_result_subscription<GnssConversionNode, RelativePositionWithCovarianceStamped>(
    "ecef_position", node.get(),
    [&number_of_received_msgs](
      const RelativePositionWithCovarianceStamped::SharedPtr) {number_of_received_msgs++;});

  const auto dt{std::chrono::milliseconds{100LL}};
  const auto max_wait_time{std::chrono::seconds{2LL}};
  auto time_passed{std::chrono::milliseconds{0LL}};
  while (time_passed < max_wait_time) {
    publisher->publish(msg);
    rclcpp::spin_some(node);
    rclcpp::spin_some(get_fake_node());
    std::this_thread::sleep_for(dt);
    time_passed += dt;
  }
  if (number_of_received_msgs > 0) {
    FAIL() << "A message was converted while it shouldn't have been.";
  }
  // We never received a message which is what we expected.
  SUCCEED();
}

/// @test Test that wrong initialization throws an exception.
TEST_F(TestGnssConversionNode, WrongInitialization) {
  EXPECT_THROW(std::make_shared<GnssConversionNode>(rclcpp::NodeOptions{}), std::runtime_error);
}
