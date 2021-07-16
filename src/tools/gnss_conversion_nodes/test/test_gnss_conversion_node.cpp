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
//
// Co-developed by Tier IV, Inc. and Apex.AI, Inc.

#include <fake_test_node/fake_test_node.hpp>
#include <gnss_conversion_nodes/gnss_conversion_node.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <gtest/gtest.h>

#include <memory>
#include <string>
#include <vector>

using autoware::gnss_conversion_nodes::GnssConversionNode;
using autoware_auto_msgs::msg::RelativePositionWithCovarianceStamped;

namespace
{

geometry_msgs::msg::TransformStamped get_tf__ecef__enu(
  const sensor_msgs::msg::NavSatFix & msg,
  const geometry_msgs::msg::TransformStamped::_child_frame_id_type & child_frame_id,
  const geometry_msgs::msg::TransformStamped::_header_type::_frame_id_type & frame_id)
{
  geometry_msgs::msg::TransformStamped tf_msg;
  GeographicLib::Geocentric wgs84_to_ecef_convertor{
    GeographicLib::Constants::WGS84_a(),
    GeographicLib::Constants::WGS84_f()};
  std::vector<autoware::common::types::float64_t> r__ecef__enu(9UL, 0.0);
  wgs84_to_ecef_convertor.Forward(
    msg.latitude, msg.longitude, msg.altitude,
    tf_msg.transform.translation.x, tf_msg.transform.translation.y, tf_msg.transform.translation.z,
    r__ecef__enu);
  tf_msg.child_frame_id = child_frame_id;
  tf_msg.header.frame_id = frame_id;
  tf_msg.header.stamp = msg.header.stamp;
  tf2::Matrix3x3 tf__ecef__enu{
    r__ecef__enu[0], r__ecef__enu[1], r__ecef__enu[2],
    r__ecef__enu[3], r__ecef__enu[4], r__ecef__enu[5],
    r__ecef__enu[6], r__ecef__enu[7], r__ecef__enu[8]};
  tf2::Quaternion q;
  tf__ecef__enu.getRotation(q);
  tf_msg.transform.rotation = tf2::toMsg(q);
  return tf_msg;
}

}  // namespace

using TestGnssConversionNode = autoware::tools::testing::FakeTestNode;

/// @test Test that we can convert a message from GNSS to ECEF frame coordinates.
TEST_F(TestGnssConversionNode, PublishAndReceiveMsgWithNoTfConversion) {
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
  auto publisher = create_publisher<sensor_msgs::msg::NavSatFix>("wgs84_position");
  auto subscription = create_subscription<RelativePositionWithCovarianceStamped>(
    "gnss_position", *node,
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

/// @test Test conversion of a message from GNSS to local frame coordinates, using the ENU <-> ECEF
///     conversion coming from TF.
TEST_F(TestGnssConversionNode, PublishAndReceiveMsgConvertToEnu) {
  sensor_msgs::msg::NavSatFix msg{};
  msg.header.stamp.set__sec(42).set__nanosec(42);
  msg.header.frame_id = "fix";
  msg.position_covariance_type = msg.COVARIANCE_TYPE_DIAGONAL_KNOWN;
  msg.status.status = msg.status.STATUS_FIX;
  msg.position_covariance = std::array<autoware::common::types::float64_t, 9UL>{};
  // Coordinate of the Hofbraeuhaus in Munich.
  msg.longitude = 11.5777366;
  msg.latitude = 48.1376098;
  msg.altitude = 515.0;  // Meters above sea level;

  // Create the node.
  rclcpp::NodeOptions node_options{};
  const std::vector<autoware::common::types::float64_t> override_variances{1.0, 2.0, 3.0};
  node_options.append_parameter_override("override_variances", override_variances);
  node_options.append_parameter_override("output_frame_id", "map");
  const auto node{std::make_shared<GnssConversionNode>(node_options)};

  // Set a transformation between ENU <-> ECEF to the TF buffer. We will call the ENU frame "map".
  node->tf_buffer().setTransform(get_tf__ecef__enu(msg, "map", "earth"), "test_node");

  // Check that the received messages are properly converted.
  RelativePositionWithCovarianceStamped::SharedPtr last_msg{};
  auto publisher = create_publisher<sensor_msgs::msg::NavSatFix>("wgs84_position");
  auto subscription = create_subscription<RelativePositionWithCovarianceStamped>(
    "gnss_position", *node,
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
  // We expect the coordinates to be 0 as the base of the ENU frame is exactly the ECEF coordinate
  // we are sending out.
  EXPECT_EQ("map", last_msg->header.frame_id);
  EXPECT_NEAR(last_msg->position.x, 0.0, 1.0);
  EXPECT_NEAR(last_msg->position.y, 0.0, 1.0);
  EXPECT_NEAR(last_msg->position.z, 0.0, 1.0);
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
  auto publisher = create_publisher<sensor_msgs::msg::NavSatFix>("wgs84_position");
  auto subscription = create_subscription<RelativePositionWithCovarianceStamped>(
    "gnss_position", *node,
    [&number_of_received_msgs](
      const RelativePositionWithCovarianceStamped::SharedPtr) {number_of_received_msgs++;});

  const auto dt{std::chrono::milliseconds{100LL}};
  const auto max_wait_time{std::chrono::seconds{1LL}};
  auto time_passed{std::chrono::milliseconds{0LL}};
  while (time_passed < max_wait_time) {
    publisher->publish(msg);
    rclcpp::spin_some(node);
    rclcpp::spin_some(get_fake_node());
    std::this_thread::sleep_for(dt);
    time_passed += dt;
  }
  ASSERT_EQ(0, number_of_received_msgs) << "We should receive no messages in this test";
}

/// @test Test that when tf is not available and a different frame is asked the node throws.
TEST_F(TestGnssConversionNode, ConversionThrowsWhenNoTfAvailable) {
  sensor_msgs::msg::NavSatFix msg{};
  msg.header.stamp.set__sec(42).set__nanosec(42);
  msg.header.frame_id = "fix";
  msg.position_covariance_type = msg.COVARIANCE_TYPE_DIAGONAL_KNOWN;
  msg.status.status = msg.status.STATUS_FIX;
  msg.position_covariance = std::array<autoware::common::types::float64_t, 9UL>{};

  rclcpp::NodeOptions node_options{};
  const std::vector<autoware::common::types::float64_t> override_variances{1.0, 2.0, 3.0};
  node_options.append_parameter_override("override_variances", override_variances);
  node_options.append_parameter_override("output_frame_id", "non_existing_frame");
  const auto node{std::make_shared<GnssConversionNode>(node_options)};

  std::int32_t number_of_received_msgs{};
  auto publisher = create_publisher<sensor_msgs::msg::NavSatFix>("wgs84_position");
  auto subscription = create_subscription<RelativePositionWithCovarianceStamped>(
    "gnss_position", *node,
    [&number_of_received_msgs](
      const RelativePositionWithCovarianceStamped::SharedPtr) {number_of_received_msgs++;});

  const auto dt{std::chrono::milliseconds{100LL}};
  const auto max_wait_time{std::chrono::seconds{1LL}};
  auto time_passed{std::chrono::milliseconds{0LL}};
  while (time_passed < max_wait_time) {
    publisher->publish(msg);
    rclcpp::spin_some(node);
    rclcpp::spin_some(get_fake_node());
    std::this_thread::sleep_for(dt);
    time_passed += dt;
  }
  ASSERT_EQ(0, number_of_received_msgs) << "We should receive no messages in this test";
}

/// @test Test that wrong initialization throws an exception.
TEST_F(TestGnssConversionNode, WrongInitialization) {
  EXPECT_THROW(std::make_shared<GnssConversionNode>(rclcpp::NodeOptions{}), std::runtime_error);
}
