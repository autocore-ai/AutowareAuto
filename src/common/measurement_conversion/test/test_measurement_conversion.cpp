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

/// \copyright Copyright 2021 Apex.AI, Inc.
/// All rights reserved.

#include <gtest/gtest.h>

#include <measurement_conversion/measurement_conversion.hpp>
#include <common/types.hpp>

using autoware::common::state_estimation::StampedMeasurement2dPose64;
using autoware::common::state_estimation::StampedMeasurement2dSpeed64;
using autoware::common::state_estimation::StampedMeasurement2dPoseAndSpeed64;
using autoware::common::state_estimation::message_to_measurement;
using autoware::common::state_vector::variable::X;
using autoware::common::state_vector::variable::Y;
using autoware::common::state_vector::variable::X_VELOCITY;
using autoware::common::state_vector::variable::Y_VELOCITY;
using autoware::common::types::float32_t;

/// \test Create a measurement from odometry.
TEST(Measurement2dConversionTest, odom) {
  nav_msgs::msg::Odometry msg{};
  msg.header.frame_id = "map";
  msg.header.stamp.sec = 42;
  msg.header.stamp.nanosec = 0;
  msg.pose.pose.position.x = 42.0;
  msg.pose.pose.position.y = 23.0;
  // Rotation around z axis by 90 degrees.
  msg.pose.pose.orientation.x = 0.0;
  msg.pose.pose.orientation.y = 0.0;
  msg.pose.pose.orientation.z = 0.7071068;
  msg.pose.pose.orientation.w = 0.7071068;
  msg.pose.covariance[0] = 1.0;
  msg.pose.covariance[7] = 2.0;
  msg.twist.twist.linear.x = 23.0;
  msg.twist.twist.linear.y = 42.0;
  msg.twist.covariance[0] = 3.0;
  msg.twist.covariance[7] = 4.0;
  const auto measurement = message_to_measurement<StampedMeasurement2dPoseAndSpeed64>(msg);
  EXPECT_DOUBLE_EQ(measurement.measurement.state().at<X>(), 42.0);
  EXPECT_DOUBLE_EQ(measurement.measurement.state().at<Y>(), 23.0);
  const auto x_idx = measurement.measurement.state().index_of<X>();
  const auto y_idx = measurement.measurement.state().index_of<Y>();
  EXPECT_DOUBLE_EQ(measurement.measurement.covariance()(x_idx, x_idx), 1.0);
  EXPECT_DOUBLE_EQ(measurement.measurement.covariance()(y_idx, y_idx), 2.0);

  const auto kPrecision = 0.00001;
  // Note that the expected values for x and y are switched because of the 90 deg rotation.
  EXPECT_NEAR(measurement.measurement.state().at<X_VELOCITY>(), -42.0, kPrecision);
  EXPECT_NEAR(measurement.measurement.state().at<Y_VELOCITY>(), 23.0, kPrecision);
  // Note that the expected values for x and y are switched because of the 90 deg rotation.
  const auto x_speed_idx = measurement.measurement.state().index_of<X_VELOCITY>();
  const auto y_speed_idx = measurement.measurement.state().index_of<Y_VELOCITY>();
  EXPECT_NEAR(measurement.measurement.covariance()(x_speed_idx, x_speed_idx), 4.0, kPrecision);
  EXPECT_NEAR(measurement.measurement.covariance()(y_speed_idx, y_speed_idx), 3.0, kPrecision);

  EXPECT_EQ(measurement.timestamp.time_since_epoch(), std::chrono::seconds{42LL});
}

/// \test Create a measurement from pose.
TEST(Measurement2dConversionTest, pose) {
  geometry_msgs::msg::PoseWithCovarianceStamped msg{};
  msg.header.frame_id = "map";
  msg.header.stamp.sec = 42;
  msg.header.stamp.nanosec = 0;
  msg.pose.pose.position.x = 42.0;
  msg.pose.pose.position.y = 23.0;
  msg.pose.covariance[0] = 1.0;
  msg.pose.covariance[7] = 2.0;
  const auto measurement = message_to_measurement<StampedMeasurement2dPose64>(
    msg);
  EXPECT_DOUBLE_EQ(measurement.measurement.state().vector().x(), 42.0);
  EXPECT_DOUBLE_EQ(measurement.measurement.state().vector().y(), 23.0);
  EXPECT_DOUBLE_EQ(measurement.measurement.covariance()(0, 0), 1.0);
  EXPECT_DOUBLE_EQ(measurement.measurement.covariance()(1, 1), 2.0);
  EXPECT_EQ(
    measurement.timestamp.time_since_epoch(),
    std::chrono::seconds{42LL});
}

/// \test Create a measurement from a relative pose.
TEST(Measurement2dConversionTest, RelativePose) {
  autoware_auto_msgs::msg::RelativePositionWithCovarianceStamped msg{};
  msg.header.frame_id = "map";
  msg.header.stamp.sec = 42;
  msg.header.stamp.nanosec = 0;
  msg.position.x = 42.0;
  msg.position.y = 23.0;
  msg.covariance[0] = 1.0;
  msg.covariance[4] = 2.0;
  msg.covariance[8] = 3.0;
  const auto measurement = message_to_measurement<StampedMeasurement2dPose64>(msg);
  EXPECT_DOUBLE_EQ(measurement.measurement.state().vector().x(), 42.0);
  EXPECT_DOUBLE_EQ(measurement.measurement.state().vector().y(), 23.0);
  EXPECT_DOUBLE_EQ(measurement.measurement.covariance()(0, 0), 1.0);
  EXPECT_DOUBLE_EQ(measurement.measurement.covariance()(1, 1), 2.0);
  EXPECT_EQ(
    measurement.timestamp.time_since_epoch(),
    std::chrono::seconds{42LL});
}

/// \test Create a measurement from twist.
TEST(Measurement2dConversionTest, twist) {
  geometry_msgs::msg::TwistWithCovarianceStamped msg{};
  msg.header.frame_id = "map";
  msg.header.stamp.sec = 42;
  msg.header.stamp.nanosec = 0;
  msg.twist.twist.linear.x = 23.0;
  msg.twist.twist.linear.y = 42.0;
  msg.twist.covariance[0] = 3.0;
  msg.twist.covariance[7] = 4.0;
  const auto measurement = message_to_measurement<StampedMeasurement2dSpeed64>(
    msg);
  EXPECT_DOUBLE_EQ(measurement.measurement.state().vector()[0], 23.0);
  EXPECT_DOUBLE_EQ(measurement.measurement.state().vector()[1], 42.0);
  EXPECT_DOUBLE_EQ(measurement.measurement.covariance()(0, 0), 3.0);
  EXPECT_DOUBLE_EQ(measurement.measurement.covariance()(1, 1), 4.0);

  EXPECT_EQ(
    measurement.timestamp.time_since_epoch(),
    std::chrono::seconds{42LL});
}
