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

/// \copyright Copyright 2021 Apex.AI, Inc.
/// All rights reserved.

#include <gtest/gtest.h>

#include <measurement_conversion/measurement_conversion.hpp>

using autoware::common::state_estimation::StampedMeasurement2dPose;
using autoware::common::state_estimation::StampedMeasurement2dSpeed;
using autoware::common::state_estimation::StampedMeasurement2dPoseAndSpeed;
using autoware::common::state_estimation::message_to_measurement;

/// \test Create a measurement from odometry.
TEST(Measurement2dConversionTest, odom) {
  nav_msgs::msg::Odometry msg{};
  msg.header.frame_id = "map";
  msg.header.stamp.sec = 42;
  msg.header.stamp.nanosec = 0;
  msg.pose.pose.position.x = 42.0;
  msg.pose.pose.position.y = 23.0;
  msg.pose.covariance[0] = 1.0;
  msg.pose.covariance[7] = 2.0;
  msg.twist.twist.linear.x = 23.0;
  msg.twist.twist.linear.y = 42.0;
  msg.twist.covariance[0] = 3.0;
  msg.twist.covariance[7] = 4.0;
  const auto measurement = message_to_measurement<StampedMeasurement2dPoseAndSpeed>(
    msg);
  EXPECT_FLOAT_EQ(measurement.measurement.state().vector().x(), 42.0F);
  EXPECT_FLOAT_EQ(measurement.measurement.state().vector().y(), 23.0F);
  EXPECT_FLOAT_EQ(measurement.measurement.covariance()(0, 0), 1.0F);
  EXPECT_FLOAT_EQ(measurement.measurement.covariance()(1, 1), 2.0F);

  EXPECT_FLOAT_EQ(measurement.measurement.state().vector()[2], 23.0F);
  EXPECT_FLOAT_EQ(measurement.measurement.state().vector()[3], 42.0F);
  EXPECT_FLOAT_EQ(measurement.measurement.covariance()(2, 2), 3.0F);
  EXPECT_FLOAT_EQ(measurement.measurement.covariance()(3, 3), 4.0F);

  EXPECT_EQ(
    measurement.timestamp.time_since_epoch(),
    std::chrono::seconds{42LL});
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
  const auto measurement = message_to_measurement<StampedMeasurement2dPose>(
    msg);
  EXPECT_FLOAT_EQ(measurement.measurement.state().vector().x(), 42.0F);
  EXPECT_FLOAT_EQ(measurement.measurement.state().vector().y(), 23.0F);
  EXPECT_FLOAT_EQ(measurement.measurement.covariance()(0, 0), 1.0F);
  EXPECT_FLOAT_EQ(measurement.measurement.covariance()(1, 1), 2.0F);
  EXPECT_EQ(
    measurement.timestamp.time_since_epoch(),
    std::chrono::seconds{42LL});
}

/// \test Create a measurement from tist.
TEST(Measurement2dConversionTest, twist) {
  geometry_msgs::msg::TwistWithCovarianceStamped msg{};
  msg.header.frame_id = "map";
  msg.header.stamp.sec = 42;
  msg.header.stamp.nanosec = 0;
  msg.twist.twist.linear.x = 23.0;
  msg.twist.twist.linear.y = 42.0;
  msg.twist.covariance[0] = 3.0;
  msg.twist.covariance[7] = 4.0;
  const auto measurement = message_to_measurement<StampedMeasurement2dSpeed>(
    msg);
  EXPECT_FLOAT_EQ(measurement.measurement.state().vector()[0], 23.0F);
  EXPECT_FLOAT_EQ(measurement.measurement.state().vector()[1], 42.0F);
  EXPECT_FLOAT_EQ(measurement.measurement.covariance()(0, 0), 3.0F);
  EXPECT_FLOAT_EQ(measurement.measurement.covariance()(1, 1), 4.0F);

  EXPECT_EQ(
    measurement.timestamp.time_since_epoch(),
    std::chrono::seconds{42LL});
}
